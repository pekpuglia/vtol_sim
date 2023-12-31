use std::ops::Add;

use crate::reference_frame::{ReferenceFrame, ConvertToFrame, SCREEN_FRAME};

use control_systems::UnitySystem;
use control_systems::{NegativeFeedback, Series, StateVector, Parallel, IntoSV};
use nalgebra::Vector2;

use crate::controllers::{PID, PD};

use super::*;
use super::angle_controlled_bicopter::*;

#[derive(Clone, Copy)]
pub struct BicopterPositionInputReceiver {
    mouse_screen_pos: Vector2<f64>
}

impl BicopterPositionInputReceiver {
    fn update_mouse_position(&mut self, ev: &egaku2d::glutin::event::Event<'_, ()>) {
        if let Event::WindowEvent { event: WindowEvent::CursorMoved { position, .. }, .. } = ev {
            self.mouse_screen_pos = Vector2::new(position.x, position.y);
        }
    }

    fn position_target(&self, world_ref_frame: &ReferenceFrame) -> Vector2<f64> {
        self.mouse_screen_pos.position_to_frame(&SCREEN_FRAME, world_ref_frame)
    }
}

#[derive(Clone, Copy)]
struct PositionTrackerToForceAngle {
    position_pids: Parallel<PID, PID>
}

impl DynamicalSystem for PositionTrackerToForceAngle {
    const STATE_VECTOR_SIZE: usize = <Parallel<PID, PID> as DynamicalSystem>::STATE_VECTOR_SIZE;

    const INPUT_SIZE      : usize = <Parallel<PID, PID> as DynamicalSystem>::INPUT_SIZE;

    const OUTPUT_SIZE     : usize = <Parallel<PID, PID> as DynamicalSystem>::OUTPUT_SIZE + 1;

    fn xdot(&self, t: f64, 
        x: DVector<f64>, 
        u: DVector<f64>) -> DVector<f64> {
        self.position_pids.xdot(t, x, u)
    }

    fn y(&self, t: f64, 
        x: DVector<f64>, 
        u: DVector<f64>) -> DVector<f64> {
        let fx_fy = self.position_pids.y(t, x, u);

        let fx = fx_fy[0]; let fy = fx_fy[1];

        let f = fx.powi(2).add(fy.powi(2)).sqrt();

        let theta = -fx.atan2(fy);

        dvector![
            f,
            theta,
            0.0
        ]
    }
}

#[derive(Clone, Copy)]
pub struct PositionFeedbackAdapter;

impl DynamicalSystem for PositionFeedbackAdapter {
    const STATE_VECTOR_SIZE: usize = 0;

    const INPUT_SIZE      : usize = 6;

    const OUTPUT_SIZE     : usize = 4;

    fn xdot(&self, t: f64, 
        x: DVector<f64>, 
        u: DVector<f64>) -> DVector<f64> {
        dvector![]
    }

    fn y(&self, t: f64, 
        x: DVector<f64>, 
        u: DVector<f64>) -> DVector<f64> {
        //x vx y vy
        dvector![
            u[0],
            u[3],
            u[1],
            u[4]
        ]
    }
}

type PositionFeedbackLoop = NegativeFeedback<Series<PositionTrackerToForceAngle, AngleControlledFeedback>, PositionFeedbackAdapter>;

#[derive(Clone)]
struct PositionControlledBicopter {
    system: PositionFeedbackLoop,
    x: nalgebra::DVector<f64>,
    u: nalgebra::DVector<f64>,
    pub ref_frame: ReferenceFrame,
    position_receiver: BicopterPositionInputReceiver
}

// //abstrair p vehicle
impl ode_solvers::System<ode_solvers::DVector<f64>> for PositionControlledBicopter {
    fn system(&self, x: f64, y: &ode_solvers::DVector<f64>, dy: &mut ode_solvers::DVector<f64>) {
        dy.copy_from_slice(self.system.xdot(
            x, 
            nalgebra::DVector::from_row_slice(y.as_slice()), 
            self.u.clone()).as_slice())
    }
}

// //esse também
impl PositionControlledBicopter {
    fn update(&mut self, dt: f64) {
        let mut stepper = Rk4::new(
            self.clone(),
            0.0,
            ode_solvers::DVector::from_row_slice(self.x.as_slice()),
            dt,
            dt/5.0
        );

        let _stats = stepper.integrate();

        self.x.copy_from_slice(stepper.y_out().last().expect("should have integrated at least 1 step").as_slice());
    }
}

impl Component for PositionControlledBicopter {
    fn draw(&mut self, canvas: &mut egaku2d::SimpleCanvas, dt: f32, paused: bool) {
        if !paused {
            self.update(dt as f64);
        }

        let sv = StateVector::<PositionFeedbackLoop>::new(self.x.clone());

        let position_error = self.system.error(0.0, sv.data.clone(), self.u.clone());

        let force_moment = self.system
            .dir_ref()
            .y1(0.0, 
                sv.dirx().data, 
                position_error
        );

        let angle_error = self.system
            .dir_ref()
            .ds2_ref()
            .error(0.0, 
                sv.dirx().x2().data, 
                force_moment
        );

        let thrusts = self.system
            .dir_ref()
            .ds2_ref()
            .dir_ref()
            .y1(0.0, 
                sv.dirx().x2().dirx().data, 
                angle_error
        );

        self.system
            .dir_ref()
            .ds2_ref()
            .dir_ref()
            .ds2_ref()
            .body_centered_geometry(&sv
                .dirx()
                .x2()
                .dirx()
                .x2()
                .data, &thrusts, &self.ref_frame)
            .iter()
            .map(|geom| geom.draw(canvas))
            .last();
    }

    fn receive_event(&mut self, ev: &Event<'_, ()>) {
        self.position_receiver.update_mouse_position(ev);
        let position_target = self.position_receiver.position_target(&self.ref_frame);
        self.u = dvector![
            position_target[0], 0.0,
            position_target[1], 0.0
        ]

    }
}

impl Vehicle for PositionControlledBicopter {
    fn set_reference_frame(&mut self, new_ref_frame: &ReferenceFrame) {
        self.ref_frame = *new_ref_frame;
    }

    fn x(&self) -> &DVector<f64> {
        &self.x
    }
}

use super::bicopter_main;

pub fn main() {

    let ref_frame = ReferenceFrame::new_from_screen_frame(
        &Vector2::x(), 
        &-Vector2::y(), 
        &Vector2::new(0.0,0.0));

        let ang_controller: AngleController = Series::new(
            Parallel::new(
                UnitySystem{}, 
                PD::new(18367.3, 6000.0)), 
            HAL{}
        );
    
        let angle_direct_path: AngleDirectPath = Series::new(
            ang_controller, 
            BicopterDynamicalModel::new(
                    1000.0, 
                    1.0, 
                    100.0, 
                    40.0)
        );
    
        let angle_controlled_feedback: AngleControlledFeedback = NegativeFeedback::new(
            angle_direct_path, 
            AngleFeedbackAdapter{}
        );

    let position_feedback_loop = NegativeFeedback::new(
        Series::new(PositionTrackerToForceAngle { position_pids: Parallel::new(
            PID::new(1.26, 0.0, 1.61), 
            PID::new(2.66, 0.73, 1.75)) }
            , angle_controlled_feedback), PositionFeedbackAdapter{}
    );

    let initial_state_vector: StateVector<PositionFeedbackLoop> = [0.0, 0.0].into_sv::<PositionTrackerToForceAngle>()
        .series(
            [WID as f64/2.0, -HEI as f64/2.0, 0.0, 0.0, 0.0, 0.0].into_sv::<AngleControlledFeedback>()
        ).feedback([].into_sv::<PositionFeedbackAdapter>());

    bicopter_main(PositionControlledBicopter{
        system: position_feedback_loop,
        position_receiver: BicopterPositionInputReceiver { mouse_screen_pos: Vector2::zeros() },
        ref_frame,
        u: dvector![0.0,0.0,0.0,0.0],
        x: initial_state_vector.data
    })
}