use std::ops::Add;

use crate::graphical_utils::Component;
use crate::reference_frame::{ReferenceFrame, ConvertToFrame, SCREEN_FRAME};
use crate::vehicles::{vehicle_main, GenericVehicle, InputReceiver, PhysicalModel};

use control_systems::UnitySystem;
use control_systems::{NegativeFeedback, Series, StateVector, Parallel, IntoSV};
use nalgebra::{vector, Vector2};

use crate::vehicles::{controllers::{PID, PD}, Vehicle};

use super::*;
use super::angle_controlled_bicopter::*;

#[derive(Clone, Copy)]
pub struct BicopterPositionInputReceiver {
}

impl InputReceiver for BicopterPositionInputReceiver {
    fn u(&self, ev: &Event<'_, ()>) -> Option<DVector<f64>> {
        if let Event::WindowEvent { event: WindowEvent::CursorMoved { position, .. }, .. } = ev {
            Some(dvector![position.x, position.y])
        } else {
            None
        }
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

type PositionControlledBicopter = GenericVehicle<PositionFeedbackLoop, BicopterPositionInputReceiver>;

impl Component for PositionControlledBicopter {
    fn draw(&mut self, canvas: &mut egaku2d::SimpleCanvas, dt: f32, paused: bool) {
        if !paused {
            self.update(dt as f64);
        }

        let sv = StateVector::<PositionFeedbackLoop>::new(self.x.clone());

        let position_error = self.model.error(0.0, sv.data.clone(), self.u.clone());

        let force_moment = self.model
            .dir_ref()
            .y1(0.0, 
                sv.dirx().data, 
                position_error
        );

        let angle_error = self.model
            .dir_ref()
            .ds2_ref()
            .error(0.0, 
                sv.dirx().x2().data, 
                force_moment
        );

        let thrusts = self.model
            .dir_ref()
            .ds2_ref()
            .dir_ref()
            .y1(0.0, 
                sv.dirx().x2().dirx().data, 
                angle_error
        );

        self.model
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
        let u = self.input.u(ev);
        if let Some(new_u) = u {
            let position_target = vector![new_u[0], new_u[1]].position_to_frame(&SCREEN_FRAME, &self.ref_frame);
            self.u = dvector![
                position_target[0], 0.0,
                position_target[1], 0.0
            ]
        }
    }
}

impl Vehicle for PositionControlledBicopter {
    fn set_reference_frame(&mut self, new_ref_frame: &ReferenceFrame) {
        self.ref_frame = *new_ref_frame;
    }

    fn x(&self) -> &DVector<f64> {
        &self.x
    }

    fn x_mut(&mut self) -> &mut DVector<f64> {
        &mut self.x
    }
}

const WID: f64 = 600.0;
const HEI: f64 = 480.0;

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

    vehicle_main(PositionControlledBicopter{
        model: position_feedback_loop,
        input: BicopterPositionInputReceiver {},
        ref_frame,
        u: DVector::zeros(PositionFeedbackLoop::INPUT_SIZE),
        x: initial_state_vector.data
    }, WID, HEI)
}