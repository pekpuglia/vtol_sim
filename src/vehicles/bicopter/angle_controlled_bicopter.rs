use crate::{
    graphical_utils::Component, reference_frame::ReferenceFrame, vehicles::{
        controllers::PD, screen_center_x, vehicle_main, GenericVehicle, InputReceiver, PhysicalModel}};

use super::*;
use control_systems::{NegativeFeedback, Series, Parallel};
use derive_new::new;
use nalgebra::Vector2;

#[derive(new, Clone)]
pub struct BicopterForceAngleInputReceiver {
    pub force_gain: f64,
    pub angle_gain: f64
}

impl BicopterForceAngleInputReceiver {
    fn targets(&self, ev: &egaku2d::glutin::event::Event<'_, ()>) -> Option<nalgebra::DVector<f64>> {
        if let Event::WindowEvent {event: WindowEvent::CursorMoved { device_id: _, position: p, .. }, window_id: _} = ev {
            
            let force = self.force_gain * (1.0 - (p.y)/HEI as f64);

            let angle = self.angle_gain * ((p.x)/WID as f64 - 1.0/2.0);
            // 3o - thetadot? remover
            Some(dvector![force, angle, 0.0])
        } else {
            None
        }
    }
}

impl InputReceiver for BicopterForceAngleInputReceiver {
    fn u(&self, ev: &Event<'_, ()>) -> Option<DVector<f64>> {
        self.targets(ev)
    }
}

#[derive(new, Clone)]
pub struct HAL;

impl DynamicalSystem for HAL {
    const STATE_VECTOR_SIZE: usize = 0;

    const INPUT_SIZE      : usize = 2;

    const OUTPUT_SIZE     : usize = 2;

    fn xdot(&self, _t: f64, 
        _x: nalgebra::DVector<f64>, 
        _u: nalgebra::DVector<f64>) -> nalgebra::DVector<f64> {
        dvector![]
    }

    fn y(&self, _t: f64, 
        _x: nalgebra::DVector<f64>, 
        u: nalgebra::DVector<f64>) -> nalgebra::DVector<f64> {
        let force = u[0];
        let moment = u[1];
        let l_thrust = force / 2.0 - moment / 2.0;
        let r_thrust = force / 2.0 + moment / 2.0;

        dvector![l_thrust, r_thrust]
    }
}

#[derive(new, Clone)]
pub struct AngleFeedbackAdapter;

impl DynamicalSystem for AngleFeedbackAdapter {
    const STATE_VECTOR_SIZE: usize = 0;

    const INPUT_SIZE      : usize = 6;

    const OUTPUT_SIZE     : usize = 3;

    fn xdot(&self, _t: f64, 
        _x: nalgebra::DVector<f64>, 
        _u: nalgebra::DVector<f64>) -> nalgebra::DVector<f64> {
        dvector![]
    }

    fn y(&self, _t: f64, 
        _x: nalgebra::DVector<f64>, 
        u: nalgebra::DVector<f64>) -> nalgebra::DVector<f64> {
        //força, angulo, v ang
        dvector![0.0, u[2], u[5]]
    }
}

use control_systems::UnitySystem;

pub type AngleController = Series<Parallel<UnitySystem<1>, crate::vehicles::controllers::PD>, HAL>;

pub type AngleDirectPath = Series<AngleController, BicopterDynamicalModel>;

pub type AngleControlledFeedback = NegativeFeedback<AngleDirectPath, AngleFeedbackAdapter>;

type AngleFeedbackBicopter = GenericVehicle<AngleControlledFeedback, BicopterForceAngleInputReceiver>;

impl Component for AngleFeedbackBicopter {
    fn draw(&mut self, canvas: &mut egaku2d::SimpleCanvas, dt: f32, paused: bool) {
        if !paused {
            self.update(dt as f64);

        }
        //algo errado aqui!!!  hb
        let output = self.model.y(0.0, self.x.clone(), self.u.clone());
        let error = self.u.clone() - self.model.rev_ref().y(0.0, dvector![], output);
        let thrusts = self.model
            .dir_ref()
            .ds1_ref()
            .y(0.0, dvector![], error.clone());

        self.model
            .dir_ref()
            .ds2_ref()
            .body_centered_geometry(&self.x, &thrusts, &self.ref_frame)
            .iter()
            .map(|geom| geom.draw(canvas))
            .last();
    }

    fn receive_event(&mut self, ev: &Event<'_, ()>) {
        match self.input.u(ev) {
            Some(targets) => {self.u = targets},
            None => {},
        }
    }
}

const WID: f64 = 600.0;
const HEI: f64 = 480.0;

#[allow(unused)]
pub fn main() {

    let ref_frame = ReferenceFrame::new_from_screen_frame(
        &Vector2::x(), 
        &-Vector2::y(), 
        &Vector2::new(0.0,0.0)
    );

    let ang_controller: AngleController = Series::new(
        Parallel::new(
            UnitySystem{}, 
            PD::new(18367.3, 6000.0)), 
        HAL{}
    );

    let direct_path: AngleDirectPath = Series::new(
        ang_controller, 
        BicopterDynamicalModel::new(
                1000.0, 
                1.0, 
                100.0, 
                40.0)
    );

    let angle_controlled_feedback: AngleControlledFeedback = NegativeFeedback::new(
        direct_path, 
        AngleFeedbackAdapter{}
    );

    let angle_feedback_bicopter = AngleFeedbackBicopter {
        model: angle_controlled_feedback,
        input: BicopterForceAngleInputReceiver { 
            force_gain: 200.0, 
            angle_gain: -std::f64::consts::FRAC_PI_2},
        x: screen_center_x(WID, HEI, AngleControlledFeedback::STATE_VECTOR_SIZE),
        u: DVector::zeros(AngleControlledFeedback::INPUT_SIZE),
        ref_frame
    };
    
    vehicle_main(angle_feedback_bicopter, WID, HEI)
}