use std::ops::Add;

use crate::reference_frame::{ReferenceFrame, ConvertToFrame, SCREEN_FRAME};

use super::*;
use control_systems::{NegativeFeedback, Series};
use derive_new::new;
use nalgebra::Vector2;

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

#[derive(new, Clone)]
pub struct AnglePDControllerHAL {
    pub kp: f64,
    pub kd: f64,
}

impl DynamicalSystem for AnglePDControllerHAL {
    const STATE_VECTOR_SIZE: usize = 0;

    const INPUT_SIZE      : usize = 3;

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

        let theta_error = u[1];
        let theta_dot = u[2];
        let moment = self.kp * theta_error + self.kd * theta_dot;
        let l_thrust = force / 2.0 + moment / 2.0;
        let r_thrust = force / 2.0 - moment / 2.0;

        dvector![l_thrust, r_thrust]
    }
}

struct PIDController {
    kp: f64,
    kd: f64,
    ki: f64
}

impl DynamicalSystem for PIDController {
    const STATE_VECTOR_SIZE: usize = 1;
    //x, xdot
    const INPUT_SIZE      : usize = 2;

    const OUTPUT_SIZE     : usize = 1;

    fn xdot(&self, t: f64, 
        x: DVector<f64>, 
        u: DVector<f64>) -> DVector<f64> {
        dvector![u[0]]
    }

    fn y(&self, t: f64, 
        x: DVector<f64>, 
        u: DVector<f64>) -> DVector<f64> {
        dvector![self.kp*u[0] + self.kd*u[1] + self.ki*x[0]]
    }
}

struct PositionErrorToForceAndMomentConverter {
    x_pid: PIDController,
    y_pid: PIDController
}

impl DynamicalSystem for PositionErrorToForceAndMomentConverter {
    const STATE_VECTOR_SIZE: usize = 2;

    const INPUT_SIZE      : usize = 4;

    const OUTPUT_SIZE     : usize = 2;

    fn xdot(&self, t: f64, 
        x: DVector<f64>, 
        u: DVector<f64>) -> DVector<f64> {
        let mut ret = self.x_pid.xdot(t, 
            x.rows(0, 1).into(), 
            u.rows(0, 2).into()
        );
        
        ret.extend(
            self.y_pid.xdot(t, 
                x.rows(1, 1).into(), 
                u.rows(2, 2).into()).iter().copied()
        );
        ret
    }

    fn y(&self, t: f64, 
        x: DVector<f64>, 
        u: DVector<f64>) -> DVector<f64> {
        let fx = self.x_pid.y(t, 
            x.rows(0, 1).into(), 
            u.rows(0, 2).into())[0];

        let fy = self.y_pid.y(t, 
            x.rows(1, 1).into(), 
            u.rows(2, 2).into())[0];

        let f = fx.powi(2).add(fy.powi(2)).sqrt();

        let theta = -fx.atan2(fy);

        dvector![
            f,
            theta,
            0.0
        ]
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
            x[0],
            x[3],
            x[1],
            x[4]
        ]
    }
}

type AngleFeedbackLoop = NegativeFeedback<Series<AnglePDControllerHAL, BicopterDynamicalModel>, AngleFeedbackAdapter>;

type PositionFeedbackLoop = NegativeFeedback<Series<PositionErrorToForceAndMomentConverter, AngleFeedbackLoop>, PositionFeedbackAdapter>;

// #[derive(new, Clone)]
// pub struct AngleFeedbackBicopter {
//     plant: NegativeFeedback<Series<AnglePDControllerHAL, BicopterDynamicalModel>, AngleFeedbackAdapter>,
//     #[new(value="dvector![
//         WID as f64/2.0,
//         -HEI as f64/2.0,
//         0.0,
//         0.0,
//         0.0,
//         0.0
//     ]")]
//     pub x: nalgebra::DVector<f64>,
//     #[new(value="dvector![
//         0.0,
//         0.0
//     ]")]
//     u: nalgebra::DVector<f64>,
//     pub ref_frame: ReferenceFrame,
//     input_receiver: BicopterForceAngleInputReceiver,
//     future_receiver: BicopterPositionInputReceiver
// }


// impl ode_solvers::System<ode_solvers::DVector<f64>> for AngleFeedbackBicopter {
//     fn system(&self, x: f64, y: &ode_solvers::DVector<f64>, dy: &mut ode_solvers::DVector<f64>) {
//         dy.copy_from_slice(self.plant.xdot(
//             x, 
//             nalgebra::DVector::from_row_slice(y.as_slice()), 
//             self.u.clone()).as_slice())
//     }
// }

// impl AngleFeedbackBicopter {
//     fn update(&mut self, dt: f64) {
//         let mut stepper = Rk4::new(
//             self.clone(),
//             0.0,
//             ode_solvers::DVector::from_row_slice(self.x.as_slice()),
//             dt,
//             dt/5.0
//         );

//         let _stats = stepper.integrate();

//         self.x.copy_from_slice(stepper.y_out().last().expect("should have integrated at least 1 step").as_slice());
//     }
// }

// impl Component for AngleFeedbackBicopter {
//     fn draw(&mut self, canvas: &mut egaku2d::SimpleCanvas, dt: f32, paused: bool) {
//         if !paused {
//             self.update(dt as f64);

//         }

//         dbg!(self.future_receiver.position_target(&self.ref_frame));

//         let output = self.plant.y(0.0, self.x.clone(), self.u.clone());
//         let error = self.u.clone() - self.plant.rev_ref().y(0.0, dvector![], output);
//         let thrusts = self.plant.dir_ref().ds1_ref().y(0.0, dvector![], error.clone());

//         self.plant
//             .dir_ref()
//             .ds2_ref()
//             .body_centered_geometry(&self.x, &thrusts, &self.ref_frame)
//             .iter()
//             .map(|geom| geom.draw(canvas))
//             .last();
//     }

//     fn receive_event(&mut self, ev: &Event<'_, ()>) {
//         match self.input_receiver.targets(&self.ref_frame, ev) {
//             Some(targets) => {self.u = targets},
//             None => {},
//         }
//         self.future_receiver.update_mouse_position(ev);
//     }
// }

// impl Vehicle for AngleFeedbackBicopter {
//     fn set_reference_frame(&mut self, new_ref_frame: &ReferenceFrame) {
//         self.ref_frame = *new_ref_frame;
//     }

//     fn x(&self) -> &DVector<f64> {
//         &self.x
//     }
// }

use super::bicopter_main;

pub fn main() {

    // let ref_frame = ReferenceFrame::new_from_screen_frame(
    //     &Vector2::x(), 
    //     &-Vector2::y(), 
    //     &Vector2::new(0.0,0.0));

    // bicopter_main(
    //     AngleFeedbackBicopter::new(
    //         NegativeFeedback::new(
    //             Series::new(
    //                 AnglePDControllerHAL { kp: 1000.0, kd: 2000.0 },
    //                 BicopterDynamicalModel::new(
    //                     1000.0, 
    //                     1.0, 
    //                     100.0, 
    //                     40.0
    //                 )
    //             ), 
    //             AngleFeedbackAdapter::new()
    //         ), ref_frame,
    //         BicopterForceAngleInputReceiver { force_gain: 200.0, angle_gain: -std::f64::consts::FRAC_PI_2},
    //         BicopterPositionInputReceiver{ mouse_screen_pos: Vector2::zeros() }
    //     )
    // )
}