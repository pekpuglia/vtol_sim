use crate::{background::Background, reference_frame::{ReferenceFrame, SCREEN_FRAME}};

use super::*;
use control_systems::{NegativeFeedback, Series};
use derive_new::new;
use egaku2d::glutin::event::{KeyboardInput, VirtualKeyCode};
use nalgebra::Vector2;

#[derive(new, Clone)]
struct BicopterForceAngleInputReceiver {
    force_gain: f64,
    angle_gain: f64
}

impl BicopterForceAngleInputReceiver {
    fn targets(&self, ev: &egaku2d::glutin::event::Event<'_, ()>) -> Option<nalgebra::DVector<f64>> {
        if let Event::WindowEvent {event: WindowEvent::CursorMoved { device_id: _, position: p, .. }, window_id: _} = ev {
            
            let force = self.force_gain * (1.0 - (p.y)/HEI as f64);

            let angle = self.angle_gain * ((p.x)/WID as f64 - 1.0/2.0);
            Some(dvector![force, angle, 0.0])
        } else {
            None
        }
    }
}

#[derive(new, Clone)]
struct PDController {
    kp: f64,
    kd: f64,
}

impl DynamicalSystem for PDController {
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

#[derive(new, Clone)]
struct AngleFeedbackAdapter;

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

#[derive(new, Clone)]
struct AngleFeedbackBicopter {
    plant: NegativeFeedback<Series<PDController, BicopterDynamicalModel>, AngleFeedbackAdapter>,
    #[new(value="dvector![
        WID as f64/2.0,
        -HEI as f64/2.0,
        0.0,
        0.0,
        0.0,
        0.0
    ]")]
    x: nalgebra::DVector<f64>,
    #[new(value="dvector![
        0.0,
        0.0
    ]")]
    u: nalgebra::DVector<f64>,
    ref_frame: ReferenceFrame,
    input_receiver: BicopterForceAngleInputReceiver
}

impl ode_solvers::System<ode_solvers::DVector<f64>> for AngleFeedbackBicopter {
    fn system(&self, x: f64, y: &ode_solvers::DVector<f64>, dy: &mut ode_solvers::DVector<f64>) {
        dy.copy_from_slice(self.plant.xdot(
            x, 
            nalgebra::DVector::from_row_slice(y.as_slice()), 
            self.u.clone()).as_slice())
    }
}

impl AngleFeedbackBicopter {
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

impl Component for AngleFeedbackBicopter {
    fn draw(&mut self, canvas: &mut egaku2d::SimpleCanvas, dt: f32, paused: bool) {
        if !paused {
            self.update(dt as f64);

        }

        let output = self.plant.y(0.0, self.x.clone(), self.u.clone());
        let error = self.u.clone() - self.plant.rev_ref().y(0.0, dvector![], output);
        let thrusts = self.plant.dir_ref().ds1_ref().y(0.0, dvector![], error.clone());

        self.plant
            .dir_ref()
            .ds2_ref()
            .body_centered_geometry(&self.x, &thrusts, &self.ref_frame)
            .iter()
            .map(|geom| geom.draw(canvas))
            .last();
    }

    fn receive_event(&mut self, ev: &Event<'_, ()>) {
        match self.input_receiver.targets(ev) {
            Some(targets) => {self.u = targets},
            None => {},
        }
    }
}

enum CameraOptions {
    VehicleCentered,
    Fixed
}

struct World {
    bicopter: AngleFeedbackBicopter,
    background: Background,
    camera_option: CameraOptions,
    camera_option_toggle: bool
}

impl Component for World {
    fn draw(&mut self, canvas: &mut SimpleCanvas, dt: f32, paused: bool) {
        match self.camera_option {
            CameraOptions::VehicleCentered => {
                let ref_frame = ReferenceFrame::new_from_screen_frame(
                    &Vector2::x(), 
                    &-Vector2::y(), 
                &Vector2::new(WID as f64/2.0 - self.bicopter.x[0], HEI as f64/2.0 + self.bicopter.x[1]));
                self.bicopter.ref_frame = ref_frame;
                self.background.ref_frame = ref_frame;
            },
            CameraOptions::Fixed => {},
        }
        self.background.draw(canvas, dt, paused);
        self.bicopter.draw(canvas, dt, paused)
    }

    fn receive_event(&mut self, ev: &Event<'_, ()>) {
        if let Event::WindowEvent { 
            event: WindowEvent::KeyboardInput { 
                input: KeyboardInput { virtual_keycode: Some(VirtualKeyCode::V), .. }, .. }, .. } = ev 
            {
            if !self.camera_option_toggle {
                self.camera_option = match self.camera_option {
                    CameraOptions::VehicleCentered => CameraOptions::Fixed,
                    CameraOptions::Fixed => CameraOptions::VehicleCentered,
                };
                self.camera_option_toggle = true;
            } else {
                self.camera_option_toggle = false;
            }
        }
        self.background.receive_event(ev);
        self.bicopter.receive_event(ev);
    }
}

pub fn bicopter_main() {
    let ev_loop = egaku2d::glutin::event_loop::EventLoop::new();

    let ref_frame = ReferenceFrame::new_from_screen_frame(
        &Vector2::x(), 
        &-Vector2::y(), 
        &Vector2::new(0.0,0.0));

    let mut drawer = Drawer::new(
        60, 
        WID as usize, 
        HEI as usize, 
        "test", 
        &ev_loop,
        vec![
            Box::new(World { 
                bicopter: AngleFeedbackBicopter::new(
                    NegativeFeedback::new(
                        Series::new(
                            PDController { kp: 1000.0, kd: 2000.0 },
                            BicopterDynamicalModel::new(
                                1000.0, 
                                1.0, 
                                100.0, 
                                40.0
                            )
                        ), 
                        AngleFeedbackAdapter::new()
                    ), ref_frame,
                    BicopterForceAngleInputReceiver { force_gain: 200.0, angle_gain: -std::f64::consts::FRAC_PI_2}
                ), background: Background::new(
                    Vector2::new(0.0,0.0), 
                    100.0, 
                    [1.0,0.0,0.0,0.3], 
                    [0.0,1.0,0.0,0.3], 
                    WID.into(), 
                    HEI.into()), camera_option: CameraOptions::Fixed, camera_option_toggle: false })
        ]);

    ev_loop.run(move |event, _, control_flow| main_loop(event, control_flow, &mut drawer));
}