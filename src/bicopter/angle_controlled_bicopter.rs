use super::*;
use control_systems::{NegativeFeedback, Series};
use derive_new::new;

#[derive(new)]
struct BicopterForceAngleInputReceiver {
    force_gain: f64,
    angle_gain: f64
}

impl BicopterForceAngleInputReceiver {
    fn targets(&self, ev: &egaku2d::glutin::event::Event<'_, ()>) -> Option<nalgebra::DVector<f64>> {
        if let Event::WindowEvent {event: WindowEvent::CursorMoved { device_id: _, position: p, .. }, window_id: _} = ev {
            
            let force = self.force_gain * (1.0 - (p.y)/HEI as f64);

            let angle = self.angle_gain * ((p.x)/WID as f64 - 1.0/2.0);

            Some(dvector![force, angle])
        } else {
            None
        }
    }
}

#[derive(new)]
struct PDController {
    kp: f64,
    kd: f64
}

impl DynamicalSystem for PDController {
    const STATE_VECTOR_SIZE: usize = 0;

    const INPUT_SIZE      : usize = 2;

    const OUTPUT_SIZE     : usize = 2;

    fn xdot(&self, t: f64, 
        x: nalgebra::DVector<f64>, 
        u: nalgebra::DVector<f64>) -> nalgebra::DVector<f64> {
        todo!()
    }

    fn y(&self, t: f64, 
        x: nalgebra::DVector<f64>, 
        u: nalgebra::DVector<f64>) -> nalgebra::DVector<f64> {
        todo!()
    }
}

#[derive(new)]
struct AngleFeedbackAdapter;

impl DynamicalSystem for AngleFeedbackAdapter {
    const STATE_VECTOR_SIZE: usize = 0;

    const INPUT_SIZE      : usize = 6;

    const OUTPUT_SIZE     : usize = 2;

    fn xdot(&self, t: f64, 
        x: nalgebra::DVector<f64>, 
        u: nalgebra::DVector<f64>) -> nalgebra::DVector<f64> {
        dvector![]
    }

    fn y(&self, t: f64, 
        x: nalgebra::DVector<f64>, 
        u: nalgebra::DVector<f64>) -> nalgebra::DVector<f64> {
        dvector![0.0, x[2]]
    }
}

#[derive(new)]
struct AngleFeedbackBicopter {
    plant: NegativeFeedback<Series<PDController, BicopterDynamicalModel>, AngleFeedbackAdapter>,
    #[new(value="dvector![
        WID as f64/2.0,
        HEI as f64/2.0,
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
    input_receiver: BicopterForceAngleInputReceiver
}

impl AngleFeedbackBicopter {
    fn update(&mut self, dt: f64) {
        
    }
}