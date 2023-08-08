use super::*;
use control_systems::{NegativeFeedback, Series};
use derive_new::new;

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

            Some(dvector![force, angle])
        } else {
            None
        }
    }
}

#[derive(new, Clone)]
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

#[derive(new, Clone)]
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

#[derive(new, Clone)]
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
    }
}

impl Component for AngleFeedbackBicopter {
    fn draw(&mut self, canvas: &mut egaku2d::SimpleCanvas, dt: f32, paused: bool) {
        if !paused {
            self.update(dt as f64);
        }

        let prop_dir = BicopterDynamicalModel::propeller_direction(&self.x).map(|x| x as f32);

        let (left, right) = {
            let tmp = self.plant.left_right_positions(&self.x);

            (tmp.0.map(|x| x as f32), tmp.1.map(|x| x as f32))
        };

        let l_thrust = self.u[0] as f32;
        let r_thrust = self.u[1] as f32;

        //corpo
        canvas
            .lines(5.0)
            .add(left.into(),right.into())
            .send_and_uniforms(canvas)
            .with_color([1.0, 1.0, 1.0, 1.0])
            .draw();

        //empuxos
        canvas
            .arrows(2.0)
            .add(left.into(), (left + 0.7 * l_thrust * prop_dir).into())
            .send_and_uniforms(canvas)
            .with_color([1.0, 0.0, 0.0, 1.0])
            .draw();

        canvas
            .arrows(2.0)
            .add(right.into(), (right + 0.7 * r_thrust * prop_dir).into())
            .send_and_uniforms(canvas)
            .with_color([0.0, 0.0, 1.0, 1.0])
            .draw();
    }

    fn receive_event(&mut self, ev: &Event<'_, ()>) {
        match self.input_receiver.targets(ev) {
            Some(targets) => {self.u = targets},
            None => {},
        }
    }
}