use control_systems::DynamicalSystem;
use nalgebra::dvector;

#[derive(Clone, Copy, derive_new::new)]
struct PD {
    kp: f64,
    kd: f64
}

impl DynamicalSystem for PD {
    const STATE_VECTOR_SIZE: usize = 0;

    const INPUT_SIZE      : usize = 2;

    const OUTPUT_SIZE     : usize = 1;

    fn xdot(&self, t: f64, 
        x: nalgebra::DVector<f64>, 
        u: nalgebra::DVector<f64>) -> nalgebra::DVector<f64> {
        dvector![]
    }

    fn y(&self, t: f64, 
        x: nalgebra::DVector<f64>, 
        u: nalgebra::DVector<f64>) -> nalgebra::DVector<f64> {
        dvector![self.kp * x[0] + self.kd * x[1]]
    }
}

#[derive(Clone, Copy, derive_new::new)]
struct PID {
    kp: f64,
    ki: f64,
    kd: f64
}

impl DynamicalSystem for PID {
    const STATE_VECTOR_SIZE: usize = 1;

    const INPUT_SIZE      : usize = 2;

    const OUTPUT_SIZE     : usize = 1;

    fn xdot(&self, t: f64, 
        x: nalgebra::DVector<f64>, 
        u: nalgebra::DVector<f64>) -> nalgebra::DVector<f64> {
        dvector![u[0]]
    }

    fn y(&self, t: f64, 
        x: nalgebra::DVector<f64>, 
        u: nalgebra::DVector<f64>) -> nalgebra::DVector<f64> {
        dvector![self.kp * u[0] + self.kd * u[1] + self.ki * x[0]]
    }
}