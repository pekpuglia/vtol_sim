use nalgebra::{Vector2, dvector, Rotation2};

pub use control_systems::DynamicalSystem;

#[derive(Clone, Copy)]
pub struct BicopterDynamicalModel {
    inertia: f64,
    mass: f64,
    gravity: f64,
    prop_dist: f64,
}

impl BicopterDynamicalModel {
    pub fn new(inertia: f64,
        mass: f64,
        gravity: f64,
        prop_dist: f64) -> BicopterDynamicalModel {
        BicopterDynamicalModel { inertia, mass, gravity, prop_dist }
    }

    pub fn propeller_direction(x: &nalgebra::DVector<f64>) -> Vector2<f64> {
        Vector2::new(x[2].sin(), -x[2].cos())
    }

    pub fn left_right_positions(&self, x: & nalgebra::DVector<f64>) -> (Vector2<f64>, Vector2<f64>) {
        let prop_dir = BicopterDynamicalModel::propeller_direction(x);
     
        let left_right = Rotation2::new(std::f64::consts::FRAC_PI_2) * prop_dir;
        let left = Vector2::new(x[0], x[1]) - self.prop_dist/2.0 * left_right;
        let right = Vector2::new(x[0], x[1]) + self.prop_dist/2.0 * left_right;
        (left, right)
    }
}

/*
    State Vector:
    0 x
    1 y
    2 theta
    3 xdot
    4 ydot
    5 thetadot

    Input:
    lthrust
    rthrust

    Output:
    State Vector
*/
impl DynamicalSystem for BicopterDynamicalModel {
    const STATE_VECTOR_SIZE: usize = 6;

    const INPUT_SIZE      : usize = 2;

    const OUTPUT_SIZE     : usize = 6;

    fn xdot(&self, _t: f64, 
        x: nalgebra::DVector<f64>, 
        u: nalgebra::DVector<f64>) -> nalgebra::DVector<f64> {
        let thrust = 
            (u[0] + u[1]) * BicopterDynamicalModel::propeller_direction(&x);
        dvector![
            x[3],
            x[4],
            x[5],
            thrust.x / self.mass,
            thrust.y / self.mass + self.gravity,
            self.prop_dist * (u[0] - u[1]) / self.inertia
        ]
    }

    fn y(&self, _t: f64, 
        x: nalgebra::DVector<f64>, 
        _u: nalgebra::DVector<f64>) -> nalgebra::DVector<f64> {
        x
    }
}