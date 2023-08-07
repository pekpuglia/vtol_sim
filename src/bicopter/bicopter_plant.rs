use nalgebra::{Vector2, dvector, Rotation2};

use control_systems::DynamicalSystem;

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

// //remover estado
// #[derive(Clone)]
// pub struct BicopterPlant {
//     model: BicopterDynamicalModel,
//     x: DVector<f64>, //state vector
//     u: DVector<f64> //input
// }


// impl System<Vector6<f64>> for BicopterPlant {
//     fn system(&self, x: f64, y: &Vector6<f64>, dy: &mut Vector6<f64>) {
//         dy.copy_from_slice(self.model.xdot(x, DVector::from_row_slice(y.as_slice()), self.u).as_slice())
//     }
// }

// impl BicopterPlant {

//     pub fn propeller_direction(&self) -> Vector2<f32> {
//         Vector2::new(self.angle_rad.sin(), -self.angle_rad.cos())
//     }

//     pub fn pos(&self) -> Vector2<f32> {
//         self.pos
//     }

//     pub fn left_right_positions(&self) -> (Vector2<f32>, Vector2<f32>) {
//         let prop_dir = self.propeller_direction();
        
//         let left_right = Rotation2::new(std::f32::consts::FRAC_PI_2) * prop_dir;

//         let left = self.pos - self.prop_dist as f32/2.0 * left_right;

//         let right =  self.pos() + self.prop_dist as f32/2.0 * left_right;

//         (left, right)
//     }

//     pub fn thrusts(&self) -> (f32, f32) {
//         (self.l_thrust as f32, self.r_thrust as f32)
//     }

//     pub fn prop_dist(&self) -> f64 {
//         self.prop_dist
//     }

//     pub fn update(&mut self, dt: f64) {
//         let mut stepper = Rk4::new(*self, 
//             0.0, 
//             self.state_vector(), 
//             dt, 
//         dt/5.0);

//         let _stats = stepper.integrate();

//         let final_state_vector = stepper.y_out().last().expect("should have integrated at least 1 step");

//         self.pos.x     = final_state_vector.x as f32;
//         self.pos.y     = final_state_vector.y as f32;
//         self.angle_rad = final_state_vector.z as f32;
//         self.vel.x     = final_state_vector.w as f32;
//         self.vel.y     = final_state_vector.a as f32;
//         self.ang_vel   = final_state_vector.b as f32;
//     }

//     fn state_vector(&self) -> Vector6<f64> {
//         Vector6::new(
//         self.pos.x as f64, 
//         self.pos.y as f64, 
//         self.angle_rad as f64,
//         self.vel.x as f64, 
//         self.vel.y as f64, 
//         self.ang_vel as f64)
//     }

//     pub fn new(
//         pos: Vector2<f32>,
//         vel: Vector2<f32>,
//         //sentido horário!
//         angle_rad: f32,
//         ang_vel: f32,
//         inertia: f64,
//         mass: f64,
//         gravity: f64,
//         prop_dist: f64,
//         l_thrust: f64,
//         r_thrust:f64
//      ) -> BicopterPlant {
//         BicopterPlant {
//             pos,
//             vel,
//             angle_rad,
//             ang_vel,
//             inertia,
//             mass,
//             gravity,
//             prop_dist,
//             l_thrust,
//             r_thrust
//         }
//     }
// }