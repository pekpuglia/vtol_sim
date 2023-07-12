use nalgebra::{Rotation2, Vector2};
use ode_solvers::Vector6;
use ode_solvers::{Rk4, System};

#[derive(Clone, Copy)]
pub struct BicopterPlant {
    pos: Vector2<f32>,
    vel: Vector2<f32>,
    //sentido horário!
    angle_rad: f32,
    ang_vel: f32,
    inertia: f32,
    mass: f32,
    gravity: f32,
    prop_dist: f32,
    pub l_thrust: f32,
    pub r_thrust:f32
}

impl System<Vector6<f64>> for BicopterPlant {
    fn system(&self, _x: f64, y: &Vector6<f64>, dy: &mut Vector6<f64>) {
        let thrust = (self.l_thrust + self.r_thrust) * self.propeller_direction();
        dy.x = y.w;
        dy.y = y.a;
        dy.z = y.b;
        dy.w = (thrust.x / self.mass) as f64;
        dy.a = (thrust.y / self.mass + self.gravity) as f64;
        dy.b = (self.prop_dist * (self.l_thrust - self.r_thrust) / self.inertia) as f64;
    }
}

impl BicopterPlant {

    pub fn propeller_direction(&self) -> Vector2<f32> {
        Vector2::new(self.angle_rad.sin(), -self.angle_rad.cos())
    }

    pub fn pos(&self) -> Vector2<f32> {
        self.pos
    }

    pub fn left_right_positions(&self) -> (Vector2<f32>, Vector2<f32>) {
        let prop_dir = self.propeller_direction();
        
        let left_right = Rotation2::new(std::f32::consts::FRAC_PI_2) * prop_dir;

        let left = self.pos - self.prop_dist/2.0 * left_right;

        let right =  self.pos() + self.prop_dist/2.0 * left_right;

        (left, right)
    }

    pub fn thrusts(&self) -> (f32, f32) {
        (self.l_thrust, self.r_thrust)
    }

    pub fn prop_dist(&self) -> f32 {
        self.prop_dist
    }

    pub fn update(&mut self, dt: f64) {
        let mut stepper = Rk4::new(*self, 
            0.0, 
            self.state_vector(), 
            dt, 
        dt/5.0);

        let _stats = stepper.integrate();

        let final_state_vector = stepper.y_out().last().expect("should have integrated at least 1 step");

        self.pos.x     = final_state_vector.x as f32;
        self.pos.y     = final_state_vector.y as f32;
        self.angle_rad = final_state_vector.z as f32;
        self.vel.x     = final_state_vector.w as f32;
        self.vel.y     = final_state_vector.a as f32;
        self.ang_vel   = final_state_vector.b as f32;
    }

    fn state_vector(&self) -> Vector6<f64> {
        Vector6::new(
        self.pos.x as f64, 
        self.pos.y as f64, 
        self.angle_rad as f64,
        self.vel.x as f64, 
        self.vel.y as f64, 
        self.ang_vel as f64)
    }

    pub fn new(
        pos: Vector2<f32>,
        vel: Vector2<f32>,
        //sentido horário!
        angle_rad: f32,
        ang_vel: f32,
        inertia: f32,
        mass: f32,
        gravity: f32,
        prop_dist: f32,
        l_thrust: f32,
        r_thrust:f32
     ) -> BicopterPlant {
        BicopterPlant {
            pos,
            vel,
            angle_rad,
            ang_vel,
            inertia,
            mass,
            gravity,
            prop_dist,
            l_thrust,
            r_thrust
        }
    }
}