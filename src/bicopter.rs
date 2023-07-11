use egaku2d::glutin::event::{Event, WindowEvent, MouseButton, ElementState};
use nalgebra::{Rotation2, Vector2};
use ode_solvers::Vector6;
use ode_solvers::{Rk4, System};

use crate::graphical_utils::*;
const WID: f32 = 600.0;

const HEI: f32 = 480.0;

#[derive(Clone, Copy)]
struct Bicopter {
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
    r_thrust:f32,
    max_thrust: f32,
    paused: bool
}

impl System<Vector6<f64>> for Bicopter {
    fn system(&self, x: f64, y: &Vector6<f64>, dy: &mut Vector6<f64>) {
        let thrust = (self.l_thrust + self.r_thrust) * self.propeller_direction();
        dy.x = y.w;
        dy.y = y.a;
        dy.z = y.b;
        dy.w = (thrust.x / self.mass) as f64;
        dy.a = (thrust.y / self.mass + self.gravity) as f64;
        dy.b = (self.prop_dist * (self.l_thrust - self.r_thrust) / self.inertia) as f64;
    }
}

impl Bicopter {

    fn propeller_direction(&self) -> Vector2<f32> {
        Vector2::new(self.angle_rad.sin(), -self.angle_rad.cos())
    }

    fn update(&mut self, dt: f64) {
        let mut stepper = Rk4::new(*self, 
            0.0, 
            self.state_vector(), 
            dt, 
        dt/5.0);

        let stats = stepper.integrate();

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
}

impl Component for Bicopter {
    fn draw(&mut self, canvas: &mut egaku2d::SimpleCanvas, dt: f32) {
        if !self.paused {
            self.update(dt as f64);
        }

        let prop_dir = self.propeller_direction();
        
        let left_right = Rotation2::new(std::f32::consts::FRAC_PI_2) * prop_dir;

        let left = self.pos - self.prop_dist/2.0 * left_right;

        let right =  self.pos + self.prop_dist/2.0 * left_right;

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
            .add(left.into(), (left + 70.0 * self.l_thrust/self.max_thrust * prop_dir).into())
            .send_and_uniforms(canvas)
            .with_color([1.0, 0.0, 0.0, 1.0])
            .draw();

        canvas
            .arrows(2.0)
            .add(right.into(), (right + 70.0 * self.r_thrust/self.max_thrust * prop_dir).into())
            .send_and_uniforms(canvas)
            .with_color([0.0, 0.0, 1.0, 1.0])
            .draw();

    }

    fn receive_event(&mut self, ev: &egaku2d::glutin::event::Event<'_, ()>) {
        if let Event::WindowEvent {event: WindowEvent::CursorMoved { device_id: _, position: p, .. }, window_id: _} = ev {
            let total_thrust = 2.0 * self.max_thrust * (1.0 - (p.y as f32)/HEI);

            let moment = 1e-1 * self.max_thrust * self.prop_dist * ((p.x as f32)/WID - 1.0/2.0);

            self.l_thrust = total_thrust / 2.0 + moment / (2.0 * self.prop_dist);
            self.r_thrust = total_thrust / 2.0 - moment / (2.0 * self.prop_dist);
            dbg!(total_thrust, moment);
        }

        if let Event::WindowEvent { window_id, 
            event: WindowEvent::MouseInput { 
                device_id, state, button, modifiers } } = ev {
            match (state, button) {
                (ElementState::Pressed, MouseButton::Right) => {self.paused = !self.paused},
                _ => {}
            }
        }
    }
}

pub fn bicopter_main() {
    let ev_loop = egaku2d::glutin::event_loop::EventLoop::new();
    
    let mut drawer = Drawer::new(
        30, 
        WID as usize, 
        HEI as usize, 
        "test", 
        &ev_loop,
        vec![
            Box::new(Bicopter{
                pos:[WID/2.0,HEI/2.0].into(),
                angle_rad: 0.0,
                mass:1.0,
                inertia:100.0,
                gravity:100.0,
                prop_dist:40.0,
                max_thrust:100.0,
                l_thrust:0.0,
                r_thrust:0.0, 
                vel: [0.0,0.0].into(), 
                ang_vel: 0.0,
                paused: true })
        ]);

    ev_loop.run(move |event, _, control_flow| main_loop(event, control_flow, &mut drawer));
}
