use egaku2d::glutin::event::{Event, WindowEvent, MouseButton, ElementState};
use nalgebra::{Rotation2, Vector2};
use ode_solvers::Vector6;
use ode_solvers::{Rk4, System};

use crate::graphical_utils::*;
const WID: f32 = 600.0;

const HEI: f32 = 480.0;

mod bicopter_physics;

struct Bicopter {
    bicopter_model: bicopter_physics::BicopterModel,
    paused: bool,
    max_thrust: f32
}

impl Component for Bicopter {
    fn draw(&mut self, canvas: &mut egaku2d::SimpleCanvas, dt: f32) {
        if !self.paused {
            self.bicopter_model.update(dt as f64);
        }

        let prop_dir = self.bicopter_model.propeller_direction();

       let (left, right) = self.bicopter_model.left_right_positions();

        let (l_thrust, r_thrust) = self.bicopter_model.thrusts();

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
            .add(left.into(), (left + 70.0 * l_thrust/self.max_thrust * prop_dir).into())
            .send_and_uniforms(canvas)
            .with_color([1.0, 0.0, 0.0, 1.0])
            .draw();

        canvas
            .arrows(2.0)
            .add(right.into(), (right + 70.0 * r_thrust/self.max_thrust * prop_dir).into())
            .send_and_uniforms(canvas)
            .with_color([0.0, 0.0, 1.0, 1.0])
            .draw();

    }

    fn receive_event(&mut self, ev: &egaku2d::glutin::event::Event<'_, ()>) {
        if let Event::WindowEvent {event: WindowEvent::CursorMoved { device_id: _, position: p, .. }, window_id: _} = ev {
            let total_thrust = 2.0 * self.max_thrust * (1.0 - (p.y as f32)/HEI);

            let moment = 1e-1 * self.max_thrust * self.bicopter_model.prop_dist() * ((p.x as f32)/WID - 1.0/2.0);

            self.bicopter_model.l_thrust = total_thrust / 2.0 + moment / (2.0 * self.bicopter_model.prop_dist());
            self.bicopter_model.r_thrust = total_thrust / 2.0 - moment / (2.0 * self.bicopter_model.prop_dist());
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
                // pos:[WID/2.0,HEI/2.0].into(),
                // angle_rad: 0.0,
                // mass:1.0,
                // inertia:100.0,
                // gravity:100.0,
                // prop_dist:40.0,
                // max_thrust:100.0,
                // l_thrust:0.0,
                // r_thrust:0.0, 
                // vel: [0.0,0.0].into(), 
                // ang_vel: 0.0,
                paused: true,
                max_thrust: 100.0,
                bicopter_model: bicopter_physics::BicopterModel::new(
                    [WID/2.0,HEI/2.0].into(),
                    [0.0,0.0].into(),
                    0.0,
                    0.0,
                    100.0,
                    1.0,
                    100.0,
                    40.0,
                    0.0,
                    0.0
                )})
        ]);

    ev_loop.run(move |event, _, control_flow| main_loop(event, control_flow, &mut drawer));
}
