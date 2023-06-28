mod graphical_utils;
use graphical_utils::*;
use cgmath::{Vector2, Matrix2, InnerSpace, SquareMatrix};

struct Ball {
    pos: Vector2<f32>,
    vel: Vector2<f32>,
    damp: f32,
    r: f32,
    is_held: bool,
    current_mouse_pos: Vector2<f32>,
    last_mouse_pos: Vector2<f32>
}

fn reflect(axis: Vector2<f32>, to_reflect: Vector2<f32>) -> Vector2<f32> {
    let norm_axis = axis.normalize();
    let axis_base = Matrix2::new(norm_axis.x, norm_axis.y, -norm_axis.y, norm_axis.x);
    axis_base * Matrix2::new(1.0, 0.0, 0.0, -1.0) * axis_base.invert().expect("axis_base should be isometric") * to_reflect
}

impl Ball {
    fn new(pos: [f32;2], r: f32, damp:f32) -> Ball {
        Ball {pos: pos.into(), vel: [0.0,0.0].into(), 
            r, is_held: false, 
            current_mouse_pos: [0.0,0.0].into(), 
            last_mouse_pos: [0.0, 0.0].into(), damp}
    }

    fn dynamics(&mut self, dt: f32) {
        match self.is_held {
            true => {
                self.pos = self.current_mouse_pos;
                self.vel = (self.current_mouse_pos - self.last_mouse_pos) / dt;
            },
            false => {
                self.vel -= self.vel * self.damp;
                self.pos = self.pos + dt * self.vel;
                if self.pos.x < 0.0 || self.pos.x > 600.0 {
                    self.vel = reflect([0.0, 1.0].into(), self.vel)
                }

                if self.pos.y < 0.0 || self.pos.y > 480.0 {
                    self.vel = reflect([1.0, 0.0].into(), self.vel)
                }
            },
        }
    }
}

use egaku2d::glutin::event::{Event, WindowEvent, ElementState, MouseButton};
impl Component for Ball {
    fn draw(&mut self, canvas: &mut egaku2d::SimpleCanvas, dt: f32) {
        self.dynamics(dt);
        canvas
            .circles()
            .add(self.pos.into())
            .send_and_uniforms(canvas, self.r)
            .with_color([1.0,1.0,1.0,1.0])
            .draw();
    }

    fn receive_event(&mut self, ev: &Event<'_, ()>) {
        if let Event::WindowEvent { event: WindowEvent::MouseInput { device_id: _, state, button, ..}, .. } = ev {
            match (state, button) {
                (ElementState::Pressed, MouseButton::Left) => {
                    let delta_x = self.current_mouse_pos[0] - self.pos[0];
                    let delta_y = self.current_mouse_pos[1] - self.pos[1];
                    self.is_held = (delta_x.powi(2) + delta_y.powi(2)).sqrt() < self.r;
                },
                (ElementState::Released, MouseButton::Left) => {self.is_held = false}
                _ => {}
            }
        }

        if let Event::WindowEvent {event: WindowEvent::CursorMoved { device_id: _, position: p, .. }, window_id: _} = ev {
            self.last_mouse_pos = self.current_mouse_pos;
            self.current_mouse_pos = Vector2::new(p.x as f32, p.y as f32);
        }
    }
}

fn main() {
    let ev_loop = egaku2d::glutin::event_loop::EventLoop::new();
    
    let mut drawer = Drawer::new(
        60, 
        600, 
        480, 
        "test", 
        &ev_loop,
        vec![
            Box::new(Ball::new([300.0, 240.0], 30.0, 0.01))
        ]);

    ev_loop.run(move |event, _, control_flow| main_loop(event, control_flow, &mut drawer));
}
