use crate::graphical_utils::*;
use crate::math_helpers::*;
use nalgebra::Vector2;
use ode_solvers::Vector4;
use nalgebra::geometry::Rotation2;
use ode_solvers::{System, Dopri5};
const WID: f32 = 600.0;
const HEI: f32 = 480.0;

#[derive(enum_map::Enum, Debug, Clone, Copy)]
enum Walls {
    Top,
    Right,
    Left,
    Bottom
}

#[derive(Clone, Copy)]
struct Ball {
    pos: Vector2<f32>,
    vel: Vector2<f32>,
    damp: f64,
    r: f32,
    is_held: bool,
    current_mouse_pos: Vector2<f32>,
    last_mouse_pos: Vector2<f32>,
    //normals p/ dentro do mapa
    wall_point_and_normals: enum_map::EnumMap<Walls, (Vector2<f32>, Vector2<f32>)>,
}

impl System<Vector4<f64>> for Ball {
    fn system(&self, _x: f64, y: &Vector4<f64>, dy: &mut Vector4<f64>) {
        dy.x = y.z;
        dy.y = y.w;
        dy.z = -y.z*self.damp;
        dy.w = -y.w*self.damp;
    }

    fn solout(&mut self, _x: f64, y: &Vector4<f64>, _dy: &Vector4<f64>) -> bool {
        let current_pos = Vector2::new(y.x, y.y).cast();
        let first = self.wall_point_and_normals
            .iter()
            .map(|el| (el.0, el.1.1.dot(&(current_pos - el.1.0))))
            .filter(|(_wall, dot)| *dot < 0.0)
            .nth(0);
        first.is_some()
    }
}

impl Ball {
    fn new(pos: [f32;2], r: f32, damp: f64) -> Ball {
        Ball {pos: pos.into(), vel: [0.0,0.0].into(), 
            r, is_held: false, 
            current_mouse_pos: [0.0,0.0].into(), 
            last_mouse_pos: [0.0, 0.0].into(), damp,
            wall_point_and_normals: enum_map::enum_map! {
                Walls::Bottom => ([0.0, HEI-r].into(), [0.0, -1.0].into()),
                Walls::Left => ([r, 0.0].into(), [1.0, 0.0].into()),
                Walls::Right => ([WID-r, 0.0].into(), [-1.0, 0.0].into()),
                Walls::Top => ([0.0, r].into(), [0.0, 1.0].into()), },
        }
    }

    fn dynamics(&mut self, dt: f32) {
        match self.is_held {
            true => {
                self.pos = self.current_mouse_pos
                    .zip_map(&Vector2::new(self.r, self.r), |v1, v2| v1.max(v2))
                    .zip_map(&Vector2::new(WID-self.r, HEI-self.r), |v1, v2| v1.min(v2));
                self.vel = (self.current_mouse_pos - self.last_mouse_pos) / dt;
            },
            false => {
                let mut remaining_dt = dt;
                while remaining_dt > 0.0 {
                    let mut integrator = Dopri5::new(
                        *self, 
                        0.0, 
                        dt.into(), 
                       dt as f64 /5.0 ,
                        Vector4::new(self.pos.x, self.pos.y, self.vel.x, self.vel.y).cast(),
                        1e-8,
                        1e-8);

                    let _stats = integrator.integrate();

                    let new_state_vec = integrator.y_out().last().expect("should have integrated at least 1 step").to_owned();
                    self.pos = Vector2::new(new_state_vec.x as f32, new_state_vec.y as f32)
                        .zip_map(&Vector2::new(self.r, self.r), |v1, v2| v1.max(v2))
                        .zip_map(&Vector2::new(WID-self.r, HEI-self.r), |v1, v2| v1.min(v2));
                    self.vel = Vector2::new(new_state_vec.z as f32, new_state_vec.w as f32);
                    
                    let free_movement_time = integrator.x_out().last().expect("should have integrated at least 1 step").to_owned() as f32;
                    
                    let next_collision = self.wall_point_and_normals
                        .iter()
                        .map(|el| (el.0, el.1.1.dot(&(Vector2::new(new_state_vec.x as f32, new_state_vec.y as f32) - el.1.0))))
                        .filter(|(_wall, dot)| *dot < 0.0)
                        .nth(0).unzip().0;

                    if let Some(wall) = next_collision {
                        if free_movement_time < remaining_dt {
                            self.vel = reflect(Rotation2::new(std::f32::consts::FRAC_PI_2)*self.wall_point_and_normals[wall].1, self.vel)
                        }
                    }
                    remaining_dt -= free_movement_time;
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
            .send_and_uniforms(canvas, 2.0*self.r)
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
                (ElementState::Pressed, MouseButton::Right) => {
                    self.pos = self.current_mouse_pos;
                    self.vel = [0.0, 0.0].into();
                }
                _ => {}
            }
        }

        if let Event::WindowEvent {event: WindowEvent::CursorMoved { device_id: _, position: p, .. }, window_id: _} = ev {
            self.last_mouse_pos = self.current_mouse_pos;
            self.current_mouse_pos = Vector2::new(p.x as f32, p.y as f32);
        }
    }
}

pub fn ball_main() {
    let ev_loop = egaku2d::glutin::event_loop::EventLoop::new();
    
    let mut drawer = Drawer::new(
        30, 
        WID as usize, 
        HEI as usize, 
        "test", 
        &ev_loop,
        vec![
            Box::new(Ball::new([WID/2.0, HEI/2.0], 30.0, 0.1))
        ]);

    ev_loop.run(move |event, _, control_flow| main_loop(event, control_flow, &mut drawer));
}
