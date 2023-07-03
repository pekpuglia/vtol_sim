mod graphical_utils;
use std::cmp::min;

use graphical_utils::*;
use cgmath::{Vector2, Matrix2, InnerSpace, SquareMatrix};

const WID: f32 = 600.0;
const HEI: f32 = 480.0;

struct Ball {
    pos: Vector2<f32>,
    vel: Vector2<f32>,
    damp: f32,
    r: f32,
    is_held: bool,
    current_mouse_pos: Vector2<f32>,
    last_mouse_pos: Vector2<f32>,
    wall_point_and_directions: enum_map::EnumMap<Walls, (Vector2<f32>, Vector2<f32>)>,
}

fn reflect(axis: Vector2<f32>, to_reflect: Vector2<f32>) -> Vector2<f32> {
    let norm_axis = axis.normalize();
    let axis_base = Matrix2::new(norm_axis.x, norm_axis.y, -norm_axis.y, norm_axis.x);
    axis_base * Matrix2::new(1.0, 0.0, 0.0, -1.0) * axis_base.invert().expect("axis_base should be isometric") * to_reflect
}

fn intersection_lambda(pos1: Vector2<f32>, dir1: Vector2<f32>, pos2: Vector2<f32>, dir2: Vector2<f32>) -> Result<f32, ()> {
    //pos1 + lambda1 * dir1 = pos2 + lambda2 * dir2
    //[dir1 -dir2] * [lambda1; lambda2] = pos2 - pos1
    //returns lambda1
    let a = Matrix2::from_cols(dir1, -dir2);
    
    Ok((a.invert().ok_or(())? * (pos2 - pos1)).x)
}

#[cfg(test)]
mod test {
    use super::*;
    #[test]
    fn test_reflect() {
        let axis = Vector2::new(1.0, 0.0);
        assert!((Vector2::new(1.0, -1.0) - reflect(axis, [1.0, 1.0].into())).magnitude().abs() < 1e-5);
        assert!((Vector2::new(1.0, -1.0) - reflect(-axis, [1.0, 1.0].into())).magnitude().abs() < 1e-5);
    }

    #[test]
    fn test_intersect() {
        let case1 = intersection_lambda([0.0, 0.0].into(), [1.0, 0.0].into(), [0.0, 1.0].into(), [1.0, 0.0].into());
        assert!(matches!(case1, Err(())));
        let case2 = intersection_lambda([0.0, 0.0].into(), [2.0, 0.0].into(), [1.0, 1.0].into(), [0.0, 1.0].into());
        assert!(case2.is_ok_and(|val| (val - 0.5).abs() < 1e-5));
    }
}

#[derive(enum_map::Enum, Debug, Clone, Copy)]
enum Walls {
    Top,
    Right,
    Left,
    Bottom
}

impl Ball {
    fn new(pos: [f32;2], r: f32, damp:f32) -> Ball {
        Ball {pos: pos.into(), vel: [0.0,0.0].into(), 
            r, is_held: false, 
            current_mouse_pos: [0.0,0.0].into(), 
            last_mouse_pos: [0.0, 0.0].into(), damp,
            wall_point_and_directions: enum_map::enum_map! {
                Walls::Bottom => ([0.0, HEI-r].into(), [1.0, 0.0].into()),
                Walls::Left => ([r, 0.0].into(), [0.0, 1.0].into()),
                Walls::Right => ([WID-r, 0.0].into(), [0.0, 1.0].into()),
                Walls::Top => ([0.0, r].into(), [1.0, 0.0].into())
            }, }
    }

    fn calculate_next_collision(&self) -> Option<(f32, Walls)> {
        let mut collision_with_walls = self.wall_point_and_directions
            .iter()
            .map(|(key, (p2, d2))| 
                (intersection_lambda(self.pos, self.vel, p2.to_owned(), d2.to_owned()), key))
            .map(|min_opt| min_opt.0.ok().map(|time_opt| (time_opt, min_opt.1)))
            .flatten()
            .collect::<Vec<(f32, Walls)>>();
        collision_with_walls.sort_by(|a, b| a.0.partial_cmp(&b.0).expect("no NaNs should be here, exploding..."));

        collision_with_walls.get(0).copied()
        // for (key, (p2, d2)) in self.wall_point_and_directions {
        //     let collision_dt = intersection_lambda(self.pos, self.vel, p2, d2);

        //     match collision_dt {
        //         //collides this frame
        //         Ok(col_dt) if col_dt >= 0.0 && col_dt <= dt => {
        //             // self.pos += self.vel * col_dt;
        //             // self.vel = reflect(d2, self.vel);
        //             // self.pos += self.vel * (dt - col_dt);
        //             safe_dt = safe_dt.min(col_dt);
        //             collision = Some(key);
        //         },
        //         //no collision this frame
        //         Ok(_) | Err(_) => {
        //             // self.pos += self.vel * dt;
        //         },
        //     }
        // }
    }

    fn dynamics(&mut self, dt: f32) {
        match self.is_held {
            true => {
                self.pos = self.current_mouse_pos
                    .zip([0.0   + self.r, 0.0   + self.r].into(), f32::max)
                    .zip([WID - self.r, HEI - self.r].into(), f32::min);
                self.vel = (self.current_mouse_pos - self.last_mouse_pos) / dt;
            },
            false => {
                self.vel -= self.vel * self.damp;
                
                let mut safe_dt = dt;
                let mut collision = None;
                for (key, (p2, d2)) in self.wall_point_and_directions {
                    let collision_dt = intersection_lambda(self.pos, self.vel, p2, d2);

                    match collision_dt {
                        //collides this frame
                        Ok(col_dt) if col_dt >= 0.0 && col_dt <= dt => {
                            // self.pos += self.vel * col_dt;
                            // self.vel = reflect(d2, self.vel);
                            // self.pos += self.vel * (dt - col_dt);
                            safe_dt = safe_dt.min(col_dt);
                            collision = Some(key);
                        },
                        //no collision this frame
                        Ok(_) | Err(_) => {
                            // self.pos += self.vel * dt;
                        },
                    }
                }

                self.pos += self.vel * safe_dt;
                match collision {
                    Some(wall) => {
                        self.vel = reflect(self.wall_point_and_directions[wall].1, self.vel);
                        self.pos += self.vel * (dt - safe_dt);
                    },
                    None => {},
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
        WID as usize, 
        HEI as usize, 
        "test", 
        &ev_loop,
        vec![
            Box::new(Ball::new([WID/2.0, HEI/2.0], 30.0, 0.01))
        ]);

    ev_loop.run(move |event, _, control_flow| main_loop(event, control_flow, &mut drawer));
}
