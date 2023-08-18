use std::{time::Instant, ops::{Div, Range}};

use egaku2d::{glutin::{event::{Event, WindowEvent, VirtualKeyCode, ElementState, MouseButton}, event_loop::{ControlFlow, EventLoop}}, SimpleCanvas};
use nalgebra::Vector2;

use crate::reference_frame::{ReferenceFrame, Geometry, GeometryTypes};

pub trait Component {
    fn draw(&mut self, canvas: &mut SimpleCanvas, dt: f32, paused: bool);
    fn receive_event(&mut self, ev: &Event<'_, ()>);
}

pub struct World {
    ref_frame: ReferenceFrame,
    tile_size: f64,
    color1: [f32; 4],
    color2: [f32; 4],
    screen_height: f64,
    screen_width: f64
}

impl World {
    pub fn new(origin_screen_frame: Vector2<f64>, tile_size: f64, color1: [f32; 4], color2: [f32;4], screen_width: f64, screen_height: f64) -> World {
        World { 
            ref_frame: ReferenceFrame::new_from_screen_frame(
                &Vector2::x(), 
                &-Vector2::y(), 
                &origin_screen_frame), 
            tile_size, 
            color1, 
            color2,
            screen_height,
            screen_width, 
        }
    }
}

impl Component for World {    
    fn draw(&mut self, canvas: &mut SimpleCanvas, dt: f32, paused: bool) {
        let screen_origin = -self.ref_frame.origin_screen_frame;

        let x_start_index = (screen_origin.x / self.tile_size).floor() as i32;
        let x_end_index = (screen_origin.x + self.screen_width).div(self.tile_size).ceil() as i32;

        let y_start_index = (screen_origin.y-self.screen_height).div(self.tile_size).floor() as i32;
        let y_end_index = (screen_origin.y).div(self.tile_size).ceil() as i32;
        
        Range{ start: x_start_index, end: x_end_index}
            .flat_map(
                |x_el| Range{ start: y_start_index, end: y_end_index }.map(move |y_el| (x_el, y_el)))
            .map(|(x_el, y_el)| match (x_el+y_el) % 2 == 0 {
                true => (self.color1, x_el as f64, y_el as f64),
                false => (self.color2, x_el as f64, y_el as f64),
                } 
            )
            .map(|(color, x_i, y_i)| 
                Geometry::new(color, self.ref_frame, GeometryTypes::AARect { 
                    lower: self.tile_size * Vector2::new(x_i, y_i), 
                    upper: self.tile_size * Vector2::new(x_i + 1.0, y_i + 1.0) 
                }).draw(canvas)
            ).last();
    }

    fn receive_event(&mut self, ev: &Event<'_, ()>) {
    }
}


pub struct Drawer {
    system: egaku2d::WindowedSystem,
    timer: egaku2d::RefreshTimer,
    components: Vec<Box<dyn Component>>,
    last_draw: Instant,
    paused: bool
}

impl Drawer {
    pub fn new(fps: u32, width: usize, height: usize, window_name: &str, event_loop: &EventLoop<()>, components: Vec<Box<dyn Component>>) -> Drawer {
        Drawer {
            system: egaku2d::WindowedSystem::new([width, height], event_loop, window_name),
            timer: egaku2d::RefreshTimer::new(((1000 as f64) / (fps as f64)) as usize),
            components,
            last_draw: Instant::now(),
            paused: true
        }
    }

    fn draw(&mut self)  {
        if self.timer.is_ready() {
            let now = Instant::now();
            let dt = now.duration_since(self.last_draw).as_secs_f32();
            self.last_draw = now;
            let canvas = self.system.canvas_mut();
            canvas.clear_color([0.0,0.0,0.0]);
            self.components.iter_mut().for_each(|c| c.draw(canvas, dt, self.paused));
            self.system.swap_buffers()
        }
    }
    fn broadcast_events(&mut self, ev: &Event<'_,()>) {
        self.components.iter_mut().for_each(|c| c.receive_event(ev));
    }
}

//enviar eventos pros componentes
pub fn main_loop(ev: Event<'_, ()>, control_flow: &mut ControlFlow, drawer: &mut Drawer) {
    drawer.broadcast_events(&ev);
    match ev {
        Event::WindowEvent { event: wev, .. } => {
            match wev {
                WindowEvent::KeyboardInput { input, ..} => {
                    if let Some(VirtualKeyCode::Escape) = input.virtual_keycode {
                        *control_flow = ControlFlow::Exit;
                    }
                },
                WindowEvent::CloseRequested => {*control_flow = ControlFlow::Exit},
                WindowEvent::MouseInput {state, button, .. } => {
                    match (state, button) {
                        (ElementState::Pressed, MouseButton::Right) => {drawer.paused = !drawer.paused},
                        _ => {}
                    }
            }
                _ => {}
            }
        },
        
        Event::MainEventsCleared => {drawer.draw();},
        _ => {}
    }
}