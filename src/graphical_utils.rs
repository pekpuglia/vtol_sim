use std::time::Instant;

use egaku2d::{glutin::{event::{Event, WindowEvent, VirtualKeyCode}, event_loop::{ControlFlow, EventLoop}}, SimpleCanvas};

pub trait Component {
    fn draw(&mut self, canvas: &mut SimpleCanvas, dt: f32);
    fn receive_event(&mut self, ev: &Event<'_, ()>);
}



pub struct Drawer {
    system: egaku2d::WindowedSystem,
    timer: egaku2d::RefreshTimer,
    components: Vec<Box<dyn Component>>,
    last_draw: Instant,
}

impl Drawer {
    pub fn new(fps: u32, width: usize, height: usize, window_name: &str, event_loop: &EventLoop<()>, components: Vec<Box<dyn Component>>) -> Drawer {
        Drawer {
            system: egaku2d::WindowedSystem::new([width, height], event_loop, window_name),
            timer: egaku2d::RefreshTimer::new(((1 as f64) / (fps as f64)) as usize),
            components,
            last_draw: Instant::now(),
        }
    }

    fn draw(&mut self)  {
        if self.timer.is_ready() {
            let now = Instant::now();
            let dt = now.duration_since(self.last_draw).as_secs_f32();
            self.last_draw = now;

            let canvas = self.system.canvas_mut();
            canvas.clear_color([0.0,0.0,0.0]);
            self.components.iter_mut().for_each(|c| c.draw(canvas, dt));
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
                _ => {}
            }
        },
        
        Event::MainEventsCleared => {drawer.draw();},
        _ => {}
    }
}