use egaku2d::glutin::{event::{Event, WindowEvent, VirtualKeyCode}, event_loop::{ControlFlow, EventLoop}};

pub struct Drawer {
    system: egaku2d::WindowedSystem,
    timer: egaku2d::RefreshTimer
}

impl Drawer {
    pub fn new(fps: u32, width: usize, height: usize, window_name: &str, event_loop: &EventLoop<()>) -> Drawer {
        Drawer {
            system: egaku2d::WindowedSystem::new([width, height], event_loop, window_name),
            timer: egaku2d::RefreshTimer::new(((1 as f64) / (fps as f64)) as usize),
        }
    }

    fn draw(&mut self)  {
        if self.timer.is_ready() {
            let canvas = self.system.canvas_mut();
            canvas.clear_color([0.0,0.0,0.0]);
            canvas.rects()
            .add([100.0, 500.0, 50.0, 430.0])
            .send_and_uniforms(canvas)
            .with_color([1.0,1.0,1.0,1.0]).draw();
            self.system.swap_buffers()
        }
    }
}


pub fn main_loop(ev: Event<'_, ()>, control_flow: &mut ControlFlow, drawer: &mut Drawer) {
    match ev {
        Event::WindowEvent { event: wev, .. } => {
            match wev {
                WindowEvent::KeyboardInput { input, ..} => {
                    if let Some(VirtualKeyCode::Escape) = input.virtual_keycode {
                        *control_flow = ControlFlow::Exit;
                    }
                },
                _ => {}
            }
        },
        Event::MainEventsCleared => {drawer.draw();},
        _ => {}
    }
}