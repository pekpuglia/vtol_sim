mod graphical_utils;
use graphical_utils::*;

struct Ball {
    x: f32,
    y: f32,
    r: f32
}

use egaku2d::glutin::event::{Event, WindowEvent};
impl Component for Ball {
    fn draw(&self, canvas: &mut egaku2d::SimpleCanvas, dt: f64) {
        canvas
            .circles()
            .add([self.x, self.y])
            .send_and_uniforms(canvas, self.r)
            .with_color([1.0,1.0,1.0,1.0])
            .draw();
    }

    fn receive_event(&mut self, ev: &Event<'_, ()>) {
        if let Event::WindowEvent {event: WindowEvent::CursorMoved { device_id: _, position: p, .. }, window_id: _} = ev {
            self.x = p.x as f32;
            self.y = p.y as f32;
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
            Box::new(Ball{ x: 300.0, y: 240.0, r: 10.0 })
        ]);

    ev_loop.run(move |event, _, control_flow| main_loop(event, control_flow, &mut drawer));
}
