use egaku2d::glutin::{self, event::VirtualKeyCode, event_loop::ControlFlow};

struct Drawer {

}
use glutin::event::{Event, WindowEvent};
fn event_handler(ev: Event<'_, ()>, control_flow: &mut ControlFlow, drawer: Drawer) {
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
        Event::MainEventsCleared => {drawer;},
        _ => {}
    }
}

fn main() {
    let ev_loop = glutin::event_loop::EventLoop::new();
    let mut glsys = egaku2d::WindowedSystem::new(
        [600, 480], 
        &ev_loop, 
        "test window");



    let mut timer = egaku2d::RefreshTimer::new(16);

    ev_loop.run(move |event, _, control_flow| match event {
        glutin::event::Event::WindowEvent {event: wev, .. } => {
            match wev {
                glutin::event::WindowEvent::KeyboardInput { input, ..} => {
                    if let Some(VirtualKeyCode::Escape) = input.virtual_keycode {
                        *control_flow = ControlFlow::Exit;
                    }
                },
                _ => {}
            }
        },
        glutin::event::Event::MainEventsCleared => {
            if timer.is_ready() {
                let canvas = glsys.canvas_mut();
                canvas.clear_color([0.0,0.0,0.0]);
                canvas.rects()
                .add([100.0, 500.0, 50.0, 430.0])
                .send_and_uniforms(canvas)
                .with_color([1.0,1.0,1.0,1.0]).draw();
                glsys.swap_buffers()
            }
        }
        _ => {}
    });
}
