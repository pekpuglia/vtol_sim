use egaku2d::glutin::{ event::VirtualKeyCode, event_loop::ControlFlow};

mod graphical_utils;
use graphical_utils::*;

fn main() {
    let ev_loop = egaku2d::glutin::event_loop::EventLoop::new();
    
    let mut drawer = Drawer::new(60, 600, 480, "test", &ev_loop);

    ev_loop.run(move |event, _, control_flow| main_loop(event, control_flow, &mut drawer));
}
