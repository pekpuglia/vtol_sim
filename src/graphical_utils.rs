use std::time::Instant;

use egaku2d::glutin::{
    event::{
        WindowEvent, 
        VirtualKeyCode, 
        ElementState, 
        MouseButton
    }, event_loop::{
        ControlFlow, 
        EventLoop}};

pub use egaku2d::SimpleCanvas;
pub use egaku2d::glutin::event::Event;

pub trait Component {
    fn draw(&mut self, canvas: &mut SimpleCanvas, dt: f32, paused: bool);
    fn receive_event(&mut self, ev: &Event<'_, ()>);
}

pub struct Drawer<CT: Component> {
    system: egaku2d::WindowedSystem,
    timer: egaku2d::RefreshTimer,
    main_component: CT,
    last_draw: Instant,
    paused: bool
}

impl<CT: Component> Drawer<CT> {
    pub fn new(fps: u32, width: usize, height: usize, window_name: &str, event_loop: &EventLoop<()>, main_component: CT) -> Drawer<CT> {
        Drawer {
            system: egaku2d::WindowedSystem::new([width, height], event_loop, window_name),
            timer: egaku2d::RefreshTimer::new(((1000 as f64) / (fps as f64)) as usize),
            main_component,
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
            self.main_component.draw(canvas, dt, self.paused);
            self.system.swap_buffers()
        }
    }
    fn broadcast_events(&mut self, ev: &Event<'_,()>) {
        self.main_component.receive_event(ev)
    }
}

//enviar eventos pros componentes
pub fn main_loop(ev: Event<'_, ()>, control_flow: &mut ControlFlow, drawer: &mut Drawer<impl Component>) {
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