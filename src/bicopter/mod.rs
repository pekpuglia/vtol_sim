use egaku2d::glutin::event::{Event, WindowEvent, VirtualKeyCode, KeyboardInput};
use nalgebra::{dvector, Vector2, DVector};
use ode_solvers::Rk4;
use crate::reference_frame::ReferenceFrame;
use crate::{background::Background, graphical_utils::*};

const WID: f32 = 600.0;

const HEI: f32 = 480.0;

mod bicopter_dynamics;
use bicopter_dynamics::{BicopterDynamicalModel, DynamicalSystem};


pub mod plain_bicopter;

pub mod angle_controlled_bicopter;

pub mod position_controlled_bicopter;

pub trait Vehicle {
    fn set_reference_frame(&mut self, new_ref_frame: &ReferenceFrame);
    fn x(&self) -> &DVector<f64>;
}

enum CameraOptions {
    VehicleCentered,
    Fixed
}

struct World<VehicleType> {
    bicopter: VehicleType,
    background: Background,
    camera_option: CameraOptions,
    camera_option_toggle: bool
}

impl<VehicleType: Component + Vehicle> Component for World<VehicleType> {
    fn draw(&mut self, canvas: &mut SimpleCanvas, dt: f32, paused: bool) {
        match self.camera_option {
            CameraOptions::VehicleCentered => {
                let ref_frame = ReferenceFrame::new_from_screen_frame(
                    &Vector2::x(), 
                    &-Vector2::y(), 
                &Vector2::new(WID as f64/2.0 - self.bicopter.x()[0], HEI as f64/2.0 + self.bicopter.x()[1]));
                self.bicopter.set_reference_frame(&ref_frame);
                self.background.ref_frame = ref_frame;
            },
            CameraOptions::Fixed => {},
        }
        self.background.draw(canvas, dt, paused);
        self.bicopter.draw(canvas, dt, paused)
    }

    fn receive_event(&mut self, ev: &Event<'_, ()>) {
        if let Event::WindowEvent { 
            event: WindowEvent::KeyboardInput { 
                input: KeyboardInput { virtual_keycode: Some(VirtualKeyCode::V), .. }, .. }, .. } = ev 
            {
            if !self.camera_option_toggle {
                self.camera_option = match self.camera_option {
                    CameraOptions::VehicleCentered => CameraOptions::Fixed,
                    CameraOptions::Fixed => CameraOptions::VehicleCentered,
                };
                self.camera_option_toggle = true;
            } else {
                self.camera_option_toggle = false;
            }
        }
        self.background.receive_event(ev);
        self.bicopter.receive_event(ev);
    }
}

fn bicopter_main(bicopter: impl Component + Vehicle + 'static) {
    let ev_loop = egaku2d::glutin::event_loop::EventLoop::new();

    let mut drawer = Drawer::new(
        60, 
        WID as usize, 
        HEI as usize, 
        "test", 
        &ev_loop,
        vec![
            Box::new(World { 
                bicopter: bicopter, 
                background: Background::new(
                    Vector2::new(0.0,0.0), 
                    100.0, 
                    [1.0,0.0,0.0,0.3], 
                    [0.0,1.0,0.0,0.3], 
                    WID.into(), 
                    HEI.into()), 
                camera_option: CameraOptions::Fixed, 
                camera_option_toggle: false })
        ]);

    ev_loop.run(move |event, _, control_flow| main_loop(event, control_flow, &mut drawer));
}