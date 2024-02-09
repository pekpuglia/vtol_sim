mod controllers;

use crate::{
    reference_frame::ReferenceFrame,
    background::Background,
    graphical_utils::Component};

use egaku2d::{
    glutin::event::{Event, WindowEvent, KeyboardInput, VirtualKeyCode}, SimpleCanvas};

pub mod bicopter;

pub mod plane;

use nalgebra::{DVector, Vector2};



pub trait Vehicle {
    fn set_reference_frame(&mut self, new_ref_frame: &ReferenceFrame);
    fn x(&self) -> &DVector<f64>;
}

pub enum CameraOptions {
    VehicleCentered,
    Fixed
}

const WID: f32 = 600.0;

const HEI: f32 = 480.0;

pub struct World<VehicleType> {
    pub bicopter: VehicleType,
    pub background: Background,
    pub camera_option: CameraOptions,
    pub camera_option_toggle: bool
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