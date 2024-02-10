mod controllers;

use crate::{
    background::Background, 
    graphical_utils::{Component, Drawer, main_loop}, 
    reference_frame::ReferenceFrame};

use egaku2d::{
    glutin::event::{Event, WindowEvent, KeyboardInput, VirtualKeyCode}, SimpleCanvas};

pub mod bicopter;

pub mod plane;

use nalgebra::{DVector, Vector2};

pub trait Vehicle: Component {
    fn set_reference_frame(&mut self, new_ref_frame: &ReferenceFrame);
    fn x(&self) -> &DVector<f64>;
}

pub enum CameraOptions {
    VehicleCentered,
    Fixed
}

pub struct World<VehicleType> {
    pub screen_width: f64,
    pub screen_height: f64,
    pub vehicle: VehicleType,
    pub background: Background,
    pub camera_option: CameraOptions,
    pub camera_option_toggle: bool
}

impl<VehicleType: Vehicle> Component for World<VehicleType> {
    fn draw(&mut self, canvas: &mut SimpleCanvas, dt: f32, paused: bool) {
        match self.camera_option {
            CameraOptions::VehicleCentered => {
                let ref_frame = ReferenceFrame::new_from_screen_frame(
                    &Vector2::x(), 
                    &-Vector2::y(), 
                &Vector2::new(self.screen_width/2.0 - self.vehicle.x()[0], self.screen_height/2.0 + self.vehicle.x()[1]));
                self.vehicle.set_reference_frame(&ref_frame);
                self.background.ref_frame = ref_frame;
            },
            CameraOptions::Fixed => {},
        }
        self.background.draw(canvas, dt, paused);
        self.vehicle.draw(canvas, dt, paused)
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
        self.vehicle.receive_event(ev);
    }
}

fn vehicle_main(vehicle: impl Vehicle + 'static, width: f64, height: f64) {
    let ev_loop = egaku2d::glutin::event_loop::EventLoop::new();

    let mut drawer = Drawer::new(
        60, 
        width as usize, 
        height as usize, 
        "test", 
        &ev_loop,
        World { 
            screen_width: width,
            screen_height: height,
            vehicle, 
            background: Background::new(
                Vector2::new(0.0,0.0), 
                100.0, 
                [1.0,0.0,0.0,0.3], 
                [0.0,1.0,0.0,0.3], 
                width, 
                height), 
            camera_option: CameraOptions::Fixed, 
            camera_option_toggle: false }
        );

    ev_loop.run(move |event, _, control_flow| main_loop(event, control_flow, &mut drawer));
}