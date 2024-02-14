mod controllers;

use crate::{
    background::Background, geometry::Geometry, graphical_utils::{main_loop, Component, Drawer}, reference_frame::ReferenceFrame};

use control_systems::DynamicalSystem;
use egaku2d::{
    glutin::event::{Event, WindowEvent, KeyboardInput, VirtualKeyCode}, SimpleCanvas};

pub mod bicopter;

pub mod plane;

use nalgebra::{DVector, Vector2};
use ode_solvers::System;

pub trait PhysicalModel: DynamicalSystem {
    //make body_centered_frame better
    //doesn't need type?
    //doesn't need ref_frame (always screen_frame?)
    fn body_centered_frame(x: &nalgebra::DVector<f64>, ref_frame: &ReferenceFrame) -> ReferenceFrame;
    //receive bodyframe as input?
    fn body_centered_geometry(&self, x: &nalgebra::DVector<f64>, u: &nalgebra::DVector<f64>, ref_frame: &ReferenceFrame) -> Vec<Geometry>;
}

pub trait InputReceiver {
    fn u(&self, ev: &Event<'_, ()>) -> Option<DVector<f64>>;
}

//vehicle = DynamicalSystem + InputReceiver + estado!
#[derive(Clone)]
pub struct GenericVehicle<DS: DynamicalSystem, IR: InputReceiver> {
    model: DS,
    input: IR,
    x: DVector<f64>,
    u: DVector<f64>,
    ref_frame: ReferenceFrame
}

impl<DS, IR> ode_solvers::System<f64, ode_solvers::DVector<f64>> for GenericVehicle<DS, IR> 
where
    DS: DynamicalSystem,
    IR: InputReceiver
{
    fn system(&self, x: f64, y: &ode_solvers::DVector<f64>, dy: &mut ode_solvers::DVector<f64>) {
        dy.copy_from_slice(
            self.model.xdot(x, nalgebra::DVector::from_row_slice(y.as_slice()), self.u.clone()).as_slice()
        )
    }
}

impl<DS, IR> GenericVehicle<DS, IR>
where
    DS: DynamicalSystem + Clone,
    IR: InputReceiver + Clone
{
    fn set_reference_frame(&mut self, new_ref_frame: &crate::reference_frame::ReferenceFrame) {
        self.ref_frame = new_ref_frame.clone();
    }

    fn x(&self) -> &DVector<f64> {
        &self.x
    }

    fn update(&mut self, dt: f64) {
        let mut stepper = ode_solvers::Rk4::new(
            self.clone(), 
            0.0, 
            ode_solvers::DVector::from_row_slice(self.x().as_slice()), 
            dt, 
            dt/5.0);

        let _stats = stepper.integrate();

        self.x.copy_from_slice(stepper.y_out().last().expect("should have integrated at least 1 step").as_slice());
    }
}

pub trait Vehicle: Clone + Component + System<f64, ode_solvers::DVector<f64>> {
    fn set_reference_frame(&mut self, new_ref_frame: &ReferenceFrame);
    fn x(&self) -> &DVector<f64>;
    fn x_mut(&mut self) -> &mut DVector<f64>;

    fn update(&mut self, dt: f64) {
        let mut stepper = ode_solvers::Rk4::new(
            self.clone(), 
            0.0, 
            ode_solvers::DVector::from_row_slice(self.x().as_slice()), 
            dt, 
            dt/5.0);

        let _stats = stepper.integrate();

        self.x_mut().copy_from_slice(stepper.y_out().last().expect("should have integrated at least 1 step").as_slice());
    }
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