mod plane_dynamics;

use std::f64::consts::PI;

use nalgebra::{DVector, vector, dvector, Vector2};

use crate::{bicopter::{World, Vehicle, CameraOptions}, graphical_utils::{Component, Drawer, main_loop}, reference_frame::{SCREEN_FRAME, ReferenceFrame}, background::Background};

use self::plane_dynamics::PlaneDynamicalModel;

const WID: f32 = 600.0;

const HEI: f32 = 480.0;

struct Plane {
    model: plane_dynamics::PlaneDynamicalModel,
    x: DVector<f64>,
    u: DVector<f64>,
    ref_frame: ReferenceFrame
}

impl Component for Plane {
    fn draw(&mut self, canvas: &mut egaku2d::SimpleCanvas, dt: f32, paused: bool) {
        self.model
            .body_centered_geometry(&self.x, &self.u, &self.ref_frame)
            .iter()
            .map(|geom| geom.draw(canvas))
            .last();
    }

    fn receive_event(&mut self, ev: &egaku2d::glutin::event::Event<'_, ()>) {
        ()
    }
}

impl Vehicle for Plane {
    fn set_reference_frame(&mut self, new_ref_frame: &crate::reference_frame::ReferenceFrame) {
        self.ref_frame = new_ref_frame.clone();
    }

    fn x(&self) -> &DVector<f64> {
        &self.x
    }
}

pub fn main() {
    let ev_loop = egaku2d::glutin::event_loop::EventLoop::new();

    let mut drawer = Drawer::new(
        60, 
        WID as usize, 
        HEI as usize, 
        "test", 
        &ev_loop,
        vec![
            Box::new(World { 
                bicopter: Plane {
                    model: PlaneDynamicalModel::new(
                        40.0, 
                        5.0, 
                        20.0, 
                        3.0, 
                        80.0, 
                        -0.5),
                    ref_frame: ReferenceFrame::new_from_screen_frame(
                        &Vector2::x(), &-Vector2::y(), &Vector2::new(WID as f64 / 2.0, HEI as f64 / 2.0)),
                    u: dvector![0.0, 0.0],
                    x: dvector![
                        0.0, 0.0, PI / 6.0, 
                        0.0, 0.0, 0.0]
                }, 
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

        ev_loop.run(move |event,_ , control_flow| main_loop(event, control_flow, &mut drawer))
}