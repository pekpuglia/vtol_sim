use egaku2d::glutin::event::{Event, WindowEvent};
use nalgebra::{dvector, Vector2, DVector};
use ode_solvers::Rk4;
use crate::reference_frame::ReferenceFrame;
use crate::{background::Background, graphical_utils::*};
use crate::vehicles::{Vehicle, World, CameraOptions};


const WID: f32 = 600.0;

const HEI: f32 = 480.0;

mod bicopter_dynamics;
use bicopter_dynamics::{BicopterDynamicalModel, DynamicalSystem};


pub mod plain_bicopter;

pub mod angle_controlled_bicopter;

pub mod position_controlled_bicopter;


fn bicopter_main(bicopter: impl Component + Vehicle + 'static) {
    let ev_loop = egaku2d::glutin::event_loop::EventLoop::new();

    let mut drawer = Drawer::new(
        60, 
        WID as usize, 
        HEI as usize, 
        "test", 
        &ev_loop,
        World { 
            vehicle: bicopter, 
            background: Background::new(
                Vector2::new(0.0,0.0), 
                100.0, 
                [1.0,0.0,0.0,0.3], 
                [0.0,1.0,0.0,0.3], 
                WID.into(), 
                HEI.into()), 
            camera_option: CameraOptions::Fixed, 
            camera_option_toggle: false }
        );

    ev_loop.run(move |event, _, control_flow| main_loop(event, control_flow, &mut drawer));
}