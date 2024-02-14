use egaku2d::glutin::event::{Event, WindowEvent};
use nalgebra::{dvector, Vector2, DVector};
use crate::reference_frame::ReferenceFrame;

mod bicopter_dynamics;
use bicopter_dynamics::{BicopterDynamicalModel, DynamicalSystem};


pub mod plain_bicopter;

pub mod angle_controlled_bicopter;

pub mod position_controlled_bicopter;