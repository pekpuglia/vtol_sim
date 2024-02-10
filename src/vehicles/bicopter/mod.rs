use egaku2d::glutin::event::{Event, WindowEvent};
use nalgebra::{dvector, Vector2, DVector};
use ode_solvers::Rk4;
use crate::reference_frame::ReferenceFrame;
use crate::{background::Background, graphical_utils::*};
use crate::vehicles::{Vehicle, World, CameraOptions};


const WID: f64 = 600.0;

const HEI: f64 = 480.0;

mod bicopter_dynamics;
use bicopter_dynamics::{BicopterDynamicalModel, DynamicalSystem};


pub mod plain_bicopter;

pub mod angle_controlled_bicopter;

pub mod position_controlled_bicopter;