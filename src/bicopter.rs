use egaku2d::glutin::event::{Event, WindowEvent};
use nalgebra::{dvector};
use ode_solvers::{Rk4};

use crate::graphical_utils::*;
const WID: f32 = 600.0;

const HEI: f32 = 480.0;

mod bicopter_dynamics;
use bicopter_dynamics::{BicopterDynamicalModel, DynamicalSystem};

pub mod plain_bicopter;

pub mod angle_controlled_bicopter;
