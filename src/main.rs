use vehicles::{bicopter, plane, rocket};

mod graphical_utils;
mod reference_frame;
mod geometry;
mod background;

mod ball;

mod vehicles;

//obs: player controller != (feedback) controller

//closing a control loop: better to use actuator that acts directly on measured quantity

//todo
//add config file
//add ui with control options
//saturations??
//geometry: circle radius field actually is diameter!!
//zoom?
//make reference frames safer, better use them for initial conditions
//eliminate dvector? enum map?
//manage dependency spreading
//add vtol
//add rocket
//use real units
//cli to choose sim mode
fn main() {
    bicopter::angle_controlled_bicopter::main()
}
