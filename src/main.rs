use vehicles::{bicopter, plane};

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
//zoom?
//make reference frames safer
//eliminate dvector? enum map?
//manage dependency spreading
//add vtol
//add rocket
//use real units
//cli to choose sim mode
fn main() {
    bicopter::plain_bicopter::main()
}
