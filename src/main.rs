mod graphical_utils;
mod reference_frame;
mod geometry;
mod background;


mod controllers;

mod ball;

mod bicopter;

mod plane;

//obs: player controller != (feedback) controller

//closing a control loop: better to use actuator that acts directly on measured quantity

//todo
//add config file
//add ui with control options
//saturations??
//zoom?
//make reference frames safer
//eliminate dvector? enum map?
//abstract world, vehicle - change "components field"
//add vtol
//add rocket
//use real units?
fn main() {
    plane::main()
}
