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
//change "components field"
//don't use global WID, HEI!
//manage dependency spreading
//add vtol
//add rocket
//use real units?
fn main() {
    vehicles::bicopter::plain_bicopter::main()
}
