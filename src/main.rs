mod graphical_utils;
mod reference_frame;
mod geometry;
mod background;

mod ball;

mod bicopter;

//obs: player controller != (feedback) controller

//todo
//add config file
//add ui with control options
//apply background to plain bicopter
//make position-controlled bicopter
//zoom?
//make reference frames safer
//eliminate dvector? enum map?
//add plane
//add vtol
//add rocket
fn main() {
    bicopter::angle_controlled_bicopter::main()
}
