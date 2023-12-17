mod graphical_utils;
mod reference_frame;
mod geometry;
mod background;


mod controllers;

mod ball;

mod bicopter;

mod plane;

//obs: player controller != (feedback) controller

//todo
//add config file
//add ui with control options
//saturations??
//zoom?
//make reference frames safer
//eliminate dvector? enum map?
//add plane
//add vtol
//add rocket
fn main() {
    plane::main()
}
