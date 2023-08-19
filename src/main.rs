mod graphical_utils;
mod reference_frame;
mod ball;
mod math_helpers;

mod bicopter;

//obs: player controller != (feedback) controller

//todo
//refactor module hierarchy
//add config file
//add ui with control options
//add world, camera
//add plane
//add vtol
//add rocket
fn main() {
    bicopter::angle_controlled_bicopter::bicopter_main()
}
