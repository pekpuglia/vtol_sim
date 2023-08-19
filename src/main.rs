mod graphical_utils;
mod reference_frame;
mod geometry;
mod world;
mod ball;
// mod math_helpers;

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
    ball::ball_main()
}
