mod graphical_utils;
mod ball;
mod math_helpers;

mod bicopter;

//obs: player controller != (feedback) controller

//todo
//abstract controller
//dynamical systems modelling
//add config file
//add ui with control options
//add background
//add camera
//add plane
//add vtol
fn main() {
    bicopter::bicopter_main()
}
