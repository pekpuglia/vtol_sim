use control_systems::DynamicalSystem;
use nalgebra::{DVector, Vector2};

use crate::reference_frame::ReferenceFrame;

use super::{vehicle_main, GenericVehicle, InputReceiver, PhysicalModel};

const WID: f64 = 600.0;
const HEI: f64 = 480.0;

//assumes vacuum for now
struct RocketModel {
    dry_mass: f64,
    initial_propellant_mass: f64,
    exhaust_velocity: f64,
    body_length: f64,
    body_cross_section: f64
}

impl DynamicalSystem for RocketModel {
    const STATE_VECTOR_SIZE: usize = 7;

    const INPUT_SIZE      : usize = 2;

    const OUTPUT_SIZE     : usize = 7;

    fn xdot(&self, t: f64, 
        x: nalgebra::DVector<f64>, 
        u: nalgebra::DVector<f64>) -> nalgebra::DVector<f64> {
        todo!()
    }

    fn y(&self, t: f64, 
        x: nalgebra::DVector<f64>, 
        u: nalgebra::DVector<f64>) -> nalgebra::DVector<f64> {
        todo!()
    }
}

impl PhysicalModel for RocketModel {
    fn body_centered_frame(x: &nalgebra::DVector<f64>, ref_frame: &crate::reference_frame::ReferenceFrame) -> crate::reference_frame::ReferenceFrame {
        todo!()
    }

    fn body_centered_geometry(&self, x: &nalgebra::DVector<f64>, u: &nalgebra::DVector<f64>, ref_frame: &crate::reference_frame::ReferenceFrame) -> Vec<crate::geometry::Geometry> {
        todo!()
    }
}

struct GimbalThrustInputReceiver {}

impl InputReceiver for GimbalThrustInputReceiver {
    fn u(&self, ev: &egaku2d::glutin::event::Event<'_, ()>) -> Option<nalgebra::DVector<f64>> {
        todo!()
    }
}

type PlainRocket = GenericVehicle<RocketModel, GimbalThrustInputReceiver>;

pub fn main() {
    vehicle_main(PlainRocket {
        input: GimbalThrustInputReceiver{},
        model: RocketModel { 
            dry_mass: 10.0, 
            initial_propellant_mass: 5.5, 
            exhaust_velocity: 1500.0, 
            body_length: 60.0, 
            body_cross_section: 10.0 },
        ref_frame: ReferenceFrame::new_from_screen_frame(
            &Vector2::x(), 
            &-Vector2::y(), 
            &Vector2::new(WID/2.0, HEI/2.0)),
        u: DVector::zeros(RocketModel::INPUT_SIZE),
        x: DVector::zeros(RocketModel::STATE_VECTOR_SIZE)
    }, WID, HEI);
}