use control_systems::DynamicalSystem;
use egaku2d::glutin::{dpi::PhysicalPosition, event::{Event, WindowEvent}};
use nalgebra::{dvector, DVector, Matrix2, Vector2};

use crate::{geometry::{Geometry, GeometryTypes}, graphical_utils::Component, reference_frame::ReferenceFrame};

use super::{vehicle_main, GenericVehicle, InputReceiver, PhysicalModel};

const WID: f64 = 600.0;
const HEI: f64 = 480.0;

//assumes vacuum for now
#[derive(Clone, Copy)]
struct RocketModel {
    dry_mass: f64,
    dry_com: f64, //cm from nose in body_lengths
    initial_propellant_mass: f64,
    exhaust_velocity: f64,
    body_length: f64,
    body_cross_section: f64,
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
    //abstract
    fn body_centered_frame(x: &nalgebra::DVector<f64>, ref_frame: &crate::reference_frame::ReferenceFrame) -> crate::reference_frame::ReferenceFrame {
        let sign = Matrix2::<f64>::from(ref_frame).determinant().signum();
        ReferenceFrame::new_from_frame(
            &Vector2::new(x[2].cos(), x[2].sin()), 
            & (sign * Vector2::new(x[2].sin(), -x[2].cos())),
            &Vector2::new(x[0], x[1]),
            ref_frame)
    }

    fn body_centered_geometry(&self, x: &nalgebra::DVector<f64>, u: &nalgebra::DVector<f64>, ref_frame: &crate::reference_frame::ReferenceFrame) -> Vec<crate::geometry::Geometry> {
        let body_tip = self.dry_com * self.body_length * Vector2::x();
        let body_base = -(1.0-self.dry_com) * self.body_length * Vector2::x(); 
        
        let body = Geometry::new(
            [1.0, 1.0, 1.0, 1.0].into(), 
            *ref_frame, 
            GeometryTypes::Line { 
                p1: body_base, 
                p2: body_tip, 
                thickness: (self.body_cross_section/2.0) as f32 }
        );

        let nose = Geometry::new(
            [1.0, 1.0, 1.0, 1.0].into(), 
            *ref_frame, 
            GeometryTypes::Circle { 
                center: body_tip, 
                radius: self.body_cross_section as f32 }
        );

        let engine_exit = body_base - self.body_cross_section * Vector2::new(u[1].cos(), -u[1].sin());

        let engine = Geometry::new(
            [0.0, 0.0, 1.0, 1.0].into(), 
            *ref_frame, 
            GeometryTypes::Line { 
                p1: engine_exit, 
                p2: body_base, 
                thickness: (self.body_cross_section/2.0) as f32 }
        );

        let thrust_arrow = Geometry::new(
            [1.0, 0.0, 0.0, 1.0].into(), 
            *ref_frame, 
            GeometryTypes::Arrow { 
                start: engine_exit, 
                end: engine_exit - 2.0 * u[0] * Vector2::new(u[1].cos(), -u[1].sin()), 
                thickness: (self.body_cross_section/3.0) as f32}
            );

        vec![body, nose, engine, thrust_arrow]
    }
}

#[derive(Clone, Copy)]
struct GimbalThrustInputReceiver {
    thrust_gain: f64,
    gimbal_gain: f64
}

impl InputReceiver for GimbalThrustInputReceiver {
    fn u(&self, ev: &egaku2d::glutin::event::Event<'_, ()>) -> Option<nalgebra::DVector<f64>> {
        if let Event::WindowEvent { event: WindowEvent::CursorMoved { position: PhysicalPosition { x, y }, ..}, .. } = ev {

            let thrust = (1.0 - y / HEI as f64) * self.thrust_gain;

            let gimbal = (x / WID as f64 - 0.5) * self.gimbal_gain;

            Some(dvector![thrust, gimbal])
        } else {
            None
        }
    }
}

type PlainRocket = GenericVehicle<RocketModel, GimbalThrustInputReceiver>;

impl Component for PlainRocket {
    //abstract!!!
    fn draw(&mut self, canvas: &mut egaku2d::SimpleCanvas, dt: f32, paused: bool) {
        if !paused {
            self.update(dt as f64);
        }

        self.model
            .body_centered_geometry(
                &self.x, 
                &self.u, 
                &self.ref_frame)
            .iter()
            .map(|geom| geom.draw(canvas))
            .last();
    }
    //easy to abstract
    fn receive_event(&mut self, ev: &egaku2d::glutin::event::Event<'_, ()>) {
        match self.input.u(ev) {
            Some(u) => {self.u = u},
            None => (),
        }
    }
}

pub fn main() {
    vehicle_main(PlainRocket {
        input: GimbalThrustInputReceiver{thrust_gain: 30.0, gimbal_gain: std::f64::consts::PI / 18.0},
        model: RocketModel { 
            dry_mass: 10.0, 
            dry_com: 0.3,
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