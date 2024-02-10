mod plane_dynamics;

use control_systems::DynamicalSystem;
use egaku2d::glutin::event::{Event, WindowEvent};
use nalgebra::{DVector, dvector, Vector2};
use ode_solvers::Rk4;

use crate::{vehicles::{World, Vehicle, CameraOptions}, graphical_utils::{Component, Drawer, main_loop}, reference_frame::{ReferenceFrame}, background::Background};

use self::plane_dynamics::{PlaneDynamicalModel, AerodynamicModel, LiftModel, MomentModel, DragModel};

use super::vehicle_main;

const WID: f64 = 600.0;

const HEI: f64 = 480.0;

//todo
//abstract input receivers!!
#[derive(Clone, Copy)]
struct PlaneThrustAndElevatorInputReceiver {
    thrust_gain: f64,
    elevator_gain: f64
}

impl PlaneThrustAndElevatorInputReceiver {
    fn u(&self, ev: &Event<'_, ()>) -> Option<DVector<f64>> {
        if let Event::WindowEvent {event: WindowEvent::CursorMoved { device_id: _, position: p, .. }, window_id: _} = ev {
            
            let thrust = (1.0 - p.y / HEI as f64) * self.thrust_gain;

            let elevator = (p.x / WID as f64 - 0.5) * self.elevator_gain;

            Some(dvector![thrust, elevator])
        } else {
            None
        }
    }
}

#[derive(Clone)]
struct Plane {
    model: plane_dynamics::PlaneDynamicalModel,
    thrust_elevator_input: PlaneThrustAndElevatorInputReceiver,
    x: DVector<f64>,
    u: DVector<f64>,
    ref_frame: ReferenceFrame
}

impl Plane {
    fn update(&mut self, dt: f64) {
        let mut stepper = Rk4::new(
            self.clone(), 
            0.0, 
            ode_solvers::DVector::from_row_slice(self.x.as_slice()), 
            dt, 
            dt/5.0);

        let _stats = stepper.integrate();

        self.x.copy_from_slice(stepper.y_out().last().expect("should have integrated at least 1 step").as_slice());
    }
}

impl ode_solvers::System<f64, ode_solvers::DVector<f64>> for Plane {
    fn system(&self, x: f64, y: &ode_solvers::DVector<f64>, dy: &mut ode_solvers::DVector<f64>) {
        dy.copy_from_slice(
            self.model.xdot(x, nalgebra::DVector::from_row_slice(y.as_slice()), self.u.clone()).as_slice()
        )
    }
}

impl Component for Plane {
    fn draw(&mut self, canvas: &mut egaku2d::SimpleCanvas, dt: f32, paused: bool) {
        if !paused {
            self.update(dt as f64);
        }

        dbg!(AerodynamicModel::alpha(&self.x), self.model.aero_model.lift.cl(&self.x, &self.u));

        match self.x.iter().any(|val| val.is_nan()) {
            true => {panic!("wtf nan")},
            false => ()
        };

        self.model
            .body_centered_geometry(&self.x, &self.u, &self.ref_frame)
            .iter()
            .map(|geom| geom.draw(canvas))
            .last();
    }

    fn receive_event(&mut self, ev: &egaku2d::glutin::event::Event<'_, ()>) {
        match self.thrust_elevator_input.u(ev) {
            Some(u) => {self.u = u;},
            None => (),
        }
    }
}

impl Vehicle for Plane {
    fn set_reference_frame(&mut self, new_ref_frame: &crate::reference_frame::ReferenceFrame) {
        self.ref_frame = new_ref_frame.clone();
    }

    fn x(&self) -> &DVector<f64> {
        &self.x
    }
}

#[allow(unused)]
pub fn main() {

    let aero = AerodynamicModel::new(
        LiftModel::new(5.0, -0.035, 0.314, 0.1),
        MomentModel::new(-0.1, -0.5, -0.05, -0.7),
        DragModel::new(0.03, 0.05, 0.05),
        1.225,
        0.1,
        1.0
    );

    vehicle_main(Plane {
        model: PlaneDynamicalModel::new(
            40.0, 
            5.0, 
            20.0, 
            3.0, 
            80.0,
            0.25,
            -0.67,
            aero,
            100.0,
            40.0,
            1e-1
            ),
        thrust_elevator_input: PlaneThrustAndElevatorInputReceiver { thrust_gain: 5000.0, elevator_gain: 1.0 },
        ref_frame: ReferenceFrame::new_from_screen_frame(
            &Vector2::x(), &-Vector2::y(), &Vector2::new(WID as f64 / 2.0, HEI as f64 / 2.0)),
        u: dvector![0.0, 0.0],
        x: dvector![
            0.0, 0.0, 0.0, 
            200.0, 0.0, 0.0]
    }, WID, HEI);
}