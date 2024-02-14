use crate::vehicles::{screen_center_x, vehicle_main, GenericVehicle, InputReceiver, PhysicalModel, Vehicle};
use super::*;



#[derive(Clone)]
struct BicopterForceMomentInputReceiver {
    force_gain: f64,
    moment_gain: f64
}

impl BicopterForceMomentInputReceiver {
    fn new(force_gain: f64, moment_gain: f64) -> BicopterForceMomentInputReceiver {
        BicopterForceMomentInputReceiver { 
            force_gain, 
            moment_gain}
    }

    fn thrusts(&self, ev: &egaku2d::glutin::event::Event<'_, ()>) -> Option<nalgebra::DVector<f64>> {
        if let Event::WindowEvent {event: WindowEvent::CursorMoved { device_id: _, position: p, .. }, window_id: _} = ev {
            
            //p.y p baixo
            let force = self.force_gain * (1.0 - (p.y)/HEI as f64);

            let moment = self.moment_gain * ((p.x)/WID as f64 - 1.0/2.0);

            let l_thrust = force / 2.0 - moment / 2.0;
            let r_thrust = force / 2.0 + moment / 2.0;

            Some(dvector![l_thrust, r_thrust])
        } else {
            None
        }
        
    }
}

impl InputReceiver for BicopterForceMomentInputReceiver {
    fn u(&self, ev: &Event<'_, ()>) -> Option<DVector<f64>> {
        self.thrusts(ev)
    }
}

type PlainBicopter = GenericVehicle<BicopterDynamicalModel, BicopterForceMomentInputReceiver>;

impl Component for PlainBicopter {
    fn draw(&mut self, canvas: &mut egaku2d::SimpleCanvas, dt: f32, paused: bool) {
        if !paused {
            self.update(dt as f64);
        }

        //corpo
        self.model.body_centered_geometry(&self.x, &self.u, &self.ref_frame)
            .iter()
            .map(|geom| geom.draw(canvas))
            .last();

    }

    fn receive_event(&mut self, ev: &egaku2d::glutin::event::Event<'_, ()>) {
        match self.input.u(ev) {
            Some(thrusts) => {self.u = thrusts},
            None => {},
        }
    }
}

impl Vehicle for PlainBicopter {
    fn set_reference_frame(&mut self, new_ref_frame: &ReferenceFrame) {
        self.ref_frame = *new_ref_frame;
    }

    fn x(&self) -> &DVector<f64> {
        &self.x
    }

    fn x_mut(&mut self) -> &mut DVector<f64> {
        &mut self.x
    }
}

const WID: f64 = 600.0;
const HEI: f64 = 480.0;

#[allow(unused)]
pub fn main() {
    //make better use of this
    let ref_frame = ReferenceFrame::new_from_screen_frame(
        &Vector2::x(), 
        &-Vector2::y(), 
        &Vector2::new(0.0,0.0));

    vehicle_main(
        PlainBicopter{
            model: BicopterDynamicalModel::new(
                1000.0, 
                1.0, 
                100.0, 
            40.0),
            input: BicopterForceMomentInputReceiver::new(200.0, -25.0 ),
            x: screen_center_x(WID, HEI, BicopterDynamicalModel::STATE_VECTOR_SIZE),
            u: DVector::zeros(2),
            ref_frame,
        }, WID, HEI);
}
