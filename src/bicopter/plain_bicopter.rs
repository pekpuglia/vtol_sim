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
            
            let force = self.force_gain * (1.0 - (p.y)/HEI as f64);

            let moment = self.moment_gain * ((p.x)/WID as f64 - 1.0/2.0);

            let l_thrust = force / 2.0 + moment / 2.0;
            let r_thrust = force / 2.0 - moment / 2.0;

            Some(dvector![l_thrust, r_thrust])
        } else {
            None
        }
        
    }
}
#[derive(Clone, derive_new::new)]
struct PlainBicopter {
    plant: BicopterDynamicalModel,
    #[new(value="dvector![
        WID as f64/2.0,
        -HEI as f64/2.0,
        0.0,
        0.0,
        0.0,
        0.0
    ]")]
    x: nalgebra::DVector<f64>,
    #[new(value="dvector![
        0.0,
        0.0
    ]")]
    u: nalgebra::DVector<f64>,
    ref_frame: ReferenceFrame,
    input_receiver: BicopterForceMomentInputReceiver
}

impl ode_solvers::System<ode_solvers::DVector<f64>> for PlainBicopter {
    fn system(&self, x: f64, y: &ode_solvers::DVector<f64>, dy: &mut ode_solvers::DVector<f64>) {
        dy.copy_from_slice(self.plant.xdot(
            x, 
            nalgebra::DVector::from_row_slice(y.as_slice()), 
            self.u.clone()).as_slice())
    }
}

impl PlainBicopter {
    // fn new(plant: BicopterDynamicalModel, input_receiver: BicopterForceMomentInputReceiver) -> PlainBicopter {
    //     PlainBicopter { 
    //         plant, 
    //         x: dvector![
    //             WID as f64/2.0,
    //             HEI as f64/2.0,
    //             0.0,
    //             0.0,
    //             0.0,
    //             0.0
    //         ], u: dvector![
    //             0.0,
    //             0.0
    //         ], input_receiver }
    // }

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

impl Component for PlainBicopter {
    fn draw(&mut self, canvas: &mut egaku2d::SimpleCanvas, dt: f32, paused: bool) {
        if !paused {
            self.update(dt as f64);
        }

        //corpo
        self.plant.body_centered_geometry(&self.x, &self.u, &self.ref_frame)
            .iter()
            .map(|geom| geom.draw(canvas))
            .last();

    }

    fn receive_event(&mut self, ev: &egaku2d::glutin::event::Event<'_, ()>) {
        match self.input_receiver.thrusts(ev) {
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
}

pub fn main() {

    let ref_frame = ReferenceFrame::new_from_screen_frame(
        &Vector2::x(), 
        &-Vector2::y(), 
        &Vector2::new(0.0,0.0));

    bicopter_main(
        PlainBicopter::new(
            BicopterDynamicalModel::new(
                1000.0, 
                1.0, 
                100.0, 
            40.0), 
                ref_frame, 
            BicopterForceMomentInputReceiver::new(200.0, -25.0 )
    ));
}
