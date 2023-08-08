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
#[derive(Clone)]
struct PlainBicopter {
    plant: BicopterDynamicalModel,
    //mudar
    x: nalgebra::DVector<f64>,
    u: nalgebra::DVector<f64>,
    input_receiver: BicopterForceMomentInputReceiver
}

impl ode_solvers::System<ode_solvers::DVector<f64>> for PlainBicopter {
    fn system(&self, x: f64, y: &ode_solvers::DVector<f64>, dy: &mut ode_solvers::DVector<f64>) {
        dy.copy_from_slice(self.plant.xdot(x, nalgebra::DVector::from_row_slice(y.as_slice()), self.u.clone()).as_slice())
    }
}

impl PlainBicopter {
    fn new(plant: BicopterDynamicalModel, input_receiver: BicopterForceMomentInputReceiver) -> PlainBicopter {
        PlainBicopter { 
            plant, 
            x: dvector![
                WID as f64/2.0,
                HEI as f64/2.0,
                0.0,
                0.0,
                0.0,
                0.0
            ], u: dvector![
                0.0,
                0.0
            ], input_receiver }
    }

    fn update(&mut self, dt: f64) {
        let mut stepper = Rk4::new(
            self.clone(), 
            0.0, 
            ode_solvers::DVector::from_row_slice(self.x.as_slice()), 
            dt, 
            dt/5.0);

        let stats = stepper.integrate();

        self.x.copy_from_slice(stepper.y_out().last().expect("should have integrated at least 1 step").as_slice());
    }
}

impl Component for PlainBicopter {
    fn draw(&mut self, canvas: &mut egaku2d::SimpleCanvas, dt: f32, paused: bool) {
        if !paused {
            self.update(dt as f64);
        }

        let prop_dir = BicopterDynamicalModel::propeller_direction(&self.x).map(|x| x as f32);

        let (left, right) = {
            let tmp = self.plant.left_right_positions(&self.x);

            (tmp.0.map(|x| x as f32), tmp.1.map(|x| x as f32))
        };

        let l_thrust = self.u[0] as f32;
        let r_thrust = self.u[1] as f32;

        //corpo
        canvas
            .lines(5.0)
            .add(left.into(),right.into())
            .send_and_uniforms(canvas)
            .with_color([1.0, 1.0, 1.0, 1.0])
            .draw();

        //empuxos
        canvas
            .arrows(2.0)
            .add(left.into(), (left + 0.7 * l_thrust * prop_dir).into())
            .send_and_uniforms(canvas)
            .with_color([1.0, 0.0, 0.0, 1.0])
            .draw();

        canvas
            .arrows(2.0)
            .add(right.into(), (right + 0.7 * r_thrust * prop_dir).into())
            .send_and_uniforms(canvas)
            .with_color([0.0, 0.0, 1.0, 1.0])
            .draw();

    }

    fn receive_event(&mut self, ev: &egaku2d::glutin::event::Event<'_, ()>) {
        match self.input_receiver.thrusts(ev) {
            Some(thrusts) => {self.u = thrusts},
            None => {},
        }
    }
}

pub fn bicopter_main() {
    let ev_loop = egaku2d::glutin::event_loop::EventLoop::new();
    
    let mut drawer = Drawer::new(
        30, 
        WID as usize, 
        HEI as usize, 
        "test", 
        &ev_loop,
        vec![
            Box::new(PlainBicopter::new(
                BicopterDynamicalModel::new(
                    1000.0, 
                    1.0, 
                    100.0, 
                40.0), 
                BicopterForceMomentInputReceiver { force_gain: 200.0, moment_gain: 25.0 })) 
        ]);

    ev_loop.run(move |event, _, control_flow| main_loop(event, control_flow, &mut drawer));
}
