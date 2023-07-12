use egaku2d::glutin::event::{Event, WindowEvent};

use crate::graphical_utils::*;
const WID: f32 = 600.0;

const HEI: f32 = 480.0;

mod bicopter_plant;

struct BicopterForceMomentInputReceiver {
    force_gain: f32,
    moment_gain: f32,
    force: f32,
    moment: f32
}

impl BicopterForceMomentInputReceiver {
    fn new(force_gain: f32, moment_gain: f32) -> BicopterForceMomentInputReceiver {
        BicopterForceMomentInputReceiver { 
            force_gain, 
            moment_gain, 
            force: 0.0, 
            moment: 0.0 }
    }

    fn set_force_moment(&mut self, ev: &egaku2d::glutin::event::Event<'_, ()>) {
        if let Event::WindowEvent {event: WindowEvent::CursorMoved { device_id: _, position: p, .. }, window_id: _} = ev {
            
            self.force = self.force_gain * (1.0 - (p.y as f32)/HEI);

            self.moment = self.moment_gain * ((p.x as f32)/WID - 1.0/2.0);

        }
    }

    //transformar em HAL
    fn control_bicopter(&self, bicopter: &mut bicopter_plant::BicopterPlant) {
        bicopter.l_thrust = self.force / 2.0 + self.moment / (2.0 * bicopter.prop_dist());
        bicopter.r_thrust = self.force / 2.0 - self.moment / (2.0 * bicopter.prop_dist());
    }
}

struct Bicopter {
    plant: bicopter_plant::BicopterPlant,
    controller: BicopterForceMomentInputReceiver
}

impl Component for Bicopter {
    fn draw(&mut self, canvas: &mut egaku2d::SimpleCanvas, dt: f32, paused: bool) {
        if !paused {
            self.controller.control_bicopter(&mut self.plant);
            self.plant.update(dt as f64);
        }

        let prop_dir = self.plant.propeller_direction();

       let (left, right) = self.plant.left_right_positions();

        let (l_thrust, r_thrust) = self.plant.thrusts();

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
        self.controller.set_force_moment(ev)
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
            Box::new(Bicopter{
                plant:bicopter_plant::BicopterPlant::new(
                    [WID/2.0,HEI/2.0].into(),
                    [0.0,0.0].into(),
                    0.0,
                    0.0,
                    1000.0,
                    1.0,
                    100.0,
                    40.0,
                    0.0,
                    0.0),
                controller: BicopterForceMomentInputReceiver::new(200.0, 1000.0) })
        ]);

    ev_loop.run(move |event, _, control_flow| main_loop(event, control_flow, &mut drawer));
}
