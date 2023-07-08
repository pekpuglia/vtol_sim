use cgmath::Angle;
use cgmath::Rad;
use cgmath::Vector2;

use crate::graphical_utils::*;
use crate::math_helpers::*;
const WID: f32 = 600.0;

const HEI: f32 = 480.0;

struct Bicopter {
    pos: Vector2<f32>,
    //sentido horário!
    angle: Rad<f32>,
    mass: f32,
    inertia: f32,
    prop_dist: f32,
    max_thrust: f32,
    l_thrust: f32,
    r_thrust:f32
}



impl Component for Bicopter {
    fn draw(&mut self, canvas: &mut egaku2d::SimpleCanvas, dt: f32) {
        let left_right = Vector2::new(self.angle.cos(), self.angle.sin());
        canvas
            .lines(5.0)
            .add((self.pos - self.prop_dist/2.0 * left_right).into(), (self.pos + self.prop_dist/2.0 * left_right).into())
            .send_and_uniforms(canvas)
            .with_color([1.0, 1.0, 1.0, 1.0])
            .draw();
    }

    fn receive_event(&mut self, ev: &egaku2d::glutin::event::Event<'_, ()>) {
        
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
                pos:[WID/2.0,HEI/2.0].into(),
                angle:Rad(std::f32::consts::FRAC_PI_4),
                mass:1.1,
                inertia:1e-2,
                prop_dist:40.0,
                max_thrust:0.7,
                l_thrust:0.0,
                r_thrust:0.0})
        ]);

    ev_loop.run(move |event, _, control_flow| main_loop(event, control_flow, &mut drawer));
}
