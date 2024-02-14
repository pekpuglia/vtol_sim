use std::f64::consts::FRAC_PI_2;

use control_systems::DynamicalSystem;
use nalgebra::{Matrix2, Vector2, vector, dvector, DVector, Rotation2};

use crate::{geometry::{Geometry, GeometryTypes}, reference_frame::ReferenceFrame, vehicles::PhysicalModel};

#[derive(derive_new::new, Clone, Copy)]
pub struct LiftModel {
    cl_a: f64,
    alpha_0: f64,
    alpha_m: f64,
    cl_delta: f64,
}
impl LiftModel {
    pub fn cl(&self, x: &DVector<f64>, u: &DVector<f64>) -> f64 {
        let alpha_bar = AerodynamicModel::alpha(x)-self.alpha_0;

        (match alpha_bar.abs() <= 2.0*(self.alpha_m - self.alpha_0).abs() {
            true => self.cl_a * (self.alpha_m-self.alpha_0)/FRAC_PI_2 * 
                (FRAC_PI_2 * alpha_bar / 
                (self.alpha_m - self.alpha_0)).sin(),
            false => 0.0
        }) + self.cl_delta * u[1]
        // self.cl_a * (AerodynamicModel::alpha(x) - self.alpha_0) + self.cl_delta * u[1]
    }
}

//neutral point
#[derive(derive_new::new, Clone, Copy)]
pub struct MomentModel {
    cm_0: f64,
    cm_delta: f64,
    cm_q: f64,
    x_np_c: f64,
} //add stall
impl MomentModel {
    fn cm(&self, x: &DVector<f64>, u: &DVector<f64>) -> f64 {
        self.cm_0 + self.cm_delta * u[1] + self.cm_q * x[5]
    }
}

#[derive(derive_new::new, Clone, Copy)]
pub struct DragModel {
    cd_0: f64,
    cd_alpha: f64,
    cd_alpha2: f64
} //add stall
impl DragModel {
    fn cd(&self, x: &DVector<f64>, u: &DVector<f64>) -> f64 {
        let alpha = AerodynamicModel::alpha(x);
        self.cd_0 + self.cd_alpha * alpha + self.cd_alpha2 * alpha.powi(2)
    }
}

#[derive(derive_new::new, Clone, Copy)]
pub struct AerodynamicModel {
    pub lift: LiftModel,
    pub moment: MomentModel,
    pub drag: DragModel,
    pub rho: f64,
    pub sref: f64,
    pub cref: f64
}
impl AerodynamicModel {
    pub fn alpha(x: &DVector<f64>) -> f64 {
        let body_direction = Vector2::new(x[2].cos(), x[2].sin());

        let velocity = vector![x[3], x[4]];

        //angle(u, v) * sign(u x v)
        body_direction.angle(&velocity) * (velocity.x * body_direction.y - velocity.y * body_direction.x).signum()
    }
}

#[derive(derive_new::new, Clone, Copy)]
pub struct PlaneDynamicalModel {
    //graphical parameters
    main_chord: f64,
    main_thickness: f64,
    tail_chord: f64,
    tail_thickness: f64,
    tail_le_to_main_le: f64,
    elevator_x_c: f64,
    x_cg_c: f64,
    pub aero_model: AerodynamicModel,
    gravity: f64,
    mass: f64,
    inertia: f64
}


impl PhysicalModel for PlaneDynamicalModel {

    fn body_centered_frame(x: &nalgebra::DVector<f64>, ref_frame: &ReferenceFrame) -> ReferenceFrame {
        let sign = Matrix2::<f64>::from(ref_frame).determinant().signum();
        ReferenceFrame::new_from_frame(
            &Vector2::new(x[2].cos(), x[2].sin()), 
            & (sign * Vector2::new(x[2].sin(), -x[2].cos())),
            &Vector2::new(x[0], x[1]),
            ref_frame)
    }


    //x: xcg ycg theta ...
     fn body_centered_geometry(&self, x: &nalgebra::DVector<f64>, u: &nalgebra::DVector<f64>, ref_frame: &ReferenceFrame) -> Vec<Geometry> {
        let frame = PlaneDynamicalModel::body_centered_frame(x, ref_frame);

        let main_leading_edge = Vector2::new(
            -self.x_cg_c*self.main_chord, 
            0.0
        );

        let main_trailing_edge = Vector2::new(
            main_leading_edge[0] - self.main_chord,
            0.0
        );

        let tail_leading_edge = 
            main_leading_edge - vector![self.tail_le_to_main_le, 0.0]
        ;

        let tail_elevator_hinge = 
            tail_leading_edge - vector![self.tail_chord * (1.0 - self.elevator_x_c), 0.0]
        ;

        //positivo pra baixo!
        let elevator_trailing_edge = 
            tail_elevator_hinge + self.tail_chord * self.elevator_x_c * Vector2::new(
                -u[1].cos(), -u[1].sin()
        );

        vec![
            //main wing
            Geometry::new(
                [1.0, 1.0, 1.0, 1.0], 
                frame, 
                GeometryTypes::new_line(
                    main_trailing_edge, 
                    main_leading_edge, 
                    self.main_thickness as f32)
            ),
            //tail
            Geometry::new(
                [1.0, 1.0, 1.0, 1.0], 
                frame, 
                GeometryTypes::new_line(
                    tail_elevator_hinge, 
                    tail_leading_edge, 
                    self.tail_thickness as f32)
            ),
            //elevator
            Geometry::new(
                [0.0, 0.0, 1.0, 1.0], 
                frame, 
                GeometryTypes::new_arrow(
                    tail_elevator_hinge, 
                    tail_elevator_hinge + 5.0 * (elevator_trailing_edge - tail_elevator_hinge), 
                    self.tail_thickness as f32)
            ),
            //thrust
            Geometry::new(
                [1.0, 0.0, 0.0, 1.0], 
                frame, 
                GeometryTypes::new_arrow(
                    main_leading_edge, 
                    main_leading_edge + vector![u[0]/10.0, 0.0], 
                    2.0)
            )
        ]
    }
}

impl DynamicalSystem for PlaneDynamicalModel {
    const STATE_VECTOR_SIZE: usize = 6;

    const INPUT_SIZE      : usize = 2;

    const OUTPUT_SIZE     : usize = 6;

    fn xdot(&self, t: f64, 
        x: nalgebra::DVector<f64>, 
        u: nalgebra::DVector<f64>) -> nalgebra::DVector<f64> {
        let cl = self.aero_model.lift.cl(&x, &u);
        let cd = self.aero_model.drag.cd(&x, &u);
        let cm = self.aero_model.moment.cm(&x, &u);

        let qS = 0.5 * self.aero_model.rho * self.aero_model.sref * (x[3].powi(2) + x[4].powi(2)).sqrt();

        let force = 
            // lift
            qS * cl * vector![-x[4], x[3]] +
            - qS * cd * vector![x[3], x[4]] +
            u[0] * Rotation2::new(x[2]).matrix() * Vector2::x()
        ;

        let moment = qS * cm * self.aero_model.cref + qS * cl * self.aero_model.cref * (self.aero_model.moment.x_np_c - self.x_cg_c);

        // let moment = 0.0;

        dvector![
            x[3],
            x[4],
            x[5],
            force.x / self.mass,
            force.y / self.mass - self.gravity,
            moment / self.inertia
        ]
    }

    fn y(&self, t: f64, 
        x: nalgebra::DVector<f64>, 
        u: nalgebra::DVector<f64>) -> nalgebra::DVector<f64> {
        x
    }
}