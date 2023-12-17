use control_systems::DynamicalSystem;
use nalgebra::{Matrix2, Vector2, vector, dvector, DVector};

use crate::{geometry::{Geometry, GeometryTypes}, reference_frame::{SCREEN_FRAME, ReferenceFrame}};

#[derive(derive_new::new)]
pub struct LiftModel {
    cl_a: f64,
    alpha_0: f64,
    alpha_m: f64,
    cl_delta: f64,
}
impl LiftModel {
    fn cl(&self, x: &DVector<f64>, u: &DVector<f64>) -> f64 {
        self.cl_a * (AerodynamicModel::alpha(x) - self.alpha_0)
    }
}

//neutral point
#[derive(derive_new::new)]
pub struct MomentModel {
    cm_0: f64,
    cm_delta: f64,
    cm_q: f64,
    x_np_c: f64,
}
impl MomentModel {
    fn cm(&self, x: &DVector<f64>, u: &DVector<f64>) -> f64 {
        self.cm_0
    }
}

#[derive(derive_new::new)]
pub struct DragModel {
    cd_0: f64,
    ki: f64,
}
impl DragModel {
    fn cd(&self, x: DVector<f64>, u: &DVector<f64>) -> f64 {
        self.cd_0
    }
}

#[derive(derive_new::new)]
pub struct AerodynamicModel {
    lift: LiftModel,
    moment: MomentModel,
    drag: DragModel,
    rho: f64,
    sref: f64,
    cref: f64
}
impl AerodynamicModel {
    fn alpha(x: &DVector<f64>) -> f64 {
        x[2] - x[4].atan2(x[3])
    }
}

#[derive(derive_new::new)]
pub struct PlaneDynamicalModel {
    //graphical parameters
    main_chord: f64,
    main_thickness: f64,
    tail_chord: f64,
    tail_thickness: f64,
    tail_le_to_main_le: f64,
    elevator_x_c: f64,
    x_cg_c: f64,
    aero_model: AerodynamicModel,
    gravity: f64,
    mass: f64,
    inertia: f64
}


impl PlaneDynamicalModel {

    pub fn body_centered_frame(x: &nalgebra::DVector<f64>, ref_frame: &ReferenceFrame) -> ReferenceFrame {
        let sign = Matrix2::<f64>::from(ref_frame).determinant().signum();
        ReferenceFrame::new_from_frame(
            &Vector2::new(x[2].cos(), x[2].sin()), 
            & (sign * Vector2::new(x[2].sin(), -x[2].cos())),
            &Vector2::new(x[0], x[1]),
            ref_frame)
    }


    //x: xcg ycg theta ...
    pub fn body_centered_geometry(&self, x: &nalgebra::DVector<f64>, u: &nalgebra::DVector<f64>, ref_frame: &ReferenceFrame) -> Vec<Geometry> {
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
                    main_leading_edge + vector![u[0]*50.0, 0.0], 
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
        dvector![

        ]
    }

    fn y(&self, t: f64, 
        x: nalgebra::DVector<f64>, 
        u: nalgebra::DVector<f64>) -> nalgebra::DVector<f64> {
        x
    }
}