use nalgebra::{Matrix2, Vector2, vector};

use crate::{geometry::{Geometry, GeometryTypes}, reference_frame::{SCREEN_FRAME, ReferenceFrame}};



#[derive(derive_new::new)]
pub struct PlaneDynamicalModel {
    main_chord: f64,
    main_thickness: f64,
    tail_chord: f64,
    tail_thickness: f64,
    tail_le_to_main_le: f64,
    x_cg_c: f64
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

        let tail_trailing_edge = 
            tail_leading_edge - vector![self.tail_chord, 0.0]
        ;

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
                    tail_trailing_edge, 
                    tail_leading_edge, 
                    self.tail_thickness as f32)
            )
        ]
    }
}