use nalgebra::{Vector2, dvector, Matrix2};
use crate::reference_frame::ReferenceFrame;
use crate::geometry::{Geometry, GeometryTypes};
pub use control_systems::DynamicalSystem;

#[derive(Clone, Copy)]
pub struct BicopterDynamicalModel {
    inertia: f64,
    mass: f64,
    gravity: f64,
    prop_dist: f64,
}

impl BicopterDynamicalModel {
    pub fn new(inertia: f64,
        mass: f64,
        gravity: f64,
        prop_dist: f64) -> BicopterDynamicalModel {
        BicopterDynamicalModel { inertia, mass, gravity, prop_dist }
    }

    pub fn body_centered_frame(x: &nalgebra::DVector<f64>, ref_frame: &ReferenceFrame) -> ReferenceFrame {
        let sign = Matrix2::<f64>::from(ref_frame).determinant().signum();
        ReferenceFrame::new_from_frame(
            &Vector2::new(x[2].cos(), x[2].sin()), 
            & (sign * Vector2::new(x[2].sin(), -x[2].cos())),
            &Vector2::new(x[0], x[1]),
            ref_frame)
    }

    pub fn body_centered_geometry(&self, x: &nalgebra::DVector<f64>, u: &nalgebra::DVector<f64>, ref_frame: &ReferenceFrame) -> Vec<Geometry> {
        let frame = BicopterDynamicalModel::body_centered_frame(x, ref_frame);
        let left = Vector2::new(-self.prop_dist/2.0, 0.0);
        let right = Vector2::new(self.prop_dist/2.0, 0.0);


        vec![
            Geometry::new(
                [1.0,1.0,1.0,1.0], 
                frame,
                GeometryTypes::new_line(
                    left, 
                    right, 
                    5.0)
            ),
            Geometry::new(
                [1.0, 0.0, 0.0, 1.0], 
                frame,
                GeometryTypes::new_arrow(
                    left, 
                    (left + 0.7 * u[0] * Vector2::y()).into(), 
                    2.0)
            ),
            Geometry::new(
                [0.0, 0.0, 1.0, 1.0], 
                frame,
                GeometryTypes::new_arrow(
                    right.into(), 
                    (right + 0.7 * u[1] * Vector2::y()).into(), 
                    2.0)
            )
        ]
    }
}

/*
    State Vector:
    0 x
    1 y
    2 theta
    3 xdot
    4 ydot
    5 thetadot

    Input:
    lthrust
    rthrust

    Output:
    State Vector
*/
impl DynamicalSystem for BicopterDynamicalModel {
    const STATE_VECTOR_SIZE: usize = 6;

    const INPUT_SIZE      : usize = 2;

    const OUTPUT_SIZE     : usize = 6;
    //assume sistema x p direita, y p cima!
    fn xdot(&self, _t: f64, 
        x: nalgebra::DVector<f64>, 
        u: nalgebra::DVector<f64>) -> nalgebra::DVector<f64> {

        dvector![
            x[3],
            x[4],
            x[5],
            - (u[0] + u[1]) * x[2].sin() / self.mass,
              (u[0] + u[1]) * x[2].cos() / self.mass - self.gravity,
            self.prop_dist * (u[0] - u[1]) / self.inertia
        ]
    }

    fn y(&self, _t: f64, 
        x: nalgebra::DVector<f64>, 
        _u: nalgebra::DVector<f64>) -> nalgebra::DVector<f64> {
        x
    }
}