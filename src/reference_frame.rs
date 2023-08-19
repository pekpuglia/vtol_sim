//colocar conversor de referencial?

//screen frame: (0,0) no canto superior esquerdo, x p direita, y p baixo
//Background frame (0,0) configurável, x p direita, y p esquerda
//etc
use nalgebra::{Vector2, Matrix2};

#[derive(Clone, Copy, Debug)]
pub struct ReferenceFrame {
    pub x_unit_vector_screen_frame: Vector2<f64>,
    pub y_unit_vector_screen_frame: Vector2<f64>,
    pub origin_screen_frame: Vector2<f64>
}

pub const SCREEN_FRAME: ReferenceFrame = ReferenceFrame{ 
    x_unit_vector_screen_frame: Vector2::new(1.0, 0.0), 
    y_unit_vector_screen_frame: Vector2::new(0.0, 1.0), 
    origin_screen_frame: Vector2::new(0.0, 0.0) };

impl ReferenceFrame {
    pub fn new_from_screen_frame(x_vector: &Vector2<f64>, y_vector: &Vector2<f64>, origin: &Vector2<f64>) -> ReferenceFrame {
        ReferenceFrame {
            x_unit_vector_screen_frame: x_vector.normalize(),
            y_unit_vector_screen_frame: y_vector.normalize(),
            origin_screen_frame: *origin,
        }
    }

}

impl From<&ReferenceFrame> for Matrix2<f64> {
    fn from(value: &ReferenceFrame) -> Self {
        Matrix2::from_columns(&[value.x_unit_vector_screen_frame, value.y_unit_vector_screen_frame])
    }
}

pub trait ConvertToFrame {
    fn to_frame(&self, origin: &ReferenceFrame, dest: &ReferenceFrame) -> Self;
}

impl ConvertToFrame for Vector2<f64> {
    fn to_frame(&self, origin: &ReferenceFrame, dest: &ReferenceFrame) -> Self {
        let origin_mat: Matrix2<f64> = origin.into();
        let dest_mat: Matrix2<f64> = dest.into();

        dest_mat.try_inverse().expect("Reference Frame Matrices should always be invertible") * (origin_mat * self + origin.origin_screen_frame - dest.origin_screen_frame)
    }
}

#[cfg(test)]
mod test_frame_conversion {
    use nalgebra::vector;

    use super::*;
    #[test]
    fn basic_test() {
        let destination_frame = ReferenceFrame { 
            x_unit_vector_screen_frame: Vector2::new(1.0, 1.0), 
            y_unit_vector_screen_frame: Vector2::new(-1.0, 1.0), 
            origin_screen_frame: Vector2::new(1.0, 0.0) };

        let result = Vector2::new(1.0, 1.0).to_frame(&SCREEN_FRAME, &destination_frame);
        
        assert!((result - vector![1.0 / 2.0, 1.0 / 2.0]).norm() < 1e-10)
    }
}
