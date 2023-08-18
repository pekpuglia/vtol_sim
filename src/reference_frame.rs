//colocar conversor de referencial?

//screen frame: (0,0) no canto superior esquerdo, x p direita, y p baixo
//world frame (0,0) configurável, x p direita, y p esquerda
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

#[derive(derive_new::new)]
pub enum GeometryTypes {
    Circle{
        center: Vector2<f64>,
        radius: f32
    },
    AARect{
        lower: Vector2<f64>,
        upper: Vector2<f64>
    },
    Line{
        p1: Vector2<f64>,
        p2: Vector2<f64>,
        thickness: f32
    },
    Arrow{
        start: Vector2<f64>,
        end: Vector2<f64>,
        thickness: f32
    }
}

#[derive(derive_new::new)]
pub struct Geometry {
    color: [f32; 4],
    ref_frame: ReferenceFrame,
    geom_type: GeometryTypes
}

impl Geometry {
    pub fn draw(&self, canvas: &mut egaku2d::SimpleCanvas) {
        let screen_geometry = self.to_frame(&SCREEN_FRAME);
        match screen_geometry.geom_type {
            GeometryTypes::Circle { center, radius } => {
                canvas
                    .circles()
                    .add(center.map(|x| x as f32).into())
                    .send_and_uniforms(canvas, radius)
                    .with_color(self.color)
                    .draw();
            },
            GeometryTypes::AARect { lower, upper } => {
                canvas
                    .rects()
                    .add([lower[0] as f32, upper[0] as f32, lower[1] as f32, upper[1] as f32])
                    .send_and_uniforms(canvas)
                    .with_color(self.color)
                    .draw();
            },
            GeometryTypes::Line { p1, p2, thickness } => {
                canvas
                    .lines(thickness)
                    .add(p1.map(|x| x as f32).into(), p2.map(|x| x as f32).into())
                    .send_and_uniforms(canvas)
                    .with_color(self.color)
                    .draw();
            },
            GeometryTypes::Arrow { start, end, thickness } => {
                canvas
                .arrows(thickness)
                .add(start.map(|x| x as f32).into(), end.map(|x| x as f32).into())
                .send_and_uniforms(canvas)
                .with_color(self.color)
                .draw();
            },
        }
    }

    //exige que novo referencial seja isométrico
    fn to_frame(&self, dest: &ReferenceFrame) -> Geometry {
        Geometry {
            color: self.color,
            ref_frame: *dest,
            geom_type: match self.geom_type {
                GeometryTypes::Circle { center, radius } => {
                    GeometryTypes::Circle { center: center.to_frame(&self.ref_frame, dest), radius }
                },
                GeometryTypes::AARect { lower, upper } => {
                    GeometryTypes::AARect { lower: lower.to_frame(&self.ref_frame, dest), upper: upper.to_frame(&self.ref_frame, dest) }
                },
                GeometryTypes::Line { p1, p2, thickness } => {
                    GeometryTypes::Line { p1: p1.to_frame(&self.ref_frame, dest), p2: p2.to_frame(&self.ref_frame, dest), thickness }
                },
                GeometryTypes::Arrow { start, end, thickness } => {
                    GeometryTypes::Arrow { start: start.to_frame(&self.ref_frame, dest), end: end.to_frame(&self.ref_frame, dest), thickness }
                },
            },
        }
    }
}