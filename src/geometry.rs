use nalgebra::Vector2;
use crate::reference_frame::{ReferenceFrame, SCREEN_FRAME, ConvertToFrame};
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
                    GeometryTypes::Circle { center: center.position_to_frame(&self.ref_frame, dest), radius }
                },
                GeometryTypes::AARect { lower, upper } => {
                    GeometryTypes::AARect { lower: lower.position_to_frame(&self.ref_frame, dest), upper: upper.position_to_frame(&self.ref_frame, dest) }
                },
                GeometryTypes::Line { p1, p2, thickness } => {
                    GeometryTypes::Line { p1: p1.position_to_frame(&self.ref_frame, dest), p2: p2.position_to_frame(&self.ref_frame, dest), thickness }
                },
                GeometryTypes::Arrow { start, end, thickness } => {
                    GeometryTypes::Arrow { start: start.position_to_frame(&self.ref_frame, dest), end: end.position_to_frame(&self.ref_frame, dest), thickness }
                },
            },
        }
    }
}