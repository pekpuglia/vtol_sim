use crate::graphical_utils;

//colocar conversor de referencial?

#[derive(derive_new::new)]
pub enum GeometryTypes {
    Circle{
        center: [f32; 2],
        radius: f32
    },
    AARect{
        lower: [f32; 2],
        upper: [f32; 2]
    },
    Line{
        p1: [f32; 2],
        p2: [f32; 2],
        thickness: f32
    },
    Arrow{
        start: [f32; 2],
        end: [f32; 2],
        thickness: f32
    }
}

#[derive(derive_new::new)]
pub struct Geometry {
    color: [f32; 4],
    geom_type: GeometryTypes
}

impl Geometry {
    pub fn draw(&self, canvas: &mut egaku2d::SimpleCanvas) {
        match self.geom_type {
            GeometryTypes::Circle { center, radius } => {
                canvas
                    .circles()
                    .add(center)
                    .send_and_uniforms(canvas, radius)
                    .with_color(self.color)
                    .draw();
            },
            GeometryTypes::AARect { lower, upper } => {
                canvas
                    .rects()
                    .add([lower[0], lower[1], upper[0], upper[1]])
                    .send_and_uniforms(canvas)
                    .with_color(self.color)
                    .draw();
            },
            GeometryTypes::Line { p1, p2, thickness } => {
                canvas
                    .lines(thickness)
                    .add(p1, p2)
                    .send_and_uniforms(canvas)
                    .with_color(self.color)
                    .draw();
            },
            GeometryTypes::Arrow { start, end, thickness } => {
                canvas
                .arrows(thickness)
                .add(start, end)
                .send_and_uniforms(canvas)
                .with_color(self.color)
                .draw();
            },
        }
    }
}