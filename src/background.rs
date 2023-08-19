use std::ops::{Div, Range};

use crate::{reference_frame::ReferenceFrame, geometry::{Geometry, GeometryTypes}};
use nalgebra::Vector2;
use crate::graphical_utils::{Component, SimpleCanvas, Event};
pub struct Background {
    ref_frame: ReferenceFrame,
    tile_size: f64,
    color1: [f32; 4],
    color2: [f32; 4],
    screen_height: f64,
    screen_width: f64
}

impl Background {
    pub fn new(origin_screen_frame: Vector2<f64>, tile_size: f64, color1: [f32; 4], color2: [f32;4], screen_width: f64, screen_height: f64) -> Background {
        Background { 
            ref_frame: ReferenceFrame::new_from_screen_frame(
                &Vector2::x(), 
                &-Vector2::y(), 
                &origin_screen_frame), 
            tile_size, 
            color1, 
            color2,
            screen_height,
            screen_width, 
        }
    }
}

impl Component for Background {    
    fn draw(&mut self, canvas: &mut SimpleCanvas, _dt: f32, _paused: bool) {
        let screen_origin = -self.ref_frame.origin_screen_frame;

        let x_start_index = (screen_origin.x / self.tile_size).floor() as i32;
        let x_end_index = (screen_origin.x + self.screen_width).div(self.tile_size).ceil() as i32;

        let y_start_index = (screen_origin.y-self.screen_height).div(self.tile_size).floor() as i32;
        let y_end_index = (screen_origin.y).div(self.tile_size).ceil() as i32;
        
        Range{ start: x_start_index, end: x_end_index}
            .flat_map(
                |x_el| Range{ start: y_start_index, end: y_end_index }.map(move |y_el| (x_el, y_el)))
            .map(|(x_el, y_el)| match (x_el+y_el) % 2 == 0 {
                true => (self.color1, x_el as f64, y_el as f64),
                false => (self.color2, x_el as f64, y_el as f64),
                } 
            )
            .map(|(color, x_i, y_i)| 
                Geometry::new(color, self.ref_frame, GeometryTypes::AARect { 
                    lower: self.tile_size * Vector2::new(x_i, y_i), 
                    upper: self.tile_size * Vector2::new(x_i + 1.0, y_i + 1.0) 
                }).draw(canvas)
            ).last();
    }

    fn receive_event(&mut self, _ev: &Event<'_, ()>) {
    }
}