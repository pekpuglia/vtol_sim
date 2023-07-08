use cgmath::{Vector2, Matrix2, InnerSpace, SquareMatrix, Deg};

pub fn reflect(axis: Vector2<f32>, to_reflect: Vector2<f32>) -> Vector2<f32> {
    let norm_axis = axis.normalize();
    let axis_base = Matrix2::new(norm_axis.x, norm_axis.y, -norm_axis.y, norm_axis.x);
    axis_base * Matrix2::new(1.0, 0.0, 0.0, -1.0) * axis_base.invert().expect("axis_base should be isometric") * to_reflect
}

pub fn intersection_lambda(pos1: Vector2<f32>, dir1: Vector2<f32>, pos2: Vector2<f32>, dir2: Vector2<f32>) -> Result<f32, ()> {
    //pos1 + lambda1 * dir1 = pos2 + lambda2 * dir2
    //[dir1 -dir2] * [lambda1; lambda2] = pos2 - pos1
    //returns lambda1
    let a = Matrix2::from_cols(dir1, -dir2);
    
    Ok((a.invert().ok_or(())? * (pos2 - pos1)).x)
}

#[cfg(test)]
mod test {
    use super::*;
    #[test]
    fn test_reflect() {
        let axis = Vector2::new(1.0, 0.0);
        assert!((Vector2::new(1.0, -1.0) - reflect(axis, [1.0, 1.0].into())).magnitude().abs() < 1e-5);
        assert!((Vector2::new(1.0, -1.0) - reflect(-axis, [1.0, 1.0].into())).magnitude().abs() < 1e-5);
    }

    #[test]
    fn test_intersect() {
        let case1 = intersection_lambda([0.0, 0.0].into(), [1.0, 0.0].into(), [0.0, 1.0].into(), [1.0, 0.0].into());
        assert!(matches!(case1, Err(())));
        let case2 = intersection_lambda([0.0, 0.0].into(), [2.0, 0.0].into(), [1.0, 1.0].into(), [0.0, 1.0].into());
        assert!(case2.is_ok_and(|val| (val - 0.5).abs() < 1e-5));
    }
}