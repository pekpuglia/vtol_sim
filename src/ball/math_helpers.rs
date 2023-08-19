// use cgmath::{Vector2, Matrix2, InnerSpace, SquareMatrix, Deg};
use nalgebra::{Vector2, Matrix2};

pub fn reflect(axis: Vector2<f64>, to_reflect: Vector2<f64>) -> Vector2<f64> {
    let norm_axis = axis.normalize();
    let axis_base = Matrix2::new(norm_axis.x, norm_axis.y, -norm_axis.y, norm_axis.x);
    axis_base * Matrix2::new(1.0, 0.0, 0.0, -1.0) * axis_base.try_inverse().expect("axis_base should be isometric") * to_reflect
}

pub fn intersection_lambda(pos1: Vector2<f64>, dir1: Vector2<f64>, pos2: Vector2<f64>, dir2: Vector2<f64>) -> Result<f64, ()> {
    //pos1 + lambda1 * dir1 = pos2 + lambda2 * dir2
    //[dir1 -dir2] * [lambda1; lambda2] = pos2 - pos1
    //returns lambda1
    let a = Matrix2::from_columns(&[dir1, -dir2]);
    
    Ok((a.try_inverse().ok_or(())? * (pos2 - pos1)).x)
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