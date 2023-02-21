use ncollide2d::bounding_volume::AABB;
use ncollide2d::na::{Isometry2, Point2, RealField, Vector2};
use ncollide2d::partitioning::{VisitStatus, Visitor};
use ncollide2d::query::Ray;
use std::error::Error;

pub fn dist<N: RealField + Copy>(a: &Point2<N>, b: &Point2<N>) -> N {
    (a - b).norm()
}

/// Computes the intersection parameter between two parameterized lines
pub fn intersection_param<N: RealField + Copy>(
    a0: &Point2<N>,
    ad: &Vector2<N>,
    b0: &Point2<N>,
    bd: &Vector2<N>,
) -> Option<(N, N)> {
    let det: N = bd.x * ad.y - bd.y * ad.x;
    if det.abs() < N::from_f64(1e-6).unwrap() {
        return Option::None;
    }

    let dx = b0.x - a0.x;
    let dy = b0.y - a0.y;

    Some(((dy * bd.x - dx * bd.y) / det, (dy * ad.x - dx * ad.y) / det))
}

pub fn ray_intersect_aabb<N: RealField + Copy>(b: &AABB<N>, r: &Ray<N>) -> bool {
    let x_inv = N::from_f64(1.0).unwrap() / r.dir.x;
    let y_inv = N::from_f64(1.0).unwrap() / r.dir.y;
    let tx1 = (b.mins.x - r.origin.x) * x_inv;
    let tx2 = (b.maxs.x - r.origin.x) * x_inv;
    let mut t_min = tx1.min(tx2);
    let mut t_max = tx1.max(tx2);

    let ty1 = (b.mins.y - r.origin.y) * y_inv;
    let ty2 = (b.maxs.y - r.origin.y) * y_inv;

    t_min = t_min.max(ty1.min(ty2));
    t_max = t_max.min(ty1.max(ty2));

    t_max >= t_min
}

pub struct RayVisitor<'a, N: 'a + RealField + Copy, T: 'a> {
    pub ray: &'a Ray<N>,
    pub collector: &'a mut Vec<T>,
}

impl<'a, N: RealField + Copy, T: Clone> RayVisitor<'a, N, T> {
    pub fn new(ray: &'a Ray<N>, buffer: &'a mut Vec<T>) -> RayVisitor<'a, N, T> {
        RayVisitor {
            ray,
            collector: buffer,
        }
    }
}

impl<'a, N, T> Visitor<T, AABB<N>> for RayVisitor<'a, N, T>
where
    N: RealField + Copy,
    T: Clone,
{
    fn visit(&mut self, bv: &AABB<N>, t: Option<&T>) -> VisitStatus {
        if ray_intersect_aabb(bv, self.ray) {
            if let Some(t) = t {
                self.collector.push(t.clone());
            }

            VisitStatus::Continue
        } else {
            VisitStatus::Stop
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::tools2d::ray_intersect_aabb;
    use ncollide2d::bounding_volume::AABB;
    use ncollide2d::na::{Point2, Vector2};
    use ncollide2d::query::NeighborhoodGeometry::Point;
    use ncollide2d::query::Ray;

    #[test]
    fn ray_aabb_test_intersect() {
        let aabb: AABB<f64> = AABB::new(Point2::new(0.0, 0.0), Point2::new(0.5, 0.5));
        let ray: Ray<f64> = Ray::new(Point2::new(0.25, 1.0), Vector2::new(0.0, 1.0));
        assert!(ray_intersect_aabb(&aabb, &ray));
    }

    #[test]
    fn ray_aabb_test_miss() {
        let aabb: AABB<f64> = AABB::new(Point2::new(0.0, 0.0), Point2::new(0.5, 0.5));
        let ray: Ray<f64> = Ray::new(Point2::new(1.25, 1.0), Vector2::new(0.0, 1.0));
        assert!(!ray_intersect_aabb(&aabb, &ray));
    }
}
