use crate::tools2d::{dist, intersect_with_edge, intersection_param, intersections};
use nalgebra::Point;
use ncollide2d::na::{Isometry2, Point2, Vector2};
use ncollide2d::partitioning::BVH;
use ncollide2d::query::visitors::RayInterferencesCollector;
use ncollide2d::query::{PointQuery, Ray, RayCast};
use ncollide2d::shape::{ConvexPolygon, Polyline, Segment};
use std::error::Error;

pub fn farthest_pair(hull: &ConvexPolygon<f64>) -> (usize, usize) {
    let mut i0: usize = 0;
    let mut i1: usize = 0;
    let mut dist: f64 = 0.0;
    // TODO: Switch to convex hull rotating calipers algorithm
    for i in 0..hull.points().len() {
        for j in 0..hull.points().len() {
            let d: f64 = (hull.points()[i] - hull.points()[j]).norm();
            if d > dist {
                dist = d;
                i0 = i;
                i1 = j;
            }
        }
    }

    (i0, i1)
}

struct MeanSearchState {
    fraction: f64,
    distance: f64,
    point: Point2<f64>,
}

pub struct MeanInteriorPoint {
    pub center: Point2<f64>,
    pub side0: Point2<f64>,
    pub side1: Point2<f64>,
}

pub struct ClosedPolyline {
    pub line: Polyline<f64>,
    pub hull: ConvexPolygon<f64>,
}

impl ClosedPolyline {
    /// Create a new closed polyline. The provided points will be copied and adjacent duplicates
    /// removed.  If the provided points are not enclosed, the first point will be copied to the
    /// end.
    pub fn new(
        points: &Vec<Point2<f64>>,
        tol: Option<f64>,
    ) -> Result<ClosedPolyline, Box<dyn Error>> {
        // Construct the vertices we're looking for
        let mut vertices: Vec<Point2<f64>> = vec![points[0]];
        let tol_value = tol.unwrap_or(1e-6);

        for i in 0..points.len() - 1 {
            if dist(&points[i], &points[i + 1]) >= tol_value {
                vertices.push(points[i]);
            }
        }

        if dist(&vertices[0], vertices.last().unwrap()) > tol_value {
            vertices.push(vertices[0]);
        }

        let hull: ConvexPolygon<f64> = ConvexPolygon::try_from_points(&vertices).unwrap();
        let line: Polyline<f64> = Polyline::new(vertices, Option::None);

        Ok(ClosedPolyline { line, hull })
    }

    pub fn intersect_with_edge(&self, ray: &Ray<f64>, edge_index: usize) -> Option<Point2<f64>> {
        intersect_with_edge(&self.line, &ray, edge_index)
    }

    // pub fn naive_intersections(&self, ray: &Ray<f64>) -> Vec<Point2<f64>> {
    //     let mut results: Vec<Point2<f64>> = Vec::new();
    //     for (i, _) in self.line.edges().iter().enumerate() {
    //         if let Some(point) = self.intersect_with_edge(ray, i) {
    //             results.push(point);
    //         }
    //     }
    //
    //     results
    // }

    pub fn intersections(&self, ray: &Ray<f64>) -> Vec<Point2<f64>> {
        intersections(&self.line, &ray)
    }

    pub fn mean_point(
        &self,
        p0: &Point2<f64>,
        p1: &Point2<f64>,
        tol: f64,
    ) -> Option<MeanInteriorPoint> {
        let mut positive = MeanSearchState {
            fraction: 1.0,
            distance: 0.0,
            point: p1.clone(),
        };
        let mut negative = MeanSearchState {
            fraction: 0.0,
            distance: 0.0,
            point: p0.clone(),
        };
        let ray = Ray::new(p0.clone(), p1 - p0);

        let mut working: Point2<f64> = ray.point_at(0.5);
        while (positive.fraction - negative.fraction) * ray.dir.norm() > tol {
            let fraction = (positive.fraction + negative.fraction) * 0.5;
            working = ray.point_at(fraction);

            let closest = self
                .line
                .project_point(&Isometry2::<f64>::identity(), &working, false);

            let to_closest = closest.point - working;
            let distance = dist(&working, &closest.point);
            if to_closest.dot(&ray.dir) > 0.0 {
                // If this is true, then the closest point is in the direction of the ray normal
                positive.point = closest.point.clone();
                positive.fraction = fraction;
                positive.distance = distance;
            } else {
                negative.point = closest.point.clone();
                negative.fraction = fraction;
                negative.distance = distance;
            }
        }

        Some(MeanInteriorPoint {
            center: working,
            side0: negative.point,
            side1: positive.point,
        })
    }
}

#[cfg(test)]
mod tests {
    fn test_farthest_pair() {}
}
