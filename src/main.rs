use nalgebra::{Point, Point3};
use ncollide2d::na::{Isometry2, Point2, Vector2};
use ncollide2d::shape::{ConvexPolygon, Polyline, Segment};
use serde::{Deserialize, Serialize};
use serde_json::Result;
use std::fs;
use ncollide2d::query::{Ray, RayCast};
use std::f64::consts::PI;

fn main() {
    let data = fs::read_to_string("/home/matt/working/airfoil/data/E-E.json")
        .expect("Unable to read file");

    let input: InputData = serde_json::from_str(&data).expect("Failed to parse input data");

    let nominal_3d = points_from_string(&input.nominal).unwrap();
    let mut nominal_2d = to_point2(&nominal_3d);
    remove_adjacent_duplicates(&mut nominal_2d);

    let poly: Polyline<f64> = Polyline::new(nominal_2d, Option::None);
    let hull: ConvexPolygon<f64> = ConvexPolygon::try_from_points(&poly.points())
        .expect("Couldn't generate convex polygon");

    println!("{:?}", hull);
    println!("Polyline: {}", poly.points().len());
    println!("Hull: {}", hull.points().len());

    // Farthest pair
    let (i0, i1) = farthest_pair(&hull);
    let p0 = &hull.points()[i0];
    let p1 = &hull.points()[i1];
    let rough_chord = Ray::new(*p0, p1 - p0);
    let rotate_90 = Isometry2::rotation(PI / 2.0);
    let rough_n = rotate_90 * rough_chord.dir.normalize();

    println!("Max distance ({:?}, {:?}): {}", p0, p1, dist_2d(p0, p1));
    for i in 1..10 {
        let ray = Ray::new(rough_chord.point_at(i as f64 / 10.0), rough_n);
        let ints = intersections(&ray, &poly);
        print!("Ray {}: ", i);
        for p in ints {
            print!("{:?} ", p);
        }
        print!("\n");
    }


}

fn intersection_param(a0: &Point2<f64>, ad: &Vector2<f64>, b0: &Point2<f64>, bd: &Vector2<f64>) -> Option<f64> {
    let det: f64 = bd.x * ad.y - bd.y * ad.x;
    if det.abs() < 1e-6 {
        return Option::None;
    }

    let dx = b0.x - a0.x;
    let dy = b0.y - a0.y;

    Some((dy * bd.x - dx * bd.y) / det)
}

fn intersections(ray: &Ray<f64>, poly: &Polyline<f64>) -> Vec<Point2<f64>> {
    let mut results: Vec<Point2<f64>> = Vec::new();
    for (i, edge) in poly.edges().iter().enumerate() {
        let p0 = &poly.points()[edge.indices.coords[0]];
        let p1 = &poly.points()[edge.indices.coords[1]];
        let d = p1 - p0;
        let param = intersection_param(&p0, &d, &ray.origin, &ray.dir);
        if param.is_some() {
            let f = param.unwrap();
            if 0.0 <= f  && f <= 1.0 {
                results.push(ray.origin + ray.dir * param.unwrap());
            }
        }
    }
    results.sort_by(|a, b| a[0].partial_cmp(&b[0]).unwrap());
    remove_adjacent_duplicates(&mut results);
    results
}

fn farthest_pair(hull: &ConvexPolygon<f64>) -> (usize, usize) {
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

fn points_from_string(s: &str) -> Result<Vec<Point3<f64>>>
{
    let mut raw_points: Vec<Point3<f64>> = Vec::new();
    for token in s.split(";") {
        let mut pieces: [f64; 3] = [Default::default(); 3];
        for pair in token.split(",").take(3).enumerate() {
            pieces[pair.0] = pair
                .1
                .parse()
                .expect("Couldn't parse a floating point value");
        }
        raw_points.push(Point3::new(pieces[0], pieces[1], pieces[2]));
    }
    Ok(raw_points)
}

fn to_point2(points: &Vec<Point3<f64>>) -> Vec<Point2<f64>>
{
    let mut converted: Vec<Point2<f64>> = Vec::new();
    // TODO: actual plane discovery and projection
    for point in points {
        converted.push(Point2::new(point.x, point.y));
    }
    converted
}

fn remove_adjacent_duplicates(points: &mut Vec<Point2<f64>>)
{
    points.dedup_by(|a, b| dist_2d(a, b) < 1e-6);
    if dist_2d(points.first().unwrap(), points.last().unwrap()) < 1e-6 {
        points.pop();
    }
}

fn dist_2d(a: &Point2<f64>, b: &Point2<f64>) -> f64 {
    (a - b).norm()
}

#[derive(Serialize, Deserialize)]
struct InputData {
    leading_edge: Vec<f64>,
    nominal: String,
    actual: String,
}
