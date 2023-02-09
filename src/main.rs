use std::fmt::Debug;
use nalgebra::{distance, Point3, Scalar, Vector2, Vector3};
use ncollide2d::shape::{ConvexPolygon, Polyline};
use ncollide2d::na::Point2;
use serde::{Deserialize, Serialize};
use serde_json::Result;
use std::fs;

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
    println!("Max distance ({:?}, {:?}): {}", p0, p1, dist_2d(p0, p1));

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
