mod closed_polyline;
mod tools2d;

use nalgebra::{Point, Point3};
use ncollide2d::na::{Isometry2, Point2, Vector2};
use ncollide2d::query::{Ray, RayCast};
use ncollide2d::shape::{ConvexPolygon, Polyline, Segment};
use serde::{Deserialize, Serialize};
use serde_json::Result;
use std::f64::consts::PI;
use std::fs;

use crate::tools2d::dist;
use closed_polyline::{farthest_pair, ClosedPolyline};
use tools2d::{intersection_param, ray_intersect_aabb};

fn main() {
    let data = fs::read_to_string("/home/matt/working/airfoil/data/E-E.json")
        .expect("Unable to read file");

    let input: InputData = serde_json::from_str(&data).expect("Failed to parse input data");

    let nominal_3d = points_from_string(&input.nominal).unwrap();
    let mut nominal_2d = to_point2(&nominal_3d);

    let closed = ClosedPolyline::new(&nominal_2d, Some(1e-4)).unwrap();
    // for p in closed.line.points().iter() {
    //     println!("{:?}", p);
    // }

    let (i0, i1) = farthest_pair(&closed.hull);
    let p0 = &closed.hull.points()[i0];
    let p1 = &closed.hull.points()[i1];
    let rough_chord = Ray::new(*p0, p1 - p0);

    let pc = rough_chord.point_at(0.5);
    let cross_ray = Ray::new(
        pc,
        Isometry2::rotation(PI / 2.0) * rough_chord.dir.normalize(),
    );
    let intersections = closed.intersections(&cross_ray);
    let mean = closed
        .mean_point(&intersections[0], &intersections[1], 1e-4)
        .unwrap();
    for p in intersections.iter() {
        println!("{:?}", p);
    }
    println!("Center: {:?}", mean.center);
    println!(
        "Side 0: {:?} {}",
        mean.side0,
        dist(&mean.center, &mean.side0)
    );
    println!(
        "Side 1: {:?} {}",
        mean.side1,
        dist(&mean.center, &mean.side1)
    );
}

fn points_from_string(s: &str) -> Result<Vec<Point3<f64>>> {
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

fn to_point2(points: &Vec<Point3<f64>>) -> Vec<Point2<f64>> {
    let mut converted: Vec<Point2<f64>> = Vec::new();
    // TODO: actual plane discovery and projection
    for point in points {
        converted.push(Point2::new(point.x, point.y));
    }
    converted
}

#[derive(Serialize, Deserialize)]
struct InputData {
    leading_edge: Vec<f64>,
    nominal: String,
    actual: String,
}
