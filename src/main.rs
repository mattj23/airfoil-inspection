use nalgebra::{Vector2, Vector3, distance};
use serde::{Deserialize, Serialize};
use serde_json::Result;
use std::fs;

fn main() {
    let data = fs::read_to_string("/home/matt/working/airfoil/data/E-E.json")
        .expect("Unable to read file");

    let input: InputData = serde_json::from_str(&data).expect("Failed to parse input data");

    let raw_nominal = points_from_string(&input.nominal).unwrap();
    let mut nominal = Contour::from_points(&raw_nominal);
    println!("Original size: {}", nominal.points.len());
    nominal.remove_adjacent_duplicates();
    println!("Reduced size: {}", nominal.points.len());
}

fn points_from_string(s: &str) -> Result<Vec<Vector3<f64>>> {
    let mut raw_points: Vec<Vector3<f64>> = Vec::new();
    for token in s.split(";") {
        let mut pieces: [f64; 3] = [0.0; 3];
        for pair in token.split(",").take(3).enumerate() {
            pieces[pair.0] = pair
                .1
                .parse()
                .expect("Couldn't parse a floating point value");
        }
        raw_points.push(Vector3::new(pieces[0], pieces[1], pieces[2]));
    }
    Ok(raw_points)
}

struct Contour {
    points: Vec<Vector2<f64>>,
}

impl Contour {
    fn remove_adjacent_duplicates(&mut self) {
        self.points.dedup_by(|a, b| (a.x - b.x).powf(2.0) + (a.y - b.y).powf(2.0) < 1e-6);
    }

    fn from_points(points: &Vec<Vector3<f64>>) -> Contour {
        let mut result = Contour { points: Vec::new() };
        // TODO: actual plane discovery and projection
        for p in points {
            result.points.push(Vector2::new(p.x, p.y));
        }

        result
    }
}

#[derive(Serialize, Deserialize)]
struct InputData {
    leading_edge: Vec<f64>,
    nominal: String,
    actual: String,
}
