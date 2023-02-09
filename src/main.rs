use std::fs;
use serde::{Deserialize, Serialize};
use serde_json::Result;
use nalgebra::{Vector3, Vector2};

fn main() {
    let data = fs::read_to_string("/home/matt/working/airfoil/data/E-E.json")
        .expect("Unable to read file");

    let input: InputData = serde_json::from_str(&data)
        .expect("Failed to parse input data");

    let raw = points_from_string(&input.nominal).unwrap();
    for p in raw {
        println!("{}, {}, {}", p.x, p.y, p.z);
    }


}

fn points_from_string(s: &str) -> Result<Vec<Vector3<f64>>> {
    let mut raw_points: Vec<Vector3<f64>> = Vec::new();
    for token in s.split(";") {
        let mut pieces: [f64; 3] = [0.0; 3];
        for pair in token.split(",").take(3).enumerate() {
            pieces[pair.0] = pair.1.parse().expect("Couldn't parse a floating point value");
        }
        raw_points.push(Vector3::new(pieces[0], pieces[1], pieces[2]));
    }
    Ok(raw_points)
}

#[derive(Serialize, Deserialize)]
struct InputData {
    leading_edge: Vec<f64>,
    nominal: String,
    actual: String
}