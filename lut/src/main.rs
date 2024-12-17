use lerp::Lerp;
use std::env;
use std::fs::read_to_string;

fn load_values_comma_separated(filename: &str) -> Vec<f64> {
    let contents = read_to_string(filename).expect("Unable to read file");
    contents
        .split(',')
        .map(|s| s.trim().parse::<f64>().expect("Invalid number"))
        .collect()
}

fn find_interval(values: &[f64], target: f64) -> Option<usize> {
    match values.binary_search_by(|val| val.partial_cmp(&target).unwrap()) {
        Ok(index) => Some(index),
        Err(index) => {
            if index == 0 || index == values.len() {
                None
            } else {
                Some(index - 1)
            }
        }
    }
}

fn interpolate_y_to_x(xs: &[f64], ys: &[f64], y_target: f64) -> Option<f64> {
    let y_min = ys[0];
    let y_max = ys[ys.len() - 1];

    if (y_target - y_max).abs() < f64::EPSILON {
        return Some(xs[xs.len() - 1]);
    }

    let i = find_interval(ys, y_target)?;
    let y0 = ys[i];
    let y1 = ys[i + 1];
    let x0 = xs[i];
    let x1 = xs[i + 1];

    let t = (y_target - y0) / (y1 - y0);
    Some(x0.lerp(x1, t))
}

fn main() {
    let args: Vec<String> = env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <y_value>", args[0]);
        std::process::exit(1);
    }
    let y_target = args[1]
        .parse::<f64>()
        .expect("Failed to parse input as f64");

    let mut xs = load_values_comma_separated("lut_x.txt");
    let mut ys = load_values_comma_separated("lut_y.txt");

    assert_eq!(
        xs.len(),
        ys.len(),
        "X and Y arrays must have the same length."
    );

    let mut pairs: Vec<(f64, f64)> = xs.into_iter().zip(ys.into_iter()).collect();
    pairs.sort_by(|a, b| a.1.partial_cmp(&b.1).expect("Cannot compare values"));
    let (xs_sorted, ys_sorted): (Vec<_>, Vec<_>) = pairs.into_iter().unzip();

    let y_min = ys_sorted[0];
    let y_max = ys_sorted[ys_sorted.len() - 1];

    let x_result = if y_target < y_min {
        xs_sorted[0]
    } else if y_target > y_max {
        xs_sorted[xs_sorted.len() - 1]
    } else {
        interpolate_y_to_x(&xs_sorted, &ys_sorted, y_target)
            .expect("Interpolation should succeed in range")
    };

    let percentage = if y_target <= y_min {
        0.0
    } else if y_target >= y_max {
        100.0
    } else {
        ((y_target - y_min) / (y_max - y_min)) * 100.0
    };

    println!("{:.5}", percentage);
}
