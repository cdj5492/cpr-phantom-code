use bevy_math::prelude::*;

pub fn solve_rib(segments_c: Vec<f32>, thetas: Vec<f32>, errors: Vec<f32>) -> Vec<Vec2> {
    let thetas_mod: Vec<f32> = thetas
        .iter()
        .zip(errors.iter())
        .map(|(theta, error)| theta + error)
        .collect();
    let r = segments_c
        .iter()
        .zip(thetas_mod.iter())
        .map(|(segment, theta)| segment / theta)
        .collect::<Vec<f32>>();
    let segments_s: Vec<f32> = r
        .iter()
        .zip(thetas_mod.iter())
        .map(|(r, theta)| r * (2.0 * (1.0 - theta.cos())).sqrt())
        .collect();

    let mut phis = vec![0.0; thetas_mod.len()];
    phis[0] = thetas_mod[0] / 2.0;
    for i in 1..thetas_mod.len() {
        phis[i] = (thetas_mod[i - 1] + thetas_mod[i]) / 2.0;
    }

    let mut points = vec![Vec2::ZERO]; // Initialize with a zero vector
    let mut t = phis[0];
    for i in 0..thetas_mod.len() {
        let direction = Vec2::from_angle(t);
        // SAFETY: points is guaranteed to have at least one element (the initial zero vector)
        let point = points.last().unwrap() + segments_s[i] * direction;
        points.push(point);
        t += phis[i];
    }

    points
}
