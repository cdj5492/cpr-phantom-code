use bevy_math::prelude::*;

pub struct Rib {
    /// A borrowed slice of segments
    pub segments: &'static [RibSegment],
}

pub struct RibSegment {
    pub channel: usize,
    pub length: f32,
    pub error: f32,
}

impl Rib {
    /// extracts relevent thetas from incoming data stream using mapped channels
    pub fn extract_thetas(&self, data: &Vec<f32>) -> Vec<f32> {
        self.segments
            .iter()
            .map(|segment| data[segment.channel])
            .collect()
    }

    pub fn solve_rib(&self, thetas: Vec<f32>) -> Vec<Vec2> {
        let thetas_mod: Vec<f32> = self
            .segments
            .iter()
            .zip(thetas.iter())
            .map(|(segment, theta)| *theta + segment.error)
            .collect();
        let r = self
            .segments
            .iter()
            .map(|segment| segment.length)
            .zip(thetas_mod.iter())
            .map(|(segment_length, theta)| segment_length / theta)
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
}
