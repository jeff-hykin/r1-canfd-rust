//! The vendor's gripper opening -> motor angle mapping, ported from mobiman `r1_gripper_controller` (`rgc`).
//!
//! `rgc` takes an opening (0 = closed .. 100 = open; it calls it both percent and stroke and treats them as the same
//! number, `openingPercentToStroke` rgc@0x15ce90 is the identity), inverts the four-segment sine linkage of
//! `a2_arm_analysis::gripper_decode` through a lookup table, and sends the motor angle as `p_des`.

/// Segment bounds of the linkage angle (`gripper_geom::kSegLimits` rgc@0x1de790).
#[allow(clippy::approx_constant)] // the vendor's stored doubles
pub const SEGMENT_LIMITS_RAD: [f64; 5] =
    [-0.1, 0.7853981633974483, 1.5707963267948966, 2.356194490192345, 3.141592653589793];
/// Per-segment stroke bases (rgc@0x1de740, 0x1de720, 0x1de728, 0x1de738).
pub const SEGMENT_BASES: [f64; 4] = [6.696960066389071, 20.090880199167213, 33.48480033194535, 46.8787204647235];
/// Stroke at and beyond the last segment (rgc@0x1de718).
pub const FULL_STROKE: f64 = 107.15136106222513;
/// Phase inside each segment's sine (rgc@0x1de748, pi/8).
#[allow(clippy::approx_constant)] // the vendor's stored literal
pub const SEGMENT_PHASE_RAD: f64 = 0.39269908169872414;
/// Crank length the sine is scaled by (literal 17.5 in getInverseLUT rgc@0x189a20).
pub const CRANK: f64 = 17.5;
/// Samples per segment: i = 0..=300 of `lo + i/300 (hi - lo)` (getInverseLUT rgc@0x189a20, loop bound 0x12d).
pub const SAMPLES_PER_SEGMENT: u32 = 300;
/// A sample joins the table only if it rises by more than this (rgc@0x1de750).
pub const MONOTONIC_EPSILON: f64 = 1e-6;
/// Added to the interpolation denominator (strokeToAngle rgc@0x189d70, rgc@0x1de758).
pub const INTERPOLATION_EPSILON: f64 = 1e-9;
/// Linkage angle at the motor's zero (applyGripperMapping rgc@0x15de80, rgc@0x1de768).
pub const MOTOR_ZERO_LINKAGE_RAD: f64 = 0.10890854532444616;
/// Motor angle limit: p_des is clamped to [-2.8, 0] (literal -2.8 / 2.8 in applyGripperMapping rgc@0x15de80).
pub const MOTOR_TRAVEL_RAD: f64 = 2.8;
/// Fallback slope when the table is empty (stroke x -0.028, rgc@0x1de760); unreachable with the vendor constants.
pub const FALLBACK_RAD_PER_STROKE: f64 = -0.027999999999999997;
/// Openings are clamped to [0, 100] (applyGripperMapping rgc@0x15de80, buildDirectTarget rgc@0x15e6e0).
pub const MAX_OPENING: f64 = 100.0;

/// Stroke at linkage angle `theta` exactly as getInverseLUT rgc@0x189a20 evaluates it (negative results become 0).
pub fn stroke_at(theta: f64) -> f64 {
    let [s0, s1, s2, s3, s4] = SEGMENT_LIMITS_RAD;
    let (base, offset) = if s0 <= theta && theta < s1 {
        (SEGMENT_BASES[0], 0.0)
    } else if s1 <= theta && theta < s2 {
        (SEGMENT_BASES[1], s1)
    } else if s2 <= theta && theta < s3 {
        (SEGMENT_BASES[2], s2)
    } else if s3 <= theta && theta < s4 {
        (SEGMENT_BASES[3], s3)
    } else {
        return if s0 <= theta { FULL_STROKE } else { 0.0 };
    };
    let half = base - (SEGMENT_PHASE_RAD - (theta - offset)).sin() * CRANK;
    (half + half).max(0.0)
}

/// `gripper_geom::InverseLUT`: strictly increasing strokes and their linkage angles.
#[derive(Clone, Debug, PartialEq)]
pub struct InverseLut {
    pub strokes: Vec<f64>,
    pub thetas: Vec<f64>,
}

impl InverseLut {
    /// Build the table as getInverseLUT rgc@0x189a20 does (the last sample of the last segment is exactly pi).
    pub fn vendor() -> InverseLut {
        let mut table = InverseLut { strokes: Vec::new(), thetas: Vec::new() };
        for segment in 0..4 {
            let (low, high) = (SEGMENT_LIMITS_RAD[segment], SEGMENT_LIMITS_RAD[segment + 1]);
            for sample in 0..=SAMPLES_PER_SEGMENT {
                let theta = if sample == SAMPLES_PER_SEGMENT && segment == 3 {
                    high
                } else {
                    low + (f64::from(sample) / f64::from(SAMPLES_PER_SEGMENT)) * (high - low)
                };
                let stroke = stroke_at(theta);
                let accept = match table.strokes.last() {
                    None => true,
                    Some(&last) => last - MONOTONIC_EPSILON <= stroke && MONOTONIC_EPSILON < (stroke - last).abs(),
                };
                if accept {
                    table.strokes.push(stroke);
                    table.thetas.push(theta);
                }
            }
        }
        table
    }

    /// `gripper_geom::strokeToAngle` rgc@0x189d70: clamp to the table ends, else lower_bound + linear interpolation.
    pub fn stroke_to_angle(&self, stroke: f64) -> f64 {
        let (Some(&min_stroke), Some(&max_stroke)) = (self.strokes.first(), self.strokes.last()) else {
            return f64::NAN;
        };
        if stroke <= min_stroke {
            return self.thetas[0];
        }
        if max_stroke <= stroke {
            return self.thetas[self.thetas.len() - 1];
        }
        let upper = self.strokes.partition_point(|&value| value < stroke);
        let (s0, s1) = (self.strokes[upper - 1], self.strokes[upper]);
        let (t0, t1) = (self.thetas[upper - 1], self.thetas[upper]);
        t0 + ((stroke - s0) / ((s1 - s0) + INTERPOLATION_EPSILON)) * (t1 - t0)
    }

    /// `applyGripperMapping` rgc@0x15de80: opening 0..100 -> motor p_des in [-2.8, 0] (0 = closed).
    pub fn opening_to_motor_rad(&self, opening: f64) -> f64 {
        let opening = if opening < MAX_OPENING { opening.max(0.0) } else { MAX_OPENING };
        let theta = self.stroke_to_angle(opening);
        if theta.is_nan() {
            return opening * FALLBACK_RAD_PER_STROKE;
        }
        let travel = theta - MOTOR_ZERO_LINKAGE_RAD;
        if travel < 0.0 {
            0.0
        } else if travel <= MOTOR_TRAVEL_RAD {
            -travel
        } else {
            -MOTOR_TRAVEL_RAD
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arm::gripper_opening_mm;

    #[test]
    fn closed_open_and_middle() {
        let table = InverseLut::vendor();
        assert_eq!(table.strokes.len(), table.thetas.len());
        assert!(table.strokes.windows(2).all(|pair| pair[0] < pair[1]));
        assert_eq!(table.opening_to_motor_rad(0.0), 0.0);
        assert_eq!(table.opening_to_motor_rad(-5.0), 0.0);
        assert_eq!(table.opening_to_motor_rad(100.0), -2.8);
        assert_eq!(table.opening_to_motor_rad(250.0), -2.8);
        let middle = table.opening_to_motor_rad(50.0);
        assert!((middle + 1.3535259356278524).abs() < 1e-9, "{middle}");
    }

    #[test]
    fn inverse_of_the_hdas_feedback_linkage() {
        // HDAS's feedback decode (gripper_decode lpm@0x11feb0) maps the motor angle back to the same opening
        let table = InverseLut::vendor();
        for opening in [10.0, 30.0, 60.0, 90.0] {
            let motor = table.opening_to_motor_rad(opening);
            let back = gripper_opening_mm(motor).unwrap();
            assert!((back - opening).abs() < 0.05, "{opening} -> {motor} -> {back}");
        }
    }
}
