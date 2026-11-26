//! Three-wheel swerve kinematics, ported from mobiman `chassis_control_node` (`ccn`, r1_chassis_control.cpp).
//!
//! Wheel order is fl, fr, r. Contact points: fl (a, +b), fr (a, -b), r (-r, 0). Steer angle 0 = wheel along +x,
//! positive counter-clockwise. All math is f64 like ccn; HDAS rounds to f32 only when it encodes.

use serde::Serialize;

use crate::encode::ChassisTarget;

/// ccn's `.data` geometry: front wheels' x `a`, front half-track `b`, rear wheel's distance behind centre `r`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct SwerveGeometry {
    pub a_m: f64,
    pub b_m: f64,
    pub r_m: f64,
}

/// ccn symbols `a`, `b`, `r` (read by vector_control ccn@0x12a340 and the static init ccn@0x124c20).
pub const VENDOR_GEOMETRY: SwerveGeometry = SwerveGeometry { a_m: 0.16867, b_m: 0.28, r_m: 0.32733 };

/// vector_control ccn@0x12a340 scales vx and vy down to 1.5 m/s.
pub const MAX_LINEAR_M_S: f64 = 1.5;
/// ... and wz to 3 rad/s.
pub const MAX_ANGULAR_RAD_S: f64 = 3.0;
/// map_angle ccn@0x12a184 folds at pi/2 (ccn@0x19c3d8 / -pi/2 at 0x19c3e8).
const QUARTER_TURN: f64 = std::f64::consts::FRAC_PI_2;
/// ... by pi (ccn@0x19c3e0).
const HALF_TURN: f64 = std::f64::consts::PI;
/// ... and wraps by 2 pi (ccn@0x19c3f0).
const FULL_TURN: f64 = std::f64::consts::TAU;

/// A body twist.
#[derive(Clone, Copy, Debug, PartialEq, Default, Serialize)]
pub struct Twist {
    pub vx_m_s: f64,
    pub vy_m_s: f64,
    pub wz_rad_s: f64,
}

/// Which of map_angle's two `default_chassis_type` branches run (ccn@0x12a184 compares the type string twice).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub struct MapAngleBranches {
    /// First branch: fold the reference into [-pi/2, pi/2] and flip the speed.
    pub fold_to_half_plane: bool,
    /// Second branch: flip when the reference is more than pi/2 from the reference angle passed in, then wrap to +-pi.
    pub flip_toward_actual: bool,
}

impl MapAngleBranches {
    /// map_angle ccn@0x12a184 runs the first branch when `default_chassis_type == "W0"` (ccn@0x192078) and the second
    /// when it is `"W1"` (ccn@0x192080); any other type runs neither.
    pub fn for_chassis_type(chassis_type: &str) -> MapAngleBranches {
        MapAngleBranches { fold_to_half_plane: chassis_type == "W0", flip_toward_actual: chassis_type == "W1" }
    }
}

/// ccn's `default_chassis_type` initialiser is "W0" (static init ccn@0x124c20); the `chassis_type` param can override.
pub const VENDOR_CHASSIS_TYPE: &str = "W0";
/// The branches for [`VENDOR_CHASSIS_TYPE`].
pub const VENDOR_MAP_ANGLE: MapAngleBranches = MapAngleBranches { fold_to_half_plane: true, flip_toward_actual: false };

/// `map_angle` ccn@0x12a184: (steer angle to command, speed sign).
pub fn map_angle(branches: MapAngleBranches, angle_ref: f64, angle_actual: f64) -> (f64, f64) {
    let mut sign = 1.0;
    let mut mapped = angle_ref;
    if branches.fold_to_half_plane {
        if QUARTER_TURN < mapped {
            mapped -= HALF_TURN;
            sign = -1.0;
        }
        if mapped < -QUARTER_TURN {
            sign = -sign;
            mapped += HALF_TURN;
        }
    }
    if branches.flip_toward_actual {
        let error = angle_ref - angle_actual;
        if QUARTER_TURN < error.abs() {
            sign = -sign;
            mapped -= HALF_TURN.copysign(error);
        }
        if HALF_TURN < mapped.abs() {
            mapped -= FULL_TURN.copysign(mapped);
        }
    }
    (mapped, sign)
}

/// vector_control ccn@0x12a340's clamp: each component scaled to its limit keeping its sign.
fn limit(value: f64, max: f64) -> f64 {
    if max < value.abs() {
        (value * max) / value.abs()
    } else {
        value
    }
}

/// Wheel-frame (x, y) velocities of fl, fr, r for a twist (vector_control ccn@0x12a340).
pub fn wheel_vectors(geometry: &SwerveGeometry, twist: Twist) -> [(f64, f64); 3] {
    let (vx, vy, wz) = (twist.vx_m_s, twist.vy_m_s, twist.wz_rad_s);
    [
        (vx - wz * geometry.b_m, vy + wz * geometry.a_m),
        (wz * geometry.b_m + vx, vy + wz * geometry.a_m),
        (vx, vy - wz * geometry.r_m),
    ]
}

/// The IK's memory: last commanded angle per wheel (`last_angle_ref_fl/rl/rr` in ccn's .bss).
#[derive(Clone, Copy, Debug, PartialEq, Default, Serialize)]
pub struct SwerveIk {
    pub last_angle_ref_rad: [f64; 3],
}

impl SwerveIk {
    /// `vector_control` ccn@0x12a340: clamp, per-wheel vector, atan2 angle, map_angle against the last command for
    /// the angle and against the measured steer angle for the speed sign. Returns angles and signed speeds.
    pub fn solve(
        &mut self,
        geometry: &SwerveGeometry,
        branches: MapAngleBranches,
        twist: Twist,
        measured_steer_rad: &[f64; 3],
    ) -> ([f64; 3], [f64; 3]) {
        let clamped = Twist {
            vx_m_s: limit(twist.vx_m_s, MAX_LINEAR_M_S),
            vy_m_s: limit(twist.vy_m_s, MAX_LINEAR_M_S),
            wz_rad_s: limit(twist.wz_rad_s, MAX_ANGULAR_RAD_S),
        };
        let vectors = wheel_vectors(geometry, clamped);
        let mut angles = [0.0; 3];
        let mut speeds = [0.0; 3];
        for wheel in 0..3 {
            let (x, y) = vectors[wheel];
            let speed = (x * x + y * y).sqrt();
            let reference = y.atan2(x);
            angles[wheel] = map_angle(branches, reference, self.last_angle_ref_rad[wheel]).0;
            speeds[wheel] = map_angle(branches, reference, measured_steer_rad[wheel]).1 * speed;
        }
        self.last_angle_ref_rad = angles;
        (angles, speeds)
    }
}

/// Body twist from wheel speeds and steer angles: the FK in ccn's /hdas/feedback_chassis callback ccn@0x128d30.
///
/// vx, vy are the wheel-vector means; wz projects each wheel on its tangent (V_T1/V_T2/V_T3 from the static init
/// ccn@0x124c20, theta = atan(a/b)) and divides every wheel by `r`, although the front lever arm is sqrt(a^2+b^2).
pub fn forward_kinematics(geometry: &SwerveGeometry, wheel_speed_m_s: &[f64; 3], steer_angle_rad: &[f64; 3]) -> Twist {
    let theta = (geometry.a_m / geometry.b_m).atan();
    let tangent_fr = (theta.cos(), theta.sin());
    let tangent_fl = ((HALF_TURN - theta).cos(), (HALF_TURN - theta).sin());
    let tangent_r = (0.0, -1.0);
    let wheel = |index: usize| {
        let (sin, cos) = steer_angle_rad[index].sin_cos();
        (wheel_speed_m_s[index] * cos, wheel_speed_m_s[index] * sin)
    };
    let (fl, fr, r) = (wheel(0), wheel(1), wheel(2));
    let dot = |tangent: (f64, f64), vector: (f64, f64)| tangent.0 * vector.0 + tangent.1 * vector.1;
    Twist {
        vx_m_s: (fl.0 + fr.0 + r.0) / 3.0,
        vy_m_s: (fl.1 + fr.1 + r.1) / 3.0,
        wz_rad_s: (dot(tangent_fl, fl) + dot(tangent_fr, fr) + dot(tangent_r, r)) / geometry.r_m / 3.0,
    }
}

/// Rigid-body least-squares twist from the three wheel vectors (not the vendor's: exact for any rigid motion).
///
/// Solves `vx - wz py = ux`, `vy + wz px = uy` over the contact points in the normal equations.
pub fn forward_kinematics_least_squares(
    geometry: &SwerveGeometry,
    wheel_speed_m_s: &[f64; 3],
    steer_angle_rad: &[f64; 3],
) -> Twist {
    let points = [(geometry.a_m, geometry.b_m), (geometry.a_m, -geometry.b_m), (-geometry.r_m, 0.0)];
    let (mut sum_x, mut sum_y, mut sum_r2, mut sum_ux, mut sum_uy, mut sum_moment) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    for (wheel, (px, py)) in points.iter().enumerate() {
        let (sin, cos) = steer_angle_rad[wheel].sin_cos();
        let (ux, uy) = (wheel_speed_m_s[wheel] * cos, wheel_speed_m_s[wheel] * sin);
        sum_x += px;
        sum_y += py;
        sum_r2 += px * px + py * py;
        sum_ux += ux;
        sum_uy += uy;
        sum_moment += px * uy - py * ux;
    }
    // normal equations [[n, 0, -Sy], [0, n, Sx], [-Sy, Sx, Sr2]] (vx, vy, wz) = (Sux, Suy, Smoment)
    let count = points.len() as f64;
    let wz =
        (sum_moment - (sum_x * sum_uy - sum_y * sum_ux) / count) / (sum_r2 - (sum_x * sum_x + sum_y * sum_y) / count);
    Twist { vx_m_s: (sum_ux + sum_y * wz) / count, vy_m_s: (sum_uy - sum_x * wz) / count, wz_rad_s: wz }
}

/// MotorControl fields as ccn fills them for HDAS (p_des = angles, v_des = speeds then steer rates), in f32.
pub fn chassis_target(angles: &[f64; 3], speeds: &[f64; 3], steer_rates: &[f64; 3]) -> ChassisTarget {
    ChassisTarget {
        wheel_speed_m_s: speeds.map(|value| value as f32),
        steer_angle_rad: angles.map(|value| value as f32),
        steer_rate_rad_s: steer_rates.map(|value| value as f32),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn assert_twist_close(got: Twist, want: Twist, tolerance: f64) {
        let error = [got.vx_m_s - want.vx_m_s, got.vy_m_s - want.vy_m_s, got.wz_rad_s - want.wz_rad_s];
        assert!(error.iter().all(|e| e.abs() <= tolerance), "{got:?} vs {want:?}");
    }

    /// The vendor FK's output for the IK of `twist`, worked by hand: vx passes through; vy' = vy + wz (2a - r) / 3
    /// (the contact points' mean x is not 0); with L = sqrt(a^2+b^2), wz' = (vy (2a/L - 1) + wz (2L + r)) / (3r),
    /// because the tangents sum to (0, 2a/L - 1) and every wheel is divided by r.
    fn vendor_fk_of_ik(twist: Twist) -> Twist {
        let SwerveGeometry { a_m: a, b_m: b, r_m: r } = VENDOR_GEOMETRY;
        let lever = (a * a + b * b).sqrt();
        let wz = (twist.vy_m_s * (2.0 * a / lever - 1.0) + twist.wz_rad_s * (2.0 * lever + r)) / (3.0 * r);
        Twist { vx_m_s: twist.vx_m_s, vy_m_s: twist.vy_m_s + twist.wz_rad_s * (2.0 * a - r) / 3.0, wz_rad_s: wz }
    }

    #[test]
    fn hand_computed_pure_rotation() {
        let g = VENDOR_GEOMETRY;
        let vectors = wheel_vectors(&g, Twist { vx_m_s: 0.0, vy_m_s: 0.0, wz_rad_s: 1.0 });
        assert_eq!(vectors, [(-0.28, 0.16867), (0.28, 0.16867), (0.0, -0.32733)]);
        let speeds = vectors.map(|(x, y)| (x * x + y * y).sqrt());
        let angles = vectors.map(|(x, y)| y.atan2(x));
        let twist = forward_kinematics(&g, &speeds, &angles);
        // by hand: L = 0.3268785, wz = (2 L + r) / 3r = 0.9990805, vy = (2a - r) / 3 = 0.0033367
        assert!((twist.wz_rad_s - 0.9990805).abs() < 1e-6, "{twist:?}");
        assert!((twist.vy_m_s - 0.0033367).abs() < 1e-6 && twist.vx_m_s.abs() < 1e-12, "{twist:?}");
        let exact = forward_kinematics_least_squares(&g, &speeds, &angles);
        assert_twist_close(exact, Twist { vx_m_s: 0.0, vy_m_s: 0.0, wz_rad_s: 1.0 }, 1e-12);
    }

    #[test]
    fn ik_then_fk_round_trips() {
        for twist in [
            Twist { vx_m_s: 0.5, vy_m_s: 0.0, wz_rad_s: 0.0 },
            Twist { vx_m_s: -0.3, vy_m_s: 0.4, wz_rad_s: 0.0 },
            Twist { vx_m_s: 0.2, vy_m_s: -0.1, wz_rad_s: 0.7 },
            Twist { vx_m_s: -1.0, vy_m_s: 0.0, wz_rad_s: -1.2 },
        ] {
            let mut ik = SwerveIk::default();
            // settle the steer memory, then solve with the wheels where the last command put them
            let (angles, _) = ik.solve(&VENDOR_GEOMETRY, VENDOR_MAP_ANGLE, twist, &[0.0; 3]);
            let (angles, speeds) = ik.solve(&VENDOR_GEOMETRY, VENDOR_MAP_ANGLE, twist, &angles);
            let back = forward_kinematics(&VENDOR_GEOMETRY, &speeds, &angles);
            assert_twist_close(back, vendor_fk_of_ik(twist), 1e-12);
            let exact = forward_kinematics_least_squares(&VENDOR_GEOMETRY, &speeds, &angles);
            assert_twist_close(exact, twist, 1e-12);
        }
    }

    #[test]
    fn zero_twist_commands_zero_and_clamps_scale() {
        let mut ik = SwerveIk::default();
        let (angles, speeds) = ik.solve(&VENDOR_GEOMETRY, VENDOR_MAP_ANGLE, Twist::default(), &[-0.010, 0.007, -0.016]);
        assert_eq!((angles, speeds), ([0.0; 3], [0.0; 3]));
        assert_eq!(limit(3.0, MAX_LINEAR_M_S), 1.5);
        assert_eq!(limit(-4.0, MAX_ANGULAR_RAD_S), -3.0);
    }

    #[test]
    fn map_angle_folds_and_flips() {
        let fold = MapAngleBranches { fold_to_half_plane: true, flip_toward_actual: false };
        let (angle, sign) = map_angle(fold, 2.0, 0.0);
        assert!((angle - (2.0 - HALF_TURN)).abs() < 1e-12 && sign == -1.0);
        let (angle, sign) = map_angle(fold, -2.0, 0.0);
        assert!((angle - (HALF_TURN - 2.0)).abs() < 1e-12 && sign == -1.0);
        let flip = MapAngleBranches { fold_to_half_plane: false, flip_toward_actual: true };
        let (angle, sign) = map_angle(flip, 3.0, 0.0);
        assert!((angle - (3.0 - HALF_TURN)).abs() < 1e-12 && sign == -1.0);
    }
}
