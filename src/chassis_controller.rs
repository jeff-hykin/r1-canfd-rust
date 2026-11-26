//! chassis_control_node's control loop (`ccn`, `main` ccn@0x124df0), one [`ChassisController::step`] per 20 ms
//! loop iteration, turning a twist target into the per-wheel targets HDAS encodes as 0x031.
//!
//! ccn runs a fixed 20 ms loop (immediate 20 000 000 ns ccn@0x126944) with `dt = 0.02` baked into its ramp
//! (ccn@0x19c410); the caller owns that clock. Inputs arrive between steps like ccn's `spin_some` callbacks.

use serde::Serialize;

use crate::encode::ChassisTarget;
use crate::swerve::{
    forward_kinematics, MapAngleBranches, SwerveGeometry, SwerveIk, Twist, VENDOR_GEOMETRY, VENDOR_MAP_ANGLE,
};

/// ccn's loop period in seconds (ccn@0x126944 sleeps to 20 ms slots; the ramp uses 0.02 from ccn@0x19c410).
pub const LOOP_PERIOD_S: f64 = 0.02;
/// Loops without a reset before the timeout path (counter ccn@0x1d32dc saturates at 10).
pub const TIMEOUT_LOOPS: u32 = 10;
/// Startup acceleration limits (vx, vy, wz) (ccn .data @0x1d3390).
pub const DEFAULT_ACCELERATION: [f64; 3] = [2.5, 1.0, 2.5];
/// /motion_target/chassis_acc_limit caps each axis's limit (callback ccn@0x128720).
pub const ACCELERATION_CAPS: [f64; 3] = [2.5, 1.0, 1.0];
/// Jerk limit = this x the acceleration limit, per axis (profiles built in main ccn@0x124df0).
pub const JERK_PER_ACCELERATION: [f64; 3] = [10.0, 20.0, 10.0];
/// calculateNextVelocity ccn@0x129f60 clamps every ramp output (also wz) to +-1.5.
pub const RAMP_OUTPUT_LIMIT: f64 = 1.5;
/// A wheel more than pi/8 off its target angle counts as misaligned (ccn@0x19c418).
pub const MISALIGNED_RAD: f64 = 0.39269908169872414;
/// While misaligned the ramp state is clamped to +-0.07 m/s (ccn@0x19c428/0x19c430) ...
pub const MISALIGNED_LINEAR_LIMIT: f64 = 0.07;
/// ... or +-0.3 m/s when |target wz| > 0.1 (immediates in main ccn@0x124df0).
pub const MISALIGNED_TURNING_LINEAR_LIMIT: f64 = 0.3;
pub const MISALIGNED_TURNING_WZ: f64 = 0.1;
/// Brake steer pose fl, fr, r (float literals in main ccn@0x124df0).
pub const BRAKE_STEER_RAD: [f32; 3] = [0.75, -0.75, 0.0];
/// Wheel-speed low-pass: out = 0.1 x new + 0.9 x last out (ccn@0x19c420 / 0x19c438).
pub const LOW_PASS_NEW_WEIGHT: f64 = 0.1;
pub const LOW_PASS_OLD_WEIGHT: f64 = 0.9;
/// RC stick raw range and centre (mapping_function ccn@0x12a060).
pub const RC_RAW_MIN: f64 = 240.0;
pub const RC_RAW_MAX: f64 = 1807.0;
pub const RC_RAW_CENTRE: f64 = 1024.0;
/// RC mode byte that selects RC driving, and the one that hands control to software (callback ccn@0x12a0a0).
pub const RC_MODE_DRIVE: u8 = 2;
pub const RC_MODE_SOFTWARE: u8 = 5;

/// `calculateNextVelocity` ccn@0x129f60 (jerk-limited ramp toward `target`), from the disassembly.
pub fn next_velocity(
    velocity: f64,
    acceleration: f64,
    target: f64,
    max_acceleration: f64,
    max_jerk: f64,
    dt: f64,
) -> f64 {
    if max_acceleration * dt > (velocity - target).abs() {
        return target;
    }
    let next = if target > velocity {
        let (a0, a1) = if max_acceleration > acceleration {
            (acceleration, max_jerk.mul_add(dt, acceleration).min(max_acceleration))
        } else {
            (max_acceleration, max_acceleration)
        };
        let candidate = dt.mul_add((a0 + a1) * 0.5, velocity);
        if target < candidate {
            target
        } else {
            candidate
        }
    } else if target < velocity {
        let floor = -max_acceleration;
        let (a0, a1) = if floor < acceleration {
            (acceleration, (-max_jerk).mul_add(dt, acceleration).max(floor))
        } else {
            (floor, floor)
        };
        let candidate = dt.mul_add((a0 + a1) * 0.5, velocity);
        if target > candidate {
            target
        } else {
            candidate
        }
    } else {
        ((acceleration + acceleration) * 0.5).mul_add(dt, velocity)
    };
    next.clamp(-RAMP_OUTPUT_LIMIT, RAMP_OUTPUT_LIMIT)
}

/// `mapping_function` ccn@0x12a060: raw RC count -> [min, max], exactly 0 at the centre count.
pub fn map_rc_axis(raw: f32, min: f64, max: f64) -> f64 {
    let raw = f64::from(raw);
    if raw == RC_RAW_CENTRE {
        return 0.0;
    }
    (max - min).mul_add((raw - RC_RAW_MIN) / (RC_RAW_MAX - RC_RAW_MIN), min)
}

/// ccn's state: globals and the loop's locals carried across iterations.
#[derive(Clone, Debug, Serialize)]
pub struct ChassisController {
    pub geometry: SwerveGeometry,
    pub map_angle: MapAngleBranches,
    /// Target twist (vx, vy, wz) (ccn@0x1d33c0), unclamped.
    pub target: [f64; 3],
    pub acceleration: [f64; 3],
    /// Latest steer angles and wheel speeds from /hdas/feedback_chassis (ccn@0x1d3360).
    pub steer_feedback_rad: [f64; 3],
    pub wheel_speed_feedback_m_s: [f64; 3],
    pub brake: bool,
    /// `controller_control_mode` (ccn@0x1d3358): 1 = RC drives (start), 0 = software twists are accepted.
    pub rc_has_control: bool,
    loops_without_reset: u32,
    ramp_previous: [f64; 3],
    ramp_before_previous: [f64; 3],
    filtered_speed: [f64; 3],
    ik: SwerveIk,
}

/// What one step publishes: the MotorControl for HDAS and the FK speed for /motion_control/chassis_speed.
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct ChassisStep {
    pub target: ChassisTarget,
    pub measured_twist: Twist,
    pub timed_out: bool,
}

impl Default for ChassisController {
    fn default() -> ChassisController {
        ChassisController::new(VENDOR_GEOMETRY, VENDOR_MAP_ANGLE)
    }
}

impl ChassisController {
    pub fn new(geometry: SwerveGeometry, map_angle: MapAngleBranches) -> ChassisController {
        ChassisController {
            geometry,
            map_angle,
            target: [0.0; 3],
            acceleration: DEFAULT_ACCELERATION,
            steer_feedback_rad: [0.0; 3],
            wheel_speed_feedback_m_s: [0.0; 3],
            brake: false,
            rc_has_control: true,
            loops_without_reset: 0,
            ramp_previous: [0.0; 3],
            ramp_before_previous: [0.0; 3],
            filtered_speed: [0.0; 3],
            ik: SwerveIk::default(),
        }
    }

    /// /motion_target/target_speed_chassis (callback ccn@0x128484): taken only once RC handed over (mode 5).
    pub fn on_twist(&mut self, twist: Twist) {
        if self.rc_has_control {
            return;
        }
        self.target = [twist.vx_m_s, twist.vy_m_s, twist.wz_rad_s];
        self.loops_without_reset = 0;
    }

    /// /motion_target/brake_mode (callback ccn@0x128430): any message resets the timeout.
    pub fn on_brake(&mut self, brake: bool) {
        self.brake = brake;
        self.loops_without_reset = 0;
    }

    /// /motion_target/chassis_acc_limit (callback ccn@0x128720): |limit| capped per axis; does not reset the timeout.
    pub fn on_acceleration_limit(&mut self, limit: [f64; 3]) {
        self.acceleration = std::array::from_fn(|axis| limit[axis].abs().min(ACCELERATION_CAPS[axis]));
    }

    /// /controller (callback ccn@0x12a0a0): mode 2 drives from the sticks, mode 5 hands control to software.
    pub fn on_remote(&mut self, mode: u8, left_x: f32, left_y: f32, right_x: f32) {
        match mode {
            RC_MODE_DRIVE => {
                self.rc_has_control = true;
                self.loops_without_reset = 0;
                self.target =
                    [map_rc_axis(left_y, -1.5, 1.5), map_rc_axis(left_x, 1.0, -1.0), map_rc_axis(right_x, 1.0, -1.0)];
            }
            RC_MODE_SOFTWARE => self.rc_has_control = false,
            _ => {}
        }
    }

    /// /hdas/feedback_chassis (callback ccn@0x128d30): position[0..2] steer, velocity[0..2] wheel speed.
    pub fn on_feedback(&mut self, steer_rad: [f64; 3], wheel_speed_m_s: [f64; 3]) {
        self.steer_feedback_rad = steer_rad;
        self.wheel_speed_feedback_m_s = wheel_speed_m_s;
    }

    /// One iteration of the 20 ms loop in main ccn@0x124df0, in the vendor's order.
    pub fn step(&mut self) -> ChassisStep {
        let target = self.target;
        let mut ramped = [0.0; 3];
        for axis in 0..3 {
            let (previous, before) = (self.ramp_previous[axis], self.ramp_before_previous[axis]);
            let max_acceleration = self.acceleration[axis];
            ramped[axis] = if max_acceleration * LOOP_PERIOD_S < (target[axis] - previous).abs() {
                let acceleration = (previous - before) / LOOP_PERIOD_S;
                let max_jerk = max_acceleration * JERK_PER_ACCELERATION[axis];
                next_velocity(previous, acceleration, target[axis], max_acceleration, max_jerk, LOOP_PERIOD_S)
            } else {
                target[axis]
            };
        }
        let twist = Twist { vx_m_s: ramped[0], vy_m_s: ramped[1], wz_rad_s: ramped[2] };
        let steer = self.steer_feedback_rad;
        let (angles, speeds) = self.ik.solve(&self.geometry, self.map_angle, twist, &steer);
        let measured_twist = forward_kinematics(&self.geometry, &self.wheel_speed_feedback_m_s, &steer);
        let mut steer_out = angles.map(|angle| angle as f32);
        if (0..3).any(|wheel| (steer[wheel] - angles[wheel]).abs() > MISALIGNED_RAD) {
            let limit = if target[2].abs() > MISALIGNED_TURNING_WZ {
                MISALIGNED_TURNING_LINEAR_LIMIT
            } else {
                MISALIGNED_LINEAR_LIMIT
            };
            for axis in 0..2 {
                ramped[axis] = ramped[axis].clamp(-limit, limit);
                self.ramp_previous[axis] = self.ramp_previous[axis].clamp(-limit, limit);
            }
        }
        if self.brake {
            steer_out = BRAKE_STEER_RAD;
        }
        let mut speed_out = [0f32; 3];
        for wheel in 0..3 {
            let filtered = speeds[wheel].mul_add(LOW_PASS_NEW_WEIGHT, self.filtered_speed[wheel] * LOW_PASS_OLD_WEIGHT);
            self.filtered_speed[wheel] = filtered;
            speed_out[wheel] = filtered as f32;
        }
        if self.loops_without_reset < TIMEOUT_LOOPS {
            self.loops_without_reset += 1;
        }
        let timed_out = self.loops_without_reset == TIMEOUT_LOOPS;
        if timed_out {
            speed_out = [0.0; 3];
            steer_out = steer.map(|angle| angle as f32);
        }
        self.ramp_before_previous = self.ramp_previous;
        self.ramp_previous = ramped;
        ChassisStep {
            target: ChassisTarget {
                wheel_speed_m_s: speed_out,
                steer_angle_rad: steer_out,
                steer_rate_rad_s: [0.0; 3],
            },
            measured_twist,
            timed_out,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::encode::encode_chassis;

    #[test]
    fn parked_with_the_gatekeeper_sends_all_zero() {
        // the capture's 0x031: all zero at 50 Hz while parked with gatekeeper brake=false messages arriving
        let mut controller = ChassisController::default();
        controller.on_remote(RC_MODE_SOFTWARE, 1024.0, 1024.0, 1024.0);
        for _ in 0..200 {
            controller.on_brake(false);
            controller.on_feedback([-0.010, 0.007, -0.016], [0.0; 3]);
            let step = controller.step();
            assert_eq!(encode_chassis(&step.target).data, [0u8; 20]);
        }
    }

    #[test]
    fn timeout_holds_the_measured_steer() {
        let mut controller = ChassisController::default();
        controller.on_feedback([-0.010, 0.007, -0.016], [0.0; 3]);
        let steps: Vec<_> = (0..12).map(|_| controller.step()).collect();
        assert!(steps[..9].iter().all(|step| !step.timed_out) && steps[9].timed_out && steps[11].timed_out);
        // FFF6 0007 FFF0 in bytes 6..11, as the vendor would send it
        assert_eq!(&encode_chassis(&steps[11].target).data[6..12], &[0xFF, 0xF6, 0x00, 0x07, 0xFF, 0xF0]);
    }

    #[test]
    fn software_twist_ramps_and_is_ignored_until_mode_5() {
        let mut controller = ChassisController::default();
        controller.on_twist(Twist { vx_m_s: 1.0, ..Twist::default() });
        assert_eq!(controller.target, [0.0; 3]);
        controller.on_remote(RC_MODE_SOFTWARE, 1024.0, 1024.0, 1024.0);
        controller.on_twist(Twist { vx_m_s: 1.0, ..Twist::default() });
        let mut last = 0.0;
        for _ in 0..50 {
            controller.on_brake(false);
            let step = controller.step();
            let speed = step.target.wheel_speed_m_s[2];
            assert!(speed >= last && speed <= 1.0, "{speed}");
            last = speed;
        }
        assert!(last > 0.9, "{last}");
        assert_eq!(controller.step().target.steer_angle_rad, [0.0; 3]);
    }

    #[test]
    fn ramp_respects_acceleration_and_clamps() {
        assert_eq!(next_velocity(0.0, 0.0, 0.01, 2.5, 25.0, 0.02), 0.01);
        let v = next_velocity(0.0, 0.0, 1.0, 2.5, 25.0, 0.02);
        assert!((v - 0.005).abs() < 1e-12, "{v}");
        assert_eq!(next_velocity(1.49, 2.5, 3.0, 2.5, 25.0, 0.02), 1.5);
        assert_eq!(map_rc_axis(1024.0, -1.5, 1.5), 0.0);
        assert_eq!(map_rc_axis(1807.0, -1.5, 1.5), 1.5);
    }
}
