//! mobiman `r1_gripper_controller` (`rgc`) plus libmobiman's `Controller_Gripper` (`lmm`): what one gripper opening
//! command (0 closed .. 100 open) turns into on 0x051 / 0x061.
//!
//! Vendor structure: a 500 Hz control loop (rclcpp::Rate 2 ms, rgc@0x15aa10) runs [`GripperController::control`]
//! and hands `[p, v, kp, kd, tff]` to the OTG; each side's OTG thread (`reflexxesThreadFunc` lmm@0x1f7b30) steps a
//! Reflexxes Type II profile and publishes every pass (about 415 Hz, a 2 ms sleep on top of the work).

use serde::Serialize;

use crate::encode::GripperTarget;
use crate::gripper_geometry::InverseLut;

/// R1PRO gains from gripper_controller_kp_kd.toml (read in main rgc@0x159494): kp 20, kd 10.
pub const VENDOR_KP: f64 = 20.0;
pub const VENDOR_KD: f64 = 10.0;
/// ... `tff_additional` 2.0 (the FORCE-mode torque).
pub const VENDOR_TFF_ADDITIONAL: f64 = 2.0;
/// ... `cutting_ratio` 1.0.
pub const VENDOR_CUTTING_RATIO: f64 = 1.0;
/// ... OTG limits max_vel 30, max_acc 60 (jerk 60 is ignored by Type II).
pub const VENDOR_OTG_MAX_VELOCITY: f64 = 30.0;
pub const VENDOR_OTG_MAX_ACCELERATION: f64 = 60.0;
/// OTG cycle time (lmm@0x241470).
pub const OTG_CYCLE_S: f64 = 0.002;
/// R1PRO grasp force threshold (GripperState init in main rgc@0x159494; 0.5 on R1LITE).
pub const VENDOR_FORCE_THRESHOLD: f64 = 0.1;
/// GripperState `max_grasping_force` (main rgc@0x159494).
pub const VENDOR_MAX_GRASPING_FORCE: f64 = 4.0;
/// Added to the measured effort: left 0.1 (rgc@0x1de6d0, left_gripper_feedback_CB rgc@0x15c2e0), right 0.5
/// (immediate in right_gripper_feedback_CB rgc@0x15c250).
pub const VENDOR_EFFORT_OFFSET: [f64; 2] = [0.1, 0.5];
/// Startup: open to these percentages in turn (performInitialization lambda rgc@0x15e794 calls) ...
pub const VENDOR_STARTUP_OPENINGS: [f64; 4] = [50.0, 0.0, 50.0, 100.0];
/// ... holding each 2.0 s (fmov rgc@0x15e9fc); afterwards the target is forced to 100 (open).
pub const VENDOR_STARTUP_HOLD_S: f64 = 2.0;
/// The opening the controller holds once startup ends.
pub const VENDOR_OPENING_AFTER_STARTUP: f64 = 100.0;
/// performInitialization's "[Init] Collecting baseline forces..." loop (rgc@0x15a838): updateForceBaseline on the
/// latest effort of each side (rgc@0x15abf8 / 0x15ac34) for 1.0 s ...
pub const VENDOR_BASELINE_GATHER_S: f64 = 1.0;
/// ... once per 5 ms (sleep_for 5000000 ns, rgc@0x15accc); no grasp detection and no control meanwhile.
pub const VENDOR_BASELINE_GATHER_PERIOD_S: f64 = 0.005;

/// Force baseline: a window of the last 100 efforts, used once it holds 50 (updateForceBaseline rgc@0x15cec0) ...
const BASELINE_WINDOW: usize = 100;
const BASELINE_MIN_SAMPLES: usize = 50;
/// ... blended 0.001 new / 0.999 old (rgc@0x1de6d8 / 0x1de6e0).
const BASELINE_NEW_WEIGHT: f64 = 0.001;
const BASELINE_OLD_WEIGHT: f64 = 0.999;
/// Adaptive threshold EMA 0.01 / 0.99 (updateAdaptiveForceThreshold rgc@0x15d220, rgc@0x1de6e8 / 0x1de6f0).
const ADAPTIVE_NEW_WEIGHT: f64 = 0.01;
const ADAPTIVE_OLD_WEIGHT: f64 = 0.99;
/// detectGrasping rgc@0x15d2e0: |velocity| < 1.0, position > 5.0, closing = target + 0.5 < position.
const GRASP_MAX_SPEED: f64 = 1.0;
const GRASP_MIN_POSITION: f64 = 5.0;
const CLOSING_MARGIN: f64 = 0.5;
/// FORCE -> POSITION once target - position > 5 (updateGripperControl rgc@0x15df20).
const RELEASE_MARGIN: f64 = 5.0;
/// A new target more than 1.0 above the position drops FORCE back to POSITION (LeftGripperTargetPositionCB rgc@0x15eb80).
const REOPEN_MARGIN: f64 = 1.0;
/// HYBRID halves the gains (updateGripperControl rgc@0x15df20).
const HYBRID_GAIN_SCALE: f64 = 0.5;

/// `GripperControlMode` (DWARF enum: POSITION_MODE 0, FORCE_MODE 1, HYBRID_MODE 2).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum GripperMode {
    Position,
    Force,
    Hybrid,
}

/// rgc's per-side configuration (toml + main).
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct GripperConfig {
    pub kp: f64,
    pub kd: f64,
    pub tff_additional: f64,
    pub cutting_ratio: f64,
    pub force_threshold: f64,
    pub max_grasping_force: f64,
    pub effort_offset: f64,
    pub otg_max_velocity: f64,
    pub otg_max_acceleration: f64,
}

impl GripperConfig {
    /// R1PRO values for side 0 = left, 1 = right.
    pub fn vendor(side_index: usize) -> GripperConfig {
        GripperConfig {
            kp: VENDOR_KP,
            kd: VENDOR_KD,
            tff_additional: VENDOR_TFF_ADDITIONAL,
            cutting_ratio: VENDOR_CUTTING_RATIO,
            force_threshold: VENDOR_FORCE_THRESHOLD,
            max_grasping_force: VENDOR_MAX_GRASPING_FORCE,
            effort_offset: VENDOR_EFFORT_OFFSET[side_index],
            otg_max_velocity: VENDOR_OTG_MAX_VELOCITY,
            otg_max_acceleration: VENDOR_OTG_MAX_ACCELERATION,
        }
    }
}

/// One-axis online trajectory: time-optimal trapezoid toward a resting target, stepped from its own state.
/// Reflexxes Type II itself is not ported; at rest and in steady motion this gives the same targets.
#[derive(Clone, Copy, Debug, PartialEq, Default, Serialize)]
pub struct TrapezoidOtg {
    pub position: f64,
    pub velocity: f64,
}

impl TrapezoidOtg {
    pub fn step(&mut self, target: f64, max_velocity: f64, max_acceleration: f64, dt: f64) -> (f64, f64) {
        let error = target - self.position;
        let toward = if error < 0.0 { -1.0 } else { 1.0 };
        let (distance, speed) = (error.abs(), self.velocity * toward);
        // the fastest next speed that can still stop on the target after this step
        let half_step = max_acceleration * dt / 2.0;
        let stoppable = -half_step
            + (half_step * half_step + 2.0 * max_acceleration * (distance - speed * dt / 2.0).max(0.0)).sqrt();
        let next_speed =
            stoppable.min(max_velocity).min(speed + max_acceleration * dt).max(speed - max_acceleration * dt).max(0.0);
        let advance = 0.5 * (speed + next_speed) * dt;
        if distance <= advance || next_speed == 0.0 {
            *self = TrapezoidOtg { position: target, velocity: 0.0 };
        } else {
            *self = TrapezoidOtg { position: self.position + advance * toward, velocity: next_speed * toward };
        }
        (self.position, self.velocity)
    }
}

/// One side's rgc state (`GripperState`) plus its OTG.
#[derive(Clone, Debug, Serialize)]
pub struct GripperController {
    pub config: GripperConfig,
    pub mode: GripperMode,
    pub target_opening: f64,
    pub is_grasping: bool,
    pub baseline_force: f64,
    mean_compensated_force: f64,
    mean_compensated_force_sq: f64,
    adaptive_force_threshold: f64,
    #[serde(skip)]
    force_history: std::collections::VecDeque<f64>,
    /// `[p, v, kp, kd, tff]` handed to the OTG (v is never written by rgc).
    pub otg_target: [f64; 5],
    pub otg: TrapezoidOtg,
    #[serde(skip)]
    lut: InverseLut,
}

impl GripperController {
    pub fn new(config: GripperConfig, lut: InverseLut) -> GripperController {
        GripperController {
            config,
            mode: GripperMode::Position,
            target_opening: VENDOR_OPENING_AFTER_STARTUP,
            is_grasping: false,
            baseline_force: 0.0,
            mean_compensated_force: 0.0,
            mean_compensated_force_sq: 0.0,
            adaptive_force_threshold: 0.0,
            force_history: Default::default(),
            otg_target: [0.0, 0.0, config.kp, config.kd, 0.0],
            otg: TrapezoidOtg::default(),
            lut,
        }
    }

    pub fn lut(&self) -> &InverseLut {
        &self.lut
    }

    /// `/motion_target/target_position_gripper_*` (Left/RightGripperTargetPositionCB rgc@0x15eb80 / 0x15ecf4).
    pub fn set_opening(&mut self, opening: f64, measured_position: f64) {
        self.target_opening = opening.clamp(0.0, 100.0);
        if self.mode == GripperMode::Force && measured_position + REOPEN_MARGIN < self.target_opening {
            self.mode = GripperMode::Position;
            self.is_grasping = false;
        }
    }

    /// performInitialization's baseline gather (rgc@0x15a838): updateForceBaseline alone on the offset effort.
    pub fn gather_baseline(&mut self, raw_effort: f64) {
        self.update_baseline(raw_effort + self.config.effort_offset);
    }

    /// The startup sweep's buildDirectTarget (rgc@0x15e6e0) handed straight to moveJOTG (rgc@0x15e840): position
    /// gains, zero torque, no control pass.
    pub fn direct_target(&mut self, opening: f64) {
        let motor = self.lut.opening_to_motor_rad(opening.clamp(0.0, 100.0));
        self.otg_target = [motor, 0.0, self.config.kp, self.config.kd, 0.0];
    }

    /// The baseline a long-running rgc has converged to at this effort (0.001 blend per 2 ms pass, about 2 s): for
    /// joining a vendor that is already running, where no startup is replayed.
    pub fn settle_baseline(&mut self, raw_effort: f64) {
        self.baseline_force = raw_effort + self.config.effort_offset;
    }

    /// updateForceBaseline rgc@0x15cec0.
    fn update_baseline(&mut self, effort: f64) {
        self.force_history.push_back(effort);
        if self.force_history.len() > BASELINE_WINDOW {
            self.force_history.pop_front();
        }
        if !self.is_grasping && self.force_history.len() >= BASELINE_MIN_SAMPLES {
            let mean = self.force_history.iter().sum::<f64>() / self.force_history.len() as f64;
            self.baseline_force = mean * BASELINE_NEW_WEIGHT + BASELINE_OLD_WEIGHT * self.baseline_force;
        }
    }

    /// detectGrasping rgc@0x15d2e0 with updateAdaptiveForceThreshold rgc@0x15d220 inside it.
    fn detect_grasping(&mut self, position: f64, velocity: f64, effort: f64, closing: bool) -> bool {
        let compensated = effort - self.baseline_force;
        if !self.is_grasping {
            let mean = compensated * ADAPTIVE_NEW_WEIGHT + ADAPTIVE_OLD_WEIGHT * self.mean_compensated_force;
            let mean_sq =
                compensated * compensated * ADAPTIVE_NEW_WEIGHT + ADAPTIVE_OLD_WEIGHT * self.mean_compensated_force_sq;
            self.mean_compensated_force = mean;
            self.mean_compensated_force_sq = mean_sq;
            let deviation = (mean_sq - mean * mean).max(0.0).sqrt();
            self.adaptive_force_threshold = (mean.abs() + deviation).max(self.config.force_threshold * 0.5);
        }
        let threshold = self.adaptive_force_threshold.max(self.config.force_threshold);
        threshold * self.config.cutting_ratio < compensated.abs()
            && velocity.abs() < GRASP_MAX_SPEED
            && closing
            && GRASP_MIN_POSITION < position
    }

    /// One 500 Hz control pass: updateGripperControl rgc@0x15df20 on the latest feedback (opening position, its
    /// velocity, raw effort); updates `otg_target`.
    pub fn control(&mut self, position: f64, velocity: f64, raw_effort: f64) {
        let effort = raw_effort + self.config.effort_offset;
        self.update_baseline(effort);
        let stroke = self.target_opening;
        let commanded = self.lut.opening_to_motor_rad(stroke);
        let closing = stroke + CLOSING_MARGIN < position;
        let grasping = self.detect_grasping(position, velocity, effort, closing);
        self.is_grasping = grasping;
        let config = self.config;
        match self.mode {
            GripperMode::Force => {
                let force = config.tff_additional.min(config.max_grasping_force);
                self.otg_target = [self.lut.opening_to_motor_rad(position), self.otg_target[1], 0.0, 0.0, force];
                if RELEASE_MARGIN < stroke - position {
                    self.mode = GripperMode::Position;
                    self.is_grasping = false;
                }
            }
            GripperMode::Hybrid if grasping => {
                let force = config.tff_additional.min(config.max_grasping_force);
                self.otg_target = [self.lut.opening_to_motor_rad(position), self.otg_target[1], 0.0, 0.0, force];
                self.mode = GripperMode::Force;
            }
            GripperMode::Hybrid => {
                let (kp, kd) = (config.kp * HYBRID_GAIN_SCALE, config.kd * HYBRID_GAIN_SCALE);
                self.otg_target = [commanded, self.otg_target[1], kp, kd, 0.0];
            }
            GripperMode::Position => {
                self.otg_target = [commanded, self.otg_target[1], config.kp, config.kd, 0.0];
                if grasping && closing {
                    self.mode = GripperMode::Hybrid;
                }
            }
        }
    }

    /// One OTG pass (reflexxesThreadFunc lmm@0x1f7b30): p_des / v_des from the profile, gains and torque from the
    /// target, all cast to f32 as the MotorControl carries them.
    pub fn otg_step(&mut self) -> GripperTarget {
        let config = self.config;
        let (position, velocity) =
            self.otg.step(self.otg_target[0], config.otg_max_velocity, config.otg_max_acceleration, OTG_CYCLE_S);
        GripperTarget {
            position_rad: position as f32,
            velocity: velocity as f32,
            kp: self.otg_target[2] as f32,
            kd: self.otg_target[3] as f32,
            torque: self.otg_target[4] as f32,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arm::Side;
    use crate::encode::encode_gripper;

    fn settled(opening: f64, side_index: usize) -> GripperTarget {
        let mut controller = GripperController::new(GripperConfig::vendor(side_index), InverseLut::vendor());
        controller.set_opening(opening, 0.0);
        let mut last = GripperTarget::default();
        for _ in 0..2000 {
            controller.control(opening, 0.0, 0.0);
            last = controller.otg_step();
        }
        last
    }

    #[test]
    fn closed_and_open_settle_to_the_captured_bytes() {
        // the capture's constant 0x051 (left closed) and 0x061 (right open)
        assert_eq!(encode_gripper(Side::Left, &settled(0.0, 0)).data, [0, 0, 0, 0, 0x04, 0xB0, 0x05, 0xDC, 0, 0]);
        assert_eq!(
            encode_gripper(Side::Right, &settled(100.0, 1)).data,
            [0xCC, 0x98, 0, 0, 0x04, 0xB0, 0x05, 0xDC, 0, 0]
        );
    }

    #[test]
    fn otg_respects_its_limits() {
        let mut otg = TrapezoidOtg::default();
        let mut steps = 0;
        while otg.position != -2.8 {
            let before = otg.velocity;
            otg.step(-2.8, 30.0, 60.0, OTG_CYCLE_S);
            // every step but the final snap onto the target keeps both limits
            let landed = otg.position == -2.8;
            assert!(
                landed || (otg.velocity.abs() <= 30.0 && (otg.velocity - before).abs() <= 60.0 * OTG_CYCLE_S + 1e-12)
            );
            steps += 1;
            assert!(steps < 1000);
        }
        // 2.8 rad at 60 rad/s^2 without reaching 30 rad/s: 2 sqrt(2.8 / 60) = 0.43 s
        assert!((steps as f64 * OTG_CYCLE_S - 0.432).abs() < 0.02, "{steps}");
    }

    #[test]
    fn a_blocked_close_goes_hybrid_then_force() {
        let mut controller = GripperController::new(GripperConfig::vendor(0), InverseLut::vendor());
        for _ in 0..200 {
            controller.control(60.0, 0.0, 0.0);
        }
        controller.set_opening(0.0, 60.0);
        controller.control(60.0, 0.0, 3.0);
        assert_eq!(controller.mode, GripperMode::Hybrid);
        controller.control(60.0, 0.0, 3.0);
        assert_eq!(controller.mode, GripperMode::Force);
        assert_eq!(controller.otg_target[2..], [0.0, 0.0, 2.0]);
        controller.set_opening(100.0, 60.0);
        assert_eq!(controller.mode, GripperMode::Position);
    }
}
