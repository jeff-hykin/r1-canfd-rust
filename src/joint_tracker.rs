//! mobiman `r1_pro_jointTracker_demo_node` (`jt`): what dimos's arm and torso joint targets turn into on 0x050 /
//! 0x060 / 0x041. jt plans every move with Reflexxes Type II from its own state (feedback only seeds it) and adds
//! Pinocchio gravity torque on the arms; without that feed-forward the arms sag at kp 2.
//!
//! Vendor structure (LOGIC_VS_VENDOR.md, "Joint tracker"):
//! - arm planner (`Controller_A2::reflexxesThreadFunc` jt@0x2241f0): 4 ms, publishes while a move is pending.
//! - target loop (lambda jt@0x186ba0): 5 ms, re-applies each arm's target (so a holding arm publishes about every
//!   5 ms) and hands the torso planner's position to the arms' gravity model.
//! - torso planner (`Controller::reflexxesThreadFunc` jt@0x2339a0): 5 ms, publishes only while moving.

use crate::arm::{ArmFeedback, Side, ARM_JOINTS};
use crate::commander::{ArmCommander, TorsoCommander};
use crate::encode::OutgoingFrame;
use crate::gravity::{GravityModel, UrdfError};
use crate::otg::SyncOtg;
use crate::torso::{TorsoFeedback, TORSO_JOINTS};

/// Arm planner period and Reflexxes cycle (`Duration(0, 4000000)` in reflexxesThreadFunc, cycle 0.004 jt@0x249a28).
pub const ARM_CYCLE_S: f64 = 0.004;
/// Target loop period (0x4c4b40 ns, jt@0x187d2c).
pub const TARGET_LOOP_S: f64 = 0.005;
/// Torso planner period and Reflexxes cycle (0.005, jt@0x24a8f0).
pub const TORSO_CYCLE_S: f64 = 0.005;

/// joint_controller_pid_kp_kd.toml, normal mode: `R1_Pro_A2_max_vel` / `_max_acc` (jerk is ignored by Type II).
pub const VENDOR_ARM_MAX_VELOCITY: f64 = 5.0;
pub const VENDOR_ARM_MAX_ACCELERATION: f64 = 20.0;
/// `R1_Pro_A2_{left,right}_kp` / `_kd`.
pub const VENDOR_ARM_KP: f64 = 2.0;
pub const VENDOR_ARM_KD: f64 = 25.0;
/// v_des of every arm joint, whatever the target (0x3ecccccd in reflexxesThreadFunc jt@0x2241f0).
pub const VENDOR_ARM_V_DES: f32 = 0.4;
/// A target's velocity field caps the speed, up to `max_vec_limit_arm` (jt@0x248658) ...
pub const VENDOR_ARM_SPEED_CAP: [f64; ARM_JOINTS] = [3.0, 3.0, 3.0, 5.0, 5.0, 5.0, 5.0];
/// ... and the acceleration follows as 1.5 x that speed (arm target callbacks jt@0x17e1c0 / 0x17e984).
pub const VENDOR_ACCELERATION_PER_SPEED: f64 = 1.5;
/// setRatio(1.0) in main: speeds are divided by it, limits multiplied.
pub const VENDOR_RATIO: f64 = 1.0;
/// Per-joint (min, max) the target is clamped to (minmax, limits jt@0x248520..0x248578; right j2 jt@0x248580).
#[allow(clippy::approx_constant)] // the vendor's literals, deliberately not exact pi fractions
pub const VENDOR_LEFT_ARM_LIMITS: [(f64, f64); ARM_JOINTS] = [
    (-4.4506, 1.309),
    (-0.1745, 3.1416),
    (-2.356196, 2.356196),
    (-2.0944, 0.3491),
    (-2.356196, 2.356196),
    (-1.047198, 1.047198),
    (-1.5708, 1.5708),
];
#[allow(clippy::approx_constant)]
pub const VENDOR_RIGHT_ARM_J2_LIMITS: (f64, f64) = (-3.1416, 0.1745);
/// j7_soft_constraints jt@0x17d5b0: with j6 <= 0, j7 is held inside 0.8 x the polynomials in j6 (jt@0x2484f8) ...
pub const J7_SOFT_SCALE: f64 = 0.8;
/// ... upper (jt@0x2485b0) and lower (jt@0x2485e0) coefficients, x^0..x^5.
pub const J7_UPPER: [f64; 6] = [4.260748, 17.752459, 43.403286, 58.502658, 40.209266, 11.012791];
pub const J7_LOWER: [f64; 6] = [-2.2953, -3.9766, -3.333, 0.605, 2.5598, 1.0522];

/// Torso planner limits before any target (Controller ctor jt@0x236010, 0x1486e8 / 0x148690).
pub const VENDOR_TORSO_MAX_VELOCITY: f64 = 1.0;
pub const VENDOR_TORSO_MAX_ACCELERATION: f64 = 1.5;
/// A torso target's velocity field caps the speed up to `max_vec_limit_torso` (jt@0x248690).
pub const VENDOR_TORSO_SPEED_CAP: f64 = 1.5;
/// v_des the torso always sends (is_r1pro, Controller::reflexxesThreadFunc jt@0x2339a0).
pub const VENDOR_TORSO_V_DES: [f32; TORSO_JOINTS] = [0.6, 1.0, 0.6, 1.0];
/// The planner's joint-3 output is clamped to +-2.95 (jt@0x24a8d0).
pub const VENDOR_TORSO_J3_CLAMP: f64 = 2.95;
/// checkBoundingBox jt@0x22fab0 on torso_link4 in base_link (R1 Pro, jt@0x24a870..): x, z, pitch, yaw; y unchecked.
pub const VENDOR_TORSO_BOX: TorsoBox =
    TorsoBox { x: (-0.1, 0.25), z: (0.551, 1.149), pitch: (-1.05, 1.05), yaw: (-1.76, 1.76) };
/// The link the bounding box is checked on.
pub const TORSO_BOX_LINK: &str = "torso_link4";

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TorsoBox {
    pub x: (f64, f64),
    pub z: (f64, f64),
    pub pitch: (f64, f64),
    pub yaw: (f64, f64),
}

impl TorsoBox {
    pub fn contains(&self, position: [f64; 3], rpy: [f64; 3]) -> bool {
        let inside = |(low, high): (f64, f64), value: f64| low <= value && value <= high;
        inside(self.x, position[0])
            && inside(self.z, position[2])
            && inside(self.pitch, rpy[1])
            && inside(self.yaw, rpy[2])
    }
}

/// Why a target was refused (the vendor would read out of bounds or hand Reflexxes invalid limits).
#[derive(Clone, Debug, PartialEq)]
pub enum TargetError {
    Length { expected: usize, got: usize },
    NotFinite,
}

impl std::fmt::Display for TargetError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TargetError::Length { expected, got } => write!(formatter, "expected {expected} positions, got {got}"),
            TargetError::NotFinite => write!(formatter, "non-finite position or a zero/non-finite speed cap"),
        }
    }
}

/// Everything the trackers take from the vendor's config and URDF.
#[derive(Clone, Debug)]
pub struct TrackerSettings {
    pub arm_max_velocity: f64,
    pub arm_max_acceleration: f64,
    pub arm_kp: f64,
    pub arm_kd: f64,
    /// ocs2 MobileManipulatorInterface models (root base_link, everything but torso + that arm locked at 0).
    pub left_gravity: Option<GravityModel>,
    pub right_gravity: Option<GravityModel>,
    /// torso-only model for the bounding-box FK; `None` skips the box (as a matching `torso_key` does).
    pub torso_kinematics: Option<GravityModel>,
}

fn joint_names(side: Option<Side>) -> Vec<String> {
    let mut names: Vec<String> = (1..=TORSO_JOINTS).map(|i| format!("torso_joint{i}")).collect();
    if let Some(side) = side {
        names.extend((1..=ARM_JOINTS).map(|i| format!("{}_arm_joint{i}", side.name())));
    }
    names
}

fn model(urdf: &str, side: Option<Side>) -> Result<GravityModel, UrdfError> {
    let names = joint_names(side);
    GravityModel::from_urdf(urdf, &names.iter().map(String::as_str).collect::<Vec<_>>())
}

impl TrackerSettings {
    /// The robot's launch (fast_mode false, torso_key empty) with the gravity and FK models from `urdf`, Galaxea's
    /// r1_pro.urdf; `None` runs without feed-forward or bounding box.
    pub fn vendor(urdf: Option<&str>) -> Result<TrackerSettings, UrdfError> {
        Ok(TrackerSettings {
            arm_max_velocity: VENDOR_ARM_MAX_VELOCITY,
            arm_max_acceleration: VENDOR_ARM_MAX_ACCELERATION,
            arm_kp: VENDOR_ARM_KP,
            arm_kd: VENDOR_ARM_KD,
            left_gravity: urdf.map(|urdf| model(urdf, Some(Side::Left))).transpose()?,
            right_gravity: urdf.map(|urdf| model(urdf, Some(Side::Right))).transpose()?,
            torso_kinematics: urdf.map(|urdf| model(urdf, None)).transpose()?,
        })
    }
}

/// The loop periods a caller runs the trackers at.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Periods {
    pub arm: f64,
    pub target_loop: f64,
    pub torso: f64,
}

pub const PERIODS: Periods = Periods { arm: ARM_CYCLE_S, target_loop: TARGET_LOOP_S, torso: TORSO_CYCLE_S };

fn polynomial(coefficients: &[f64; 6], x: f64) -> f64 {
    coefficients.iter().rev().fold(0.0, |sum, coefficient| sum * x + coefficient)
}

/// One arm: `Controller_A2` plus its target callback.
#[derive(Clone, Debug)]
pub struct ArmTracker {
    pub side: Side,
    commander: ArmCommander,
    gravity: Option<GravityModel>,
    kp: f64,
    kd: f64,
    limits: [(f64, f64); ARM_JOINTS],
    default_velocity: f64,
    default_acceleration: f64,
    /// Planner state; `None` until the first feedback seeds it (enable() blocks on it, StartOTG then unsubscribes).
    pub otg: Option<SyncOtg<ARM_JOINTS>>,
    /// The last accepted target (zeros before any, like the vendor's global).
    pub target: [f64; ARM_JOINTS],
    pub max_velocity: [f64; ARM_JOINTS],
    pub max_acceleration: [f64; ARM_JOINTS],
    /// `waiting_for_first_command_*`: nothing is published until a target arrives.
    pub waiting_for_first_command: bool,
    pub target_reached: bool,
    /// The torso planner's position, handed over by the target loop (set_torso_state).
    pub torso_position: [f64; TORSO_JOINTS],
}

impl ArmTracker {
    pub fn new(side: Side, settings: &TrackerSettings) -> ArmTracker {
        let mut limits = VENDOR_LEFT_ARM_LIMITS;
        if side == Side::Right {
            limits[1] = VENDOR_RIGHT_ARM_J2_LIMITS;
        }
        ArmTracker {
            side,
            commander: ArmCommander::new(side),
            gravity: if side == Side::Left { settings.left_gravity.clone() } else { settings.right_gravity.clone() },
            kp: settings.arm_kp,
            kd: settings.arm_kd,
            limits,
            default_velocity: settings.arm_max_velocity,
            default_acceleration: settings.arm_max_acceleration,
            otg: None,
            target: [0.0; ARM_JOINTS],
            max_velocity: [settings.arm_max_velocity; ARM_JOINTS],
            max_acceleration: [settings.arm_max_acceleration; ARM_JOINTS],
            waiting_for_first_command: true,
            target_reached: true,
            torso_position: [0.0; TORSO_JOINTS],
        }
    }

    /// jointStateCallback jt@0x21e140: only the first feedback matters (it seeds the planner).
    pub fn on_arm_feedback(&mut self, feedback: &ArmFeedback) {
        if self.otg.is_none() {
            self.otg = Some(SyncOtg::at_rest(feedback.position_rad));
        }
    }

    /// `/motion_target/target_joint_state_arm_*` (jt@0x17e1c0 / 0x17e984): clamp, j7 soft constraint, speed cap.
    pub fn on_target(&mut self, position: &[f64], velocity: &[f64]) -> Result<(), TargetError> {
        let position: [f64; ARM_JOINTS] =
            position.try_into().map_err(|_| TargetError::Length { expected: ARM_JOINTS, got: position.len() })?;
        if position.iter().any(|value| !value.is_finite()) {
            return Err(TargetError::NotFinite);
        }
        let mut target: [f64; ARM_JOINTS] = std::array::from_fn(|joint| {
            let (low, high) = self.limits[joint];
            position[joint].max(low).min(high)
        });
        let j6 = target[5];
        if j6 <= 0.0 {
            let (upper, lower) = (J7_SOFT_SCALE * polynomial(&J7_UPPER, j6), J7_SOFT_SCALE * polynomial(&J7_LOWER, j6));
            let (low, high) = if self.side == Side::Right { (lower, upper) } else { (-upper, -lower) };
            target[6] = target[6].max(low).min(high);
        }
        let (max_velocity, max_acceleration) = if velocity.len() == ARM_JOINTS {
            let speed: [f64; ARM_JOINTS] =
                std::array::from_fn(|joint| (velocity[joint].abs() / VENDOR_RATIO).min(VENDOR_ARM_SPEED_CAP[joint]));
            if speed.iter().any(|value| !(value.is_finite() && *value > 0.0)) {
                return Err(TargetError::NotFinite);
            }
            (speed, speed.map(|value| value * VENDOR_ACCELERATION_PER_SPEED))
        } else {
            ([self.default_velocity; ARM_JOINTS], [self.default_acceleration; ARM_JOINTS])
        };
        self.target = target;
        self.max_velocity = max_velocity.map(|value| value * VENDOR_RATIO);
        self.max_acceleration = max_acceleration.map(|value| value * VENDOR_RATIO);
        self.waiting_for_first_command = false;
        Ok(())
    }

    /// The 5 ms target loop's part for this arm (jt@0x186ba0): moveJOTG(target) and set_torso_state.
    pub fn apply_target(&mut self, torso_position: [f64; TORSO_JOINTS]) {
        self.torso_position = torso_position;
        if !self.waiting_for_first_command {
            self.target_reached = false;
        }
    }

    /// One 4 ms planner pass: the 0x050 / 0x060 frame while a move or re-applied target is pending.
    pub fn tick(&mut self) -> Option<OutgoingFrame> {
        if self.target_reached {
            return None;
        }
        let otg = self.otg.as_mut()?;
        otg.step(&self.target, &self.max_velocity, &self.max_acceleration, ARM_CYCLE_S);
        let position = otg.position;
        if otg.reached(&self.target) {
            self.target_reached = true;
        }
        let torque: [f32; ARM_JOINTS] = match &self.gravity {
            Some(model) => {
                let q: Vec<f64> = self.torso_position.iter().chain(position.iter()).copied().collect();
                let g = model.torques(&q);
                std::array::from_fn(|joint| g[TORSO_JOINTS + joint] as f32)
            }
            None => [0.0; ARM_JOINTS],
        };
        self.commander
            .set_joint_targets(
                &position.map(|value| value as f32),
                &[VENDOR_ARM_V_DES; ARM_JOINTS],
                &[self.kp as f32; ARM_JOINTS],
                &[self.kd as f32; ARM_JOINTS],
                &torque,
            )
            .ok()
    }
}

/// The torso: `Controller` (dof 4) plus its target callback.
#[derive(Clone, Debug)]
pub struct TorsoTracker {
    commander: TorsoCommander,
    kinematics: Option<GravityModel>,
    pub otg: Option<SyncOtg<TORSO_JOINTS>>,
    pub target: [f64; TORSO_JOINTS],
    pub max_velocity: [f64; TORSO_JOINTS],
    pub max_acceleration: [f64; TORSO_JOINTS],
    pub target_reached: bool,
    /// The last position sent (`send_joint`); frozen while the bounding box is violated.
    pub sent: [f64; TORSO_JOINTS],
}

impl TorsoTracker {
    pub fn new(settings: &TrackerSettings) -> TorsoTracker {
        TorsoTracker {
            commander: TorsoCommander::default(),
            kinematics: settings.torso_kinematics.clone(),
            otg: None,
            target: [0.0; TORSO_JOINTS],
            max_velocity: [VENDOR_TORSO_MAX_VELOCITY; TORSO_JOINTS],
            max_acceleration: [VENDOR_TORSO_MAX_ACCELERATION; TORSO_JOINTS],
            target_reached: true,
            sent: [0.0; TORSO_JOINTS],
        }
    }

    /// Controller::jointStateCallback jt@0x235c60: the first feedback seeds the planner.
    pub fn on_torso_feedback(&mut self, feedback: &TorsoFeedback) {
        if self.otg.is_none() {
            self.otg = Some(SyncOtg::at_rest(feedback.position_rad));
            self.sent = feedback.position_rad;
        }
    }

    /// The planner's position (`last_pos`), which the arms' gravity model uses; zeros before feedback.
    pub fn position(&self) -> [f64; TORSO_JOINTS] {
        self.otg.as_ref().map(|otg| otg.position).unwrap_or([0.0; TORSO_JOINTS])
    }

    /// `/motion_target/target_joint_state_torso` (torsotargetPositionCallback jt@0x17f150): no clamp; one moveJOTG.
    pub fn on_target(&mut self, position: &[f64], velocity: &[f64]) -> Result<(), TargetError> {
        let position: [f64; TORSO_JOINTS] =
            position.try_into().map_err(|_| TargetError::Length { expected: TORSO_JOINTS, got: position.len() })?;
        if position.iter().any(|value| !value.is_finite()) {
            return Err(TargetError::NotFinite);
        }
        if velocity.len() == TORSO_JOINTS {
            let speed: [f64; TORSO_JOINTS] =
                std::array::from_fn(|joint| velocity[joint].abs().min(VENDOR_TORSO_SPEED_CAP));
            if speed.iter().any(|value| !(value.is_finite() && *value > 0.0)) {
                return Err(TargetError::NotFinite);
            }
            self.max_velocity = speed;
            self.max_acceleration = speed.map(|value| value * VENDOR_ACCELERATION_PER_SPEED);
        }
        self.target = position;
        self.target_reached = false;
        Ok(())
    }

    /// One 5 ms planner pass: the 0x041 frame while moving.
    pub fn tick(&mut self) -> Option<OutgoingFrame> {
        if self.target_reached {
            return None;
        }
        let otg = self.otg.as_mut()?;
        let last = otg.clone();
        otg.step(&self.target, &self.max_velocity, &self.max_acceleration, TORSO_CYCLE_S);
        otg.position[2] = otg.position[2].clamp(-VENDOR_TORSO_J3_CLAMP, VENDOR_TORSO_J3_CLAMP);
        let reached = otg.reached(&self.target);
        let inside = match &self.kinematics {
            Some(model) => model
                .link_pose(&otg.position, TORSO_BOX_LINK)
                .is_some_and(|(position, rpy)| VENDOR_TORSO_BOX.contains(position, rpy)),
            None => true,
        };
        if inside {
            self.sent = otg.position;
        } else {
            // "EE pose out of bounding box, freeze at last_pos"
            *otg = SyncOtg::at_rest(last.position);
        }
        if reached {
            self.target_reached = true;
        }
        let velocity = VENDOR_TORSO_V_DES;
        Some(self.commander.set(&self.sent.map(|value| value as f32), &velocity))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arm::ArmFeedback;

    fn feedback(side: Side, position: [f64; ARM_JOINTS]) -> ArmFeedback {
        ArmFeedback {
            side,
            position_rad: position,
            velocity_rad_s: [0.0; ARM_JOINTS],
            torque_nm: [0.0; ARM_JOINTS],
            unread_tail: vec![],
        }
    }

    #[test]
    fn nothing_before_feedback_and_a_first_target() {
        let mut arm = ArmTracker::new(Side::Left, &TrackerSettings::vendor(None).unwrap());
        arm.apply_target([0.0; 4]);
        assert!(arm.tick().is_none());
        arm.on_arm_feedback(&feedback(Side::Left, [0.1; ARM_JOINTS]));
        arm.apply_target([0.0; 4]);
        assert!(arm.tick().is_none(), "waiting for the first command");
        assert!(arm.on_target(&[0.0; 6], &[]).is_err());
    }

    #[test]
    fn move_then_hold_at_the_dimos_tracking_speed() {
        let mut arm = ArmTracker::new(Side::Left, &TrackerSettings::vendor(None).unwrap());
        arm.on_arm_feedback(&feedback(Side::Left, [0.0; ARM_JOINTS]));
        let target = [0.0, 0.5, 0.0, -0.5, 0.0, 0.0, 0.0];
        // connection.py sends velocity [tracking_speed = 0.5] * 7
        arm.on_target(&target, &[0.5; ARM_JOINTS]).unwrap();
        assert_eq!(arm.max_velocity, [0.5; ARM_JOINTS]);
        assert_eq!(arm.max_acceleration, [0.75; ARM_JOINTS]);
        let mut sent = 0;
        let mut time_s = 0.0;
        // the 5 ms target loop and the 4 ms planner, on one clock
        let (mut next_apply, mut next_plan) = (0.0, 0.0);
        while time_s < 3.0 {
            if time_s >= next_apply {
                arm.apply_target([0.0; 4]);
                next_apply += TARGET_LOOP_S;
            }
            if time_s >= next_plan {
                sent += arm.tick().is_some() as usize;
                next_plan += ARM_CYCLE_S;
            }
            time_s += 0.001;
        }
        let otg = arm.otg.as_ref().unwrap();
        assert!(otg.reached(&target), "{otg:?}");
        // holding: one frame per re-applied target, about 200 Hz, never stops
        assert!(sent > 600, "{sent}");
    }

    #[test]
    fn targets_are_clamped_and_j7_is_softly_bounded() {
        let mut arm = ArmTracker::new(Side::Left, &TrackerSettings::vendor(None).unwrap());
        arm.on_target(&[9.0, -9.0, 0.0, 0.0, 0.0, -1.0, 1.5], &[]).unwrap();
        assert_eq!(arm.target[0], 1.309);
        assert_eq!(arm.target[1], -0.1745);
        // left arm, j6 = -1: upper(-1) = 0.605392, lower(-1) = -0.7491, so j7 stays inside [-0.4843, 0.5993]
        let (upper, lower) = (0.8 * polynomial(&J7_UPPER, -1.0), 0.8 * polynomial(&J7_LOWER, -1.0));
        assert!((upper - 0.8 * 0.605392).abs() < 1e-6, "{upper}");
        assert_eq!(arm.target[6], 1.5_f64.max(-upper).min(-lower));
        assert_eq!(arm.max_velocity, [VENDOR_ARM_MAX_VELOCITY; ARM_JOINTS]);
    }

    #[test]
    fn torso_moves_then_goes_quiet() {
        let mut torso = TorsoTracker::new(&TrackerSettings::vendor(None).unwrap());
        assert!(torso.on_target(&[0.0; 3], &[]).is_err());
        let feedback = TorsoFeedback { position_rad: [0.0; 4], velocity_rad_s: [0.0; 4], torque_nm: [0.0; 4] };
        torso.on_torso_feedback(&feedback);
        torso.on_target(&[0.1, -0.1, 0.05, 0.0], &[0.5; 4]).unwrap();
        let frames: Vec<_> = (0..400).filter_map(|_| torso.tick()).collect();
        assert!(!frames.is_empty() && frames.len() < 400, "{}", frames.len());
        assert_eq!(torso.sent, [0.1, -0.1, 0.05, 0.0]);
        assert!(torso.tick().is_none());
    }
}
