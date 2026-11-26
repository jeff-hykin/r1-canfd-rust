//! The send side a dimos native module calls: one commander per device, each turning a command into the frames HDAS
//! (plus chassis_control_node and r1_gripper_controller where they sit in the vendor path) would put on the bus.
//!
//! Commanders never touch a socket: they return [`OutgoingFrame`]s or push them into a [`FrameSink`]; the binary that
//! owns the bus decides to send (`Bus::send`, `transmit` feature). Rates follow the vendor:
//! - arm, torso, LED, function frames: one frame per call (HDAS sends one frame per ROS message, no timer).
//! - chassis: [`ChassisCommander::tick`] once per 20 ms (chassis_control_node's loop).
//! - grippers: [`GripperCommander::tick`] once per 2 ms (r1_gripper_controller's 500 Hz loop + OTG).
//! - fan level 0x023: [`FanCommander::tick`] every 0.5 s (HDAS's fan_control_thread).
//!
//! Feedback frames go to [`Commanders::on_frame`], which routes them to whichever commander needs them.

use serde::Serialize;

use crate::arm::{GripperFeedback, Side, ARM_JOINTS};
use crate::chassis::{MotorStatus, Remote};
use crate::chassis_controller::{ChassisController, ChassisStep, RC_MODE_SOFTWARE};
use crate::decode::{decode, Decoded};
use crate::encode::{
    encode_arm, encode_function_frame, encode_gripper, encode_led, encode_torso, ArmJointTarget, FunctionBoard,
    OutgoingFrame, TorsoJointTarget,
};
use crate::frame::Frame;
use crate::gate::{check_arm, ArmLimits, MotorControl, Rejected, LED_MAX_MODE, VENDOR_ARM_LIMITS};
use crate::gripper_controller::{
    GripperConfig, GripperController, OTG_CYCLE_S, VENDOR_BASELINE_GATHER_PERIOD_S, VENDOR_BASELINE_GATHER_S,
    VENDOR_OPENING_AFTER_STARTUP, VENDOR_STARTUP_HOLD_S, VENDOR_STARTUP_OPENINGS,
};
use crate::gripper_geometry::InverseLut;
use crate::odometry::{Pose2d, WheelOdometry};
use crate::swerve::Twist;
use crate::torso::TORSO_JOINTS;

/// Where frames go: a socket in the bus binary, a `Vec` in tests.
pub trait FrameSink {
    fn push(&mut self, frame: OutgoingFrame);
}

impl FrameSink for Vec<OutgoingFrame> {
    fn push(&mut self, frame: OutgoingFrame) {
        Vec::push(self, frame);
    }
}

/// Arm command path: HDAS's arm lambda gates (HDAS@0x155a00 / 0x1573c0), then arm_encode_impl.
#[derive(Clone, Debug, Serialize)]
pub struct ArmCommander {
    pub side: Side,
    pub limits: ArmLimits,
    /// `/calib/joint_bias` subtracted from p_des when HDAS's calib_enable is "enable" (disabled on this robot).
    pub joint_bias_rad: Option<[f32; ARM_JOINTS]>,
}

impl ArmCommander {
    pub fn new(side: Side) -> ArmCommander {
        ArmCommander { side, limits: VENDOR_ARM_LIMITS, joint_bias_rad: None }
    }

    /// One 0x050 / 0x060 frame, or the reason HDAS would drop the whole command (nothing is clamped).
    pub fn set_joint_targets(
        &self,
        position_rad: &[f32; ARM_JOINTS],
        velocity_rad_s: &[f32; ARM_JOINTS],
        kp: &[f32; ARM_JOINTS],
        kd: &[f32; ARM_JOINTS],
        torque_nm: &[f32; ARM_JOINTS],
    ) -> Result<OutgoingFrame, Rejected> {
        let command = MotorControl {
            p_des: position_rad.to_vec(),
            v_des: velocity_rad_s.to_vec(),
            kp: kp.to_vec(),
            kd: kd.to_vec(),
            t_ff: torque_nm.to_vec(),
            mode: 0,
        };
        let accepted = check_arm(&command, &self.limits, self.joint_bias_rad.as_ref())?;
        let joints = std::array::from_fn(|joint| ArmJointTarget {
            position_rad: accepted.p_des[joint],
            velocity_rad_s: accepted.v_des[joint],
            kp: accepted.kp[joint],
            kd: accepted.kd[joint],
            torque_nm: accepted.t_ff[joint],
        });
        Ok(encode_arm(self.side, &joints))
    }
}

/// Torso command path: encode_torso_impl, one frame per call. HDAS's mode checksum only guards its ROS topic, and
/// its length check is the array type here; calib bias is off on this robot.
#[derive(Clone, Debug, Default, Serialize)]
pub struct TorsoCommander {
    pub joint_bias_rad: Option<[f32; TORSO_JOINTS]>,
}

impl TorsoCommander {
    /// One 0x041 frame: target positions and per-joint speed limits (the TCU frame has no gains).
    pub fn set(&self, position_rad: &[f32; TORSO_JOINTS], velocity_rad_s: &[f32; TORSO_JOINTS]) -> OutgoingFrame {
        let bias = self.joint_bias_rad.unwrap_or([0.0; TORSO_JOINTS]);
        encode_torso(&std::array::from_fn(|joint| TorsoJointTarget {
            position_rad: if self.joint_bias_rad.is_some() {
                position_rad[joint] - bias[joint]
            } else {
                position_rad[joint]
            },
            velocity_rad_s: velocity_rad_s[joint],
        }))
    }
}

/// How the chassis gets permission to take software twists.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ChassisAuthority {
    /// Vendor: chassis_control_node follows the RC mode byte of 0x032 (via HDAS's /controller): mode 5 hands over.
    RemoteMode,
    /// What dimos runs today (chassis_gatekeeper.py remaps /controller and publishes mode 5 plus brake=false every
    /// 20 ms): software always has authority and the 10-loop timeout never fires, so a twist is held until replaced.
    Gatekeeper,
}

/// Chassis command path: chassis_control_node's loop, then chassis_encode_impl; also wheel odometry from 0x030.
#[derive(Clone, Debug, Serialize)]
pub struct ChassisCommander {
    pub controller: ChassisController,
    pub authority: ChassisAuthority,
    #[serde(skip)]
    pub odometry: WheelOdometry,
}

impl ChassisCommander {
    pub fn new(authority: ChassisAuthority) -> ChassisCommander {
        let mut controller = ChassisController::default();
        if authority == ChassisAuthority::Gatekeeper {
            controller.rc_has_control = false;
            controller.on_acceleration_limit(crate::chassis_controller::ACCELERATION_CAPS);
        }
        ChassisCommander { controller, authority, odometry: WheelOdometry::new() }
    }

    /// `/motion_target/target_speed_chassis`.
    pub fn set_twist(&mut self, twist: Twist) {
        self.controller.on_twist(twist);
    }

    /// `/motion_target/brake_mode`: steer to the brake pose (0.75, -0.75, 0).
    pub fn set_brake(&mut self, brake: bool) {
        self.controller.on_brake(brake);
    }

    pub fn on_motor_status(&mut self, timestamp_ns: u64, status: &MotorStatus) {
        let speeds = status.wheel_speed_m_s;
        self.controller.on_feedback(status.steer_angle_rad, speeds);
        self.odometry.on_motor_status(timestamp_ns, status);
    }

    /// 0x032 as HDAS publishes it on /controller (axes as float(raw)); ignored under the gatekeeper's remap.
    pub fn on_remote(&mut self, remote: &Remote) {
        if self.authority == ChassisAuthority::RemoteMode {
            self.controller.on_remote(
                remote.mode,
                f32::from(remote.left_x),
                f32::from(remote.left_y),
                f32::from(remote.right_x),
            );
        }
    }

    /// One 20 ms loop: the 0x031 frame to send and the FK twist chassis_control_node would publish.
    pub fn tick(&mut self) -> (OutgoingFrame, ChassisStep) {
        if self.authority == ChassisAuthority::Gatekeeper {
            self.controller.on_remote(RC_MODE_SOFTWARE, 0.0, 0.0, 0.0);
            self.controller.on_brake(self.controller.brake);
        }
        let step = self.controller.step();
        (crate::encode::encode_chassis(&step.target), step)
    }

    pub fn pose(&self) -> Pose2d {
        self.odometry.pose
    }
}

/// Gripper command path: r1_gripper_controller (startup, modes, OTG), then encode_Gripper.
#[derive(Clone, Debug, Serialize)]
pub struct GripperCommander {
    pub side: Side,
    pub controller: GripperController,
    /// Latest 0x058 / 0x068 feedback: opening (HDAS's linkage), velocity, torque as effort.
    pub feedback: Option<(f64, f64, f64)>,
    pub startup: GripperStartup,
}

/// Where a gripper is in performInitialization (rgc main), counted in 2 ms ticks.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum GripperStartup {
    /// "[Init] Collecting baseline forces...": baseline only, nothing sent.
    Gathering {
        ticks: u32,
        samples: u32,
    },
    /// "[Init] Performing open-close-open test motion...": direct targets, no control pass.
    Sweeping {
        ticks: u32,
    },
    /// Joining a running vendor: the first feedback settles the baseline, then control.
    Joining,
    Done,
}

/// Control ticks per startup step: 2.0 s at the 500 Hz loop (performInitialization rgc, Rate 2 ms rgc@0x15aa10).
const STARTUP_TICKS_PER_STEP: u32 = (VENDOR_STARTUP_HOLD_S / OTG_CYCLE_S) as u32;
/// 2 ms ticks in the 1 s baseline gather.
const GATHER_TICKS: u32 = (VENDOR_BASELINE_GATHER_S / OTG_CYCLE_S) as u32;

impl GripperCommander {
    /// `run_startup`: the vendor's 1 s baseline gather, then open 50 / close / 50 / open, 2 s each, once feedback
    /// arrives. Off: join a running vendor (the first feedback settles the baseline).
    pub fn new(side: Side, lut: InverseLut, run_startup: bool) -> GripperCommander {
        let side_index = if side == Side::Left { 0 } else { 1 };
        GripperCommander {
            side,
            controller: GripperController::new(GripperConfig::vendor(side_index), lut),
            feedback: None,
            startup: if run_startup {
                GripperStartup::Gathering { ticks: 0, samples: 0 }
            } else {
                GripperStartup::Joining
            },
        }
    }

    /// `/motion_target/target_position_gripper_*`: opening 0 (closed) .. 100 (open). Overwritten by the startup,
    /// whose end forces the target to 100 like the vendor.
    pub fn set(&mut self, opening: f64) {
        let position = self.feedback.map(|(position, _, _)| position).unwrap_or(0.0);
        self.controller.set_opening(opening, position);
    }

    pub fn on_feedback(&mut self, feedback: &GripperFeedback) {
        self.feedback = Some((feedback.opening_mm.unwrap_or(0.0), feedback.velocity, feedback.torque));
    }

    /// One 2 ms pass; `None` before the first feedback and during the baseline gather.
    pub fn tick(&mut self) -> Option<OutgoingFrame> {
        let (position, velocity, effort) = self.feedback?;
        match self.startup {
            GripperStartup::Gathering { ticks, samples } => {
                // the vendor samples every 5 ms: take one whenever the 2 ms clock crosses a 5 ms boundary
                let due = ((f64::from(ticks) * OTG_CYCLE_S) / VENDOR_BASELINE_GATHER_PERIOD_S) as u32 + 1;
                let samples = if samples < due {
                    self.controller.gather_baseline(effort);
                    samples + 1
                } else {
                    samples
                };
                self.startup = if ticks + 1 >= GATHER_TICKS {
                    GripperStartup::Sweeping { ticks: 0 }
                } else {
                    GripperStartup::Gathering { ticks: ticks + 1, samples }
                };
                return None;
            }
            GripperStartup::Sweeping { ticks } => {
                match VENDOR_STARTUP_OPENINGS.get((ticks / STARTUP_TICKS_PER_STEP) as usize) {
                    Some(&opening) => {
                        self.controller.direct_target(opening);
                        self.startup = GripperStartup::Sweeping { ticks: ticks + 1 };
                        return Some(encode_gripper(self.side, &self.controller.otg_step()));
                    }
                    None => {
                        self.controller.target_opening = VENDOR_OPENING_AFTER_STARTUP;
                        self.startup = GripperStartup::Done;
                    }
                }
            }
            GripperStartup::Joining => {
                self.controller.settle_baseline(effort);
                self.startup = GripperStartup::Done;
            }
            GripperStartup::Done => {}
        }
        self.controller.control(position, velocity, effort);
        Some(encode_gripper(self.side, &self.controller.otg_step()))
    }
}

/// 0x023: HDAS's fan_control_thread (lpm `_M_run` @0x124ff0) reads the Orin's thermal_zone0 every 0.5 s and sends
/// `[max(0, (millicelsius / 1000 / 10) - 2)]` through the CCU control queue.
#[derive(Clone, Copy, Debug, Default, Serialize)]
pub struct FanCommander;

/// Thermal zone the vendor reads (`cat /sys/devices/virtual/thermal/thermal_zone0/temp` in fan_control_thread).
pub const FAN_THERMAL_ZONE: &str = "/sys/devices/virtual/thermal/thermal_zone0/temp";
/// Fan frame ID and period (nanosleep 0x1dcd6500 ns = 0.5 s in fan_control_thread).
pub const FAN_COMMAND_ID: u32 = 0x023;
pub const FAN_PERIOD_S: f64 = 0.5;
/// Level = tens of degrees C minus this, floored at 0 (fan_control_thread).
const FAN_LEVEL_OFFSET: i32 = 2;

impl FanCommander {
    /// The 0x023 frame for a thermal reading in millidegrees C (47900 -> level 2, as on the robot).
    pub fn tick(&self, millicelsius: i64) -> OutgoingFrame {
        let tens_of_degrees = (millicelsius as f64 / 1000.0 / 10.0) as i32;
        OutgoingFrame { id: FAN_COMMAND_ID, data: vec![(tens_of_degrees - FAN_LEVEL_OFFSET).max(0) as u8] }
    }
}

/// One-shot CCU frames.
pub fn led(r: u8, g: u8, b: u8, mode: u8) -> OutgoingFrame {
    encode_led(r, g, b, mode.min(LED_MAX_MODE))
}

/// Function frame 1..10 to a board (HDAS's function_frame_* services); `None` outside 1..10.
pub fn function_frame(board: FunctionBoard, command: u8) -> Option<OutgoingFrame> {
    encode_function_frame(board, command)
}

/// Host-side e-stop view. HDAS never changes what it sends on e-stop (only publishes FeedbackStatus); the MCUs act.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize)]
pub struct EmergencyStopState {
    /// A 0x001 frame has arrived (there is no release frame).
    pub seen_frame: bool,
    /// The latest 0x033 had bit 10 set on any chassis motor.
    pub chassis_bit: bool,
}

/// Every commander plus the feedback routing.
pub struct Commanders {
    pub left_arm: ArmCommander,
    pub right_arm: ArmCommander,
    pub torso: TorsoCommander,
    pub chassis: ChassisCommander,
    pub left_gripper: GripperCommander,
    pub right_gripper: GripperCommander,
    pub fan: FanCommander,
    pub emergency_stop: EmergencyStopState,
}

impl Commanders {
    pub fn new(authority: ChassisAuthority, run_gripper_startup: bool) -> Commanders {
        let lut = InverseLut::vendor();
        Commanders {
            left_arm: ArmCommander::new(Side::Left),
            right_arm: ArmCommander::new(Side::Right),
            torso: TorsoCommander::default(),
            chassis: ChassisCommander::new(authority),
            left_gripper: GripperCommander::new(Side::Left, lut.clone(), run_gripper_startup),
            right_gripper: GripperCommander::new(Side::Right, lut, run_gripper_startup),
            fan: FanCommander,
            emergency_stop: EmergencyStopState::default(),
        }
    }

    /// Route one received frame to the commanders that use it; returns the decode for the caller's own use.
    pub fn on_frame(&mut self, frame: &Frame) -> Decoded {
        let decoded = decode(frame);
        match &decoded {
            Decoded::MotorStatus(status) => self.chassis.on_motor_status(frame.timestamp_ns, status),
            Decoded::Remote(remote) => self.chassis.on_remote(remote),
            Decoded::GripperFeedback(feedback) if feedback.side == Side::Left => {
                self.left_gripper.on_feedback(feedback)
            }
            Decoded::GripperFeedback(feedback) => self.right_gripper.on_feedback(feedback),
            Decoded::EmergencyStop(_) => self.emergency_stop.seen_frame = true,
            Decoded::Errors(report) if report.source == crate::status::ErrorSource::Chassis => {
                self.emergency_stop.chassis_bit = report.emergency_stop();
            }
            _ => {}
        }
        decoded
    }

    /// Both grippers' 2 ms pass into `sink`.
    pub fn tick_grippers(&mut self, sink: &mut impl FrameSink) {
        for gripper in [&mut self.left_gripper, &mut self.right_gripper] {
            if let Some(frame) = gripper.tick() {
                sink.push(frame);
            }
        }
    }

    /// The chassis 20 ms pass into `sink`.
    pub fn tick_chassis(&mut self, sink: &mut impl FrameSink) -> ChassisStep {
        let (frame, step) = self.chassis.tick();
        sink.push(frame);
        step
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arm_rejects_whole_commands_like_hdas() {
        let arm = ArmCommander::new(Side::Left);
        let zeros = [0.0f32; ARM_JOINTS];
        let mut kp = [2.0f32; ARM_JOINTS];
        assert!(arm.set_joint_targets(&zeros, &[0.4; ARM_JOINTS], &kp, &[25.0; ARM_JOINTS], &zeros).is_ok());
        kp[3] = 501.0;
        let rejected = arm.set_joint_targets(&zeros, &zeros, &kp, &zeros, &zeros).unwrap_err();
        assert_eq!(rejected, Rejected::ExceedsLimit { field: "kp", joint: 3, value: 501.0 });
        let mut kd = [0.0f32; ARM_JOINTS];
        kd[0] = 201.0;
        assert!(arm.set_joint_targets(&zeros, &zeros, &zeros, &kd, &zeros).is_err());
        let mut position = zeros;
        position[6] = f32::NAN;
        assert!(arm.set_joint_targets(&position, &zeros, &zeros, &zeros, &zeros).is_ok(), "NaN passes HDAS's gate");
    }

    #[test]
    fn fan_level_matches_the_robot() {
        assert_eq!(FanCommander.tick(47_900).data, [2]);
        assert_eq!(FanCommander.tick(15_000).data, [0]);
        assert_eq!(FanCommander.tick(71_000).data, [5]);
    }

    #[test]
    fn led_mode_is_clamped() {
        assert_eq!(led(1, 2, 3, 7).data, [1, 2, 3, 1]);
    }

    #[test]
    fn gripper_waits_for_feedback_and_runs_the_startup_sweep() {
        let mut gripper = GripperCommander::new(Side::Right, InverseLut::vendor(), true);
        assert!(gripper.tick().is_none());
        let feedback = GripperFeedback {
            side: Side::Right,
            motor_position_rad: 0.0,
            velocity: 0.0,
            torque: 0.0,
            opening_mm: Some(2.7),
            unread_tail: None,
        };
        gripper.on_feedback(&feedback);
        gripper.set(0.0);
        let frames: Vec<_> =
            (0..GATHER_TICKS + 4 * STARTUP_TICKS_PER_STEP + 1000).filter_map(|_| gripper.tick()).collect();
        assert_eq!(frames.len() as u32, 4 * STARTUP_TICKS_PER_STEP + 1000, "nothing is sent while gathering");
        assert_eq!(gripper.controller.target_opening, 100.0);
        // fully open at the end, the capture's right-gripper payload
        assert_eq!(frames.last().unwrap().data, [0xCC, 0x98, 0, 0, 0x04, 0xB0, 0x05, 0xDC, 0, 0]);
    }
}
