//! HDAS's checks between an incoming `hdas_msg/MotorControl` and the encoder (the MotorControl subscription lambdas
//! in HDAS main). A rejected command is dropped whole, never clamped: nothing goes on the bus for it.

use serde::Serialize;

use crate::arm::ARM_JOINTS;
use crate::torso::TORSO_JOINTS;

/// Why HDAS would drop a command.
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "reason", rename_all = "snake_case")]
pub enum Rejected {
    /// "ctrl_command of ... contains empty array" / wrong length.
    WrongLength { field: &'static str, expected: usize, got: usize },
    /// "exceed limit, ignored".
    ExceedsLimit { field: &'static str, joint: usize, value: f32 },
    /// "Torso control command verification failed" (the mode checksum).
    ChecksumMismatch { expected: u8, got: u8 },
}

/// One field's accepted range (inclusive).
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct Range {
    pub low: f32,
    pub high: f32,
}

impl Range {
    pub const fn symmetric(limit: f32) -> Range {
        Range { low: -limit, high: limit }
    }

    fn admits(self, value: f32) -> bool {
        self.low <= value && value <= self.high
    }
}

/// The per-element limits of the arm lambdas (HDAS@0x155a00 left / 0x1573c0 right).
#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct ArmLimits {
    pub position_rad: Range,
    pub velocity_rad_s: Range,
    pub kp: Range,
    pub kd: Range,
    pub torque_nm: Range,
}

/// HDAS's arm limits: p +-6.5, v +-40, t +-50, kp 0..500, kd 0..200 (constants in HDAS@0x155a00).
pub const VENDOR_ARM_LIMITS: ArmLimits = ArmLimits {
    position_rad: Range::symmetric(6.5),
    velocity_rad_s: Range::symmetric(40.0),
    kp: Range { low: 0.0, high: 500.0 },
    kd: Range { low: 0.0, high: 200.0 },
    torque_nm: Range::symmetric(50.0),
};

/// A MotorControl's arrays, as HDAS receives them.
#[derive(Clone, Debug, PartialEq, Default, Serialize)]
pub struct MotorControl {
    pub p_des: Vec<f32>,
    pub v_des: Vec<f32>,
    pub kp: Vec<f32>,
    pub kd: Vec<f32>,
    pub t_ff: Vec<f32>,
    pub mode: u8,
}

fn expect_len(field: &'static str, values: &[f32], expected: usize) -> Result<(), Rejected> {
    if values.len() != expected {
        return Err(Rejected::WrongLength { field, expected, got: values.len() });
    }
    Ok(())
}

fn expect_within(field: &'static str, values: &[f32], range: Range) -> Result<(), Rejected> {
    match values.iter().position(|&value| !range.admits(value)) {
        Some(joint) => Err(Rejected::ExceedsLimit { field, joint, value: values[joint] }),
        None => Ok(()),
    }
}

/// The arm lambda's checks: 7 of everything, every element inside its range, optional joint-bias subtraction.
pub fn check_arm(
    command: &MotorControl,
    limits: &ArmLimits,
    joint_bias_rad: Option<&[f32; ARM_JOINTS]>,
) -> Result<MotorControl, Rejected> {
    for (field, values) in [
        ("p_des", &command.p_des),
        ("v_des", &command.v_des),
        ("kp", &command.kp),
        ("kd", &command.kd),
        ("t_ff", &command.t_ff),
    ] {
        expect_len(field, values, ARM_JOINTS)?;
    }
    expect_within("p_des", &command.p_des, limits.position_rad)?;
    expect_within("v_des", &command.v_des, limits.velocity_rad_s)?;
    expect_within("kp", &command.kp, limits.kp)?;
    expect_within("kd", &command.kd, limits.kd)?;
    expect_within("t_ff", &command.t_ff, limits.torque_nm)?;
    let mut accepted = command.clone();
    if let Some(bias) = joint_bias_rad {
        accepted.p_des.iter_mut().zip(bias).for_each(|(position, bias)| *position -= bias);
    }
    Ok(accepted)
}

/// The torso lambda's `mode` checksum (main::anon_struct_56_4_26e2e147 HDAS@0x155214): a mix of element 0 of every
/// array, finished with murmur3's fmix32; `mode` must equal its low byte.
pub fn torso_checksum(command: &MotorControl) -> u8 {
    let first = |values: &[f32]| values.first().copied().unwrap_or(0.0).to_bits();
    let mut hash = first(&command.p_des).wrapping_mul(0x1f)
        ^ first(&command.v_des).wrapping_mul(0x25)
        ^ first(&command.kp).wrapping_mul(0x29)
        ^ first(&command.kd).wrapping_mul(0x2b)
        ^ first(&command.t_ff).wrapping_mul(0x2f)
        ^ 0x9e37_79b9;
    hash ^= hash >> 16;
    hash = hash.wrapping_mul(0x85eb_ca6b);
    hash ^= hash >> 13;
    hash = hash.wrapping_mul(0xc2b2_ae35);
    hash ^= hash >> 16;
    hash as u8
}

/// The torso lambda's checks: 4 of every array (r1pro) and the checksum, optional bias subtraction.
pub fn check_torso(
    command: &MotorControl,
    joint_bias_rad: Option<&[f32; TORSO_JOINTS]>,
) -> Result<MotorControl, Rejected> {
    for (field, values) in [
        ("p_des", &command.p_des),
        ("v_des", &command.v_des),
        ("kp", &command.kp),
        ("kd", &command.kd),
        ("t_ff", &command.t_ff),
    ] {
        expect_len(field, values, TORSO_JOINTS)?;
    }
    let expected = torso_checksum(command);
    if command.mode != expected {
        return Err(Rejected::ChecksumMismatch { expected, got: command.mode });
    }
    let mut accepted = command.clone();
    if let Some(bias) = joint_bias_rad {
        accepted.p_des.iter_mut().zip(bias).for_each(|(position, bias)| *position -= bias);
    }
    Ok(accepted)
}

/// The chassis lambda's check (HDAS@0x154ec0): p_des 3 entries, v_des 6, else dropped.
pub fn check_chassis(command: &MotorControl) -> Result<(), Rejected> {
    expect_len("p_des", &command.p_des, 3)?;
    expect_len("v_des", &command.v_des, 6)
}
