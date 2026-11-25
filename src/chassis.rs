//! The three-wheel swerve chassis (VCU): 0x030 wheel/steer feedback, 0x032 RC remote, and the 0x031 command another
//! node sends (decoded here only as observed traffic; this crate never transmits).
//!
//! Wheel order everywhere is front-left, front-right, rear. The capture was taken parked, so the wheel-speed scale
//! (m/s at the contact patch) is inferred from chassis_control_node's IK units, not checked under motion.

use serde::Serialize;

use crate::bytes::{i16s_be, tail, u32_le};

/// wheel speed counts per m/s (decode_chassis lpm@0x129290: raw / 1000; m/s is inferred, UNVERIFIED under motion).
pub const WHEEL_SPEED_COUNTS_PER_M_S: f64 = 1000.0;
/// steer angle counts per radian (decode_chassis lpm@0x129290: raw / 1000).
pub const STEER_ANGLE_COUNTS_PER_RAD: f64 = 1000.0;
/// steer rate counts per rad/s (decode_chassis lpm@0x129290: raw / 1000).
pub const STEER_RATE_COUNTS_PER_RAD_S: f64 = 1000.0;
/// Byte offset of the VCU's little-endian ms tick in 0x030 (capture only: it rises 1000/s; HDAS ignores it).
pub const VCU_TICK_OFFSET: usize = 32;
/// HDAS reads bytes 0-17 of 0x030 and nothing after (decode_chassis lpm@0x129290).
pub const MOTOR_STATUS_DECODED_LEN: usize = 18;

fn scaled<const N: usize>(raw: [i16; N], counts_per_unit: f64) -> [f64; N] {
    raw.map(|value| f64::from(value) / counts_per_unit)
}

/// 0x030 MOTOR_STATUS, 200 Hz, 48 bytes.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct MotorStatus {
    /// fl, fr, r drive speed; m/s at the contact patch (scale UNVERIFIED, robot was parked).
    pub wheel_speed_m_s: [f64; 3],
    /// fl, fr, r steer angle; 0 = wheel along +x, positive CCW (from the IK, not firmware).
    pub steer_angle_rad: [f64; 3],
    pub steer_rate_rad_s: [f64; 3],
    /// Bytes 32-35 little-endian, a ms counter (capture only, not vendor-sourced).
    pub vcu_tick_ms: Option<u32>,
    /// Bytes 18.. as received; HDAS reads none of them.
    pub unread_tail: Vec<u8>,
}

impl MotorStatus {
    pub const LEN: usize = MOTOR_STATUS_DECODED_LEN;

    pub fn decode(data: &[u8]) -> Option<MotorStatus> {
        let words = i16s_be::<9>(data, 0)?;
        let [fl, fr, r, angle_fl, angle_fr, angle_r, rate_fl, rate_fr, rate_r] = words;
        Some(MotorStatus {
            wheel_speed_m_s: scaled([fl, fr, r], WHEEL_SPEED_COUNTS_PER_M_S),
            steer_angle_rad: scaled([angle_fl, angle_fr, angle_r], STEER_ANGLE_COUNTS_PER_RAD),
            steer_rate_rad_s: scaled([rate_fl, rate_fr, rate_r], STEER_RATE_COUNTS_PER_RAD_S),
            vcu_tick_ms: u32_le(data, VCU_TICK_OFFSET),
            unread_tail: tail(data, MOTOR_STATUS_DECODED_LEN),
        })
    }
}

/// 0x032 REMOTE, 200 Hz, 12 bytes: the RC sticks as raw SBUS-style counts (240..1807, 1024 = centre per ccn).
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct Remote {
    pub left_x: i16,
    pub left_y: i16,
    pub right_x: i16,
    pub right_y: i16,
    pub mode: u8,
}

impl Remote {
    pub const LEN: usize = 9;

    /// `ProtocolChassis::decode_remote` lpm@0x1293e0 (struct serial_feedback_remote: i16 ch_0..3, u8 mode).
    pub fn decode(data: &[u8]) -> Option<Remote> {
        let [left_x, left_y, right_x, right_y] = i16s_be::<4>(data, 0)?;
        Some(Remote { left_x, left_y, right_x, right_y, mode: *data.get(8)? })
    }
}

/// 0x031 MOTOR_CTRL as another sender put it on the bus (HDAS, 50 Hz): per-wheel targets.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ChassisCommand {
    pub wheel_speed_m_s: [f64; 3],
    pub steer_angle_rad: [f64; 3],
    pub steer_rate_rad_s: [f64; 3],
}

impl ChassisCommand {
    pub const LEN: usize = 18;

    /// Inverse of `chassis_analysis::chassis_encode_impl` lpm@0x12bec0 (x1000 each, same order as 0x030).
    pub fn decode(data: &[u8]) -> Option<ChassisCommand> {
        let [fl, fr, r, angle_fl, angle_fr, angle_r, rate_fl, rate_fr, rate_r] = i16s_be::<9>(data, 0)?;
        Some(ChassisCommand {
            wheel_speed_m_s: scaled([fl, fr, r], WHEEL_SPEED_COUNTS_PER_M_S),
            steer_angle_rad: scaled([angle_fl, angle_fr, angle_r], STEER_ANGLE_COUNTS_PER_RAD),
            steer_rate_rad_s: scaled([rate_fl, rate_fr, rate_r], STEER_RATE_COUNTS_PER_RAD_S),
        })
    }
}
