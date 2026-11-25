//! The 4-joint torso (TCU): 0x040 feedback and the 0x041 command another node sends (observed only).
//!
//! R1 Pro takes the `torso_analysis` codecs (`ProtocolTorso::decode` lpm@0x140ef0 compares the product name).

use serde::Serialize;

use crate::bytes::i16s_be;

pub const TORSO_JOINTS: usize = 4;
/// position counts per radian (torso_analysis::decode_torso_impl lpm@0x1459a0: raw / 10000).
pub const POSITION_COUNTS_PER_RAD: f64 = 10000.0;
/// velocity counts per rad/s (decode_torso_impl lpm@0x1459a0: raw / 3000).
pub const VELOCITY_COUNTS_PER_RAD_S: f64 = 3000.0;
/// torque counts per Nm (decode_torso_impl lpm@0x1459a0: raw / 600).
pub const TORQUE_COUNTS_PER_NM: f64 = 600.0;
/// The decoder clamps position to +-3.14 rad (decode_torso_impl, DAT_00148aa0/aa8).
#[allow(clippy::approx_constant)] // the vendor's literal 3.14, not pi
pub const POSITION_CLAMP_RAD: f64 = 3.14;
/// ... velocity to +-10 rad/s (decode_torso_impl lpm@0x1459a0).
pub const VELOCITY_CLAMP_RAD_S: f64 = 10.0;
/// ... torque to +-50 Nm (decode_torso_impl lpm@0x1459a0).
pub const TORQUE_CLAMP_NM: f64 = 50.0;

/// 0x040 TORSO_MOTOR_FB, 200 Hz, 24 bytes; values clamped exactly as the vendor decoder does.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct TorsoFeedback {
    /// torso_joint1..4
    pub position_rad: [f64; TORSO_JOINTS],
    pub velocity_rad_s: [f64; TORSO_JOINTS],
    pub torque_nm: [f64; TORSO_JOINTS],
}

impl TorsoFeedback {
    pub const LEN: usize = 24;

    pub fn decode(data: &[u8]) -> Option<TorsoFeedback> {
        let words = i16s_be::<12>(data, 0)?;
        let field = |offset: usize, counts: f64, clamp: f64| {
            std::array::from_fn(|joint| (f64::from(words[3 * joint + offset]) / counts).clamp(-clamp, clamp))
        };
        Some(TorsoFeedback {
            position_rad: field(0, POSITION_COUNTS_PER_RAD, POSITION_CLAMP_RAD),
            velocity_rad_s: field(1, VELOCITY_COUNTS_PER_RAD_S, VELOCITY_CLAMP_RAD_S),
            torque_nm: field(2, TORQUE_COUNTS_PER_NM, TORQUE_CLAMP_NM),
        })
    }
}

/// 0x041 TORSO_MOTOR_CTRL as observed (WIM RT at 250 Hz in the capture): position and speed limit, no gains.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct TorsoCommand {
    pub position_rad: [f64; TORSO_JOINTS],
    pub velocity_rad_s: [f64; TORSO_JOINTS],
}

impl TorsoCommand {
    pub const LEN: usize = 16;

    /// Inverse of `torso_analysis::encode_torso_impl` lpm@0x144ff0 (joint j at 4j: p x10000, v x3000).
    pub fn decode(data: &[u8]) -> Option<TorsoCommand> {
        let words = i16s_be::<8>(data, 0)?;
        Some(TorsoCommand {
            position_rad: std::array::from_fn(|joint| f64::from(words[2 * joint]) / POSITION_COUNTS_PER_RAD),
            velocity_rad_s: std::array::from_fn(|joint| f64::from(words[2 * joint + 1]) / VELOCITY_COUNTS_PER_RAD_S),
        })
    }
}
