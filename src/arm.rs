//! The two A2 arms (ACU-L 0x05x, ACU-R 0x06x): joint feedback, gripper feedback, wrist F/T, motor temperatures, and
//! the arm / gripper commands other nodes send (decoded only as observed traffic; this crate never transmits).
//!
//! R1 Pro takes the `a2_arm_analysis` codecs (`ProtocolArm::decode_Arm` lpm@0x117b80 compares against "r1pro").

use serde::Serialize;

use crate::bytes::{i16s_be, tail, u16_be};

pub const ARM_JOINTS: usize = 7;
/// feedback position counts per radian (a2_arm_analysis::arm_decode_impl lpm@0x120780: raw / 4700).
pub const POSITION_COUNTS_PER_RAD: f64 = 4700.0;
/// feedback velocity counts per rad/s (arm_decode_impl lpm@0x120780: raw / 750).
pub const VELOCITY_COUNTS_PER_RAD_S: f64 = 750.0;
/// feedback torque counts per Nm (arm_decode_impl lpm@0x120780: raw / 600; agrees with /r1pro/motor_states effort).
pub const TORQUE_COUNTS_PER_NM: f64 = 600.0;
/// command position counts per radian (a2_arm_analysis::arm_encode_impl lpm@0x11f950: p x 4777).
pub const COMMAND_POSITION_COUNTS_PER_RAD: f64 = 4777.0;
/// command velocity counts per rad/s, 12-bit sign-magnitude (arm_encode_impl lpm@0x11f950: |v| x 250).
pub const COMMAND_VELOCITY_COUNTS_PER_RAD_S: f64 = 250.0;
/// command feed-forward torque counts per Nm, 12-bit sign-magnitude (arm_encode_impl lpm@0x11f950: |t| x 20).
pub const COMMAND_TORQUE_COUNTS_PER_NM: f64 = 20.0;
/// gripper command kp counts per unit (ProtocolArm::encode_Gripper lpm@0x11d220: kp x 60).
pub const GRIPPER_KP_COUNTS: f64 = 60.0;
/// gripper command kd counts per unit (encode_Gripper lpm@0x11d220: kd x 150).
pub const GRIPPER_KD_COUNTS: f64 = 150.0;
/// wrist force counts per newton (ProtocolHand::decode_wrench lpm@0x135890: raw / 150).
pub const FORCE_COUNTS_PER_N: f64 = 150.0;
/// wrist moment counts per Nm (decode_wrench lpm@0x135890: raw / 4500).
pub const MOMENT_COUNTS_PER_NM: f64 = 4500.0;
/// decode_wrench clamps force to +-200 N (lpm@0x135890).
pub const FORCE_CLAMP_N: f64 = 200.0;
/// decode_wrench clamps moment to +-7 Nm (lpm@0x135890).
pub const MOMENT_CLAMP_NM: f64 = 7.0;

/// Gripper linkage, `a2_arm_analysis::gripper_decode` lpm@0x11feb0, constants DAT_00147aa0..ae8: motor zero offset.
pub const GRIPPER_ZERO_OFFSET_RAD: f64 = 0.10890854895114899;
/// ... crank length (mm, inferred from the geometry; the opening is twice the per-finger travel).
pub const GRIPPER_CRANK_MM: f64 = 17.5;
/// ... phase inside each segment's sine (pi/8 as the vendor's float).
pub const GRIPPER_PHASE_RAD: f64 = 0.39269909262657166;
/// ... the four segments: (lower bound, upper bound, offset, base), all from DAT_00147aa0..ae8.
#[allow(clippy::approx_constant)] // the vendor's literals, deliberately not exact pi fractions
pub const GRIPPER_SEGMENTS: [(f64, f64, f64, f64); 4] = [
    (-0.1, 0.7853981852531433, 0.0, 6.69696044921875),
    (0.7853981852531433, 1.5707963705062866, 0.78539816, 20.09088134765625),
    (1.5707963705062866, 2.356194496154785, 1.57079632, 33.48480224609375),
    (2.356194496154785, 3.1415927410125732, 2.35619449, 46.87872314453125),
];

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Side {
    Left,
    Right,
}

impl Side {
    pub fn name(self) -> &'static str {
        match self {
            Side::Left => "left",
            Side::Right => "right",
        }
    }
}

/// 0x052 / 0x062 arm feedback, 200 Hz, 48 bytes: joint j at byte 6j, three big-endian i16s.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ArmFeedback {
    pub side: Side,
    /// {side}_arm_joint1..7
    pub position_rad: [f64; ARM_JOINTS],
    pub velocity_rad_s: [f64; ARM_JOINTS],
    pub torque_nm: [f64; ARM_JOINTS],
    /// Bytes 42.. (zero in the capture, unread by HDAS).
    pub unread_tail: Vec<u8>,
}

impl ArmFeedback {
    pub const LEN: usize = 42;

    pub fn decode(side: Side, data: &[u8]) -> Option<ArmFeedback> {
        let words = i16s_be::<21>(data, 0)?;
        let field =
            |offset: usize, counts: f64| std::array::from_fn(|joint| f64::from(words[3 * joint + offset]) / counts);
        Some(ArmFeedback {
            side,
            position_rad: field(0, POSITION_COUNTS_PER_RAD),
            velocity_rad_s: field(1, VELOCITY_COUNTS_PER_RAD_S),
            torque_nm: field(2, TORQUE_COUNTS_PER_NM),
            unread_tail: tail(data, Self::LEN),
        })
    }
}

/// One joint of an observed arm command; kp / kd are the raw 12-bit integers (firmware units unknown).
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ArmJointCommand {
    pub position_rad: f64,
    pub velocity_rad_s: f64,
    pub kp_raw: u16,
    pub kd_raw: u16,
    pub torque_nm: f64,
}

/// 0x050 / 0x060 arm command as another sender put it on the bus (WIM RT at 250 Hz in the capture), 64 bytes.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct ArmCommand {
    pub side: Side,
    pub joints: [ArmJointCommand; ARM_JOINTS],
}

/// 12-bit sign-magnitude (bit 11 = sign) as packed by arm_encode_impl lpm@0x11f950.
fn sign_magnitude_12(raw: u16) -> i32 {
    let magnitude = i32::from(raw & 0x7FF);
    if raw & 0x800 != 0 {
        -magnitude
    } else {
        magnitude
    }
}

impl ArmCommand {
    pub const LEN: usize = 56;

    /// Inverse of `a2_arm_analysis::arm_encode_impl` lpm@0x11f950 (joint j at byte 8j).
    pub fn decode(side: Side, data: &[u8]) -> Option<ArmCommand> {
        let joint = |index: usize| -> Option<ArmJointCommand> {
            let b: [u8; 8] = crate::bytes::array(data, 8 * index)?;
            let nibbles = |high: u8, low: u8| (u16::from(high) << 4) | u16::from(low >> 4);
            let low_nibble_first = |high: u8, low: u8| (u16::from(high & 0x0F) << 8) | u16::from(low);
            Some(ArmJointCommand {
                position_rad: f64::from(i16::from_be_bytes([b[0], b[1]])) / COMMAND_POSITION_COUNTS_PER_RAD,
                velocity_rad_s: f64::from(sign_magnitude_12(nibbles(b[2], b[3]))) / COMMAND_VELOCITY_COUNTS_PER_RAD_S,
                kp_raw: low_nibble_first(b[3], b[4]),
                kd_raw: nibbles(b[5], b[6]),
                torque_nm: f64::from(sign_magnitude_12(low_nibble_first(b[6], b[7]))) / COMMAND_TORQUE_COUNTS_PER_NM,
            })
        };
        Some(ArmCommand { side, joints: [joint(0)?, joint(1)?, joint(2)?, joint(3)?, joint(4)?, joint(5)?, joint(6)?] })
    }
}

/// 0x058 / 0x068 gripper feedback, 200 Hz, 8 bytes (`a2_arm_analysis::gripper_decode` lpm@0x11feb0).
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct GripperFeedback {
    pub side: Side,
    pub motor_position_rad: f64,
    /// Motor units, same scale as the arm joints.
    pub velocity: f64,
    pub torque: f64,
    /// Total finger opening through the vendor's sine linkage; mm is inferred. None where the vendor logs
    /// "gripper_position is out of limit" (and publishes 0).
    pub opening_mm: Option<f64>,
    /// Bytes 6-7, unread by HDAS (low bits toggle: a status or counter, unknown).
    pub unread_tail: Option<u16>,
}

/// The vendor's motor angle -> opening mapping (gripper_decode lpm@0x11feb0); 0 = closed, about 107 fully open.
pub fn gripper_opening_mm(motor_position_rad: f64) -> Option<f64> {
    let angle = GRIPPER_ZERO_OFFSET_RAD - motor_position_rad;
    GRIPPER_SEGMENTS
        .iter()
        .find(|(low, high, _, _)| *low < angle && angle < *high)
        .map(|(_, _, offset, base)| 2.0 * (base - GRIPPER_CRANK_MM * (GRIPPER_PHASE_RAD - (angle - offset)).sin()))
}

impl GripperFeedback {
    pub const LEN: usize = 6;

    pub fn decode(side: Side, data: &[u8]) -> Option<GripperFeedback> {
        let [position, velocity, torque] = i16s_be::<3>(data, 0)?;
        let motor_position_rad = f64::from(position) / POSITION_COUNTS_PER_RAD;
        Some(GripperFeedback {
            side,
            motor_position_rad,
            velocity: f64::from(velocity) / VELOCITY_COUNTS_PER_RAD_S,
            torque: f64::from(torque) / TORQUE_COUNTS_PER_NM,
            opening_mm: gripper_opening_mm(motor_position_rad),
            unread_tail: u16_be(data, 6),
        })
    }
}

/// 0x051 / 0x061 gripper command as observed (HDAS, about 415 Hz), 10 bytes of big-endian i16s.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct GripperCommand {
    pub side: Side,
    pub motor_position_rad: f64,
    pub velocity: f64,
    pub kp: f64,
    pub kd: f64,
    pub torque: f64,
}

impl GripperCommand {
    pub const LEN: usize = 10;

    /// Inverse of `ProtocolArm::encode_Gripper` lpm@0x11d220 (p x4700, v x750, kp x60, kd x150, t x600).
    pub fn decode(side: Side, data: &[u8]) -> Option<GripperCommand> {
        let [position, velocity, kp, kd, torque] = i16s_be::<5>(data, 0)?;
        Some(GripperCommand {
            side,
            motor_position_rad: f64::from(position) / POSITION_COUNTS_PER_RAD,
            velocity: f64::from(velocity) / VELOCITY_COUNTS_PER_RAD_S,
            kp: f64::from(kp) / GRIPPER_KP_COUNTS,
            kd: f64::from(kd) / GRIPPER_KD_COUNTS,
            torque: f64::from(torque) / TORQUE_COUNTS_PER_NM,
        })
    }
}

/// 0x057 / 0x067 wrist force/torque, 100 Hz, 12 bytes; all zero in the capture (no sensor values streamed).
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Wrench {
    pub side: Side,
    pub force_n: [f64; 3],
    pub moment_nm: [f64; 3],
}

impl Wrench {
    pub const LEN: usize = 12;

    pub fn decode(side: Side, data: &[u8]) -> Option<Wrench> {
        let raw = i16s_be::<6>(data, 0)?;
        Some(Wrench {
            side,
            force_n: std::array::from_fn(|i| {
                (f64::from(raw[i]) / FORCE_COUNTS_PER_N).clamp(-FORCE_CLAMP_N, FORCE_CLAMP_N)
            }),
            moment_nm: std::array::from_fn(|i| {
                (f64::from(raw[3 + i]) / MOMENT_COUNTS_PER_NM).clamp(-MOMENT_CLAMP_NM, MOMENT_CLAMP_NM)
            }),
        })
    }
}

/// 0x05D / 0x06D per-joint MOS and winding temperatures, 1 Hz, 64 bytes (`acu_temp_decode` lpm@0x1218e0).
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct MotorTemperatures {
    pub side: Side,
    /// Degrees C, no offset, joint1..7.
    pub mos_c: [u8; ARM_JOINTS],
    pub winding_c: [u8; ARM_JOINTS],
}

impl MotorTemperatures {
    pub const LEN: usize = 14;

    pub fn decode(side: Side, data: &[u8]) -> Option<MotorTemperatures> {
        let pairs: [u8; 14] = crate::bytes::array(data, 0)?;
        Some(MotorTemperatures {
            side,
            mos_c: std::array::from_fn(|joint| pairs[2 * joint]),
            winding_c: std::array::from_fn(|joint| pairs[2 * joint + 1]),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gripper_linkage_is_zero_when_closed_and_bounded() {
        let closed = gripper_opening_mm(GRIPPER_ZERO_OFFSET_RAD).unwrap();
        assert!(closed.abs() < 1e-6, "{closed}");
        assert!(gripper_opening_mm(GRIPPER_ZERO_OFFSET_RAD - 3.2).is_none());
    }

    #[test]
    fn arm_command_sign_magnitude() {
        let mut data = [0u8; 64];
        // joint 0: v = -0.4 (mag 100 | sign), kp 2, kd 25, t = -3.45 (mag 69 | sign)
        let v = 0x800u16 | 100;
        let t = 0x800u16 | 69;
        data[2] = (v >> 4) as u8;
        data[3] = ((v & 0xF) << 4) as u8;
        let kd = 25u16;
        data[4] = 2;
        data[5] = (kd >> 4) as u8;
        data[6] = (((kd & 0xF) << 4) | (t >> 8)) as u8;
        data[7] = (t & 0xFF) as u8;
        let joint = &ArmCommand::decode(Side::Left, &data).unwrap().joints[0];
        assert_eq!((joint.velocity_rad_s, joint.kp_raw, joint.kd_raw, joint.torque_nm), (-0.4, 2, 25, -3.45));
    }
}
