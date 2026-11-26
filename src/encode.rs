//! Command encoders: ports of HDAS's `libprotocol_modules.so` (lpm) encode functions, byte for byte.
//!
//! Inputs are `f32` because HDAS takes them from `hdas_msg/MotorControl` (`float32[]` fields) and does every
//! multiply in single precision; outputs are the exact payloads HDAS writes. Each clamp writes the vendor's literal
//! raw value (not `limit x scale`), and each in-range value is truncated toward zero like the vendor's `(int)` cast.
//! Nothing here sends: an [`OutgoingFrame`] is only bytes and an ID.

use serde::Serialize;

use crate::arm::{Side, ARM_JOINTS};
use crate::torso::TORSO_JOINTS;

/// A frame to put on the bus: 11-bit standard ID and CAN FD payload (sent with BRS, see `frame::VENDOR_HOST_FLAGS`).
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct OutgoingFrame {
    pub id: u32,
    pub data: Vec<u8>,
}

/// Left / right arm command IDs (ProtocolArm::encode_Arm lpm@0x11d720).
pub const ARM_COMMAND_ID: [u32; 2] = [0x050, 0x060];
/// Left / right gripper command IDs (ProtocolArm::encode_Gripper lpm@0x11d220).
pub const GRIPPER_COMMAND_ID: [u32; 2] = [0x051, 0x061];
/// Torso command ID (ProtocolTorso::encode lpm@0x1412d0).
pub const TORSO_COMMAND_ID: u32 = 0x041;
/// Chassis command ID (ProtocolChassis::encode_chassis lpm@0x129b40).
pub const CHASSIS_COMMAND_ID: u32 = 0x031;
/// LED command ID (ProtocolCCU::encode_command_led lpm@0x125454).
pub const LED_COMMAND_ID: u32 = 0x022;
/// Function-frame IDs: torso / left arm / right arm (encode_command_torso/left/right lpm@0x125db0/0x125830/0x125af0).
pub const FUNCTION_FRAME_ID: [u32; 3] = [0x043, 0x053, 0x063];
/// Chassis auto-calibration ID (encode_command_chassis_auto_calibration lpm@0x126070).
pub const CHASSIS_AUTO_CALIBRATION_ID: u32 = 0x034;
/// The auto-calibration payload byte; the vendor ignores its argument (`movi v0.16b,#0x20` lpm@0x1260dc).
pub const CHASSIS_AUTO_CALIBRATION_BYTE: u8 = 0x20;

/// Arm frame length (operator_new(0x40) in encode_Arm lpm@0x11d720's a2 branch); bytes 56..63 stay zero.
pub const ARM_COMMAND_LEN: usize = 64;
/// Gripper frame length (operator_new(10) in encode_Gripper lpm@0x11d220).
pub const GRIPPER_COMMAND_LEN: usize = 10;
/// Torso frame length (operator_new(0x10) in ProtocolTorso::encode lpm@0x1412d0).
pub const TORSO_COMMAND_LEN: usize = 16;
/// Chassis frame length (operator_new(0x14) in encode_chassis lpm@0x129b40); bytes 18..19 stay zero.
pub const CHASSIS_COMMAND_LEN: usize = 20;

impl Side {
    fn index(self) -> usize {
        match self {
            Side::Left => 0,
            Side::Right => 1,
        }
    }
}

/// `(int)(value * scale)` as the vendor's two bytes: f32 multiply, truncation toward zero, low 16 bits big-endian.
fn truncated_be(value: f32, scale: f32) -> [u8; 2] {
    ((value * scale) as i32 as u16).to_be_bytes()
}

/// The vendor's common clamp: `high < value` writes `high_raw`, `value < -high` writes `low_raw`, else truncate.
fn clamped_be(value: f32, scale: f32, high: f32, high_raw: u16, low_raw: u16) -> [u8; 2] {
    if high < value {
        high_raw.to_be_bytes()
    } else if value < -high {
        low_raw.to_be_bytes()
    } else {
        truncated_be(value, scale)
    }
}

/// The vendor's gain clamp: `high < value` writes `high_raw`, `value <= 0` writes 0, else truncate.
fn gain_be(value: f32, scale: f32, high: f32, high_raw: u16) -> [u8; 2] {
    if high < value {
        high_raw.to_be_bytes()
    } else if value <= 0.0 {
        [0, 0]
    } else {
        truncated_be(value, scale)
    }
}

/// One joint of an `hdas_msg/MotorControl` arm command, as HDAS hands it to the encoder (kp / kd are raw integers).
#[derive(Clone, Copy, Debug, PartialEq, Default, Serialize)]
pub struct ArmJointTarget {
    pub position_rad: f32,
    pub velocity_rad_s: f32,
    pub kp: f32,
    pub kd: f32,
    pub torque_nm: f32,
}

/// a2 arm position: `6.28 < p` -> 0x752F, `p < -6.28` -> 0x8AD1, else p x 4777 (arm_encode_impl lpm@0x11f950).
const ARM_POSITION_LIMIT_RAD: f32 = 6.28;
const ARM_POSITION_SCALE: f32 = 4777.0;
const ARM_POSITION_HIGH_RAW: u16 = 0x752F;
const ARM_POSITION_LOW_RAW: u16 = 0x8AD1;
/// a2 arm velocity: 12-bit sign-magnitude, `8 < |v|` -> magnitude 0x7D0, else |v| x 250 (arm_encode_impl lpm@0x11f950).
const ARM_VELOCITY_LIMIT_RAD_S: f32 = 8.0;
const ARM_VELOCITY_SCALE: f32 = 250.0;
/// a2 arm feed-forward torque: 12-bit sign-magnitude, `100 < |t|` -> 0x7D0, else |t| x 20 (arm_encode_impl lpm@0x11f950).
const ARM_TORQUE_LIMIT_NM: f32 = 100.0;
const ARM_TORQUE_SCALE: f32 = 20.0;
/// The saturated 11-bit magnitude both sign-magnitude fields write (0x7D / 0xFD high bytes, arm_encode_impl).
const ARM_SATURATED_MAGNITUDE: u16 = 0x7D0;
/// a2 arm gains: raw 12-bit integers, `4095 < k` -> 0xFFF (arm_encode_impl lpm@0x11f950).
const ARM_GAIN_LIMIT: f32 = 4095.0;
const ARM_GAIN_SATURATED: u16 = 0xFFF;
/// 12-bit sign bit (arm_encode_impl lpm@0x11f950 `| 0x800`).
const SIGN_BIT_12: u16 = 0x800;

/// Sign-magnitude 12-bit field: the vendor tests `value < 0` for the sign and `limit < |value|` for saturation.
fn arm_sign_magnitude(value: f32, scale: f32, limit: f32) -> u16 {
    let magnitude = value.abs();
    let bits = if limit < magnitude { ARM_SATURATED_MAGNITUDE } else { (magnitude * scale) as u32 as u16 & 0x7FF };
    if value < 0.0 {
        bits | SIGN_BIT_12
    } else {
        bits
    }
}

/// kp: `4095 < kp` -> 0xFFF, `0 <= kp` -> (int)kp, else (negative or NaN) 0 (arm_encode_impl lpm@0x11f950).
fn arm_kp(kp: f32) -> u16 {
    if ARM_GAIN_LIMIT < kp {
        ARM_GAIN_SATURATED
    } else if 0.0 <= kp {
        kp as i32 as u16 & 0xFFF
    } else {
        0
    }
}

/// kd: `4095 < kd` -> 0xFFF, `kd < 0` -> 0, else (int)kd (NaN truncates to 0) (arm_encode_impl lpm@0x11f950).
fn arm_kd(kd: f32) -> u16 {
    if ARM_GAIN_LIMIT < kd {
        ARM_GAIN_SATURATED
    } else if kd < 0.0 {
        0
    } else {
        kd as i32 as u16 & 0xFFF
    }
}

/// Port of `a2_arm_analysis::arm_encode_impl` lpm@0x11f950: joint j at byte 8j, p i16 | v 12 | kp 12 | kd 12 | t 12.
pub fn encode_arm(side: Side, joints: &[ArmJointTarget; ARM_JOINTS]) -> OutgoingFrame {
    let mut data = vec![0u8; ARM_COMMAND_LEN];
    for (joint, target) in joints.iter().enumerate() {
        let position = clamped_be(
            target.position_rad,
            ARM_POSITION_SCALE,
            ARM_POSITION_LIMIT_RAD,
            ARM_POSITION_HIGH_RAW,
            ARM_POSITION_LOW_RAW,
        );
        let velocity = arm_sign_magnitude(target.velocity_rad_s, ARM_VELOCITY_SCALE, ARM_VELOCITY_LIMIT_RAD_S);
        let kp = arm_kp(target.kp);
        let kd = arm_kd(target.kd);
        let torque = arm_sign_magnitude(target.torque_nm, ARM_TORQUE_SCALE, ARM_TORQUE_LIMIT_NM);
        let bytes = &mut data[8 * joint..8 * joint + 8];
        bytes[0..2].copy_from_slice(&position);
        bytes[2] = (velocity >> 4) as u8;
        bytes[3] = ((velocity & 0xF) << 4) as u8 | (kp >> 8) as u8;
        bytes[4] = kp as u8;
        bytes[5] = (kd >> 4) as u8;
        bytes[6] = ((kd & 0xF) << 4) as u8 | (torque >> 8) as u8;
        bytes[7] = torque as u8;
    }
    OutgoingFrame { id: ARM_COMMAND_ID[side.index()], data }
}

/// A gripper `hdas_msg/MotorControl` (element 0 of each array) as HDAS hands it to encode_Gripper.
#[derive(Clone, Copy, Debug, PartialEq, Default, Serialize)]
pub struct GripperTarget {
    /// Motor angle (rad); r1_gripper_controller maps an opening percent onto -2.8..0.
    pub position_rad: f32,
    pub velocity: f32,
    pub kp: f32,
    pub kd: f32,
    pub torque: f32,
}

/// encode_Gripper lpm@0x11d220: p x 4700, `6.5 < p` -> 0x7756, `p < -6.5` -> 0x88AA.
const GRIPPER_POSITION_LIMIT: f32 = 6.5;
const GRIPPER_POSITION_SCALE: f32 = 4700.0;
const GRIPPER_POSITION_HIGH_RAW: u16 = 0x7756;
const GRIPPER_POSITION_LOW_RAW: u16 = 0x88AA;
/// ... v x 750, `40 < |v|` -> 0x7530 / 0x8AD0.
const GRIPPER_VELOCITY_LIMIT: f32 = 40.0;
const GRIPPER_VELOCITY_SCALE: f32 = 750.0;
/// ... kp x 60, `500 < kp` -> 0x7530, `kp <= 0` -> 0.
const GRIPPER_KP_LIMIT: f32 = 500.0;
const GRIPPER_KP_SCALE: f32 = 60.0;
/// ... kd x 150, `200 < kd` -> 0x7530, `kd <= 0` -> 0.
const GRIPPER_KD_LIMIT: f32 = 200.0;
const GRIPPER_KD_SCALE: f32 = 150.0;
/// ... t x 600, `50 < |t|` -> 0x7530 / 0x8AD0.
const GRIPPER_TORQUE_LIMIT: f32 = 50.0;
const GRIPPER_TORQUE_SCALE: f32 = 600.0;
/// The +-30000 literals encode_Gripper, encode_torso_impl and chassis_encode_impl write when they saturate.
const SATURATED_HIGH_RAW: u16 = 0x7530;
const SATURATED_LOW_RAW: u16 = 0x8AD0;

/// Port of `ProtocolArm::encode_Gripper` lpm@0x11d220: five big-endian i16s.
pub fn encode_gripper(side: Side, target: &GripperTarget) -> OutgoingFrame {
    let fields = [
        clamped_be(
            target.position_rad,
            GRIPPER_POSITION_SCALE,
            GRIPPER_POSITION_LIMIT,
            GRIPPER_POSITION_HIGH_RAW,
            GRIPPER_POSITION_LOW_RAW,
        ),
        clamped_be(
            target.velocity,
            GRIPPER_VELOCITY_SCALE,
            GRIPPER_VELOCITY_LIMIT,
            SATURATED_HIGH_RAW,
            SATURATED_LOW_RAW,
        ),
        gain_be(target.kp, GRIPPER_KP_SCALE, GRIPPER_KP_LIMIT, SATURATED_HIGH_RAW),
        gain_be(target.kd, GRIPPER_KD_SCALE, GRIPPER_KD_LIMIT, SATURATED_HIGH_RAW),
        clamped_be(target.torque, GRIPPER_TORQUE_SCALE, GRIPPER_TORQUE_LIMIT, SATURATED_HIGH_RAW, SATURATED_LOW_RAW),
    ];
    OutgoingFrame { id: GRIPPER_COMMAND_ID[side.index()], data: fields.concat() }
}

/// One torso joint: target position and speed limit; the TCU frame carries no gains.
#[derive(Clone, Copy, Debug, PartialEq, Default, Serialize)]
pub struct TorsoJointTarget {
    pub position_rad: f32,
    pub velocity_rad_s: f32,
}

/// torso_analysis::encode_torso_impl lpm@0x144ff0: p x 10000, `3.14 < p` -> 0x7AA8, `p < -3.14` -> 0x8558.
#[allow(clippy::approx_constant)] // the vendor's literal 3.14, not pi
const TORSO_POSITION_LIMIT_RAD: f32 = 3.14;
const TORSO_POSITION_SCALE: f32 = 10000.0;
const TORSO_POSITION_HIGH_RAW: u16 = 0x7AA8;
const TORSO_POSITION_LOW_RAW: u16 = 0x8558;
/// ... v x 3000, `10 < |v|` -> 0x7530 / 0x8AD0.
const TORSO_VELOCITY_LIMIT_RAD_S: f32 = 10.0;
const TORSO_VELOCITY_SCALE: f32 = 3000.0;

/// Port of `torso_analysis::encode_torso_impl` lpm@0x144ff0: joint j at 4j, (p, v) big-endian i16s.
pub fn encode_torso(joints: &[TorsoJointTarget; TORSO_JOINTS]) -> OutgoingFrame {
    let mut data = Vec::with_capacity(TORSO_COMMAND_LEN);
    for target in joints {
        data.extend(clamped_be(
            target.position_rad,
            TORSO_POSITION_SCALE,
            TORSO_POSITION_LIMIT_RAD,
            TORSO_POSITION_HIGH_RAW,
            TORSO_POSITION_LOW_RAW,
        ));
        data.extend(clamped_be(
            target.velocity_rad_s,
            TORSO_VELOCITY_SCALE,
            TORSO_VELOCITY_LIMIT_RAD_S,
            SATURATED_HIGH_RAW,
            SATURATED_LOW_RAW,
        ));
    }
    OutgoingFrame { id: TORSO_COMMAND_ID, data }
}

/// Per-wheel chassis targets in the order chassis_control_node fills MotorControl (fl, fr, r).
#[derive(Clone, Copy, Debug, PartialEq, Default, Serialize)]
pub struct ChassisTarget {
    /// v_des[0..2]: drive speed, m/s at the contact patch.
    pub wheel_speed_m_s: [f32; 3],
    /// p_des[0..2]: steer angle.
    pub steer_angle_rad: [f32; 3],
    /// v_des[3..5]: steer rate.
    pub steer_rate_rad_s: [f32; 3],
}

/// chassis_encode_impl lpm@0x12bec0: every field x 1000.
const CHASSIS_SCALE: f32 = 1000.0;
/// ... wheel speed `2 < v` -> 0x07D0, `v < -2` -> 0xF830.
const CHASSIS_WHEEL_SPEED_LIMIT: f32 = 2.0;
const CHASSIS_WHEEL_SPEED_HIGH_RAW: u16 = 0x07D0;
const CHASSIS_WHEEL_SPEED_LOW_RAW: u16 = 0xF830;
/// ... steer angle `30 < p` -> 0x7530, `p < -30` -> 0x8AD0.
const CHASSIS_STEER_ANGLE_LIMIT: f32 = 30.0;
/// ... steer rate `5 < v` -> 0x1388, `v < -5` -> 0xEC78.
const CHASSIS_STEER_RATE_LIMIT: f32 = 5.0;
const CHASSIS_STEER_RATE_HIGH_RAW: u16 = 0x1388;
const CHASSIS_STEER_RATE_LOW_RAW: u16 = 0xEC78;

/// Port of `chassis_analysis::chassis_encode_impl` lpm@0x12bec0: speeds at 0/2/4, angles 6/8/10, rates 12/14/16.
pub fn encode_chassis(target: &ChassisTarget) -> OutgoingFrame {
    let mut data = Vec::with_capacity(CHASSIS_COMMAND_LEN);
    for speed in target.wheel_speed_m_s {
        data.extend(clamped_be(
            speed,
            CHASSIS_SCALE,
            CHASSIS_WHEEL_SPEED_LIMIT,
            CHASSIS_WHEEL_SPEED_HIGH_RAW,
            CHASSIS_WHEEL_SPEED_LOW_RAW,
        ));
    }
    for angle in target.steer_angle_rad {
        data.extend(clamped_be(angle, CHASSIS_SCALE, CHASSIS_STEER_ANGLE_LIMIT, SATURATED_HIGH_RAW, SATURATED_LOW_RAW));
    }
    for rate in target.steer_rate_rad_s {
        data.extend(clamped_be(
            rate,
            CHASSIS_SCALE,
            CHASSIS_STEER_RATE_LIMIT,
            CHASSIS_STEER_RATE_HIGH_RAW,
            CHASSIS_STEER_RATE_LOW_RAW,
        ));
    }
    data.resize(CHASSIS_COMMAND_LEN, 0);
    OutgoingFrame { id: CHASSIS_COMMAND_ID, data }
}

/// Port of `ProtocolCCU::encode_command_led` lpm@0x125454: `[r, g, b, mode]` straight from hdas_msg/LEDControl.
pub fn encode_led(r: u8, g: u8, b: u8, mode: u8) -> OutgoingFrame {
    OutgoingFrame { id: LED_COMMAND_ID, data: vec![r, g, b, mode] }
}

/// Which board a function frame addresses.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FunctionBoard {
    Torso,
    LeftArm,
    RightArm,
}

/// HDAS's function-frame services accept command 1..10 (`cmp (cmd-1), #9` in functionFrameCallback* HDAS@0x14ea00..).
pub const FUNCTION_FRAME_COMMANDS: std::ops::RangeInclusive<u8> = 1..=10;

/// Port of `ProtocolCCU::encode_command_torso/left/right` lpm@0x125db0/0x125830/0x125af0 plus the HDAS service's
/// 1..10 check; `None` where HDAS rejects the request. Code meanings are unknown (MCU firmware).
pub fn encode_function_frame(board: FunctionBoard, command: u8) -> Option<OutgoingFrame> {
    if !FUNCTION_FRAME_COMMANDS.contains(&command) {
        return None;
    }
    let id = match board {
        FunctionBoard::Torso => FUNCTION_FRAME_ID[0],
        FunctionBoard::LeftArm => FUNCTION_FRAME_ID[1],
        FunctionBoard::RightArm => FUNCTION_FRAME_ID[2],
    };
    Some(OutgoingFrame { id, data: vec![command] })
}

/// Port of `ProtocolCCU::encode_command_chassis_auto_calibration` lpm@0x126070: always `[0x20]` on 0x034.
pub fn encode_chassis_auto_calibration() -> OutgoingFrame {
    OutgoingFrame { id: CHASSIS_AUTO_CALIBRATION_ID, data: vec![CHASSIS_AUTO_CALIBRATION_BYTE] }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arm::{ArmCommand, GripperCommand};
    use crate::chassis::ChassisCommand;
    use crate::torso::TorsoCommand;

    fn joint(position_rad: f32, velocity_rad_s: f32, kp: f32, kd: f32, torque_nm: f32) -> ArmJointTarget {
        ArmJointTarget { position_rad, velocity_rad_s, kp, kd, torque_nm }
    }

    #[test]
    fn arm_clamps_write_the_vendor_literals() {
        let mut joints = [ArmJointTarget::default(); ARM_JOINTS];
        joints[0] = joint(7.0, 9.0, 5000.0, 5000.0, 101.0);
        joints[1] = joint(-7.0, -9.0, -1.0, -1.0, -101.0);
        joints[2] = joint(f32::NAN, f32::NAN, f32::NAN, f32::NAN, f32::NAN);
        let data = encode_arm(Side::Left, &joints).data;
        assert_eq!(&data[0..8], &[0x75, 0x2F, 0x7D, 0x0F, 0xFF, 0xFF, 0xF7, 0xD0]);
        assert_eq!(&data[8..16], &[0x8A, 0xD1, 0xFD, 0x00, 0x00, 0x00, 0x0F, 0xD0]);
        assert_eq!(&data[16..24], &[0; 8]);
        assert_eq!(data.len(), 64);
        assert_eq!(&data[56..], &[0; 8]);
    }

    #[test]
    fn arm_hold_command_round_trips() {
        let joints = [joint(-0.5, -0.4, 2.0, 25.0, -3.45); ARM_JOINTS];
        let frame = encode_arm(Side::Right, &joints);
        assert_eq!(frame.id, 0x060);
        let decoded = ArmCommand::decode(Side::Right, &frame.data).unwrap();
        for joint in decoded.joints {
            assert_eq!((joint.kp_raw, joint.kd_raw), (2, 25));
            assert!((joint.position_rad + 0.5).abs() < 1.0 / 4777.0);
            assert!((joint.velocity_rad_s + 0.4).abs() < 1.0 / 250.0 + 1e-9);
            assert!((joint.torque_nm + 3.45).abs() < 1.0 / 20.0 + 1e-9);
        }
    }

    #[test]
    fn gripper_clamps_and_round_trip() {
        let open = GripperTarget { position_rad: -2.8, velocity: 0.0, kp: 20.0, kd: 10.0, torque: 0.0 };
        assert_eq!(encode_gripper(Side::Right, &open).data, [0xCC, 0x98, 0, 0, 0x04, 0xB0, 0x05, 0xDC, 0, 0]);
        let wild = GripperTarget { position_rad: 7.0, velocity: -41.0, kp: 501.0, kd: 0.0, torque: 51.0 };
        assert_eq!(encode_gripper(Side::Left, &wild).data, [0x77, 0x56, 0x8A, 0xD0, 0x75, 0x30, 0, 0, 0x75, 0x30]);
        let low = GripperTarget { position_rad: -7.0, velocity: 41.0, kp: -1.0, kd: 201.0, torque: -51.0 };
        assert_eq!(encode_gripper(Side::Left, &low).data, [0x88, 0xAA, 0x75, 0x30, 0, 0, 0x75, 0x30, 0x8A, 0xD0]);
        let decoded = GripperCommand::decode(Side::Right, &encode_gripper(Side::Right, &open).data).unwrap();
        assert_eq!((decoded.motor_position_rad, decoded.kp, decoded.kd), (-2.8, 20.0, 10.0));
    }

    #[test]
    fn torso_clamps_and_round_trip() {
        let joints = [
            TorsoJointTarget { position_rad: 4.0, velocity_rad_s: 11.0 },
            TorsoJointTarget { position_rad: -4.0, velocity_rad_s: -11.0 },
            TorsoJointTarget { position_rad: 0.1565, velocity_rad_s: 0.6 },
            TorsoJointTarget { position_rad: -0.3656, velocity_rad_s: 1.0 },
        ];
        let data = encode_torso(&joints).data;
        assert_eq!(&data[0..8], &[0x7A, 0xA8, 0x75, 0x30, 0x85, 0x58, 0x8A, 0xD0]);
        let decoded = TorsoCommand::decode(&data).unwrap();
        assert!((decoded.position_rad[2] - 0.1565).abs() <= 1e-4 && (decoded.velocity_rad_s[3] - 1.0).abs() <= 1e-3);
    }

    #[test]
    fn chassis_clamps_and_round_trip() {
        let target = ChassisTarget {
            wheel_speed_m_s: [2.5, -2.5, 0.25],
            steer_angle_rad: [31.0, -31.0, -0.5],
            steer_rate_rad_s: [6.0, -6.0, 1.0],
        };
        let data = encode_chassis(&target).data;
        assert_eq!(&data[0..4], &[0x07, 0xD0, 0xF8, 0x30]);
        assert_eq!(&data[6..10], &[0x75, 0x30, 0x8A, 0xD0]);
        assert_eq!(&data[12..16], &[0x13, 0x88, 0xEC, 0x78]);
        assert_eq!(data.len(), 20);
        let decoded = ChassisCommand::decode(&data).unwrap();
        assert_eq!((decoded.wheel_speed_m_s[2], decoded.steer_angle_rad[2], decoded.steer_rate_rad_s[2]), (0.25, -0.5, 1.0));
    }

    #[test]
    fn small_frames() {
        assert_eq!(encode_led(1, 2, 3, 1), OutgoingFrame { id: 0x022, data: vec![1, 2, 3, 1] });
        assert_eq!(encode_function_frame(FunctionBoard::RightArm, 3).unwrap(), OutgoingFrame { id: 0x063, data: vec![3] });
        assert!(encode_function_frame(FunctionBoard::Torso, 0).is_none());
        assert!(encode_function_frame(FunctionBoard::Torso, 11).is_none());
        assert_eq!(encode_chassis_auto_calibration().data, [0x20]);
    }
}
