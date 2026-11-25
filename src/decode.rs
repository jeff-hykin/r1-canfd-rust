//! [`Frame`] -> [`Decoded`]: one variant per kind of frame on the bus. Nothing is dropped: an ID with no decoder is
//! `Unknown`, a known ID too short for its layout is `Malformed`.

use serde::Serialize;

use crate::arm::{ArmCommand, ArmFeedback, GripperCommand, GripperFeedback, MotorTemperatures, Side, Wrench};
use crate::chassis::{ChassisCommand, MotorStatus, Remote};
use crate::frame::Frame;
use crate::hand::{Hand, HAND_LEN};
use crate::imu::{Imu, ImuLocation};
use crate::power::Bms;
use crate::status::{AcuVersion, ArmStatus, Board, EmergencyStop, ErrorReport, Version};
use crate::torso::{TorsoCommand, TorsoFeedback};

/// 0x034 with this length is the auto-calibration request, not VCU_ERROR (encode_command_chassis_auto_calibration
/// lpm@0x126070).
pub const AUTO_CALIBRATION_LEN: usize = 1;
/// VCU_ERROR's length: 6 u32 words (decode_vcu_error lpm@0x133100).
pub const VCU_ERROR_LEN: usize = 24;
/// TORSO_ERROR's words: 4 u32s (decode_tcu_error lpm@0x131cf0).
pub const TORSO_ERROR_LEN: usize = 16;
/// CHASSIS_ERROR's words: 6 u16s (decode_chassis_error lpm@0x1326a0).
pub const CHASSIS_ERROR_LEN: usize = 12;
/// LED command [r, g, b, mode] (ProtocolCCU::encode_command_led lpm@0x125454).
pub const LED_LEN: usize = 4;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FunctionTarget {
    Torso,
    LeftArm,
    RightArm,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum Decoded {
    EmergencyStop(EmergencyStop),
    Imu(Imu),
    Bms(Bms),
    MotorStatus(MotorStatus),
    Remote(Remote),
    Errors(ErrorReport),
    Version {
        board: Board,
        version: Version,
    },
    TorsoFeedback(TorsoFeedback),
    ArmFeedback(ArmFeedback),
    ArmStatus(ArmStatus),
    AcuVersion(AcuVersion),
    Hand(Hand),
    Wrench(Wrench),
    GripperFeedback(GripperFeedback),
    MotorTemperatures(MotorTemperatures),
    /// Commands other nodes put on the bus, decoded as observed traffic.
    ChassisCommand(ChassisCommand),
    TorsoCommand(TorsoCommand),
    ArmCommand(ArmCommand),
    GripperCommand(GripperCommand),
    HandCommand(Hand),
    Led {
        r: u8,
        g: u8,
        b: u8,
        mode: u8,
    },
    /// 0x043 / 0x053 / 0x063, one byte 1..10 (ProtocolCCU::encode_command_torso/left/right lpm@0x125db0/830/af0).
    FunctionFrame {
        target: FunctionTarget,
        command: u8,
    },
    /// 0x034, one byte (the vendor always sends 0x20).
    ChassisAutoCalibration {
        command: u8,
    },
    Unknown {
        id: u32,
        data: Vec<u8>,
    },
    Malformed {
        id: u32,
        data: Vec<u8>,
        needed: usize,
    },
}

/// Decode one frame. 29-bit IDs are never this bus's, so they come back `Unknown`.
pub fn decode(frame: &Frame) -> Decoded {
    if frame.extended {
        return Decoded::Unknown { id: frame.id, data: frame.data.clone() };
    }
    decode_payload(frame.id, &frame.data)
}

/// Decode a standard-ID payload.
pub fn decode_payload(id: u32, data: &[u8]) -> Decoded {
    let malformed = |needed| Decoded::Malformed { id, data: data.to_vec(), needed };
    let side = |left: u32| if id == left { Side::Left } else { Side::Right };
    match id {
        0x001 => Decoded::EmergencyStop(EmergencyStop { data: data.to_vec() }),
        0x020 => Imu::decode(ImuLocation::Chassis, data).map(Decoded::Imu).unwrap_or_else(|| malformed(Imu::LEN)),
        0x045 => Imu::decode(ImuLocation::Torso, data).map(Decoded::Imu).unwrap_or_else(|| malformed(Imu::LEN)),
        0x021 => Bms::decode(data).map(Decoded::Bms).unwrap_or_else(|| malformed(Bms::LEN)),
        0x022 => match *data {
            [r, g, b, mode, ..] => Decoded::Led { r, g, b, mode },
            _ => malformed(LED_LEN),
        },
        0x025 | 0x035 | 0x046 => {
            let board = match id {
                0x025 => Board::Ccu,
                0x035 => Board::Vcu,
                _ => Board::Tcu,
            };
            Version::decode(data)
                .map(|version| Decoded::Version { board, version })
                .unwrap_or_else(|| malformed(Version::LEN))
        }
        0x030 => MotorStatus::decode(data).map(Decoded::MotorStatus).unwrap_or_else(|| malformed(MotorStatus::LEN)),
        0x031 => {
            ChassisCommand::decode(data).map(Decoded::ChassisCommand).unwrap_or_else(|| malformed(ChassisCommand::LEN))
        }
        0x032 => Remote::decode(data).map(Decoded::Remote).unwrap_or_else(|| malformed(Remote::LEN)),
        0x033 => ErrorReport::decode_chassis(data).map(Decoded::Errors).unwrap_or_else(|| malformed(CHASSIS_ERROR_LEN)),
        0x034 if data.len() == AUTO_CALIBRATION_LEN => Decoded::ChassisAutoCalibration { command: data[0] },
        0x034 => ErrorReport::decode_vcu(data).map(Decoded::Errors).unwrap_or_else(|| malformed(VCU_ERROR_LEN)),
        0x040 => {
            TorsoFeedback::decode(data).map(Decoded::TorsoFeedback).unwrap_or_else(|| malformed(TorsoFeedback::LEN))
        }
        0x041 => TorsoCommand::decode(data).map(Decoded::TorsoCommand).unwrap_or_else(|| malformed(TorsoCommand::LEN)),
        0x043 | 0x053 | 0x063 => {
            let target = match id {
                0x043 => FunctionTarget::Torso,
                0x053 => FunctionTarget::LeftArm,
                _ => FunctionTarget::RightArm,
            };
            data.first().map(|&command| Decoded::FunctionFrame { target, command }).unwrap_or_else(|| malformed(1))
        }
        0x044 => ErrorReport::decode_torso(data).map(Decoded::Errors).unwrap_or_else(|| malformed(TORSO_ERROR_LEN)),
        0x050 | 0x060 => {
            ArmCommand::decode(side(0x050), data).map(Decoded::ArmCommand).unwrap_or_else(|| malformed(ArmCommand::LEN))
        }
        0x051 | 0x061 => GripperCommand::decode(side(0x051), data)
            .map(Decoded::GripperCommand)
            .unwrap_or_else(|| malformed(GripperCommand::LEN)),
        0x052 | 0x062 => ArmFeedback::decode(side(0x052), data)
            .map(Decoded::ArmFeedback)
            .unwrap_or_else(|| malformed(ArmFeedback::LEN)),
        0x054 | 0x064 => ArmStatus::decode(side(0x054), data).map(Decoded::ArmStatus).unwrap_or_else(|| malformed(2)),
        0x055 | 0x065 if data.len() == HAND_LEN => {
            Hand::decode(side(0x055), data).map(Decoded::HandCommand).unwrap_or_else(|| malformed(HAND_LEN))
        }
        0x055 | 0x065 => {
            AcuVersion::decode(side(0x055), data).map(Decoded::AcuVersion).unwrap_or_else(|| malformed(Version::LEN))
        }
        0x056 | 0x066 => Hand::decode(side(0x056), data).map(Decoded::Hand).unwrap_or_else(|| malformed(HAND_LEN)),
        0x057 | 0x067 => {
            Wrench::decode(side(0x057), data).map(Decoded::Wrench).unwrap_or_else(|| malformed(Wrench::LEN))
        }
        0x058 | 0x068 => GripperFeedback::decode(side(0x058), data)
            .map(Decoded::GripperFeedback)
            .unwrap_or_else(|| malformed(GripperFeedback::LEN)),
        0x05D | 0x06D => MotorTemperatures::decode(side(0x05D), data)
            .map(Decoded::MotorTemperatures)
            .unwrap_or_else(|| malformed(MotorTemperatures::LEN)),
        _ => Decoded::Unknown { id, data: data.to_vec() },
    }
}

impl Decoded {
    /// The serde tag ("imu", "arm_feedback", ...).
    pub fn kind(&self) -> &'static str {
        match self {
            Decoded::EmergencyStop(_) => "emergency_stop",
            Decoded::Imu(_) => "imu",
            Decoded::Bms(_) => "bms",
            Decoded::MotorStatus(_) => "motor_status",
            Decoded::Remote(_) => "remote",
            Decoded::Errors(_) => "errors",
            Decoded::Version { .. } => "version",
            Decoded::TorsoFeedback(_) => "torso_feedback",
            Decoded::ArmFeedback(_) => "arm_feedback",
            Decoded::ArmStatus(_) => "arm_status",
            Decoded::AcuVersion(_) => "acu_version",
            Decoded::Hand(_) => "hand",
            Decoded::Wrench(_) => "wrench",
            Decoded::GripperFeedback(_) => "gripper_feedback",
            Decoded::MotorTemperatures(_) => "motor_temperatures",
            Decoded::ChassisCommand(_) => "chassis_command",
            Decoded::TorsoCommand(_) => "torso_command",
            Decoded::ArmCommand(_) => "arm_command",
            Decoded::GripperCommand(_) => "gripper_command",
            Decoded::HandCommand(_) => "hand_command",
            Decoded::Led { .. } => "led",
            Decoded::FunctionFrame { .. } => "function_frame",
            Decoded::ChassisAutoCalibration { .. } => "chassis_auto_calibration",
            Decoded::Unknown { .. } => "unknown",
            Decoded::Malformed { .. } => "malformed",
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nothing_is_dropped() {
        assert_eq!(decode_payload(0x023, &[2]), Decoded::Unknown { id: 0x023, data: vec![2] });
        assert_eq!(decode_payload(0x052, &[0; 10]), Decoded::Malformed { id: 0x052, data: vec![0; 10], needed: 42 });
        assert!(matches!(decode_payload(0x001, &[]), Decoded::EmergencyStop(_)));
        assert_eq!(decode_payload(0x034, &[0x20]), Decoded::ChassisAutoCalibration { command: 0x20 });
        let frame = Frame { timestamp_ns: 0, id: 0x020, extended: true, flags: 0, data: vec![0; 20] };
        assert!(matches!(decode(&frame), Decoded::Unknown { .. }));
    }
}
