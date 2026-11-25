//! Safety and housekeeping frames: the 0x001 e-stop, the error words (0x033 chassis, 0x034 VCU, 0x044 torso,
//! 0x054 / 0x064 arms) with the vendor's bit tables, and the board / motor versions.
//!
//! Tables and name maps come from `error_analysis`'s static init (lpm `__static_initialization_and_destruction_0`
//! @0x1124a0, `_GLOBAL__sub_I_protocol_error.cpp`); the bit order is from the binary only, no live fault confirmed it.

use serde::Serialize;

use crate::arm::Side;
use crate::bytes::{array, tail, u16_be, u32_be};

/// `ChassiserrorCodeToString`, bit k -> name.
pub const CHASSIS_ERROR_BITS: [&str; 11] = [
    "MOTOR_DISCONNECT",
    "RECEIVE_TIMEOUT",
    "POS_EXCEED",
    "VEL_EXCEED",
    "TOR_EXCEED",
    "OVER_VOLTAGE",
    "UNDER_VOLTAGE",
    "OVER_CURRENT",
    "MOS_OVER_TEMP",
    "MOTOR_OVER_TEMP",
    "EMERGENCY_STOP",
];
/// Bit of a chassis error word that means the e-stop is pressed (`ChassiserrorCodeToString[10]`).
pub const CHASSIS_EMERGENCY_STOP_BIT: u32 = 10;
/// `VcuErrorCodeToString`.
pub const VCU_ERROR_BITS: [&str; 26] = [
    "DISCONNECT",
    "BUS_OVERVOLTAGE",
    "BUS_UNDERVOLTAGE",
    "ENCODER_ELECTRICAL_ANGLE_FAULT",
    "DRV_DRIVER_MALFUNCTION",
    "BUS_OVERCURRENT",
    "A_PHASE_OVERCURRENT",
    "B_PHASE_OVERCURRENT",
    "C_PHASE_OVERCURRENT",
    "BOARD_OVER_TEMPERATURE",
    "COIL_TEMPERATURE",
    "ENCODER_FAILED",
    "CURRENT_SENSOR_MALFUNCTION",
    "BLOCKAGE_PROTECTION",
    "ANGLE_OVER_LIMIT",
    "VEL_OVER_LIMIT",
    "CAN_COMMUNICATION_LOST",
    "ENCODER_FRAME_HEADER_VERIFICATION_FAILED",
    "ENCODER_MULTI_TURN_FAULT",
    "ENCODER_MULTI_TURN_INTERFACE_FAULT",
    "ENCODER_EXTERNAL_INPUT_FAULT",
    "ENCODER_READING_FAULT",
    "ENCODER_ERR_OFFS",
    "ENCODER_ERR_CFG",
    "ILLEGAL_FIRMWARE",
    "GATE_DRIVER_INITIALIZATION_FAILED",
];
/// `TorsoerrorCodeToString`.
pub const TORSO_ERROR_BITS: [&str; 31] = [
    "MOTOR_DISABLE",
    "DISCONNECT",
    "POS_JUMP",
    "RECEIVE_TIMEOUT",
    "POS_EXCEED",
    "BUS_OVERVOLTAGE",
    "BUS_UNDERVOLTAGE",
    "ENCODER_ELECTRICAL_ANGLE_FAULT",
    "DRV_DRIVER_MALFUNCTION",
    "BUS_OVERCURRENT",
    "A_PHASE_OVERCURRENT",
    "B_PHASE_OVERCURRENT",
    "C_PHASE_OVERCURRENT",
    "BOARD_OVER_TEMPERATURE",
    "COIL_TEMPERATURE",
    "ENCODER_FAILED",
    "CURRENT_SENSOR_MALFUNCTION",
    "BLOCKAGE_PROTECTION",
    "MOTOR_ANGLE_OVER_LIMIT",
    "MOTOR_VEL_OVER_LIMIT",
    "CAN_COMMUNICATION_LOST",
    "ENCODER_FRAME_HEADER_VERIFICATION_FAILED",
    "ENCODER_MULTI_TURN_FAULT",
    "ENCODER_MULTI_TURN_INTERFACE_FAULT",
    "ENCODER_EXTERNAL_INPUT_FAULT",
    "ENCODER_READING_FAULT",
    "ENCODER_ERR_OFFS",
    "ENCODER_ERR_CFG",
    "ILLEGAL_FIRMWARE",
    "GATE_DRIVER_INITIALIZATION_FAILED",
    "ZERO_POINT_DRIFT",
];
/// `ArmerrorCodeToString`.
pub const ARM_ERROR_BITS: [&str; 16] = [
    "MOTOR_DISABLE",
    "DISCONNECT",
    "POS_JUMP",
    "TOR_JUMP",
    "RECEIVE_TIMEOUT",
    "POS_EXCEED",
    "VEL_EXCEED",
    "TOR_EXCEED",
    "OVER_VOLTAGE",
    "UNDER_VOLTAGE",
    "OVER_CURRENT",
    "MOS_OVER_TEMP",
    "MOTOR_OVER_TEMP",
    "COMMUNICATION_LOSS",
    "OVERLOAD",
    "ENCODER_ERR",
];
/// The short (non-64-byte) arm error layout only reads bits 0..8 (decode_acu_error lpm@0x12f0d0 loops to 9).
pub const ARM_SHORT_LAYOUT_BITS: usize = 9;

/// `chassis_name_map`.
pub const CHASSIS_MOTOR_NAMES: [&str; 6] =
    ["wheel_motor_fl", "wheel_motor_fr", "wheel_motor_r", "steer_motor_fl", "steer_motor_fr", "steer_motor_r"];
/// `torso_name_map`; decode_vcu_error lpm@0x133100 indexes this one too (sic), so VCU motors are w1..w6.
pub const TORSO_MOTOR_NAMES: [&str; 6] = ["w1", "w2", "w3", "w4", "w5", "w6"];
/// `arm2_name_map` (the first seven are `arm_name_map`).
pub const ARM_MOTOR_NAMES: [&str; 8] =
    ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5", "arm_joint6", "arm_joint7", "gripper"];

/// One motor's error word and the names of its set bits.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct MotorError {
    pub motor: &'static str,
    pub code: u32,
    pub errors: Vec<&'static str>,
}

impl MotorError {
    fn new(motor: &'static str, code: u32, table: &[&'static str]) -> MotorError {
        let errors = table.iter().enumerate().filter(|(bit, _)| code >> bit & 1 == 1).map(|(_, name)| *name).collect();
        MotorError { motor, code, errors }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ErrorSource {
    /// 0x033 CHASSIS_ERROR, event-driven.
    Chassis,
    /// 0x034 VCU_ERROR, 1 Hz.
    Vcu,
    /// 0x044 TORSO_ERROR, 1 Hz.
    Torso,
}

/// 0x033 / 0x034 / 0x044: one error word per motor.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ErrorReport {
    pub source: ErrorSource,
    pub motors: Vec<MotorError>,
    /// Bytes after the error words, unread by HDAS (0x044 carries a constant `7c260024`).
    pub unread_tail: Vec<u8>,
}

impl ErrorReport {
    /// `error_analysis::decode_chassis_error` lpm@0x1326a0: 6 big-endian u16s.
    pub fn decode_chassis(data: &[u8]) -> Option<ErrorReport> {
        let motors = (0..6)
            .map(|i| {
                Some(MotorError::new(CHASSIS_MOTOR_NAMES[i], u32::from(u16_be(data, 2 * i)?), &CHASSIS_ERROR_BITS))
            })
            .collect::<Option<_>>()?;
        Some(ErrorReport { source: ErrorSource::Chassis, motors, unread_tail: tail(data, 12) })
    }

    /// `error_analysis::decode_vcu_error` lpm@0x133100: 6 big-endian u32s.
    pub fn decode_vcu(data: &[u8]) -> Option<ErrorReport> {
        Self::decode_u32_words(ErrorSource::Vcu, data, &TORSO_MOTOR_NAMES, &VCU_ERROR_BITS)
    }

    /// `error_analysis::decode_tcu_error` lpm@0x131cf0: 4 big-endian u32s (w1..w4).
    pub fn decode_torso(data: &[u8]) -> Option<ErrorReport> {
        Self::decode_u32_words(ErrorSource::Torso, data, &TORSO_MOTOR_NAMES[..4], &TORSO_ERROR_BITS)
    }

    fn decode_u32_words(
        source: ErrorSource,
        data: &[u8],
        names: &[&'static str],
        table: &[&'static str],
    ) -> Option<ErrorReport> {
        let motors = names
            .iter()
            .enumerate()
            .map(|(i, name)| Some(MotorError::new(name, u32_be(data, 4 * i)?, table)))
            .collect::<Option<_>>()?;
        Some(ErrorReport { source, motors, unread_tail: tail(data, 4 * names.len()) })
    }

    pub fn any_error(&self) -> bool {
        self.motors.iter().any(|motor| motor.code != 0)
    }

    /// Chassis EMERGENCY_STOP bit set on any motor (only 0x033 carries it).
    pub fn emergency_stop(&self) -> bool {
        self.source == ErrorSource::Chassis
            && self.motors.iter().any(|motor| motor.code >> CHASSIS_EMERGENCY_STOP_BIT & 1 == 1)
    }
}

/// A board version: each byte as decimal, 0xff read as 0 (`chassis_analysis::decode_version` lpm@0x12c2b0 and
/// its torso / ccu twins).
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct Version {
    pub hardware_raw: [u8; 3],
    pub software_raw: [u8; 3],
    /// "v{b0}.{b1}.{b2}"
    pub hardware: String,
    pub software: String,
}

fn version_string(raw: [u8; 3]) -> String {
    let part = |byte: u8| if byte == 0xFF { 0 } else { byte };
    format!("v{}.{}.{}", part(raw[0]), part(raw[1]), part(raw[2]))
}

impl Version {
    pub const LEN: usize = 6;

    pub fn decode(data: &[u8]) -> Option<Version> {
        Self::at(data, 0)
    }

    fn at(data: &[u8], offset: usize) -> Option<Version> {
        let hardware_raw = array::<3>(data, offset)?;
        let software_raw = array::<3>(data, offset + 3)?;
        Some(Version {
            hardware_raw,
            software_raw,
            hardware: version_string(hardware_raw),
            software: version_string(software_raw),
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Board {
    /// 0x025 CCU_VERSION (not seen in the capture; maybe boot only).
    Ccu,
    /// 0x035 VCU_VERSION, 2 Hz.
    Vcu,
    /// 0x046 TCU_VERSION, 1 Hz.
    Tcu,
}

/// 0x055 / 0x065 ACU_VERSION, 8 bytes (`a1_arm_analysis::version_decode` lpm@0x122180; not seen on this robot).
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct AcuVersion {
    pub side: Side,
    pub version: Version,
    /// Byte 7: 1 "MIT", 2 "PID", else "UNKNOWN".
    pub control_mode: Option<u8>,
}

impl AcuVersion {
    pub fn decode(side: Side, data: &[u8]) -> Option<AcuVersion> {
        Some(AcuVersion { side, version: Version::decode(data)?, control_mode: data.get(7).copied() })
    }

    pub fn control_mode_name(&self) -> &'static str {
        match self.control_mode {
            Some(1) => "MIT",
            Some(2) => "PID",
            _ => "UNKNOWN",
        }
    }
}

/// One arm motor's firmware version packed in 3 bytes (decode_acu_error lpm@0x12f0d0, bytes 40+3i).
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct MotorVersion {
    pub motor: &'static str,
    /// (b0 >> 4, b0 & 15, b1)
    pub software: [u8; 3],
    /// (b2 >> 4, b2 & 15)
    pub hardware: [u8; 2],
}

impl MotorVersion {
    /// The vendor's text: "software_version:v1.2.10,hardware_version:v15.15".
    pub fn vendor_string(&self) -> String {
        let [major, minor, patch] = self.software;
        let [hardware_major, hardware_minor] = self.hardware;
        format!("software_version:v{major}.{minor}.{patch},hardware_version:v{hardware_major}.{hardware_minor}")
    }
}

/// 0x054 / 0x064, 1 Hz: the 64-byte VERSION_AND_ERROR layout (ACU version, 8 error words, 7 motor versions), or
/// the short ARM_ERROR layout (7 u16 words, bits 0..8) for any other length.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct ArmStatus {
    pub side: Side,
    /// None in the short layout.
    pub acu_version: Option<Version>,
    pub motors: Vec<MotorError>,
    pub motor_versions: Vec<MotorVersion>,
}

/// Length that selects the VERSION_AND_ERROR layout (decode_acu_error lpm@0x12f0d0).
pub const ARM_STATUS_FULL_LEN: usize = 64;

impl ArmStatus {
    /// `error_analysis::decode_acu_error` lpm@0x12f0d0.
    pub fn decode(side: Side, data: &[u8]) -> Option<ArmStatus> {
        if data.len() != ARM_STATUS_FULL_LEN {
            let words = data.len().min(2 * 7) / 2;
            let motors = (0..words)
                .map(|i| {
                    let code = u32::from(u16_be(data, 2 * i)?);
                    Some(MotorError::new(ARM_MOTOR_NAMES[i], code, &ARM_ERROR_BITS[..ARM_SHORT_LAYOUT_BITS]))
                })
                .collect::<Option<_>>()?;
            return Some(ArmStatus { side, acu_version: None, motors, motor_versions: Vec::new() });
        }
        let motors = ARM_MOTOR_NAMES
            .iter()
            .enumerate()
            .map(|(i, name)| Some(MotorError::new(name, u32_be(data, 8 + 4 * i)?, &ARM_ERROR_BITS)))
            .collect::<Option<_>>()?;
        let motor_versions = ARM_MOTOR_NAMES[..7]
            .iter()
            .enumerate()
            .map(|(i, motor)| {
                let [b0, b1, b2] = array::<3>(data, 40 + 3 * i)?;
                Some(MotorVersion { motor, software: [b0 >> 4, b0 & 15, b1], hardware: [b2 >> 4, b2 & 15] })
            })
            .collect::<Option<_>>()?;
        Some(ArmStatus { side, acu_version: Some(Version::at(data, 0)?), motors, motor_versions })
    }

    pub fn any_error(&self) -> bool {
        self.motors.iter().any(|motor| motor.code != 0)
    }
}

/// 0x001 EMERGENCY_STOP: the frame's arrival is the event (`error_analysis::decode` lpm@0x133b20 ignores the
/// payload); there is no release frame.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct EmergencyStop {
    pub data: Vec<u8>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chassis_emergency_stop_bit() {
        let mut data = [0u8; 12];
        data[6..8].copy_from_slice(&(1u16 << CHASSIS_EMERGENCY_STOP_BIT | 1).to_be_bytes());
        let report = ErrorReport::decode_chassis(&data).unwrap();
        assert!(report.emergency_stop() && report.any_error());
        assert_eq!(report.motors[3].motor, "steer_motor_fl");
        assert_eq!(report.motors[3].errors, ["MOTOR_DISCONNECT", "EMERGENCY_STOP"]);
        assert!(!ErrorReport::decode_torso(&[0u8; 20]).unwrap().emergency_stop());
    }

    #[test]
    fn version_reads_ff_as_zero() {
        let version = Version::decode(&[0xff, 0xff, 0xff, 1, 1, 12, 0, 0]).unwrap();
        assert_eq!((version.hardware.as_str(), version.software.as_str()), ("v0.0.0", "v1.1.12"));
    }
}
