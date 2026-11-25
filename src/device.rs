//! Every ID HDAS knows (`Dispatcher::Dispatcher` disp@0x10b500 fills `canIDToDevice`) plus the command IDs seen
//! on the bus, with who sends it and how well its decode is proven. The upper nibble names the board:
//! 0x02x CCU, 0x03x VCU (chassis), 0x04x TCU (torso), 0x05x ACU-L (left arm), 0x06x ACU-R (right arm).

use serde::Serialize;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Direction {
    /// Feedback an MCU broadcasts unprompted.
    ToHost,
    /// A command some host node sends (decoded here only as observed traffic).
    ToBoard,
    /// The ID is used both ways (told apart by length).
    Both,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Evidence {
    /// Decoded from the vendor code and checked against the capture.
    Verified,
    /// Layout checked, but the capture only had zeros / constants, or a scale is inferred (see the note).
    Partial,
    /// From the vendor code only; the ID was not in the capture.
    CodeOnly,
    /// Seen on the bus, meaning unknown.
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
pub struct Device {
    pub id: u32,
    /// The vendor's name (dispatcher map), or a descriptive one for IDs it does not map.
    pub name: &'static str,
    pub direction: Direction,
    pub evidence: Evidence,
    pub note: &'static str,
}

const fn entry(id: u32, name: &'static str, direction: Direction, evidence: Evidence, note: &'static str) -> Device {
    Device { id, name, direction, evidence, note }
}

use Direction::{Both, ToBoard, ToHost};
use Evidence::{CodeOnly, Partial, Unknown, Verified};

pub const DEVICES: &[Device] = &[
    entry(0x001, "EMERGENCY_STOP", ToHost, CodeOnly, "arrival is the event; payload ignored, no release frame"),
    entry(0x020, "IMU", ToHost, Verified, "chassis IMU (CCU), |a| 0.997-1.002 g; gyro deg/s inferred"),
    entry(0x021, "BMS", ToHost, Verified, "54.41 V, 98.9 %; no proven charging field (bytes 6-7 raw)"),
    entry(0x022, "LED_CMD", ToBoard, CodeOnly, "[r, g, b, mode]"),
    entry(0x023, "UNKNOWN_0x023", ToBoard, Unknown, "1 byte, always 0x02, ~1.9 Hz, ESI-flagged host sender"),
    entry(0x025, "CCU_VERSION", ToHost, CodeOnly, "not in the capture"),
    entry(0x030, "MOTOR_STATUS", ToHost, Partial, "wheel/steer feedback; parked, wheel speed scale UNVERIFIED"),
    entry(0x031, "MOTOR_CTRL", ToBoard, Partial, "chassis command (HDAS, 50 Hz); all zero while parked"),
    entry(0x032, "REMOTE", ToHost, Partial, "RC sticks; all zero (no RC paired)"),
    entry(0x033, "CHASSIS_ERROR", ToHost, CodeOnly, "event; bit 10 = EMERGENCY_STOP"),
    entry(0x034, "VCU_ERROR", Both, Partial, "24 B error words (all zero) / 1 B auto-calibration request"),
    entry(0x035, "VCU_VERSION", ToHost, Verified, "sw v1.1.12"),
    entry(0x040, "TORSO_MOTOR_FB", ToHost, Verified, "within URDF limits"),
    entry(0x041, "TORSO_MOTOR_CTRL", ToBoard, Verified, "p, v only (WIM RT, 250 Hz)"),
    entry(0x043, "TORSO_FUNCTION", ToBoard, CodeOnly, "command 1..10, meanings unknown"),
    entry(0x044, "TORSO_ERROR", ToHost, Partial, "all zero"),
    entry(0x045, "TORSO_IMU", ToHost, Verified, "pitch -13 deg matches the accelerometer"),
    entry(0x046, "TCU_VERSION", ToHost, Verified, "sw v1.1.12"),
    entry(0x050, "LEFT_ARM_CMD", ToBoard, Verified, "byte-exact against arm_encode_impl"),
    entry(0x051, "LEFT_GRIPPER_CMD", ToBoard, Verified, "constant in the capture (HDAS, ~415 Hz)"),
    entry(0x052, "LEFT_ARM", ToHost, Verified, "within URDF limits; torque matches mcap effort"),
    entry(0x053, "LEFT_ARM_FUNCTION", ToBoard, CodeOnly, "command 1..10, meanings unknown"),
    entry(0x054, "LEFT_ARM_ERROR", ToHost, Verified, "versions verified; error words all zero"),
    entry(0x055, "LEFT_ACU_VERSION", Both, CodeOnly, "8 B version (A1) / 36 B hand command"),
    entry(0x056, "LEFT_HAND", ToHost, CodeOnly, "no hand fitted"),
    entry(0x057, "LEFT_WRENCH", ToHost, Partial, "length and rate only; values all zero"),
    entry(0x058, "LEFT_GRIPPER_FB", ToHost, Verified, "opening mm inferred from the linkage"),
    entry(0x05D, "LEFT_MOTOR_TEMP_FB", ToHost, Verified, "25-39 C"),
    entry(0x060, "RIGHT_ARM_CMD", ToBoard, Verified, "byte-exact against arm_encode_impl"),
    entry(0x061, "RIGHT_GRIPPER_CMD", ToBoard, Verified, "constant in the capture (HDAS, ~415 Hz)"),
    entry(0x062, "RIGHT_ARM", ToHost, Verified, "within URDF limits"),
    entry(0x063, "RIGHT_ARM_FUNCTION", ToBoard, CodeOnly, "command 1..10, meanings unknown"),
    entry(0x064, "RIGHT_ARM_ERROR", ToHost, Verified, "versions verified; error words all zero"),
    entry(0x065, "RIGHT_ACU_VERSION", Both, CodeOnly, "8 B version (A1) / 36 B hand command"),
    entry(0x066, "RIGHT_HAND", ToHost, CodeOnly, "no hand fitted"),
    entry(0x067, "RIGHT_WRENCH", ToHost, Partial, "length and rate only; values all zero"),
    entry(0x068, "RIGHT_GRIPPER_FB", ToHost, Verified, "opening mm inferred from the linkage"),
    entry(0x06D, "RIGHT_MOTOR_TEMP_FB", ToHost, Verified, "25-39 C"),
    entry(0x070, "R1PROT_JOY", ToHost, CodeOnly, "r1pro_t only; not decoded"),
    entry(0x510, "LEFT_H1_HAND", Both, CodeOnly, "not decoded"),
    entry(0x520, "RIGHT_H1_HAND", Both, CodeOnly, "not decoded"),
];

pub fn device(id: u32) -> Option<&'static Device> {
    DEVICES.iter().find(|device| device.id == id)
}

pub fn device_name(id: u32) -> &'static str {
    device(id).map_or("?", |device| device.name)
}
