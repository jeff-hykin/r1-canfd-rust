//! Replays the 20 s capture (robot standing, arms holding, parked; 2026-11-25 20:39:28 UTC) and compares every
//! decoded field against the validated Python decoder (`r1_vendor/decode/r1can.py`), whose output for every 25th
//! frame of each ID (every frame of the low-rate IDs) is in `fixtures/golden.json` (regenerate with make_golden.py).

use std::collections::BTreeMap;

use r1_canfd::arm::Side;
use r1_canfd::power::VENDOR_BATTERY_CAPACITY;
use r1_canfd::status::{ErrorReport, MotorError};
use r1_canfd::{candump, decode, Decoded};
use serde_json::Value;

const CAPTURE: &str = include_str!("fixtures/can0_20s_20261125T203928Z.log");
const GOLDEN: &str = include_str!("fixtures/golden.json");
/// Python and Rust run the same IEEE operations; only libm sin/cos may differ in the last bits.
const TOLERANCE: f64 = 1e-12;

fn number(value: &Value, what: &str) -> f64 {
    value.as_f64().unwrap_or_else(|| panic!("{what}: not a number: {value}"))
}

fn assert_close(actual: f64, expected: &Value, what: &str) {
    let expected = number(expected, what);
    assert!(
        (actual - expected).abs() <= TOLERANCE * expected.abs().max(1.0),
        "{what}: rust {actual} python {expected}"
    );
}

fn assert_all_close(actual: &[f64], expected: &Value, what: &str) {
    let expected = expected.as_array().unwrap_or_else(|| panic!("{what}: not a list"));
    assert_eq!(actual.len(), expected.len(), "{what}: length");
    for (index, (a, e)) in actual.iter().zip(expected).enumerate() {
        assert_close(*a, e, &format!("{what}[{index}]"));
    }
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

fn assert_motor_error(actual: &MotorError, code: &Value, errors: &Value, what: &str) {
    assert_eq!(Value::from(actual.code), *code, "{what}: code");
    assert_eq!(serde_json::to_value(&actual.errors).unwrap(), *errors, "{what}: errors");
}

/// One golden sample against its Rust decode, field by field, keyed the way each Python module names them.
fn compare(id: u32, data: &[u8], decoded: &Decoded, python: &Value) {
    let field = |key: &str| python.get(key).unwrap_or_else(|| panic!("0x{id:03X}: python has no {key}: {python}"));
    let what = |key: &str| format!("0x{id:03X}.{key}");
    match decoded {
        Decoded::Imu(imu) => {
            assert_close(imu.roll_deg, field("roll_deg"), &what("roll_deg"));
            assert_close(imu.pitch_deg, field("pitch_deg"), &what("pitch_deg"));
            assert_close(imu.yaw_deg, field("yaw_deg"), &what("yaw_deg"));
            for (axis, index) in [("x", 0), ("y", 1), ("z", 2)] {
                assert_close(imu.gyro_deg_s[index], field(&format!("gyro_{axis}")), &what("gyro"));
                assert_close(imu.acceleration_g[index], field(&format!("acc_{axis}_g")), &what("acc"));
            }
            assert_close(imu.acceleration_norm_g(), field("acc_norm_g"), &what("acc_norm_g"));
            assert_all_close(&imu.vendor_quaternion_xyzw(), field("vendor_quat_xyzw"), &what("vendor_quat_xyzw"));
            assert_eq!(imu.unread_tail.map(Value::from), python.get("tail_u16").cloned(), "{}", what("tail_u16"));
        }
        Decoded::Bms(bms) => {
            assert_close(bms.voltage_v, field("voltage_V"), &what("voltage_V"));
            assert_close(bms.current_a, field("current_A"), &what("current_A"));
            assert_eq!(Value::from(bms.capacity_raw), *field("capital_raw"));
            assert_close(bms.charge_percent(VENDOR_BATTERY_CAPACITY), field("hdas_capital_pct"), &what("pct"));
            assert_eq!(bms.unread_tail.map_or(Value::Null, Value::from), *field("tail_u16"));
        }
        Decoded::MotorStatus(status) => {
            for (index, wheel) in ["fl", "fr", "r"].iter().enumerate() {
                assert_close(status.wheel_speed_m_s[index], field(&format!("vel_{wheel}")), &what("vel"));
                assert_close(status.steer_angle_rad[index], field(&format!("angle_{wheel}")), &what("angle"));
                assert_close(status.steer_rate_rad_s[index], field(&format!("angle_vel_{wheel}")), &what("angle_vel"));
            }
            assert_eq!(status.vcu_tick_ms.map_or(Value::Null, Value::from), *field("vcu_tick_ms"));
            assert_eq!(status.unread_tail, data[18..]);
        }
        Decoded::ChassisCommand(command) => {
            for (index, wheel) in ["fl", "fr", "r"].iter().enumerate() {
                assert_close(command.wheel_speed_m_s[index], field(&format!("cmd_vel_{wheel}")), &what("cmd_vel"));
                assert_close(command.steer_angle_rad[index], field(&format!("cmd_angle_{wheel}")), &what("cmd_angle"));
                assert_close(command.steer_rate_rad_s[index], field(&format!("cmd_angle_vel_{wheel}")), &what("rate"));
            }
        }
        Decoded::Remote(remote) => {
            let values = [remote.left_x, remote.left_y, remote.right_x, remote.right_y];
            for (key, value) in ["left_x", "left_y", "right_x", "right_y"].iter().zip(values) {
                assert_eq!(Value::from(value), *field(key), "{}", what(key));
            }
            assert_eq!(Value::from(remote.mode), *field("mode"));
        }
        Decoded::Errors(report) => compare_errors(id, report, python),
        Decoded::Version { version, .. } => {
            assert_eq!(version.hardware, field("hardware_version").as_str().unwrap());
            assert_eq!(version.software, field("software_version").as_str().unwrap());
        }
        Decoded::TorsoFeedback(torso) => {
            assert_all_close(&torso.position_rad, field("position"), &what("position"));
            assert_all_close(&torso.velocity_rad_s, field("velocity"), &what("velocity"));
            assert_all_close(&torso.torque_nm, field("effort"), &what("effort"));
        }
        Decoded::TorsoCommand(torso) => {
            assert_all_close(&torso.position_rad, field("p_des"), &what("p_des"));
            assert_all_close(&torso.velocity_rad_s, field("v_des"), &what("v_des"));
        }
        Decoded::ArmFeedback(arm) => {
            let side = if arm.side == Side::Left { "left" } else { "right" };
            assert_eq!(field("device").as_str().unwrap(), format!("{side}_arm_feedback"));
            assert_all_close(&arm.position_rad, field("position_rad"), &what("position_rad"));
            assert_all_close(&arm.velocity_rad_s, field("velocity_rad_s"), &what("velocity_rad_s"));
            assert_all_close(&arm.torque_nm, field("torque_nm"), &what("torque_nm"));
            assert_eq!(hex(&arm.unread_tail), field("tail_hex").as_str().unwrap());
        }
        Decoded::ArmCommand(arm) => {
            let joints = field("joints").as_array().unwrap();
            assert_eq!(joints.len(), arm.joints.len());
            for (index, (joint, python_joint)) in arm.joints.iter().zip(joints).enumerate() {
                let what = format!("0x{id:03X}.joints[{index}]");
                assert_close(joint.position_rad, &python_joint["p"], &what);
                assert_close(joint.velocity_rad_s, &python_joint["v"], &what);
                assert_eq!(Value::from(joint.kp_raw), python_joint["kp"], "{what}.kp");
                assert_eq!(Value::from(joint.kd_raw), python_joint["kd"], "{what}.kd");
                assert_close(joint.torque_nm, &python_joint["t"], &what);
            }
        }
        Decoded::GripperCommand(gripper) => {
            assert_close(gripper.motor_position_rad, field("p_des"), &what("p_des"));
            assert_close(gripper.velocity, field("v_des"), &what("v_des"));
            assert_close(gripper.kp, field("kp"), &what("kp"));
            assert_close(gripper.kd, field("kd"), &what("kd"));
            assert_close(gripper.torque, field("t_ff"), &what("t_ff"));
        }
        Decoded::GripperFeedback(gripper) => {
            assert_close(gripper.motor_position_rad, field("motor_pos_rad"), &what("motor_pos_rad"));
            // the vendor publishes 0 where the linkage is out of range; Rust says None
            assert_close(gripper.opening_mm.unwrap_or(0.0), field("position"), &what("position"));
            assert_close(gripper.velocity, field("velocity"), &what("velocity"));
            assert_close(gripper.torque, field("torque"), &what("torque"));
            let tail = gripper.unread_tail.map_or(String::new(), |tail| format!("{tail:04x}"));
            assert_eq!(tail, field("undecoded_tail").as_str().unwrap());
        }
        Decoded::Wrench(wrench) => {
            for (index, key) in ["fx", "fy", "fz"].iter().enumerate() {
                assert_close(wrench.force_n[index], field(key), &what(key));
            }
            for (index, key) in ["mx", "my", "mz"].iter().enumerate() {
                assert_close(wrench.moment_nm[index], field(key), &what(key));
            }
        }
        Decoded::MotorTemperatures(temperatures) => {
            assert_eq!(serde_json::to_value(temperatures.mos_c).unwrap(), *field("mos_temp"));
            assert_eq!(serde_json::to_value(temperatures.winding_c).unwrap(), *field("winding_temp"));
        }
        Decoded::ArmStatus(status) => {
            let side = if status.side == Side::Left { "LEFT" } else { "RIGHT" };
            assert_eq!(field("name_id").as_str().unwrap(), format!("{side}_ARM_VERSION_AND_ERROR"));
            let version = status.acu_version.as_ref().unwrap();
            assert_eq!(version.hardware, field("acu_hardware_version").as_str().unwrap());
            assert_eq!(version.software, field("acu_software_version").as_str().unwrap());
            let errors = field("errors").as_array().unwrap();
            assert_eq!(errors.len(), status.motors.len());
            for (motor, python_motor) in status.motors.iter().zip(errors) {
                assert_eq!(motor.motor, python_motor["name"].as_str().unwrap());
                assert_motor_error(motor, &python_motor["error_code"], &python_motor["error_description"], &what("e"));
            }
            let versions = field("motor_versions").as_object().unwrap();
            assert_eq!(versions.len(), status.motor_versions.len());
            for version in &status.motor_versions {
                assert_eq!(version.vendor_string(), versions[version.motor].as_str().unwrap(), "{}", version.motor);
            }
        }
        Decoded::Unknown { id: 0x023, data } => assert_eq!(Value::from(data[0]), *field("byte0")),
        other => panic!("0x{id:03X}: no comparison for {other:?} (python: {python})"),
    }
}

fn compare_errors(id: u32, report: &ErrorReport, python: &Value) {
    match id {
        // chassis.py (first in r1can's decoder order): {w1: {error_code, errors}, ...}
        0x034 => {
            assert_eq!(python.as_object().unwrap().len(), report.motors.len());
            for motor in &report.motors {
                let entry = &python[motor.motor];
                assert_motor_error(motor, &entry["error_code"], &entry["errors"], motor.motor);
            }
        }
        // torso_ccu.py: {name_id, motors: {w1: {code, errors}}, tail_hex}
        0x044 => {
            let motors = python["motors"].as_object().unwrap();
            assert_eq!(motors.len(), report.motors.len());
            for motor in &report.motors {
                assert_motor_error(motor, &motors[motor.motor]["code"], &motors[motor.motor]["errors"], motor.motor);
            }
            assert_eq!(hex(&report.unread_tail), python["tail_hex"].as_str().unwrap());
        }
        other => panic!("0x{other:03X}: error frame with no golden layout"),
    }
}

#[test]
fn every_id_matches_the_python_decoder() {
    let frames = candump::parse_str(CAPTURE).unwrap();
    let golden: BTreeMap<String, Value> = serde_json::from_str(GOLDEN).unwrap();
    let mut counts: BTreeMap<String, u64> = BTreeMap::new();
    for frame in &frames {
        *counts.entry(format!("0x{:03X}", frame.frame.id)).or_default() += 1;
    }
    assert_eq!(counts.keys().collect::<Vec<_>>(), golden.keys().collect::<Vec<_>>(), "same IDs in log and golden");
    let mut compared = 0;
    for (key, entry) in &golden {
        assert_eq!(Value::from(counts[key]), entry["count"], "{key} count");
        for sample in entry["samples"].as_array().unwrap() {
            let frame = &frames[sample["line"].as_u64().unwrap() as usize].frame;
            assert_eq!(format!("0x{:03X}", frame.id), *key);
            compare(frame.id, &frame.data, &decode(frame), &sample["decoded"]);
            compared += 1;
        }
    }
    assert!(compared > 3000, "{compared} samples compared");
}

#[test]
fn every_frame_decodes_and_only_0x023_is_unknown() {
    for line in candump::parse_str(CAPTURE).unwrap() {
        match decode(&line.frame) {
            Decoded::Unknown { id, .. } => assert_eq!(id, 0x023),
            Decoded::Malformed { id, data, needed } => panic!("0x{id:03X} malformed: {} < {needed}", data.len()),
            _ => {}
        }
    }
}
