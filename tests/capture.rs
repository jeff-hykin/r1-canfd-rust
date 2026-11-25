//! Physical sanity of the same capture: gravity, pack voltage, joints inside the r1pro URDF limits
//! (`fixtures/urdf_limits.json`, from dimos' robot assets as in PROTOCOL.md), temperatures, versions, rates.

use std::collections::BTreeMap;

use r1_canfd::arm::Side;
use r1_canfd::imu::ImuLocation;
use r1_canfd::power::VENDOR_BATTERY_CAPACITY;
use r1_canfd::status::Board;
use r1_canfd::{candump, decode, Decoded, Frame, Tracker};

const CAPTURE: &str = include_str!("fixtures/can0_20s_20261125T203928Z.log");
const URDF_LIMITS: &str = include_str!("fixtures/urdf_limits.json");

fn frames() -> Vec<Frame> {
    candump::parse_str(CAPTURE).unwrap().into_iter().map(|line| line.frame).collect()
}

fn decoded() -> Vec<Decoded> {
    frames().iter().map(decode).collect()
}

/// joint name -> (lower, upper) rad.
fn urdf_limits() -> BTreeMap<String, (f64, f64)> {
    let limits: BTreeMap<String, Vec<f64>> = serde_json::from_str(URDF_LIMITS).unwrap();
    limits.into_iter().map(|(name, values)| (name, (values[0], values[1]))).collect()
}

fn assert_within(name: &str, value: f64, (lower, upper): (f64, f64)) {
    assert!(lower <= value && value <= upper, "{name} = {value} outside [{lower}, {upper}]");
}

#[test]
fn both_imus_read_one_g_and_the_torso_tilt_agrees_with_gravity() {
    let mut seen = BTreeMap::new();
    for decoded in decoded() {
        let Decoded::Imu(imu) = decoded else { continue };
        let norm = imu.acceleration_norm_g();
        // PROTOCOL.md: |a| 0.997..1.004 g over the capture
        assert!((0.99..=1.01).contains(&norm), "{:?} |a| = {norm}", imu.location);
        let a = imu.acceleration_m_s2();
        assert!((a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt() > 9.7);
        if imu.location == ImuLocation::Torso {
            let tilt_from_gravity_deg = imu.acceleration_g[0].clamp(-1.0, 1.0).asin().to_degrees();
            assert!((imu.pitch_deg.abs() - tilt_from_gravity_deg.abs()).abs() < 0.5, "{imu:?}");
        }
        *seen.entry(format!("{:?}", imu.location)).or_insert(0) += 1;
    }
    assert_eq!(seen, BTreeMap::from([("Chassis".to_string(), 3999), ("Torso".to_string(), 1999)]));
}

#[test]
fn battery_reads_54_4_volts_and_98_9_percent() {
    let packs: Vec<_> =
        decoded().into_iter().filter_map(|d| if let Decoded::Bms(b) = d { Some(b) } else { None }).collect();
    assert_eq!(packs.len(), 40);
    for pack in packs {
        assert!((pack.voltage_v - 54.4).abs() < 0.05, "{pack:?}");
        assert_eq!(pack.current_a, 0.0);
        assert!((pack.charge_percent(VENDOR_BATTERY_CAPACITY) - 98.86).abs() < 0.01, "{pack:?}");
        assert_eq!(pack.unread_tail, Some(6));
    }
}

#[test]
fn arm_and_torso_joints_stay_inside_the_urdf_limits() {
    let limits = urdf_limits();
    let mut checked = 0;
    for decoded in decoded() {
        match decoded {
            Decoded::ArmFeedback(arm) => {
                for (index, position) in arm.position_rad.iter().enumerate() {
                    let name = format!("{}_arm_joint{}", arm.side.name(), index + 1);
                    assert_within(&name, *position, limits[&name]);
                }
                // left joint 1 holds gravity: -3.2..-3.8 Nm in /r1pro/motor_states (PROTOCOL.md)
                if arm.side == Side::Left {
                    assert!((-3.9..=-3.1).contains(&arm.torque_nm[0]), "{arm:?}");
                }
                checked += 1;
            }
            Decoded::TorsoFeedback(torso) => {
                for (index, position) in torso.position_rad.iter().enumerate() {
                    let name = format!("torso_joint{}", index + 1);
                    assert_within(&name, *position, limits[&name]);
                }
                checked += 1;
            }
            _ => {}
        }
    }
    assert_eq!(checked, 3999 + 4000 + 3999);
}

#[test]
fn grippers_left_closed_right_open() {
    for decoded in decoded() {
        let Decoded::GripperFeedback(gripper) = decoded else { continue };
        let opening = gripper.opening_mm.unwrap();
        match gripper.side {
            Side::Left => assert!((2.0..4.0).contains(&opening), "{gripper:?}"),
            Side::Right => assert!((100.0..104.0).contains(&opening), "{gripper:?}"),
        }
    }
}

#[test]
fn healthy_robot_versions_temperatures_and_no_estop() {
    let mut versions = BTreeMap::new();
    for decoded in decoded() {
        match decoded {
            Decoded::EmergencyStop(_) => panic!("no e-stop in this capture"),
            Decoded::Errors(report) => assert!(!report.any_error() && !report.emergency_stop(), "{report:?}"),
            Decoded::ArmStatus(status) => {
                assert!(!status.any_error(), "{status:?}");
                assert_eq!(status.acu_version.as_ref().unwrap().software, "v1.1.14");
                let joint7 = &status.motor_versions[6];
                // the left wrist motor reports an all-zero version, the right a real one (PROTOCOL.md open question)
                assert_eq!(joint7.software == [0, 0, 0], status.side == Side::Left, "{joint7:?}");
                assert_eq!(
                    status.motor_versions[0].vendor_string(),
                    "software_version:v1.2.10,hardware_version:v15.15"
                );
            }
            Decoded::Version { board, version } => {
                versions.insert(format!("{board:?}"), (version.hardware, version.software));
            }
            Decoded::MotorTemperatures(temperatures) => {
                for celsius in temperatures.mos_c.iter().chain(&temperatures.winding_c) {
                    assert!((20..=45).contains(celsius), "{temperatures:?}");
                }
            }
            _ => {}
        }
    }
    let expected = ("v0.0.0".to_string(), "v1.1.12".to_string());
    assert_eq!(versions, BTreeMap::from([(format!("{:?}", Board::Vcu), expected.clone()), ("Tcu".into(), expected)]));
}

#[test]
fn tracker_rates_match_the_capture_report() {
    let mut tracker = Tracker::new();
    for frame in frames() {
        tracker.observe(&frame);
    }
    let rate = |id| tracker.stats(id).unwrap().rate_hz().unwrap();
    for (id, hz) in [(0x020, 200.0), (0x021, 2.0), (0x030, 200.0), (0x040, 200.0), (0x045, 100.0), (0x052, 200.0)] {
        assert!((rate(id) - hz).abs() / hz < 0.01, "0x{id:03X}: {} Hz", rate(id));
    }
    assert!((rate(0x054) - 1.0).abs() < 0.01);
    // ESI marks HDAS's own sends: exactly 0x023, 0x031, 0x051, 0x061
    let esi: Vec<u32> = tracker.ids().filter(|(_, stats)| stats.esi_count > 0).map(|(id, _)| id).collect();
    assert_eq!(esi, [0x023, 0x031, 0x051, 0x061]);
    // nothing went silent before the log ended
    assert!(tracker.stale_ids(2.0).is_empty(), "{:?}", tracker.stale_ids(2.0));
}
