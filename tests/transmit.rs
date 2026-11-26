//! Byte-exact checks of every command encoder against the frames other senders put on the bus in the capture.
//!
//! Every command ID carries one constant payload for the whole 20 s. Where the vendor's inputs can be rebuilt from
//! first principles (chassis: zero twist; grippers: opening 0 / 100 with the vendor gains) the whole send path runs
//! from those inputs. Where they cannot (arm and torso targets came from the WIM RT node), each value is rebuilt as
//! the middle of the bin that produced its raw count, and the encoder must reproduce the bytes from it.

use std::collections::BTreeMap;

use r1_canfd::arm::Side;
use r1_canfd::candump;
use r1_canfd::encode::{encode_arm, encode_torso, ArmJointTarget, TorsoJointTarget};
use r1_canfd::frame::Frame;

const CAPTURE: &str = include_str!("fixtures/can0_20s_20261125T203928Z.log");

fn frames() -> Vec<Frame> {
    candump::parse_str(CAPTURE).unwrap().into_iter().map(|line| line.frame).collect()
}

/// Every distinct payload per ID with its count.
fn payloads(id: u32) -> BTreeMap<Vec<u8>, usize> {
    let mut seen = BTreeMap::new();
    for frame in frames().into_iter().filter(|frame| frame.id == id) {
        *seen.entry(frame.data).or_insert(0) += 1;
    }
    seen
}

/// The one payload an ID carried for the whole capture.
fn constant_payload(id: u32) -> Vec<u8> {
    let seen = payloads(id);
    assert_eq!(seen.len(), 1, "0x{id:03x} is not constant: {seen:?}");
    seen.into_keys().next().unwrap()
}

/// The middle of the truncation bin of `raw` counts: values in it truncate toward zero back to `raw`.
fn mid_bin(raw: i32, scale: f32) -> f32 {
    (raw as f32 + 0.5 * (raw.signum() as f32)) / scale
}

fn sign_magnitude(raw: u16) -> i32 {
    let magnitude = i32::from(raw & 0x7FF);
    if raw & 0x800 != 0 {
        -magnitude
    } else {
        magnitude
    }
}

#[test]
fn arm_commands_reencode_byte_for_byte() {
    for (id, side) in [(0x050, Side::Left), (0x060, Side::Right)] {
        let captured = constant_payload(id);
        let joints: [ArmJointTarget; 7] = std::array::from_fn(|joint| {
            let b = &captured[8 * joint..8 * joint + 8];
            let position = i32::from(i16::from_be_bytes([b[0], b[1]]));
            let velocity = sign_magnitude((u16::from(b[2]) << 4) | u16::from(b[3] >> 4));
            let kp = (u16::from(b[3] & 0xF) << 8) | u16::from(b[4]);
            let kd = (u16::from(b[5]) << 4) | u16::from(b[6] >> 4);
            let torque = sign_magnitude((u16::from(b[6] & 0xF) << 8) | u16::from(b[7]));
            ArmJointTarget {
                position_rad: mid_bin(position, 4777.0),
                velocity_rad_s: mid_bin(velocity, 250.0),
                kp: f32::from(kp) + 0.5,
                kd: f32::from(kd) + 0.5,
                torque_nm: mid_bin(torque, 20.0),
            }
        });
        // the hold pattern PROTOCOL.md describes: kp 2, kd 25, v 0.4 rad/s on every joint
        assert!(joints.iter().all(|joint| joint.kp == 2.5 && joint.kd == 25.5), "{joints:?}");
        let frame = encode_arm(side, &joints);
        assert_eq!(frame.id, id);
        assert_eq!(frame.data, captured, "0x{id:03x}");
    }
}

#[test]
fn torso_command_reencodes_byte_for_byte() {
    let captured = constant_payload(0x041);
    let joints: [TorsoJointTarget; 4] = std::array::from_fn(|joint| {
        let word = |offset: usize| i32::from(i16::from_be_bytes([captured[offset], captured[offset + 1]]));
        TorsoJointTarget {
            position_rad: mid_bin(word(4 * joint), 10000.0),
            velocity_rad_s: mid_bin(word(4 * joint + 2), 3000.0),
        }
    });
    assert_eq!(encode_torso(&joints).data, captured);
}
