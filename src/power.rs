//! 0x021 BMS (CCU, 2 Hz, 8 bytes): pack voltage, current and remaining capacity.
//!
//! Source: `ProtocolCCU::decode_bms` lpm@0x124754 (3 signed big-endian i16s) and HDAS lambda26 HDAS@0x16f934, which
//! publishes whole volts / amps (integer division) and `capacity * 100 / battery_capacity` as percent. This keeps the
//! full 0.01 precision. No charging flag is proven anywhere in the frame; bytes 6-7 stay raw.

use serde::Serialize;

use crate::bytes::{i16_be, u16_be};

/// voltage counts per volt (decode_bms lpm@0x124754 + HDAS@0x16f934: raw / 100; 5441 -> 54.41 V in the capture).
pub const VOLTAGE_COUNTS_PER_V: f64 = 100.0;
/// current counts per amp (same source: raw / 100, signed; which sign means charging is unproven).
pub const CURRENT_COUNTS_PER_A: f64 = 100.0;
/// HDAS ros param `battery_capacity` on this robot (launch_params_1yymxejn), the denominator of HDAS's percent.
pub const VENDOR_BATTERY_CAPACITY: f64 = 3500.0;

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Bms {
    pub voltage_v: f64,
    pub current_a: f64,
    /// Remaining capacity in the BMS's own units (3460 of 3500 in the capture).
    pub capacity_raw: i16,
    /// Bytes 6-7, not read by HDAS (a constant 6 in the capture); the likeliest home of a charging state, unproven.
    pub unread_tail: Option<u16>,
}

impl Bms {
    pub const LEN: usize = 6;

    pub fn decode(data: &[u8]) -> Option<Bms> {
        Some(Bms {
            voltage_v: f64::from(i16_be(data, 0)?) / VOLTAGE_COUNTS_PER_V,
            current_a: f64::from(i16_be(data, 2)?) / CURRENT_COUNTS_PER_A,
            capacity_raw: i16_be(data, 4)?,
            unread_tail: u16_be(data, 6),
        })
    }

    /// Charge in percent against a full-pack capacity in the same units (`VENDOR_BATTERY_CAPACITY` is HDAS's).
    pub fn charge_percent(&self, full_capacity: f64) -> f64 {
        f64::from(self.capacity_raw) * 100.0 / full_capacity
    }
}
