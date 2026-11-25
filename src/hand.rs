//! Dexterous hands, code-only (no hand fitted on this robot): 0x056 / 0x066 feedback and the 0x055 / 0x065 command
//! (observed only), both 36 bytes of 6 fingers x 3 unsigned big-endian u16s.

use serde::Serialize;

use crate::arm::Side;
use crate::bytes::u16_be;

pub const FINGERS: usize = 6;
/// finger position counts per unit (ProtocolHand::decode_hand lpm@0x138c10: u16 / 4777).
pub const POSITION_COUNTS: f64 = 4777.0;
/// Hand frames are exactly this long (decode_hand lpm@0x138c10, encode_hand lpm@0x13af00).
pub const HAND_LEN: usize = 36;

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Hand {
    pub side: Side,
    pub position: [f64; FINGERS],
    /// Raw, unscaled by the vendor.
    pub velocity_raw: [u16; FINGERS],
    pub torque_raw: [u16; FINGERS],
}

impl Hand {
    pub fn decode(side: Side, data: &[u8]) -> Option<Hand> {
        let mut words = [0u16; 3 * FINGERS];
        for (index, word) in words.iter_mut().enumerate() {
            *word = u16_be(data, 2 * index)?;
        }
        Some(Hand {
            side,
            position: std::array::from_fn(|finger| f64::from(words[3 * finger]) / POSITION_COUNTS),
            velocity_raw: std::array::from_fn(|finger| words[3 * finger + 1]),
            torque_raw: std::array::from_fn(|finger| words[3 * finger + 2]),
        })
    }
}
