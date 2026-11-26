//! The Galaxea R1 Pro CAN FD bus (`can0`), receive only: every feedback frame the CCU, VCU, TCU and both ACUs
//! broadcast, decoded as the vendor driver HDAS (atc_system V2.3.0 hotfix_2) decodes it.
//!
//! - [`frame`]: one frame (kernel or log timestamp, 11-bit ID, FD flags, payload).
//! - `bus` (Linux): the SocketCAN FD socket, an iterator of [`Frame`]s with kernel receive timestamps.
//! - [`candump`]: `candump -L` logs, so recordings run through the same decoders.
//! - [`decode()`]: [`Frame`] -> [`Decoded`], one variant per frame kind; unknown IDs come back `Unknown`, never dropped.
//! - Per device: [`imu`], [`power`] (BMS), [`chassis`], [`torso`], [`arm`] (joints, grippers, F/T, temperatures),
//!   [`status`] (e-stop, error words, versions), [`hand`]; [`device`] is the ID table.
//! - [`tracker`]: per-ID rate and staleness from frame timestamps alone.
//!
//! Commands other nodes send (arm, gripper, torso, chassis) decode as observed traffic; nothing here transmits.
//!
//! ```
//! use r1_canfd::{candump, decode, Decoded};
//! let line = "(1795639172.791601) can0 021##5154100000D840006";
//! let frame = candump::parse_line(line, 1).unwrap().unwrap().frame;
//! let Decoded::Bms(bms) = decode(&frame) else { panic!() };
//! assert_eq!(bms.voltage_v, 54.41);
//! ```

#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used, clippy::panic))]

pub mod arm;
#[cfg(target_os = "linux")]
pub mod bus;
mod bytes;
pub mod candump;
pub mod chassis;
pub mod decode;
pub mod device;
pub mod encode;
pub mod frame;
pub mod gripper_geometry;
pub mod hand;
pub mod imu;
pub mod power;
pub mod status;
pub mod torso;
pub mod tracker;

#[cfg(target_os = "linux")]
pub use bus::Bus;
pub use decode::{decode, decode_payload, Decoded};
pub use frame::Frame;
pub use tracker::Tracker;
