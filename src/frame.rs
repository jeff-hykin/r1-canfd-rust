//! One CAN (FD) frame as it came off the bus or out of a log: its time, 11-bit ID, FD flags and payload.

use serde::Serialize;

/// Bit-rate switch: the data phase ran at the data bit rate (linux/can.h `CANFD_BRS`).
pub const FLAG_BRS: u8 = 0x01;
/// Error-state indicator: the sender was error-passive (`CANFD_ESI`); on this bus it marks HDAS's own transmits.
pub const FLAG_ESI: u8 = 0x02;
/// A CAN FD frame, not classic CAN (`CANFD_FDF`; candump -L prints it as part of the flag nibble).
pub const FLAG_FDF: u8 = 0x04;

/// The vendor's `CAN_Interface::writeCANFD` (comm@0x134830) sets BRS|ESI, so frames with ESI came from libcommunication.
pub const VENDOR_HOST_FLAGS: u8 = FLAG_BRS | FLAG_ESI;

#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct Frame {
    /// CLOCK_REALTIME nanoseconds: the kernel's receive time live, the candump stamp from a log.
    pub timestamp_ns: u64,
    /// The arbitration ID (11-bit on this bus; 29-bit IDs keep their full value with `extended` set).
    pub id: u32,
    pub extended: bool,
    /// `FLAG_*` bits; `FLAG_FDF` is set for every CAN FD frame.
    pub flags: u8,
    pub data: Vec<u8>,
}

impl Frame {
    pub fn is_fd(&self) -> bool {
        self.flags & FLAG_FDF != 0
    }

    /// Sent by a host process through libcommunication (HDAS), per the ESI fingerprint in PROTOCOL.md.
    pub fn from_vendor_host(&self) -> bool {
        self.flags & FLAG_ESI != 0
    }
}
