//! The live bus (Linux): a raw SocketCAN socket with CAN FD frames on and kernel receive timestamps, read as an
//! iterator of [`Frame`]s. Receive only unless the `transmit` cargo feature is on, which adds [`Bus::send`].
//!
//! Mirrors HDAS's socket (`CAN_Interface::connect` comm@0x136b70): `socket(PF_CAN, SOCK_RAW, CAN_RAW)`, bound to the
//! interface, `CAN_RAW_FD_FRAMES = 1`, no `CAN_RAW_FILTER` (every ID). It does not configure the link (bit rates,
//! `fd on`); `ip link` stays in the boot script. Each frame carries the kernel's software receive time
//! (`SO_TIMESTAMPING`, CLOCK_REALTIME), so rates and staleness do not depend on when the caller reads.

use std::ffi::CString;
use std::io;
use std::mem;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd};

use crate::frame::{Frame, FLAG_FDF};

/// linux/can.h `CAN_RAW`.
const CAN_RAW: libc::c_int = 1;
/// linux/can/raw.h `SOL_CAN_RAW` (SOL_CAN_BASE 100 + CAN_RAW).
const SOL_CAN_RAW: libc::c_int = 101;
/// linux/can/raw.h `CAN_RAW_FD_FRAMES`.
const CAN_RAW_FD_FRAMES: libc::c_int = 5;
/// `CAN_RAW_FILTER` (linux/can/raw.h); an empty filter list receives nothing.
#[cfg(feature = "transmit")]
const CAN_RAW_FILTER: libc::c_int = 1;
/// asm-generic/socket.h `SO_TIMESTAMPING` (SO_TIMESTAMPING_OLD, which 64-bit time_t uses); also the cmsg type.
const SO_TIMESTAMPING: libc::c_int = 37;
/// linux/net_tstamp.h `SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE`.
const TIMESTAMP_SOFTWARE_RX: libc::c_uint = (1 << 3) | (1 << 4);
/// linux/can.h `CAN_EFF_FLAG` / `CAN_RTR_FLAG` / masks.
const CAN_EFF_FLAG: u32 = 0x8000_0000;
const CAN_RTR_FLAG: u32 = 0x4000_0000;
const CAN_EFF_MASK: u32 = 0x1FFF_FFFF;
const CAN_SFF_MASK: u32 = 0x7FF;
/// linux/can.h `CAN_MTU` (a classic frame) and `CANFD_MTU`.
const CAN_MTU: usize = 16;
const CANFD_MTU: usize = 72;
const CANFD_MAX_DLEN: usize = 64;

/// linux/can.h `struct canfd_frame`.
#[repr(C, align(8))]
struct CanFdFrame {
    can_id: u32,
    len: u8,
    flags: u8,
    reserved: [u8; 2],
    data: [u8; CANFD_MAX_DLEN],
}

/// linux/can.h `struct sockaddr_can` (24 bytes; the address union is unused for CAN_RAW).
#[repr(C)]
struct SockaddrCan {
    can_family: libc::sa_family_t,
    can_ifindex: libc::c_int,
    can_addr: [u64; 2],
}

fn last_error(what: &str) -> io::Error {
    let error = io::Error::last_os_error();
    io::Error::new(error.kind(), format!("{what}: {error}"))
}

fn set_option(fd: RawFd, level: libc::c_int, name: libc::c_int, value: libc::c_int, what: &str) -> io::Result<()> {
    let size = mem::size_of_val(&value) as libc::socklen_t;
    if unsafe { libc::setsockopt(fd, level, name, (&value as *const libc::c_int).cast(), size) } != 0 {
        return Err(last_error(what));
    }
    Ok(())
}

/// An open CAN FD socket on one interface, iterating every frame the bus carries.
pub struct Bus {
    fd: OwnedFd,
    interface: String,
}

impl Bus {
    /// Open and bind `interface` (can0 on the R1 Pro). The link must already be up in FD mode.
    pub fn open(interface: &str) -> io::Result<Bus> {
        let name = CString::new(interface).map_err(|_| io::Error::new(io::ErrorKind::InvalidInput, "bad interface"))?;
        let ifindex = unsafe { libc::if_nametoindex(name.as_ptr()) };
        if ifindex == 0 {
            return Err(last_error(&format!("if_nametoindex({interface})")));
        }
        let raw = unsafe { libc::socket(libc::AF_CAN, libc::SOCK_RAW | libc::SOCK_CLOEXEC, CAN_RAW) };
        if raw < 0 {
            return Err(last_error("socket(PF_CAN, SOCK_RAW, CAN_RAW)"));
        }
        let fd = unsafe { OwnedFd::from_raw_fd(raw) };
        set_option(raw, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1, "CAN_RAW_FD_FRAMES")?;
        set_option(raw, libc::SOL_SOCKET, SO_TIMESTAMPING, TIMESTAMP_SOFTWARE_RX as libc::c_int, "SO_TIMESTAMPING")?;
        let address = SockaddrCan {
            can_family: libc::AF_CAN as libc::sa_family_t,
            can_ifindex: ifindex as libc::c_int,
            can_addr: [0; 2],
        };
        let size = mem::size_of::<SockaddrCan>() as libc::socklen_t;
        if unsafe { libc::bind(raw, (&address as *const SockaddrCan).cast(), size) } != 0 {
            return Err(last_error(&format!("bind({interface})")));
        }
        Ok(Bus { fd, interface: interface.to_string() })
    }

    pub fn interface(&self) -> &str {
        &self.interface
    }

    /// Block until the next frame; retries on EINTR.
    pub fn recv(&mut self) -> io::Result<Frame> {
        loop {
            match self.recv_once() {
                Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
                result => return result,
            }
        }
    }

    fn recv_once(&mut self) -> io::Result<Frame> {
        let mut raw: CanFdFrame = unsafe { mem::zeroed() };
        let mut control = [0u64; 16];
        let mut iov = libc::iovec { iov_base: (&mut raw as *mut CanFdFrame).cast(), iov_len: CANFD_MTU };
        let mut message: libc::msghdr = unsafe { mem::zeroed() };
        message.msg_iov = &mut iov;
        message.msg_iovlen = 1;
        message.msg_control = control.as_mut_ptr().cast();
        message.msg_controllen = mem::size_of_val(&control) as _;
        let received = unsafe { libc::recvmsg(self.fd.as_raw_fd(), &mut message, 0) };
        if received < 0 {
            return Err(io::Error::last_os_error());
        }
        let is_fd = match received as usize {
            CANFD_MTU => true,
            CAN_MTU => false,
            other => return Err(io::Error::new(io::ErrorKind::InvalidData, format!("short CAN read: {other} bytes"))),
        };
        let timestamp_ns = software_timestamp_ns(&message).unwrap_or(0);
        let extended = raw.can_id & CAN_EFF_FLAG != 0;
        let id = raw.can_id & if extended { CAN_EFF_MASK } else { CAN_SFF_MASK };
        let len = if raw.can_id & CAN_RTR_FLAG != 0 { 0 } else { usize::from(raw.len).min(CANFD_MAX_DLEN) };
        let flags = if is_fd { raw.flags | FLAG_FDF } else { 0 };
        Ok(Frame { timestamp_ns, id, extended, flags, data: raw.data[..len].to_vec() })
    }
}

/// ts[0] of the SCM_TIMESTAMPING control message (software receive time, CLOCK_REALTIME).
fn software_timestamp_ns(message: &libc::msghdr) -> Option<u64> {
    let mut header = unsafe { libc::CMSG_FIRSTHDR(message) };
    while !header.is_null() {
        let cmsg = unsafe { &*header };
        if cmsg.cmsg_level == libc::SOL_SOCKET && cmsg.cmsg_type == SO_TIMESTAMPING {
            let stamps = unsafe { std::ptr::read_unaligned(libc::CMSG_DATA(header).cast::<[libc::timespec; 3]>()) };
            let software = stamps[0];
            let seconds = u64::try_from(software.tv_sec).ok()?;
            let nanos = u64::try_from(software.tv_nsec).ok()?;
            return Some(seconds * 1_000_000_000 + nanos);
        }
        header = unsafe { libc::CMSG_NXTHDR(message, header) };
    }
    None
}

/// `struct canfd_frame` for one outgoing frame: standard ID, BRS (plus ESI when copying the vendor's flags).
#[cfg(feature = "transmit")]
fn outgoing_canfd(frame: &crate::encode::OutgoingFrame, flags: u8) -> io::Result<CanFdFrame> {
    if frame.data.len() > CANFD_MAX_DLEN || frame.id > CAN_SFF_MASK {
        return Err(io::Error::new(io::ErrorKind::InvalidInput, "frame too long or ID not 11-bit"));
    }
    let mut raw: CanFdFrame = unsafe { mem::zeroed() };
    raw.can_id = frame.id;
    raw.len = frame.data.len() as u8;
    raw.flags = flags;
    raw.data[..frame.data.len()].copy_from_slice(&frame.data);
    Ok(raw)
}

#[cfg(feature = "transmit")]
impl Bus {
    /// A socket for sending only: bound like [`Bus::open`] but with an empty receive filter, so nothing queues up
    /// on it (the reading `Bus` still sees every sent frame through the kernel's local loopback).
    pub fn open_send_only(interface: &str) -> io::Result<Bus> {
        let bus = Bus::open(interface)?;
        let empty = unsafe { libc::setsockopt(bus.fd.as_raw_fd(), SOL_CAN_RAW, CAN_RAW_FILTER, std::ptr::null(), 0) };
        if empty != 0 {
            return Err(last_error("CAN_RAW_FILTER (none)"));
        }
        Ok(bus)
    }

    /// Write one frame (`write(fd, frame, 72)` as `CAN_Interface::writeCANFD` comm@0x134830 does); `flags` is
    /// `frame::FLAG_BRS`, or `frame::VENDOR_HOST_FLAGS` (BRS|ESI) to copy the vendor bit for bit.
    pub fn send(&self, frame: &crate::encode::OutgoingFrame, flags: u8) -> io::Result<()> {
        let raw = outgoing_canfd(frame, flags)?;
        let written = unsafe { libc::write(self.fd.as_raw_fd(), (&raw as *const CanFdFrame).cast(), CANFD_MTU) };
        if written < 0 {
            return Err(last_error("write(can)"));
        }
        if written as usize != CANFD_MTU {
            return Err(io::Error::new(io::ErrorKind::WriteZero, format!("short CAN write: {written} bytes")));
        }
        Ok(())
    }
}

impl Iterator for Bus {
    type Item = io::Result<Frame>;

    /// Never ends; each item is one received frame or a receive error (the socket stays usable after one).
    fn next(&mut self) -> Option<io::Result<Frame>> {
        Some(self.recv())
    }
}

impl AsRawFd for Bus {
    fn as_raw_fd(&self) -> RawFd {
        self.fd.as_raw_fd()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn kernel_struct_sizes() {
        assert_eq!(mem::size_of::<CanFdFrame>(), CANFD_MTU);
        assert_eq!(mem::size_of::<SockaddrCan>(), 24);
    }
}
