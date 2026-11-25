//! `candump -L` log lines, so recordings run through the same decoders as the live bus.
//!
//! `(1795639172.670876) can0 061##7CC98...`: stamp in seconds, interface, ID (3 hex digits = 11-bit, 8 = 29-bit),
//! `##` + one flag nibble + data for CAN FD, `#` + data (or `#R` for a remote request) for classic CAN.

use std::fmt;
use std::io::BufRead;

use crate::frame::{Frame, FLAG_FDF};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ParseError {
    pub line_number: usize,
    pub reason: &'static str,
}

impl fmt::Display for ParseError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(formatter, "candump line {}: {}", self.line_number, self.reason)
    }
}

impl std::error::Error for ParseError {}

/// A parsed log line: the interface it was captured on and the frame.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LogFrame {
    pub interface: String,
    pub frame: Frame,
}

/// "1795639172.670876" -> ns, without going through a float.
fn parse_stamp_ns(stamp: &str) -> Option<u64> {
    let stamp = stamp.strip_prefix('(')?.strip_suffix(')')?;
    let (seconds, fraction) = stamp.split_once('.').unwrap_or((stamp, ""));
    if fraction.len() > 9 || !fraction.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    let fraction_ns: u64 =
        if fraction.is_empty() { 0 } else { fraction.parse::<u64>().ok()? * 10u64.pow(9 - fraction.len() as u32) };
    seconds.parse::<u64>().ok()?.checked_mul(1_000_000_000)?.checked_add(fraction_ns)
}

fn parse_hex_bytes(hex: &str) -> Option<Vec<u8>> {
    if !hex.len().is_multiple_of(2) {
        return None;
    }
    (0..hex.len()).step_by(2).map(|i| u8::from_str_radix(hex.get(i..i + 2)?, 16).ok()).collect()
}

/// Parse one line; blank lines give `Ok(None)`.
pub fn parse_line(line: &str, line_number: usize) -> Result<Option<LogFrame>, ParseError> {
    let error = |reason| ParseError { line_number, reason };
    let mut fields = line.split_whitespace();
    let Some(stamp) = fields.next() else { return Ok(None) };
    let interface = fields.next().ok_or(error("missing interface"))?;
    let body = fields.next().ok_or(error("missing frame"))?;
    let timestamp_ns = parse_stamp_ns(stamp).ok_or(error("bad timestamp"))?;
    let (id_hex, rest) = body.split_once('#').ok_or(error("missing '#'"))?;
    let id = u32::from_str_radix(id_hex, 16).map_err(|_| error("bad ID"))?;
    let extended = id_hex.len() > 3;
    let (flags, data) = if let Some(fd) = rest.strip_prefix('#') {
        let flag_char = fd.chars().next().ok_or(error("missing FD flags"))?;
        let flags = flag_char.to_digit(16).ok_or(error("bad FD flags"))? as u8;
        let data = parse_hex_bytes(fd.get(1..).unwrap_or("")).ok_or(error("bad FD data"))?;
        (flags | FLAG_FDF, data)
    } else if rest.starts_with('R') {
        (0, Vec::new())
    } else {
        (0, parse_hex_bytes(rest).ok_or(error("bad data"))?)
    };
    let frame = Frame { timestamp_ns, id, extended, flags, data };
    Ok(Some(LogFrame { interface: interface.to_string(), frame }))
}

/// Every frame of a log, in order; stops at the first read error.
pub fn read<R: BufRead>(reader: R) -> impl Iterator<Item = Result<LogFrame, Box<dyn std::error::Error>>> {
    reader.lines().enumerate().filter_map(|(index, line)| match line {
        Ok(line) => parse_line(&line, index + 1).map_err(Into::into).transpose(),
        Err(error) => Some(Err(error.into())),
    })
}

/// A whole log held in memory (tests, short recordings).
pub fn parse_str(text: &str) -> Result<Vec<LogFrame>, ParseError> {
    text.lines().enumerate().filter_map(|(index, line)| parse_line(line, index + 1).transpose()).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fd_classic_and_remote_lines() {
        let fd = parse_line("(1795639172.670876) can0 068##5CB2EFFF70008FD64", 1).unwrap().unwrap();
        assert_eq!(fd.interface, "can0");
        assert_eq!(fd.frame.timestamp_ns, 1_795_639_172_670_876_000);
        assert_eq!(fd.frame.id, 0x068);
        assert_eq!(fd.frame.flags, 0x5 | FLAG_FDF);
        assert_eq!(fd.frame.data, [0xCB, 0x2E, 0xFF, 0xF7, 0x00, 0x08, 0xFD, 0x64]);
        let classic = parse_line("(1.5) vcan0 12345678#0102", 2).unwrap().unwrap();
        assert!(classic.frame.extended && !classic.frame.is_fd());
        assert_eq!((classic.frame.timestamp_ns, classic.frame.data.as_slice()), (1_500_000_000, &[1u8, 2][..]));
        assert!(parse_line("(2.0) can0 123#R", 3).unwrap().unwrap().frame.data.is_empty());
        assert_eq!(parse_line("   ", 4), Ok(None));
        assert_eq!(parse_line("(x) can0 123#00", 5).unwrap_err().reason, "bad timestamp");
        assert_eq!(parse_line("(1.0) can0 123##5ABC", 6).unwrap_err().reason, "bad FD data");
    }
}
