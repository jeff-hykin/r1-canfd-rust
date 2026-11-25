//! Decode the R1 Pro bus live (Linux, receive only) or from a `candump -L` log.
//!
//! r1_canfd_dump <can0 | capture.log> [--summary] [--summary-every SECONDS] [--json] [--frames N] [--id 0x052,0x020]
//!
//! Default: one decoded frame per line. `--summary`: a per-ID table (count, rate, gaps, staleness, lengths) at the
//! end; `--summary-every` prints it each time that much bus time has passed (frame timestamps, not a timer).
//! `--json`: JSON lines instead of text, for frames and summaries alike.

use std::collections::BTreeSet;
use std::io::{self, BufReader, Write};
use std::process::ExitCode;

use r1_canfd::device::{device, device_name};
use r1_canfd::{candump, decode, Frame, Tracker};

struct Options {
    source: String,
    summary: bool,
    summary_every_ns: Option<u64>,
    json: bool,
    max_frames: Option<u64>,
    ids: Option<BTreeSet<u32>>,
}

const USAGE: &str = "usage: r1_canfd_dump <can0 | capture.log> [--summary] [--summary-every SECONDS] [--json] [--frames N] [--id 0x052,0x020]";

fn parse_id(text: &str) -> Result<u32, String> {
    let digits = text.trim().trim_start_matches("0x").trim_start_matches("0X");
    u32::from_str_radix(digits, 16).map_err(|_| format!("bad ID {text:?}"))
}

fn parse_options() -> Result<Options, String> {
    let mut arguments = std::env::args().skip(1);
    let mut options = Options {
        source: String::new(),
        summary: false,
        summary_every_ns: None,
        json: false,
        max_frames: None,
        ids: None,
    };
    while let Some(argument) = arguments.next() {
        let mut value = || arguments.next().ok_or(format!("{argument} needs a value"));
        match argument.as_str() {
            "--summary" => options.summary = true,
            "--json" => options.json = true,
            "--summary-every" => {
                let seconds: f64 = value()?.parse().map_err(|_| "bad --summary-every")?;
                options.summary_every_ns = Some((seconds * 1e9) as u64);
                options.summary = true;
            }
            "--frames" => options.max_frames = Some(value()?.parse().map_err(|_| "bad --frames")?),
            "--id" => options.ids = Some(value()?.split(',').map(parse_id).collect::<Result<_, _>>()?),
            "-h" | "--help" => return Err(USAGE.to_string()),
            other if options.source.is_empty() && !other.starts_with('-') => options.source = other.to_string(),
            other => return Err(format!("unexpected argument {other:?}\n{USAGE}")),
        }
    }
    if options.source.is_empty() {
        return Err(USAGE.to_string());
    }
    Ok(options)
}

fn print_frame(out: &mut impl Write, frame: &Frame, json: bool) -> io::Result<()> {
    let decoded = decode(frame);
    if json {
        let line = serde_json::json!({
            "timestamp_ns": frame.timestamp_ns,
            "id": frame.id,
            "name": device_name(frame.id),
            "flags": frame.flags,
            "decoded": decoded,
        });
        writeln!(out, "{line}")
    } else {
        let seconds = frame.timestamp_ns as f64 / 1e9;
        writeln!(out, "{seconds:.6} 0x{:03X} {:<20} {decoded:?}", frame.id, device_name(frame.id))
    }
}

fn print_summary(out: &mut impl Write, tracker: &Tracker, json: bool) -> io::Result<()> {
    for (id, stats) in tracker.ids() {
        let staleness_ms = tracker.staleness_ns(id).unwrap_or(0) as f64 / 1e6;
        let evidence = device(id).map(|device| device.evidence);
        if json {
            let line = serde_json::json!({
                "id": id,
                "name": device_name(id),
                "evidence": evidence,
                "rate_hz": stats.rate_hz(),
                "staleness_ms": staleness_ms,
                "stats": stats,
            });
            writeln!(out, "{line}")?;
            continue;
        }
        let gap_ms = |gap: Option<u64>| gap.map_or(0.0, |gap| gap as f64 / 1e6);
        let lengths: Vec<String> = stats.lengths.iter().map(|(len, count)| format!("{len}:{count}")).collect();
        writeln!(
            out,
            "0x{id:03X} {:<20} n={:6} rate={:7.1} Hz gap={:7.2}..{:7.2} ms stale={:7.1} ms len={{{}}} esi={} {}",
            device_name(id),
            stats.count,
            stats.rate_hz().unwrap_or(0.0),
            gap_ms(stats.min_gap_ns),
            gap_ms(stats.max_gap_ns),
            staleness_ms,
            lengths.join(","),
            stats.esi_count,
            evidence.map_or("not in the device table".to_string(), |evidence| format!("{evidence:?}").to_lowercase()),
        )?;
    }
    Ok(())
}

fn frames(source: &str) -> Result<Box<dyn Iterator<Item = Result<Frame, String>>>, String> {
    if std::path::Path::new(source).is_file() {
        let file = std::fs::File::open(source).map_err(|error| format!("{source}: {error}"))?;
        let frames =
            candump::read(BufReader::new(file)).map(|line| line.map(|line| line.frame).map_err(|e| e.to_string()));
        return Ok(Box::new(frames));
    }
    open_interface(source)
}

#[cfg(target_os = "linux")]
fn open_interface(interface: &str) -> Result<Box<dyn Iterator<Item = Result<Frame, String>>>, String> {
    let bus = r1_canfd::Bus::open(interface).map_err(|error| error.to_string())?;
    Ok(Box::new(bus.map(|frame| frame.map_err(|error| error.to_string()))))
}

#[cfg(not(target_os = "linux"))]
fn open_interface(source: &str) -> Result<Box<dyn Iterator<Item = Result<Frame, String>>>, String> {
    Err(format!("{source} is not a file, and live interfaces need Linux"))
}

fn run(options: Options) -> Result<(), String> {
    let stdout = io::stdout();
    let mut out = stdout.lock();
    let mut tracker = Tracker::new();
    let mut next_summary_ns: Option<u64> = None;
    let mut seen = 0u64;
    let io_error = |error: io::Error| error.to_string();
    for frame in frames(&options.source)? {
        let frame = frame?;
        if options.ids.as_ref().is_some_and(|ids| !ids.contains(&frame.id)) {
            continue;
        }
        tracker.observe(&frame);
        if !options.summary {
            print_frame(&mut out, &frame, options.json).map_err(io_error)?;
        }
        if let Some(every_ns) = options.summary_every_ns {
            let due = *next_summary_ns.get_or_insert(frame.timestamp_ns + every_ns);
            if frame.timestamp_ns >= due {
                print_summary(&mut out, &tracker, options.json).map_err(io_error)?;
                writeln!(out).map_err(io_error)?;
                next_summary_ns = Some(due + every_ns);
            }
        }
        seen += 1;
        if options.max_frames.is_some_and(|max| seen >= max) {
            break;
        }
    }
    if options.summary {
        print_summary(&mut out, &tracker, options.json).map_err(io_error)?;
    }
    Ok(())
}

fn main() -> ExitCode {
    match parse_options().and_then(run) {
        Ok(()) => ExitCode::SUCCESS,
        Err(message) => {
            eprintln!("{message}");
            ExitCode::FAILURE
        }
    }
}
