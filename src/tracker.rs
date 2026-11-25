//! Per-ID rate and staleness, driven only by frame timestamps: no timers, no clock reads. "Now" is the newest
//! frame seen on the bus, so a replayed log reports what the live bus would have.

use std::collections::BTreeMap;

use serde::Serialize;

use crate::frame::Frame;

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize)]
pub struct IdStats {
    pub count: u64,
    pub first_ns: u64,
    pub last_ns: u64,
    pub min_gap_ns: Option<u64>,
    pub max_gap_ns: Option<u64>,
    /// Payload length -> frames.
    pub lengths: BTreeMap<usize, u64>,
    /// Frames carrying the ESI flag (HDAS's own transmits, per PROTOCOL.md).
    pub esi_count: u64,
}

impl IdStats {
    /// Mean rate over everything seen so far.
    pub fn rate_hz(&self) -> Option<f64> {
        let span_ns = self.last_ns.checked_sub(self.first_ns).filter(|&span| span > 0)?;
        Some((self.count - 1) as f64 * 1e9 / span_ns as f64)
    }

    pub fn mean_gap_ns(&self) -> Option<f64> {
        let span_ns = self.last_ns.checked_sub(self.first_ns)?;
        (self.count > 1).then(|| span_ns as f64 / (self.count - 1) as f64)
    }
}

#[derive(Clone, Debug, Default)]
pub struct Tracker {
    ids: BTreeMap<u32, IdStats>,
    newest_ns: u64,
}

impl Tracker {
    pub fn new() -> Tracker {
        Tracker::default()
    }

    pub fn observe(&mut self, frame: &Frame) {
        let stats = self.ids.entry(frame.id).or_default();
        if stats.count == 0 {
            stats.first_ns = frame.timestamp_ns;
        } else {
            let gap = frame.timestamp_ns.saturating_sub(stats.last_ns);
            stats.min_gap_ns = Some(stats.min_gap_ns.map_or(gap, |min| min.min(gap)));
            stats.max_gap_ns = Some(stats.max_gap_ns.map_or(gap, |max| max.max(gap)));
        }
        stats.count += 1;
        stats.last_ns = stats.last_ns.max(frame.timestamp_ns);
        *stats.lengths.entry(frame.data.len()).or_default() += 1;
        stats.esi_count += u64::from(frame.from_vendor_host());
        self.newest_ns = self.newest_ns.max(frame.timestamp_ns);
    }

    /// The newest frame time on the bus (the tracker's "now").
    pub fn newest_ns(&self) -> u64 {
        self.newest_ns
    }

    pub fn stats(&self, id: u32) -> Option<&IdStats> {
        self.ids.get(&id)
    }

    pub fn ids(&self) -> impl Iterator<Item = (u32, &IdStats)> {
        self.ids.iter().map(|(&id, stats)| (id, stats))
    }

    /// Bus time since `id` was last seen; None if never seen.
    pub fn staleness_ns(&self, id: u32) -> Option<u64> {
        Some(self.newest_ns.saturating_sub(self.ids.get(&id)?.last_ns))
    }

    /// IDs silent for longer than `missed_periods` of their own mean gap (the caller picks the multiple).
    pub fn stale_ids(&self, missed_periods: f64) -> Vec<u32> {
        self.ids
            .iter()
            .filter(|(_, stats)| {
                stats.mean_gap_ns().is_some_and(|gap| (self.newest_ns - stats.last_ns) as f64 > missed_periods * gap)
            })
            .map(|(&id, _)| id)
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(id: u32, timestamp_ns: u64) -> Frame {
        Frame { timestamp_ns, id, extended: false, flags: 0, data: vec![0; 8] }
    }

    #[test]
    fn rate_and_staleness_follow_frame_times() {
        let mut tracker = Tracker::new();
        for tick in 0..=100 {
            tracker.observe(&frame(0x020, tick * 5_000_000));
            if tick <= 10 {
                tracker.observe(&frame(0x021, tick * 5_000_000));
            }
        }
        let imu = tracker.stats(0x020).unwrap();
        assert!((imu.rate_hz().unwrap() - 200.0).abs() < 1e-9);
        assert_eq!((imu.min_gap_ns, imu.max_gap_ns), (Some(5_000_000), Some(5_000_000)));
        assert_eq!(tracker.staleness_ns(0x020), Some(0));
        assert_eq!(tracker.staleness_ns(0x021), Some(450_000_000));
        assert_eq!(tracker.stale_ids(3.0), vec![0x021]);
        assert_eq!(tracker.staleness_ns(0x999), None);
    }
}
