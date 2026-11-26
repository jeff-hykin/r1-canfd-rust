//! Multi-axis online trajectory generation in the spirit of Reflexxes Type II (`libReflexxesTypeII.so`, which jt
//! calls through `ReflexxesAPI::RMLPosition` every cycle): velocity- and acceleration-limited (jerk unlimited, as in
//! Type II), from the generator's own current state to a resting target, all axes time-synchronized so they arrive
//! together. Re-planned on every step, like the vendor's online use.
//!
//! Not a line-by-line port of RML Type II: each axis takes the profile "change speed to `v_c` at full acceleration,
//! cruise, brake to rest", with the slowest axis's minimum time shared by all. Same limits, same endpoints, same
//! synchronized arrival; intermediate samples can differ slightly from RML's.

/// Bisection steps: far below a micro-radian on the R1's ranges.
const BISECTIONS: usize = 100;

/// One axis's profile toward rest at `distance`: cruise speed `cruise` reached at `acceleration`, total `duration`.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Profile {
    start_velocity: f64,
    cruise: f64,
    acceleration: f64,
    duration: f64,
}

impl Profile {
    fn speed_change_time(&self) -> f64 {
        (self.cruise - self.start_velocity).abs() / self.acceleration
    }

    fn brake_time(&self) -> f64 {
        self.cruise.abs() / self.acceleration
    }

    /// Displacement and velocity `t` seconds in.
    fn sample(&self, t: f64) -> (f64, f64) {
        let (v0, vc, a) = (self.start_velocity, self.cruise, self.acceleration);
        let (t1, t3) = (self.speed_change_time(), self.brake_time());
        let t2 = (self.duration - t1 - t3).max(0.0);
        let toward_cruise = if vc >= v0 { a } else { -a };
        if t <= t1 {
            return (v0 * t + 0.5 * toward_cruise * t * t, v0 + toward_cruise * t);
        }
        let after_change = (v0 + vc) / 2.0 * t1;
        if t <= t1 + t2 {
            return (after_change + vc * (t - t1), vc);
        }
        let braking = (t - t1 - t2).min(t3);
        let toward_rest = if vc >= 0.0 { -a } else { a };
        (after_change + vc * t2 + vc * braking + 0.5 * toward_rest * braking * braking, vc + toward_rest * braking)
    }

    fn distance(&self) -> f64 {
        let (t1, t3) = (self.speed_change_time(), self.brake_time());
        let t2 = (self.duration - t1 - t3).max(0.0);
        (self.start_velocity + self.cruise) / 2.0 * t1 + self.cruise * t2 + self.cruise / 2.0 * t3
    }
}

/// Cruise speeds a profile of `duration` can use: |v_c - v0| + |v_c| <= a T, |v_c| <= v_max. `None` if it cannot
/// even stop in time.
fn cruise_range(v0: f64, max_velocity: f64, acceleration: f64, duration: f64) -> Option<(f64, f64)> {
    let budget = acceleration * duration;
    if budget + 1e-12 < v0.abs() {
        return None;
    }
    let high = ((budget + v0) / 2.0).min(max_velocity);
    let low = ((v0 - budget) / 2.0).max(-max_velocity);
    (low <= high).then_some((low, high))
}

/// The profile of `duration` covering `distance`, if one exists (distance grows with the cruise speed).
fn profile_for(distance: f64, v0: f64, max_velocity: f64, acceleration: f64, duration: f64) -> Option<Profile> {
    let (low, high) = cruise_range(v0, max_velocity, acceleration, duration)?;
    let profile = |cruise| Profile { start_velocity: v0, cruise, acceleration, duration };
    let tolerance = 1e-12 * (1.0 + distance.abs());
    if profile(low).distance() > distance + tolerance || profile(high).distance() < distance - tolerance {
        return None;
    }
    let (mut low, mut high) = (low, high);
    for _ in 0..BISECTIONS {
        let middle = 0.5 * (low + high);
        if profile(middle).distance() < distance {
            low = middle;
        } else {
            high = middle;
        }
    }
    Some(profile(0.5 * (low + high)))
}

/// The shortest time an axis needs to come to rest at `distance`.
fn minimum_time(distance: f64, v0: f64, max_velocity: f64, acceleration: f64) -> f64 {
    if !(max_velocity > 0.0 && acceleration > 0.0 && distance.is_finite() && v0.is_finite()) {
        return f64::INFINITY;
    }
    let reachable = |duration| profile_for(distance, v0, max_velocity, acceleration, duration).is_some();
    let mut low = v0.abs() / acceleration;
    if reachable(low) {
        return low;
    }
    let mut high = low.max(1e-3);
    while !reachable(high) {
        high *= 2.0;
    }
    for _ in 0..BISECTIONS {
        let middle = 0.5 * (low + high);
        if reachable(middle) {
            high = middle;
        } else {
            low = middle;
        }
    }
    high
}

/// N axes' state: position and velocity, stepped toward a resting target.
#[derive(Clone, Debug, PartialEq)]
pub struct SyncOtg<const N: usize> {
    pub position: [f64; N],
    pub velocity: [f64; N],
}

impl<const N: usize> SyncOtg<N> {
    pub fn at_rest(position: [f64; N]) -> SyncOtg<N> {
        SyncOtg { position, velocity: [0.0; N] }
    }

    /// One `cycle_s` step toward `target` (zero target velocity) under per-axis limits; the new state.
    pub fn step(
        &mut self,
        target: &[f64; N],
        max_velocity: &[f64; N],
        max_acceleration: &[f64; N],
        cycle_s: f64,
    ) -> &SyncOtg<N> {
        let distance: [f64; N] = std::array::from_fn(|axis| target[axis] - self.position[axis]);
        let duration = (0..N)
            .map(|axis| minimum_time(distance[axis], self.velocity[axis], max_velocity[axis], max_acceleration[axis]))
            .fold(0.0, f64::max);
        if !duration.is_finite() {
            // a zero or invalid limit: Reflexxes rejects the input, the axes stay where they are
            self.velocity = [0.0; N];
            return self;
        }
        for axis in 0..N {
            let (v0, vmax, a) = (self.velocity[axis], max_velocity[axis], max_acceleration[axis]);
            if duration <= cycle_s {
                // lands this cycle, as Reflexxes reports the final state reached
                self.position[axis] = target[axis];
                self.velocity[axis] = 0.0;
                continue;
            }
            let profile = profile_for(distance[axis], v0, vmax, a, duration).unwrap_or_else(|| {
                let own = minimum_time(distance[axis], v0, vmax, a);
                profile_for(distance[axis], v0, vmax, a, own).unwrap_or(Profile {
                    start_velocity: v0,
                    cruise: 0.0,
                    acceleration: a,
                    duration: own,
                })
            });
            let (moved, velocity) = profile.sample(cycle_s);
            self.position[axis] += moved;
            self.velocity[axis] = velocity;
        }
        self
    }

    /// The final state is reached (every axis at the target and at rest).
    pub fn reached(&self, target: &[f64; N]) -> bool {
        (0..N).all(|axis| self.position[axis] == target[axis] && self.velocity[axis] == 0.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rest_to_rest_axes_arrive_together_within_limits() {
        let mut otg = SyncOtg::at_rest([0.0, 0.0]);
        let (target, vmax, amax, dt) = ([1.0, 0.1], [5.0, 5.0], [20.0, 20.0], 0.004);
        let mut steps = 0usize;
        let mut arrivals = [None, None];
        while !otg.reached(&target) {
            let before = otg.velocity;
            otg.step(&target, &vmax, &amax, dt);
            steps += 1;
            for axis in 0..2 {
                assert!(otg.velocity[axis].abs() <= vmax[axis] + 1e-9);
                assert!((otg.velocity[axis] - before[axis]).abs() <= amax[axis] * dt + 1e-9, "{axis} {steps}");
                if arrivals[axis].is_none() && (otg.position[axis] - target[axis]).abs() < 1e-6 {
                    arrivals[axis] = Some(steps);
                }
            }
            assert!(steps < 1000);
        }
        // 1 rad at 20 rad/s^2 without reaching 5 rad/s: a triangle of 2 sqrt(1 / 20) = 0.447 s
        assert!((steps as f64 * dt - 0.447).abs() < 0.01, "{steps}");
        let [first, second] = arrivals.map(Option::unwrap);
        assert!(first.abs_diff(second) <= 2, "{arrivals:?}");
    }

    #[test]
    fn a_new_target_mid_motion_reverses_without_breaking_the_limits() {
        let mut otg = SyncOtg::at_rest([0.0]);
        for _ in 0..50 {
            otg.step(&[2.0], &[5.0], &[20.0], 0.004);
        }
        assert!(otg.velocity[0] > 1.0);
        let mut steps = 0;
        while !otg.reached(&[0.0]) {
            let before = otg.velocity[0];
            otg.step(&[0.0], &[5.0], &[20.0], 0.004);
            assert!((otg.velocity[0] - before).abs() <= 20.0 * 0.004 + 1e-9);
            steps += 1;
            assert!(steps < 1000);
        }
    }
}
