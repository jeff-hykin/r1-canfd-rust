//! Wheel odometry: chassis_control_node's forward kinematics (`ccn`), then a planar pose integrator, which is new
//! (neither HDAS nor ccn integrates a pose).

use serde::Serialize;

use crate::chassis::MotorStatus;
use crate::swerve::{forward_kinematics, forward_kinematics_least_squares, Twist, VENDOR_GEOMETRY};

/// World-frame planar pose from wheel feedback alone.
#[derive(Clone, Copy, Debug, PartialEq, Default, Serialize)]
pub struct Pose2d {
    pub x_m: f64,
    pub y_m: f64,
    pub yaw_rad: f64,
}

/// Which FK feeds the integrator.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum OdometryKinematics {
    /// Rigid-body least squares: exact for any rigid motion (default; the vendor FK below is biased, and the vendor
    /// never integrates a pose, so there is no vendor behaviour to match).
    #[default]
    RigidLeastSquares,
    /// ccn's FK (ccn@0x128d30) as /motion_control/chassis_speed reports it: turning at wz adds wz (2a - r) / 3 to vy
    /// and lateral motion adds a spurious wz (about 0.033 rad/s per m/s).
    Vendor,
}

/// Integrates FK twists from consecutive 0x030 frames, stepping by the frames' own timestamps (event driven).
#[derive(Clone, Debug, Default)]
pub struct WheelOdometry {
    pub kinematics: OdometryKinematics,
    pub pose: Pose2d,
    /// Body-frame twist of the latest frame.
    pub twist: Twist,
    last_timestamp_ns: Option<u64>,
}

impl WheelOdometry {
    pub fn new() -> WheelOdometry {
        WheelOdometry::default()
    }

    /// One 0x030 frame: FK its wheels, then integrate the previous twist over the gap (midpoint heading).
    pub fn on_motor_status(&mut self, timestamp_ns: u64, status: &MotorStatus) -> Pose2d {
        let (speeds, angles) = (&status.wheel_speed_m_s, &status.steer_angle_rad);
        let twist = match self.kinematics {
            OdometryKinematics::RigidLeastSquares => forward_kinematics_least_squares(&VENDOR_GEOMETRY, speeds, angles),
            OdometryKinematics::Vendor => forward_kinematics(&VENDOR_GEOMETRY, speeds, angles),
        };
        if let Some(last) = self.last_timestamp_ns {
            let dt = timestamp_ns.saturating_sub(last) as f64 * 1e-9;
            self.pose = integrate(self.pose, self.twist, dt);
        }
        self.last_timestamp_ns = Some(timestamp_ns);
        self.twist = twist;
        self.pose
    }
}

/// Exact constant-twist (arc) step of a body twist over `dt` seconds.
pub fn integrate(pose: Pose2d, twist: Twist, dt: f64) -> Pose2d {
    let dyaw = twist.wz_rad_s * dt;
    let (dx_body, dy_body) = if dyaw.abs() < f64::EPSILON {
        (twist.vx_m_s * dt, twist.vy_m_s * dt)
    } else {
        let (sin, cos) = dyaw.sin_cos();
        let (a, b) = (sin / twist.wz_rad_s, (1.0 - cos) / twist.wz_rad_s);
        (twist.vx_m_s * a - twist.vy_m_s * b, twist.vx_m_s * b + twist.vy_m_s * a)
    };
    let (sin, cos) = pose.yaw_rad.sin_cos();
    Pose2d {
        x_m: pose.x_m + cos * dx_body - sin * dy_body,
        y_m: pose.y_m + sin * dx_body + cos * dy_body,
        yaw_rad: pose.yaw_rad + dyaw,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn straight_line_and_full_circle() {
        let forward = integrate(Pose2d::default(), Twist { vx_m_s: 0.5, vy_m_s: 0.0, wz_rad_s: 0.0 }, 2.0);
        assert!((forward.x_m - 1.0).abs() < 1e-12 && forward.y_m.abs() < 1e-12);
        // 1 m/s on a 1 m radius: 2 pi seconds in 1000 steps comes back to the start
        let mut pose = Pose2d::default();
        let turn = Twist { vx_m_s: 1.0, vy_m_s: 0.0, wz_rad_s: 1.0 };
        for _ in 0..1000 {
            pose = integrate(pose, turn, std::f64::consts::TAU / 1000.0);
        }
        assert!(pose.x_m.abs() < 1e-9 && pose.y_m.abs() < 1e-9, "{pose:?}");
        assert!((pose.yaw_rad - std::f64::consts::TAU).abs() < 1e-9);
    }
}
