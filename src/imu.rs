//! 0x020 chassis IMU (CCU, 200 Hz) and 0x045 torso IMU (TCU, 100 Hz): the same 20-byte layout, 9 big-endian i16s.
//!
//! Sources: `ProtocolCCU::decode_imu` lpm@0x124600 and `ProtocolTorso::decode_imu` lpm@0x1405e0. HDAS publishes the
//! raw degrees / g values and builds its quaternion from degrees treated as radians (HDAS@0x16f930 / 0x16ad00); both
//! the correct and the vendor quaternion are here.

use serde::Serialize;

use crate::bytes::{i16s_be, u16_be};

/// roll and pitch counts per degree (decode_imu lpm@0x124600: raw / 100).
pub const ROLL_PITCH_COUNTS_PER_DEG: f64 = 100.0;
/// yaw counts per degree (decode_imu lpm@0x124600: raw / 50, so yaw spans +-655 deg).
pub const YAW_COUNTS_PER_DEG: f64 = 50.0;
/// gyro counts per deg/s (decode_imu lpm@0x124600: raw / 100; deg/s is inferred, HDAS copies the value raw).
pub const GYRO_COUNTS_PER_DEG_S: f64 = 100.0;
/// accelerometer counts per g (decode_imu lpm@0x124600: raw / 5000; |a| = 0.997..1.004 g in the capture).
pub const ACCEL_COUNTS_PER_G: f64 = 5000.0;
/// Standard gravity, for g -> m/s^2 (CGPM 1901 definition, not a tuning value).
pub const STANDARD_GRAVITY_M_S2: f64 = 9.80665;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ImuLocation {
    /// 0x020, published by HDAS as `chassis_imu` on /hdas/imu_chassis.
    Chassis,
    /// 0x045, `torso_imu` on /hdas/imu_torso.
    Torso,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Imu {
    pub location: ImuLocation,
    pub roll_deg: f64,
    pub pitch_deg: f64,
    pub yaw_deg: f64,
    /// x, y, z; unit inferred as deg/s (not proven by the vendor code).
    pub gyro_deg_s: [f64; 3],
    /// x, y, z in g; z is about -1 when level.
    pub acceleration_g: [f64; 3],
    /// Bytes 18-19, not read by HDAS (0 on 0x020, a constant 600 on 0x045).
    pub unread_tail: Option<u16>,
}

impl Imu {
    pub const LEN: usize = 18;

    pub fn decode(location: ImuLocation, data: &[u8]) -> Option<Imu> {
        let [roll, pitch, yaw, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z] = i16s_be::<9>(data, 0)?;
        let gyro = |raw: i16| f64::from(raw) / GYRO_COUNTS_PER_DEG_S;
        let accel = |raw: i16| f64::from(raw) / ACCEL_COUNTS_PER_G;
        Some(Imu {
            location,
            roll_deg: f64::from(roll) / ROLL_PITCH_COUNTS_PER_DEG,
            pitch_deg: f64::from(pitch) / ROLL_PITCH_COUNTS_PER_DEG,
            yaw_deg: f64::from(yaw) / YAW_COUNTS_PER_DEG,
            gyro_deg_s: [gyro(gyro_x), gyro(gyro_y), gyro(gyro_z)],
            acceleration_g: [accel(acc_x), accel(acc_y), accel(acc_z)],
            unread_tail: u16_be(data, 18),
        })
    }

    pub fn acceleration_norm_g(&self) -> f64 {
        self.acceleration_g.iter().map(|a| a * a).sum::<f64>().sqrt()
    }

    pub fn acceleration_m_s2(&self) -> [f64; 3] {
        self.acceleration_g.map(|a| a * STANDARD_GRAVITY_M_S2)
    }

    /// Gyro in rad/s, assuming the inferred deg/s unit.
    pub fn gyro_rad_s(&self) -> [f64; 3] {
        self.gyro_deg_s.map(f64::to_radians)
    }

    /// Orientation as (x, y, z, w) from roll/pitch/yaw in radians (ZYX).
    pub fn quaternion_xyzw(&self) -> [f64; 4] {
        rpy_to_quaternion_xyzw(self.roll_deg.to_radians(), self.pitch_deg.to_radians(), self.yaw_deg.to_radians())
    }

    /// What HDAS publishes: the same formula fed the degree values as if they were radians (HDAS@0x16f930, sic).
    pub fn vendor_quaternion_xyzw(&self) -> [f64; 4] {
        rpy_to_quaternion_xyzw(self.roll_deg, self.pitch_deg, self.yaw_deg)
    }
}

fn rpy_to_quaternion_xyzw(roll: f64, pitch: f64, yaw: f64) -> [f64; 4] {
    let (sin_roll, cos_roll) = (roll * 0.5).sin_cos();
    let (sin_pitch, cos_pitch) = (pitch * 0.5).sin_cos();
    let (sin_yaw, cos_yaw) = (yaw * 0.5).sin_cos();
    [
        cos_yaw * cos_pitch * sin_roll - sin_yaw * sin_pitch * cos_roll,
        sin_yaw * cos_pitch * sin_roll + cos_yaw * sin_pitch * cos_roll,
        sin_yaw * cos_pitch * cos_roll - cos_yaw * sin_pitch * sin_roll,
        sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_pitch * cos_roll,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn level_and_still() {
        let mut data = [0u8; 20];
        data[16..18].copy_from_slice(&(-5000i16).to_be_bytes());
        let imu = Imu::decode(ImuLocation::Chassis, &data).unwrap();
        assert_eq!(imu.acceleration_g, [0.0, 0.0, -1.0]);
        assert_eq!(imu.quaternion_xyzw(), [0.0, 0.0, 0.0, 1.0]);
        assert!(Imu::decode(ImuLocation::Torso, &data[..17]).is_none());
    }
}
