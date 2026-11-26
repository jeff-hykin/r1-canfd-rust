//! jt's live t_ff against our gravity port on the vendor URDF. The URDF is Galaxea's and is not shipped here: point
//! R1_VENDOR_URDF at a copy of the robot's mobiman/share/mobiman/urdf/R1_PRO/urdf/r1_pro.urdf, or the test skips.

use r1_canfd::gravity::GravityModel;

/// `ros2 topic echo --once` on r1-pro-0219, 2026-11-25, both arms holding (jt running, HDAS feedback).
const TORSO_Q: [f64; 4] = [0.0020000000949949026, -0.005499999970197678, -0.08669999986886978, 0.00279999990016222];
const LEFT_Q: [f64; 7] = [
    -0.00978723404255319,
    0.03489361702127659,
    0.14042553191489363,
    0.06680851063829787,
    -0.06595744680851064,
    -0.6310638297872341,
    0.08361702127659575,
];
const LEFT_P_DES: [f64; 7] = [
    -0.010212766006588936,
    0.03531914949417114,
    0.14085106551647186,
    0.06680851429700851,
    -0.0659574493765831,
    -0.6312766075134277,
    0.08361702412366867,
];
const LEFT_T_FF: [f64; 7] = [
    1.0417283773422241,
    0.5850086212158203,
    0.008788715116679668,
    0.19158071279525757,
    0.023267703130841255,
    -0.45218852162361145,
    0.07768520712852478,
];
const RIGHT_Q: [f64; 7] = [
    0.09,
    0.0019148936170212765,
    -0.06106382978723404,
    -0.2680851063829787,
    0.0,
    -0.18851063829787235,
    0.14446808510638298,
];
const RIGHT_T_FF: [f64; 7] = [
    1.0876415967941284,
    0.0858222097158432,
    0.004969343543052673,
    -0.7564567923545837,
    -0.01437430176883936,
    -0.3329215347766876,
    0.09492450952529907,
];

fn model(side: &str) -> Option<GravityModel> {
    let path = std::env::var("R1_VENDOR_URDF").ok()?;
    let urdf = std::fs::read_to_string(path).ok()?;
    let mut joints: Vec<String> = (1..=4).map(|i| format!("torso_joint{i}")).collect();
    joints.extend((1..=7).map(|i| format!("{side}_arm_joint{i}")));
    let names: Vec<&str> = joints.iter().map(String::as_str).collect();
    Some(GravityModel::from_urdf(&urdf, &names).unwrap())
}

const RIGHT_P_DES: [f64; 7] = [
    0.0904255285859108,
    0.0019148936262354255,
    -0.06106382980942726,
    -0.268510639667511,
    0.0,
    -0.18851064145565033,
    0.14446808397769928,
];

/// jt's t_ff is g(q) on [torso feedback q, arm p_des] (the planner's position, not the arm feedback): matches to f32.
#[test]
fn gravity_reproduces_jt_t_ff() {
    for (side, arm, t_ff) in [("left", LEFT_P_DES, LEFT_T_FF), ("right", RIGHT_P_DES, RIGHT_T_FF)] {
        let Some(model) = model(side) else {
            eprintln!("R1_VENDOR_URDF unset: skipped");
            return;
        };
        let q: Vec<f64> = TORSO_Q.iter().chain(arm.iter()).copied().collect();
        let g = model.torques(&q);
        for joint in 0..7 {
            assert!(
                (g[4 + joint] - t_ff[joint]).abs() < 1e-6,
                "{side} joint {joint}: {} vs {}",
                g[4 + joint],
                t_ff[joint]
            );
        }
        // the arm feedback q gives a visibly different torque, so the planner position is what jt uses
        let q_feedback: Vec<f64> =
            TORSO_Q.iter().chain(if side == "left" { LEFT_Q } else { RIGHT_Q }.iter()).copied().collect();
        assert!((model.torques(&q_feedback)[4] - t_ff[0]).abs() > 1e-3);
    }
}

/// The whole jt port holding the captured left target with the captured torso: the 0x050 bytes HDAS would encode
/// from jt's published MotorControl.
#[test]
fn arm_tracker_hold_frame_matches_jt() {
    use r1_canfd::arm::{ArmFeedback, Side};
    use r1_canfd::commander::ArmCommander;
    use r1_canfd::joint_tracker::{ArmTracker, TrackerSettings};

    let Ok(path) = std::env::var("R1_VENDOR_URDF") else {
        eprintln!("R1_VENDOR_URDF unset: skipped");
        return;
    };
    let urdf = std::fs::read_to_string(path).unwrap();
    let settings = TrackerSettings::vendor(Some(&urdf)).unwrap();
    let mut arm = ArmTracker::new(Side::Left, &settings);
    arm.on_arm_feedback(&ArmFeedback {
        side: Side::Left,
        position_rad: LEFT_P_DES.map(|value| value as f32 as f64),
        velocity_rad_s: [0.0; 7],
        torque_nm: [0.0; 7],
        unread_tail: vec![],
    });
    arm.on_target(&LEFT_P_DES.map(|value| value as f32 as f64), &[0.5; 7]).unwrap();
    arm.apply_target(TORSO_Q);
    let ours = arm.tick().unwrap();
    let theirs = ArmCommander::new(Side::Left)
        .set_joint_targets(
            &LEFT_P_DES.map(|value| value as f32),
            &[0.4; 7],
            &[2.0; 7],
            &[25.0; 7],
            &LEFT_T_FF.map(|value| value as f32),
        )
        .unwrap();
    assert_eq!(ours.data, theirs.data);
    // the torso pose on the robot sits inside jt's bounding box (or the torso would freeze at startup)
    let torso = settings.torso_kinematics.as_ref().unwrap();
    let (position, rpy) = torso.link_pose(&TORSO_Q, "torso_link4").unwrap();
    assert!(r1_canfd::joint_tracker::VENDOR_TORSO_BOX.contains(position, rpy), "{position:?} {rpy:?}");
}
