# Send-side logic: ours vs the vendor

This compares the r1-canfd send side, one command path at a time, with the decompiled Galaxea R1 Pro stack (HDAS
atc_system V2.3.0 hotfix_2 plus the mobiman nodes that sit between dimos and HDAS).

**Address prefixes.** Addresses are Ghidra addresses (ELF vaddr + 0x100000).
- `lpm` = HDAS `libprotocol_modules.so`
- `comm` = `libcommunication.so`
- `HDAS` = the HDAS binary
- `ccn` = `mobiman/chassis_control_node`
- `rgc` = `mobiman/r1_gripper_controller`
- `lmm` = `libmobiman.so`
- `jt` = `r1_pro_jointTracker_demo_node`
- `ocs2pi` = `libocs2_pinocchio_interface.so` (Galaxea's ocs2 build, `install/ocs2_pinocchio_interface/lib`)

**What we replace.** dimos publishes these topics today (`connection.py`). No services, no brake or mode topics.

| dimos topic | Vendor node that consumes it | Reaches HDAS as | CAN ID |
|---|---|---|---|
| `/motion_target/target_speed_chassis` | chassis_control_node (with dimos's `chassis_gatekeeper.py`) | `/motion_control/control_chassis` | 0x031 |
| `/motion_target/target_joint_state_arm_*` | jointTracker | `/motion_control/control_arm_*` | 0x050 / 0x060 |
| `/motion_target/target_joint_state_torso` | jointTracker | `/motion_control/control_torso` | 0x041 |
| `/motion_target/target_position_gripper_*` (0..100) | r1_gripper_controller | `/motion_control/control_gripper_*` | 0x051 / 0x061 |

HDAS itself also sends 0x023 (fan level) on its own.

**Every path has the same TX tail**, in HDAS and in ours.

| Step | Vendor | Ours |
|---|---|---|
| Socket | `CAN_RAW` with FD frames | Same (`bus.rs`) |
| Write | `writeCANFD` comm@0x134830: synchronous in the ROS callback thread, boost `async_write` from a stack buffer | Commanders return frames; only a binary with the `transmit` feature can call `Bus::send`, a plain 72-byte `write` |
| Flags | Always `flags = 0x03` (BRS\|ESI) | The caller chooses `FLAG_BRS`, or `VENDOR_HOST_FLAGS` to match the vendor |
| Write errors | Dropped silently; no error callback is set | Returned to the caller |

## Arms (0x050 / 0x060)

| Step | Vendor | Ours (`joint_tracker::ArmTracker`, then `commander::ArmCommander`, `gate::check_arm`, `encode::encode_arm`) |
|---|---|---|
| Input | jt (walkthrough below): dimos's 7 targets, clamped, j7 soft limit, speed cap from the message's velocity; Reflexxes Type II every 4 ms; p_des = planner position, v_des = 0.4, kp = 2, kd = 25, t_ff = Pinocchio gravity on [torso planner q, planner q] | `ArmTracker`: the same intake, a synchronized trapezoid planner with the same limits, the same gains, `gravity::GravityModel` on Galaxea's `r1_pro.urdf`. `ArmCommander::set_joint_targets` stays available for callers giving all five arrays. |
| Gates | HDAS lambda20/21 HDAS@0x155a00/0x1573c0: 7 elements per array, otherwise dropped. Inclusive bounds p ±6.5, v ±40, kp [0, 500], kd [0, 200], t ±50; any violation drops the whole message (log throttled to 1/s). NaN passes (`b.mi`/`b.gt`). `mode` unused. | Same bounds and the same whole-message drop (`Rejected::ExceedsLimit`). NaN passes. Length is enforced by the array type. |
| Bias | When `calib_enable == "enable"`, `p -= bias`. Disabled on this robot. | `joint_bias_rad: Option<_>`, `None` by default |
| Encode | `a2_arm_analysis::arm_encode_impl` lpm@0x11f950, f32 math, truncation, the vendor's literal clamp values (0x752F/0x8AD1, 0x7D0 saturation, 0xFFF) | A line-by-line port. NaN encodes to 0. |
| Rate | One frame per message; jt publishes every 4 ms while moving and about every 5 ms while holding (its 5 ms target loop re-arms the planner) | The module runs the 4 ms planner and the 5 ms target loop; one frame per planner pass that has work |
| Timeout | None in HDAS or jt: jt holds its last target with gravity torque indefinitely | Same |
| E-stop | HDAS keeps forwarding; 0x001 is only published as status (HDAS@0x170180). jt has an RC-mode 0x10/0x11 "go home" path. | Same as HDAS: `Commanders::emergency_stop` is exposed and the frames are unchanged. The jt home path is not ported. |
| Startup | jt blocks until each limb's first feedback, seeds the planner from it, then unsubscribes; it publishes nothing for an arm until that arm's first target. Some vendor scripts call function frames 2, 5, 1 (`set_arm_mode_mit.sh`); the stock R1PROBody session does not. | Same: the first 0x052 / 0x062 seeds the planner, nothing is sent before the first target |

## Torso (0x041)

| Step | Vendor | Ours (`joint_tracker::TorsoTracker`, then `TorsoCommander::set`, `encode_torso`) |
|---|---|---|
| Input | jt torso target callback jt@0x17f150: 4 positions, no clamp; velocity (4) sets the speed cap min(\|v\|, 1.5) and acceleration 1.5 × that; one planner start per message. Torso loop jt@0x2339a0, 5 ms: p from Reflexxes, joint 3 clamped to ±2.95, v_des = [0.6, 1.0, 0.6, 1.0]; kp/kd/t_ff filled but **never encoded**; a bounding box on torso_link4 freezes the command | `TorsoTracker`: the same intake, planner, clamp, v_des and bounding box (FK on the same URDF) |
| Gates | HDAS@0x155214: 4 elements per array; `mode` must equal an fmix32 checksum of element 0 of each array (jt computes it at jt@0x235344; 42 on the robot today). No range checks. | The length is the type. The checksum is skipped because it only guards HDAS's ROS topic; it is ported as `gate::torso_checksum` / `check_torso` for anything taking ROS input. |
| Encode | `torso_analysis::encode_torso_impl` lpm@0x144ff0: p ×10000 (±3.14 → 0x7AA8/0x8558), v ×3000 (±10 → 0x7530/0x8AD0) | Port |
| Rate | 5 ms while moving; silent once the target is reached, until the next message | Same |
| Timeout / e-stop | None | Same |
| Startup | Blocks on the first `/hdas/feedback_torso`, seeds the planner, unsubscribes; nothing is sent before a target | Same (the first 0x042 seeds it) |

## Chassis (0x031)

| Step | Vendor (ccn `main` ccn@0x124df0, then HDAS) | Ours (`ChassisCommander`, `ChassisController::step`) |
|---|---|---|
| Authority | ccn starts in RC mode (`controller_control_mode` = 1, ccn@0x1d3358). Twists are ignored until `/controller` reports mode 5; mode 2 drives from the sticks (ccn@0x12a0a0). On the robot, dimos's gatekeeper remaps `/controller` and publishes mode 5 at 250 Hz. | `ChassisAuthority::RemoteMode` follows the 0x032 mode byte (vendor ccn). `ChassisAuthority::Gatekeeper` reproduces today's deployment: mode 5 plus brake = false every loop. |
| Input | `/motion_target/target_speed_chassis` → target (vx, vy, wz), unclamped (ccn@0x128484) | `set_twist` |
| Acceleration limits | (2.5, 1.0, 2.5) at start; `/chassis_acc_limit` caps them at (2.5, 1, 1) (ccn@0x128720). The gatekeeper sends (2.5, 1, 1). | Same; `Gatekeeper` applies (2.5, 1, 1) |
| Ramp | For each axis, if a·0.02 < \|target − prev\|: `calculateNextVelocity` ccn@0x129f60 (jerk = 10a / 20a / 10a, output clamped to ±1.5, wz included); otherwise the target is taken directly | `next_velocity`, a port from the disassembly with the same `mul_add`s |
| IK | `vector_control` ccn@0x12a340: clamp vx and vy to ±1.5 and wz to ±3 by scaling. Wheel vectors fl (vx−wb, vy+wa), fr (vx+wb, vy+wa), r (vx, vy−wr) with a, b, r = 0.16867, 0.28, 0.32733. `atan2`, then `map_angle` ccn@0x12a184 against the last command (angle) and against the steer feedback (speed sign). The robot's launch uses `chassis_type: W1`, the flip-toward-actual branch. | `SwerveIk::solve`, with W1 by default (`VENDOR_MAP_ANGLE`) and W0 available |
| Misalignment | Any wheel more than π/8 from its commanded angle clamps the ramp state to ±0.07 m/s (±0.3 when \|target wz\| > 0.1). This cycle's speeds are not affected. | Same |
| Brake | p_des = (0.75, −0.75, 0) | `set_brake` |
| Low-pass | v = 0.1·IK + 0.9·previous output, per wheel | Same |
| Steer rates | v_des[3..5] are always 0 | Same |
| Timeout | A counter is reset by each twist (in software mode), each brake message, and each RC mode-2 message. After 10 loops (200 ms): speeds 0, p_des = steer feedback. Filter and ramp state are kept. With the gatekeeper's 50 Hz brake messages this never fires, so the last twist is held until a new one replaces it. | `RemoteMode`: the vendor timeout. `Gatekeeper`: no timeout, as deployed. |
| Gate | HDAS@0x154ec0: p_des must be 3 long and v_des 6. No range checks. | By type |
| Encode | `chassis_encode_impl` lpm@0x12bec0: ×1000; speed ±2 → 0x07D0/0xF830, angle ±30 → 0x7530/0x8AD0, rate ±5 → 0x1388/0xEC78 | Port |
| Rate | ccn's 20 ms loop (immediate ccn@0x126944), with HDAS sending one frame per message | `tick()` every 20 ms from the caller's clock |
| E-stop | None in ccn or HDAS | None (exposed only) |
| Startup | Nothing extra: no auto-calibration, no wait for feedback | Same; `encode_chassis_auto_calibration` exists but is never called |
| FK | ccn@0x128d30: wheel-vector mean for vx and vy; wz from tangent projections, with every wheel divided by r (biased: wz reads 0.09 % low on a pure spin, and spinning leaks (2a − r)/3·wz into vy) | `swerve::forward_kinematics` is the port. Odometry defaults to a rigid least-squares FK. |
| Odometry | None anywhere in the vendor stack | `WheelOdometry`: exact arc integration, stepped by 0x030's own timestamps |

## Grippers (0x051 / 0x061)

| Step | Vendor (rgc + lmm `Controller_Gripper`) | Ours (`GripperCommander`, `GripperController`) |
|---|---|---|
| Input | JointState `position[0]` = opening 0..100, clamped (rgc@0x15eb80). A new target more than 1 above the measured position drops FORCE back to POSITION. | `set(opening)` |
| Startup | performInitialization (rgc main): waits up to 5 s for feedback ("[Init] Waiting for gripper feedback..."), then for 1.0 s calls updateForceBaseline alone on each side's offset effort every 5 ms (rgc@0x15a838, calls at rgc@0x15abf8 / 0x15ac34; no detection, no control). Then sweeps 50 → 0 → 50 → 100 %, each held 2 s, as buildDirectTarget (rgc@0x15e6e0: position gains, zero torque) handed straight to moveJOTG (lambda rgc@0x15e794); no control pass, so no grasp detection during the sweep. Then forces the target to 100. Commands that arrive meanwhile are overwritten. | `GripperCommander::new(.., run_startup)`: `GripperStartup::Gathering` (one sample per 5 ms boundary of the 2 ms clock: 200 samples, nothing sent), then `Sweeping` (`direct_target` + OTG step, no `control`), then control. `run_startup = false` (`Joining`) is for joining a vendor that already ran: the first feedback settles the baseline to its effort, as a long-running rgc's has converged (0.001 blend per 2 ms pass). |
| Mapping | `applyGripperMapping` rgc@0x15de80: LUT inverse of the linkage (rgc@0x189a20 / 0x189d70), then p = −(θ − 0.1089) clamped to [−2.8, 0] | `gripper_geometry::InverseLut`: same samples, filter and interpolation |
| Modes | `updateGripperControl` rgc@0x15df20. POSITION: kp 20, kd 10, t 0. HYBRID: kp 10, kd 5. FORCE: p = map(measured position), kp = kd = 0, t = min(2.0, 4.0). Grasp is detected (rgc@0x15d2e0) when \|effort − baseline\| > max(adaptive, 0.1)·1.0, \|vel\| < 1, the jaw is closing, and position > 5. Effort offsets are +0.1 left, +0.5 right. | Port |
| OTG | Reflexxes Type II, vmax 30, amax 60, 2 ms cycle. Steps from its own state (feedback is dropped after `StartOTG`) and publishes every pass, about 415 Hz (a 2 ms sleep on top of the work). | `TrapezoidOtg`: time-optimal trapezoid with the same limits. `tick()` every 2 ms does control and one OTG step. |
| Gates | HDAS@0x1520d4/0x1537d0: one element each, the arm bounds, drop on violation | `gate::check_gripper` (ROS input only; the commander's values are always in range) |
| Encode | `encode_Gripper` lpm@0x11d220 | Port |
| Timeout / e-stop | None; the last target is re-sent forever | Same |

## Joint tracker walkthrough (jt, `r1_pro_jointTracker_demo_node`)

Launched by `start_mobiman_r1pro_joint_pid.sh` (`ros2 launch mobiman r1_pro_jointTrackerdemo_launch.py`, fast_mode false, torso_key empty, `joint_controller_pid_kp_kd.toml`, `R1_PRO/urdf/r1_pro.urdf`, `r1_pro_task_{left,right,torso}.info`). Not stripped (DWARF), so every name below is the vendor's.

| Step | Vendor | Ours (`joint_tracker`) |
|---|---|---|
| Objects | main jt@0x178fb0 builds two `Controller_A2` (7-DOF arms, ctor jt@0x2228a0) and one `Controller` (4-DOF torso, ctor jt@0x236010); `setRatio(1.0)`, `enable_torso()` on both arms | `ArmTracker` ×2, `TorsoTracker`; `VENDOR_RATIO` = 1 |
| Feedback | `enable()` jt@0x220b74 blocks until `/hdas/feedback_arm_*` (or `_torso`) arrives; the callback jt@0x21e140 copies the position into `last_pos`. `StartOTG()` jt@0x21fe70 then drops the subscription: from here on the planner integrates its own output | `on_arm_feedback` / `on_torso_feedback` seed the planner once and are ignored afterwards |
| Arm target | `lefttargetPositionCallback` jt@0x17e1c0 / right jt@0x17e984: clears `waiting_for_first_command`, copies position[0..6] (no size check), clamps with `minmax` to the per-joint limits (jt@0x248520..578; right j2 jt@0x248580), then `j7_soft_constraints` jt@0x17d5b0: with j6 ≤ 0, j7 is clamped to 0.8 × quintic polynomials of j6 (jt@0x2485b0 upper, jt@0x2485e0 lower; mirrored for the left arm). A 7-long velocity sets vmax = min(\|v\| / ratio, [3, 3, 3, 5, 5, 5, 5]) (jt@0x248658) and amax = jerk = 1.5 vmax; otherwise the toml's `R1_Pro_A2_max_vel` 5 / `_acc` 20 / `_jerk` 100. `effort` is never read. dimos sends velocity = `tracking_speed` 0.5, so its moves run at 0.5 rad/s and 0.75 rad/s² | `ArmTracker::on_target`: same clamp, j7 rule and limits. Refuses non-7 positions, non-finite values and zero speeds |
| Target loop | lambda thread jt@0x186ba0, every 5 ms (0x4c4b40 ns jt@0x187d2c, resyncs when > 10 ms late): for each arm past its first target `moveJOTG(target)` (re-arms the planner), then `set_torso_state(torso.last_pos)` on both arms, publishes `/joint_states` | `apply_target(torso.position())` from the module's 5 ms `arm_targets` loop |
| Arm planner | `Controller_A2::reflexxesThreadFunc` jt@0x2241f0: 4 ms absolute-deadline loop, `RMLPosition` (cycle 0.004 jt@0x249a28; phase synchronization if possible; target velocity 0; current velocity/acceleration fed back from its own output), runs while `!target_reached`, sets it on `ResultValue == 1` (final state) | `tick()` from the 4 ms `arms` loop: `otg::SyncOtg` step, `target_reached` on arrival |
| Arm command | p_des = (f32) planner position; v_des = 0.4 for every joint (0x3ecccccd); kp = `R1_Pro_A2_*_kp` 2, kd = `_kd` 25; mode 0. t_ff = `getTorque_each_joint_with_gravity` ocs2pi@0x1ab4a0 = `pinocchio::computeGeneralizedGravity` on q = [torso last_pos (4), planner position (7)], elements 4..10 × mass_coef 1 (`loadCompensatedInfo` jt@0x21fb40). The model: ocs2 `MobileManipulatorInterface` on `r1_pro.urdf`, root base_link, the task file's `removeJoints` locking wheels, steering, the other arm and all fingers at 0. No \|τ\| ≤ 100 cut while torso_enabled; `set_target_ee_wrench` is stored but never used | `GravityModel` on the same URDF with the same joints movable (torso 1-4, that arm 1-7), all others frozen at 0; the same fields. Live check (robot echo 2026-11-25): t_ff to 1e-8 on both arms, the whole 0x050 hold frame byte-identical |
| Torso target | `torsotargetPositionCallback` jt@0x17f150: 4 positions required, no clamp; 4 velocities set vmax = min(\|v\|, 1.5) (jt@0x248690), amax = 1.5 vmax; copies the target unless an RC home is active; one `moveJOTG` per message | `TorsoTracker::on_target` |
| Torso planner | `Controller::reflexxesThreadFunc` jt@0x2339a0, 5 ms (cycle 0.005 jt@0x24a8f0), starting limits vel 1, acc 1.5 (jt@0x1486e8 / 0x148690). Joint 3 clamped to ±2.95 (jt@0x24a8d0; a second clamp writes index 4, out of bounds). With torso_key ≠ "galaxea_Torso_202309" (empty on this robot): KDL FK base_link → torso_link4, `checkBoundingBox` jt@0x22fab0 x ∈ [−0.1, 0.25], z ∈ [0.551, 1.149], pitch ±1.05, yaw ±1.76; outside → back to last_pos, velocity 0, command frozen. p_des = last sent; v_des [0.6, 1.0, 0.6, 1.0]; kp [140, 200, 120, 20], kd [10, 50, 5, 1] hardcoded; t_ff = torso gravity, 0 above 100; mode = checksum. Silent once reached | `TorsoTracker::tick`: same clamp, box (KDL `GetRPY` convention) and freeze; the 0x041 frame carries only p and v. Today's torso pose is inside the box (`tests/gravity_vendor.rs`) |
| RC | `controllerCallback` jt@0x17da30 on HDAS's `/controller`: mode 0x11 / 0x10 sends both arms to [0, 0, 0, −1.5, 0, 0, 0] and the torso to [0, 0, 0, 0] / [1.74, −2.64, −0.9, 0] and ignores dimos torso targets meanwhile | Not ported |
| Timeouts | None; no feedback watchdog (feedback is not read after startup), no e-stop handling; arms re-assert the last target forever | Same |

## Fan level (0x023) and other CCU frames

| Frame | Vendor | Ours |
|---|---|---|
| 0x023 | HDAS `fan_control_thread` (lpm `_M_run` @0x124ff0): every 0.5 s, `[max(0, (int)(thermal_zone0 m°C / 10000) − 2)]` through the CCU control queue (100 µs drain). The robot read 47.9 °C, giving `02`, which matches the capture. | `FanCommander::tick(millicelsius)`. The caller reads `FAN_THERMAL_ZONE` every 0.5 s. |
| LED 0x022 | HDAS@0x151784 clamps mode to ≤ 1, then sends `[r, g, b, mode]` | `commander::led` |
| Function frames 0x043 / 0x053 / 0x063 | `/HDAS/function_frame_*` services, command 1..10 only; never sent at startup | `commander::function_frame` |
| Auto-calibration 0x034 | Service only; always `[0x20]` | `encode_chassis_auto_calibration` |

## Byte-exact evidence (`tests/transmit.rs`, capture `can0_20s_20261125T203928Z.log`)

| ID | Inputs rebuilt from | Result |
|---|---|---|
| 0x031 | First principles: parked, zero twist, gatekeeper authority; the whole `ChassisCommander` loop over the capture's 0x030 feedback | 993/993 identical (all zero) |
| 0x051 | First principles: opening 0, the vendor gains, the whole `GripperCommander` over the captured 0x058 | 8583/8583 identical |
| 0x061 | First principles: opening 100, joining a running rgc (baseline settled), the whole `GripperCommander` over the captured 0x068 | Identical from the moment the OTG reaches the open angle (0.43 s) to the end; POSITION gains throughout |
| 0x050 / 0x060 | Mid-bin inputs rebuilt from the captured bytes (WIM RT was the sender in that capture) | Byte-exact re-encode (kp 2, kd 25, v 0.4 confirmed) |
| 0x050 hold | jt's live MotorControl (`ros2 topic echo /motion_control/control_arm_left`) and the torso feedback; `ArmTracker` holding the same target with gravity from the vendor URDF (`tests/gravity_vendor.rs`, needs `R1_VENDOR_URDF`) | Identical bytes; t_ff within 1e-8 on both arms |
| 0x041 | Mid-bin inputs rebuilt from the captured bytes (WIM RT sender) | Byte-exact re-encode |
| 0x023 | The robot's thermal reading, 47.9 °C | Identical |
| 0x022 / 0x043 / 0x053 / 0x063 / 0x034 (1 B) | Not in the capture | Round-trip through the decoders plus clamp cases only |

## Deliberate differences

| Difference | Why |
|---|---|
| jt's Reflexxes Type II is replaced by `otg::SyncOtg`: per joint "reach a cruise speed at full acceleration, cruise, brake", the slowest joint's minimum time shared by all, re-planned every cycle from its own state | Same limits, endpoints, arrival time and holding bytes (the hold frame is byte-identical to jt's, `tests/gravity_vendor.rs`); intermediate samples of a move can differ slightly from RML's phase-synchronized profiles. Porting RML Type II line by line (LGPL, about 5k lines) was not needed for move + hold. |
| Gravity is our own recursive sum, not Pinocchio | `computeGeneralizedGravity` on a fixed-base tree is that sum; it reproduces jt's live t_ff to f32 on both arms |
| A target with the wrong length, a non-finite position, or a zero speed cap is refused | jt reads out of bounds on a short array, keeps zeros as the target on an empty one, and hands Reflexxes invalid limits on a zero speed |
| jt's RC "go home" (`controllerCallback` jt@0x17da30, modes 0x10 / 0x11) is not ported | It moves both arms and the torso to fixed poses on an RC switch; nothing in dimos's path relies on it |
| Gripper OTG is a trapezoid, not Reflexxes Type II (`TrapezoidOtg`) | Same limits and the same resting bytes; the intermediate samples of a move can differ slightly. The vendor's OTG also runs about 17 % slower than its own limits (a 2 ms step every ~2.4 ms); ours steps exactly at the caller's 2 ms tick. |
| No ROS or DDS: no torso checksum on the direct API, and no 1/s throttled log | The checksum only protects HDAS's ROS topic |
| `Bus::send` is behind the off-by-default `transmit` feature; commanders only return frames | Nothing can transmit by accident (tests open no socket) |
| Odometry uses the rigid least-squares FK by default (`OdometryKinematics::Vendor` gives ccn's) | The vendor FK is biased and the vendor never integrates a pose, so there is nothing to match |
| `ChassisAuthority::Gatekeeper` (no timeout) vs `RemoteMode` (the vendor's 200 ms timeout) is a choice the caller must make | Today's deployment is Gatekeeper, which holds the last twist forever. Prefer RemoteMode's timeout, or have dimos keep sending its 0.1 s stop. |
| Write errors are returned, not dropped | Better diagnostics; frame content is unchanged |

## Open unknowns, the ones that block driving first

1. **Gripper grasp state on a cold start.** SOLVED for the capture: `/hdas/feedback_gripper_*` echoed on the robot (2026-11-25) equals our 0x058 / 0x068 decode (right 99.78 mm, velocity −0.012, effort 0.0; left 3.09 mm, effort −0.00333 = raw −2 / 600). The replay mismatch came from our controller starting with a zero baseline while the capture's rgc had been running: the jaw sat at 102 mm, above target + 0.5, so rgc's "closing" test held and only its converged baseline kept detectGrasping off. Still true of the vendor itself: a cold rgc start with a jaw reading above 100.5 mm ends its startup with a baseline of about 14 % of the effort and would switch that side to FORCE (kp 0, t 2.0). Today the right jaw reads 99.78 mm.
2. **Enable sequence.** HDAS sends nothing at startup, and the MCUs accept commands as they arrive. Vendor scripts send function frames 2, 5, 1 (inferred: disable, MIT mode, enable) or 2, 6, 1 (6 = PID?), but the stock session doesn't. Codes 3, 4 and 7–10 are unknown (MCU firmware not disassembled). Whether the ACUs come up enabled after a power cycle without these frames is unproven.
3. **MCU command timeout.** What each board does when 0x031, 0x041, 0x050 or 0x051 stop: hold, brake or disable. There are RECEIVE_TIMEOUT error bits, but they have never been observed. HDAS and the mobiman nodes never stop sending on their own, so the firmware's behaviour is the only safety net.
4. **E-stop release.** No host-side handling exists anywhere. What the bus shows while the e-stop is pressed and released is unrecorded.
5. **Arm kp/kd units.** Raw 12-bit integers. Only replay vendor values (2 / 25).
6. **Gravity feed-forward.** SOLVED: `computeGeneralizedGravity` only (no Coriolis; the velocity argument is unused), Pinocchio's default (0, 0, −9.81) in base_link (chassis tilt ignored), on [torso planner q, arm planner q] with every other joint of `r1_pro.urdf` locked at 0. Galaxea's URDF differs from dimos's `r1pro_2026.urdf` (gripper link 0.246 vs 0.093 kg, torso joint offsets), so ours reads the robot's copy (`gravity_urdf`).
7. **0x032 RC mode values.** Mode 2 is RC drive and 5 is software, per ccn; the physical switch states (SWA–SWD) behind them are firmware-side.
8. **Sender takeover.** HDAS currently sends 0x041 and 0x050 (flag 7). In the capture WIM RT did (flag 5). Two senders on one ID must never run together; the takeover order is not yet written.
