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

| Step | Vendor | Ours (`commander::ArmCommander`, `gate::check_arm`, `encode::encode_arm`) |
|---|---|---|
| Input | jt (Reflexxes Type II, `Controller_A2::reflexxesThreadFunc` jt@0x2241f0, 4 ms) publishes p_des = next planner position, v_des = constant 0.4, kp = 2, kd = 25 (`joint_controller_pid_kp_kd.toml`), t_ff = Pinocchio gravity torque | `set_joint_targets(pos, vel, kp, kd, torque)`: the caller supplies all five arrays. **jt is not ported** (see deliberate differences). |
| Gates | HDAS lambda20/21 HDAS@0x155a00/0x1573c0: 7 elements per array, otherwise dropped. Inclusive bounds p ±6.5, v ±40, kp [0, 500], kd [0, 200], t ±50; any violation drops the whole message (log throttled to 1/s). NaN passes (`b.mi`/`b.gt`). `mode` unused. | Same bounds and the same whole-message drop (`Rejected::ExceedsLimit`). NaN passes. Length is enforced by the array type. |
| Bias | When `calib_enable == "enable"`, `p -= bias`. Disabled on this robot. | `joint_bias_rad: Option<_>`, `None` by default |
| Encode | `a2_arm_analysis::arm_encode_impl` lpm@0x11f950, f32 math, truncation, the vendor's literal clamp values (0x752F/0x8AD1, 0x7D0 saturation, 0xFFF) | A line-by-line port. NaN encodes to 0. |
| Rate | One frame per message. No HDAS timer (jt sends 250 Hz). | One frame per call |
| Timeout | None in HDAS or jt: jt holds its last target with gravity torque indefinitely | None: the caller decides |
| E-stop | HDAS keeps forwarding; 0x001 is only published as status (HDAS@0x170180). jt has an RC-mode 0x10/0x11 "go home" path. | Same as HDAS: `Commanders::emergency_stop` is exposed and the frames are unchanged. The jt home path is not ported. |
| Startup | jt publishes nothing for a side until dimos sends that side's first target. Some vendor scripts call function frames 2, 5, 1 (`set_arm_mode_mit.sh`); the stock R1PROBody session does not. | Nothing is sent until the first `set_joint_targets` |

## Torso (0x041)

| Step | Vendor | Ours (`TorsoCommander::set`, `encode_torso`) |
|---|---|---|
| Input | jt torso loop (`Controller::reflexxesThreadFunc` jt@0x2339a0, 5 ms): p from Reflexxes, v_des overwritten to [0.6, 1.0, 0.6, 1.0], kp/kd/t_ff filled but **never encoded**; freezes outside a pose box | `set(pos[4], speed_limit[4])` |
| Gates | HDAS@0x155214: 4 elements per array; `mode` must equal an fmix32 checksum of element 0 of each array. No range checks. | The length is the type. The checksum is skipped because it only guards HDAS's ROS topic; it is ported as `gate::torso_checksum` / `check_torso` for anything taking ROS input. |
| Encode | `torso_analysis::encode_torso_impl` lpm@0x144ff0: p ×10000 (±3.14 → 0x7AA8/0x8558), v ×3000 (±10 → 0x7530/0x8AD0) | Port |
| Rate / timeout / e-stop / startup | Event-driven; no timeout; e-stop not handled; nothing at startup | Same |

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
| Startup | Waits up to 5 s for feedback, then gathers a 1 s force baseline. Sweeps 50 → 0 → 50 → 100 %, holding each 2 s (rgc@0x15e794), then forces the target to 100. Commands that arrive meanwhile are overwritten. | `GripperCommander::new(.., run_startup_sweep)`. Sends nothing until feedback arrives, then sweeps in 1000 2-ms ticks per step. The 1 s baseline gather is folded into the normal per-tick baseline update. |
| Mapping | `applyGripperMapping` rgc@0x15de80: LUT inverse of the linkage (rgc@0x189a20 / 0x189d70), then p = −(θ − 0.1089) clamped to [−2.8, 0] | `gripper_geometry::InverseLut`: same samples, filter and interpolation |
| Modes | `updateGripperControl` rgc@0x15df20. POSITION: kp 20, kd 10, t 0. HYBRID: kp 10, kd 5. FORCE: p = map(measured position), kp = kd = 0, t = min(2.0, 4.0). Grasp is detected (rgc@0x15d2e0) when \|effort − baseline\| > max(adaptive, 0.1)·1.0, \|vel\| < 1, the jaw is closing, and position > 5. Effort offsets are +0.1 left, +0.5 right. | Port |
| OTG | Reflexxes Type II, vmax 30, amax 60, 2 ms cycle. Steps from its own state (feedback is dropped after `StartOTG`) and publishes every pass, about 415 Hz (a 2 ms sleep on top of the work). | `TrapezoidOtg`: time-optimal trapezoid with the same limits. `tick()` every 2 ms does control and one OTG step. |
| Gates | HDAS@0x1520d4/0x1537d0: one element each, the arm bounds, drop on violation | `gate::check_gripper` (ROS input only; the commander's values are always in range) |
| Encode | `encode_Gripper` lpm@0x11d220 | Port |
| Timeout / e-stop | None; the last target is re-sent forever | Same |

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
| 0x061 | Opening 100: the settled OTG bytes match (`gripper_controller` unit test). In the full replay, the captured right feedback trips our grasp detector (see unknown 1). | Steady-state exact; replay mismatch is an open item |
| 0x050 / 0x060 | Mid-bin inputs rebuilt from the captured bytes. The sender in the capture was WIM RT, not jt/HDAS, so the true inputs are unknown. | Byte-exact re-encode (kp 2, kd 25, v 0.4 confirmed) |
| 0x041 | Mid-bin inputs rebuilt from the captured bytes (WIM RT sender) | Byte-exact re-encode |
| 0x023 | The robot's thermal reading, 47.9 °C | Identical |
| 0x022 / 0x043 / 0x053 / 0x063 / 0x034 (1 B) | Not in the capture | Round-trip through the decoders plus clamp cases only |

## Deliberate differences

| Difference | Why |
|---|---|
| jt (Reflexxes arm/torso planner and Pinocchio gravity torque) is not ported; the caller gives p, v, kp, kd, t directly | dimos already runs its own trajectory and control; a Rust Reflexxes plus gravity model is a separate project. **Until a caller supplies gravity t_ff, holding with kp = 2 will sag.** |
| Gripper OTG is a trapezoid, not Reflexxes Type II | Same limits and the same resting bytes; the intermediate samples of a move can differ slightly. The vendor's OTG also runs about 17 % slower than its own limits (a 2 ms step every ~2.4 ms); ours steps exactly at the caller's 2 ms tick. |
| No ROS or DDS: no torso checksum on the direct API, and no 1/s throttled log | The checksum only protects HDAS's ROS topic |
| `Bus::send` is behind the off-by-default `transmit` feature; commanders only return frames | Nothing can transmit by accident (tests open no socket) |
| Odometry uses the rigid least-squares FK by default (`OdometryKinematics::Vendor` gives ccn's) | The vendor FK is biased and the vendor never integrates a pose, so there is nothing to match |
| `ChassisAuthority::Gatekeeper` (no timeout) vs `RemoteMode` (the vendor's 200 ms timeout) is a choice the caller must make | Today's deployment is Gatekeeper, which holds the last twist forever. Prefer RemoteMode's timeout, or have dimos keep sending its 0.1 s stop. |
| Write errors are returned, not dropped | Better diagnostics; frame content is unchanged |

## Open unknowns, the ones that block driving first

1. **Right-gripper grasp state.** Replaying the captured right feedback (102 mm open, torque 0.013 + 0.5 offset) through our port of `detectGrasping` switches to FORCE mode (kp 0, t 2.0). The capture shows POSITION gains, so rgc must see a different position or effort on `/hdas/feedback_gripper_right` than our 0x068 decode gives (HDAS's gripper publish lambda was not traced; no gripper topic in the mcap). Check: `ros2 topic echo /hdas/feedback_gripper_right` once, read-only. Until then the right gripper may relax into FORCE when held open past 100.
2. **Enable sequence.** HDAS sends nothing at startup, and the MCUs accept commands as they arrive. Vendor scripts send function frames 2, 5, 1 (inferred: disable, MIT mode, enable) or 2, 6, 1 (6 = PID?), but the stock session doesn't. Codes 3, 4 and 7–10 are unknown (MCU firmware not disassembled). Whether the ACUs come up enabled after a power cycle without these frames is unproven.
3. **MCU command timeout.** What each board does when 0x031, 0x041, 0x050 or 0x051 stop: hold, brake or disable. There are RECEIVE_TIMEOUT error bits, but they have never been observed. HDAS and the mobiman nodes never stop sending on their own, so the firmware's behaviour is the only safety net.
4. **E-stop release.** No host-side handling exists anywhere. What the bus shows while the e-stop is pressed and released is unrecorded.
5. **Arm kp/kd units.** Raw 12-bit integers. Only replay vendor values (2 / 25).
6. **Gravity feed-forward.** jt's t_ff comes from Pinocchio on `r1_pro.urdf` with the torso joints prepended; whether it includes Coriolis terms wasn't traced. Without it the arms sag at kp = 2.
7. **0x032 RC mode values.** Mode 2 is RC drive and 5 is software, per ccn; the physical switch states (SWA–SWD) behind them are firmware-side.
8. **Sender takeover.** HDAS currently sends 0x041 and 0x050 (flag 7). In the capture WIM RT did (flag 5). Two senders on one ID must never run together; the takeover order is not yet written.
