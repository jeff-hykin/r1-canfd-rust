# r1-canfd

The Galaxea R1 Pro CAN FD bus (`can0`, 1 Mbit/s nominal, 5 Mbit/s data) in Rust, receive only: every frame the
CCU, VCU (chassis), TCU (torso) and both ACUs (arms) broadcast, decoded the way the vendor driver HDAS
(atc_system V2.3.0 hotfix_2) decodes it, reverse-engineered from its binaries.

- Bus (Linux): a raw SocketCAN socket like HDAS's own (`CAN_RAW`, `CAN_RAW_FD_FRAMES` on, no filter), each frame
  stamped with the kernel's software receive time (`SO_TIMESTAMPING`). `Bus` is an iterator of `Frame`s. It does
  not set up the link; `ip link` stays in the boot script.
- `candump -L` logs, so a recording runs through the same decoders.
- `decode(&Frame) -> Decoded`: chassis and torso IMUs, BMS, wheel/steer feedback, RC remote, torso and arm joints,
  grippers (with the vendor's linkage to an opening), wrist F/T, motor temperatures, board and motor versions, error
  words by name, and the e-stop. An ID with no decoder comes back `Unknown { id, data }`, and a known ID that is too
  short comes back `Malformed`. Nothing is dropped.
- Commands other nodes put on the bus (arm, gripper, torso, chassis, LED, function frames) decode as observed
  traffic. This crate has no transmit code at all.
- `Tracker`: per-ID count, rate, gap range, staleness and payload lengths, worked out from frame timestamps only
  (no timers), so a replayed log reports what the live bus would have.

```rust
use r1_canfd::{candump, decode, Decoded, Tracker};

let mut tracker = Tracker::new();
for line in candump::read(std::io::BufReader::new(std::fs::File::open("can0.log")?)) {
    let frame = line?.frame;
    tracker.observe(&frame);
    if let Decoded::ArmFeedback(arm) = decode(&frame) {
        println!("{} {:?} rad", arm.side.name(), arm.position_rad);
    }
}
# Ok::<(), Box<dyn std::error::Error>>(())
```

On the robot, `r1_canfd::Bus::open("can0")?` is the same loop with `for frame in bus`.

`r1_canfd_dump <can0 | capture.log> [--summary] [--summary-every SECONDS] [--json] [--frames N] [--id 0x052,0x020]`
prints decoded frames, or a per-ID summary (once at the end, or every N seconds of bus time), as text or JSON lines.

## Devices

"Verified" means decoded from the vendor code and checked against the 20 s capture. "Partial" means the layout was
checked, but the capture held only zeros or constants, or a unit is inferred. "Code-only" means the ID was not in the
capture. All fields are big-endian.

| ID | Name | Rate | Decoded as | Evidence |
|---|---|---|---|---|
| 0x001 | EMERGENCY_STOP | event | `EmergencyStop`: the frame arriving is the event (HDAS ignores the payload, and there is no release frame) | code-only |
| 0x020 / 0x045 | chassis / torso IMU | 200 / 100 Hz | `Imu`: roll, pitch, yaw in deg, gyro, acceleration in g, a correct quaternion plus HDAS's degrees-as-radians one | verified (\|a\| 0.997-1.004 g; torso pitch matches gravity); gyro deg/s **inferred** |
| 0x021 | BMS | 2 Hz | `Bms`: V, A, raw capacity, `charge_percent(3500)` | verified (54.41 V, 98.86 %); charging state **unproven**, bytes 6-7 raw, sign of current unproven |
| 0x030 | MOTOR_STATUS | 200 Hz | `MotorStatus`: fl/fr/r wheel speed, steer angle and steer rate, VCU ms tick | layout verified; wheel speed in m/s **UNVERIFIED** (robot parked) |
| 0x032 | REMOTE | 200 Hz | `Remote`: raw stick counts and mode | partial (all zero, no RC paired) |
| 0x033 | CHASSIS_ERROR | event | `Errors`: 6 motor words, bit 10 = EMERGENCY_STOP (`emergency_stop()`) | code-only |
| 0x034 | VCU_ERROR / auto-calibration | 1 Hz | `Errors` (24 B) / `ChassisAutoCalibration` (1 B) | partial (all zero) |
| 0x025 / 0x035 / 0x046 | CCU / VCU / TCU version | - / 2 / 1 Hz | `Version`: hw / sw strings, 0xff read as 0 | VCU and TCU verified (sw v1.1.12); CCU code-only |
| 0x040 | TORSO_MOTOR_FB | 200 Hz | `TorsoFeedback`: 4 joints, position, velocity, torque (vendor clamps) | verified (inside URDF limits) |
| 0x044 | TORSO_ERROR | 1 Hz | `Errors`: 4 motor words, bytes 16-19 raw | partial (all zero) |
| 0x052 / 0x062 | LEFT / RIGHT_ARM | 200 Hz | `ArmFeedback`: 7 joints, position, velocity, torque | verified (inside URDF limits; torque matches the mcap effort) |
| 0x054 / 0x064 | ARM_VERSION_AND_ERROR | 1 Hz | `ArmStatus`: ACU version, 8 error words, 7 motor versions | verified versions (ACU v1.1.14; left joint 7 motor reads all-zero); errors all zero |
| 0x055 / 0x065 | ACU_VERSION / hand command | - | `AcuVersion` (8 B) / `HandCommand` (36 B) | code-only |
| 0x056 / 0x066 | LEFT / RIGHT_HAND | - | `Hand`: 6 fingers | code-only (no hand fitted) |
| 0x057 / 0x067 | wrist F/T | 100 Hz | `Wrench`: N, Nm (vendor clamps) | partial (length and rate only, values all zero) |
| 0x058 / 0x068 | GRIPPER_FB | 200 Hz | `GripperFeedback`: motor rad, velocity, torque, opening via the vendor's sine linkage | verified (L 2.7 closed, R 102 open); opening in mm **inferred** |
| 0x05D / 0x06D | MOTOR_TEMP_FB | 1 Hz | `MotorTemperatures`: 7 x MOS and winding degC | verified (25-39 C) |
| 0x031 | chassis command (observed) | 50 Hz | `ChassisCommand` | partial (all zero, parked) |
| 0x041 | torso command (observed) | 250 Hz | `TorsoCommand`: position, speed limit | verified |
| 0x050 / 0x060 | arm command (observed) | 250 Hz | `ArmCommand`: p, v, raw kp/kd, t_ff per joint | verified (byte-exact against the vendor encoder) |
| 0x051 / 0x061 | gripper command (observed) | 415 Hz | `GripperCommand` | verified (constant) |
| 0x022, 0x043 / 0x053 / 0x063 | LED, function frames (observed) | event | `Led`, `FunctionFrame` (codes 1..10, meanings unknown) | code-only |
| 0x023 | ? | 1.9 Hz | `Unknown` (1 byte, always 0x02, sent by a host process) | **unknown** |
| 0x070, 0x510 / 0x520 | R1PROT_JOY, H1 hands | - | `Unknown` | not decoded |

Every scale factor is a `const` that cites the vendor function and address it came from (Ghidra addresses in
`libprotocol_modules.so`). 0x034's motor names are `w1`..`w6`, because `decode_vcu_error` really does index
`torso_name_map`.

## Tested against

A 20 s `candump -L` capture of the robot standing with its arms holding and the base parked
(`tests/fixtures/can0_20s_20261125T203928Z.log`, 70852 frames, 27 IDs). `tests/golden.rs` compares every field
against the validated Python decoder (`r1can.py`, output in `tests/fixtures/golden.json`, regenerate it with
`make_golden.py`): every 25th frame of each ID, and every frame of the low-rate ones. `tests/capture.rs` checks
the physics: 1 g on both IMUs, 54.4 V, every arm and torso joint inside the URDF limits, gripper states,
temperatures, versions, no errors, the per-ID rates, and that the ESI flag marks exactly HDAS's own sends.

Cross-compiles for the robot with `cargo zigbuild --release --target aarch64-unknown-linux-gnu.2.31`.

License: Apache-2.0
