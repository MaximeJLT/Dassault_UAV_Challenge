# UAV Autonomy — A QuadPlane Search-and-Loiter Stack

**ISAE-ENSMA**
Author: Maxime Jolliot

---

## Context

This codebase is the autonomous perception and navigation stack developed during the **2025–2026 Dassault UAV Challenge**, in which our ISAE-ENSMA team competed against seven other engineering schools. Unlike most competing teams, who used off-the-shelf UAV platforms and focused their effort on software, our team chose to design and build the airframe entirely in-house — mechanical structure, electronics, and system integration — over the course of nine months.

This decision shaped the scope of what made it onto the aircraft this year. The team prioritized producing a robust, fully self-built flying platform that we understand end-to-end, and the autonomy stack documented here was developed and validated in parallel in simulation, ready to be integrated on the platform for the next campaign. The team was awarded the **Jury's Favorite Prize** and finished in the top three of eight teams.

---

## Problem Statement

We were asked to build a fully autonomous mission for a fixed-wing/VTOL hybrid UAV (QuadPlane) operating in a constrained domain (≈ 100 × 50 m): the aircraft must take off vertically, switch to forward flight to search for a small ground target with a camera, **detect the target via a neural network**, return to hover above it, hold position for a fixed duration, then return to home and land — all without human intervention beyond the launch command.

On paper the problem looks like a string of well-understood building blocks: ArduPilot for the autopilot, MAVLink for the bus, YOLO for the detection, some trigonometry for the pixel-to-GPS conversion. In practice almost every one of those blocks revealed a subtle assumption that broke when applied to our hardware and our flight domain. This README documents the architecture **as it actually exists today** and the reasoning that led there. The intent is not to describe a clean design produced in one shot, but the design that survived several iterations of contact with the real system.

---

## Integration Status

The stack is **fully validated in ArduPilot SITL simulation**, including the perception thread, the mission FSM, the pixel-to-GPS conversion, the mission upload protocol, and all five state transitions. The complete autonomous mission runs end-to-end in simulation without intervention.

The stack was **not flown on the physical aircraft during the 2025–2026 campaign**, because the team's effort was committed to designing and manufacturing the airframe from scratch. The code is structured so that the transition from simulation to hardware requires only the call to `connect_udp()` to be replaced with `connect_serial()` — no logic changes are needed. The next-campaign integration plan is detailed at the end of this document.

---

## Mission Pipeline (current)

1. The Python script on the ground station opens a **MAVLink connection over a SiK telemetry radio** (UDP for SITL simulation).
2. Pre-flight check on the airspeed sensor (pitot).
3. An AUTO mission is uploaded containing: a `NAV_VTOL_TAKEOFF` to ~30 m followed by the hypodrome search pattern.
4. The aircraft arms, takes off vertically, transitions to fixed-wing, and starts flying the search loop.
5. A **YOLOv8 + ByteTrack** thread on the ground station processes the live video feed coming from the onboard VTX, publishing detections into a shared variable.
6. When a detection appears, the FSM converts the pixel coordinates into world GPS coordinates, commands a fixed-wing → VTOL transition, and uploads a new micro-mission `[NAV_WAYPOINT target + LOITER_UNLIM]`.
7. The aircraft navigates to and loiters over the target GPS in AUTO for the configured hold duration.
8. The FSM switches to **QRTL** for the final autonomous VTOL return and landing.

The aircraft remains in AUTO for the entire mission except for the final QRTL transition — this is a deliberate constraint discussed below.

---

## Repository Structure

```
uav-autonomy/
├── ground_station
│   ├── controller_fsm.py      # Main entry — mission FSM
│   ├── arm_pipeline.py        # Takeoff, arming, FW↔VTOL transitions, failsafes
│   ├── connection.py          # MAVLink connection (serial/UDP), pitot check
│   ├── goto.py                # Target navigation via LOITER_UNLIM micro-missions
│   ├── mission_upload.py      # Robust mission upload (handles MAVProxy echoes)
│   ├── read_gps.py            # GLOBAL_POSITION_INT helpers
│   └── hypodrome.waypoints    # Search pattern (QGC WPL format)
│
├── Perception (ml/)
│   ├── NN.py                  # YOLOv8 + ByteTrack detection thread
│   └── conversion.py          # Pixel (u,v) → GPS (lat, lon)
│
├── Utilities
│   └── run_sitl.sh            # Launch ArduPlane SITL
│
└── Legacy / unused today
    ├── gimbal.py, gimbal_test.py     # See "Pivot 2" below
    ├── yaw.py, velocity.py           # Reserved for future moving-target work
```

---

## Architectural Evolution

The current code is the result of five design pivots, each driven by a concrete problem encountered during integration or SITL testing.

### Pivot 1 — Stay in AUTO for the entire mission

**Initial hypothesis.** Use the mode that fits each phase: `AUTO` for the search loop, `GUIDED` to redirect toward the detected target, `QLOITER` to hover above it, then `QRTL` to return home. ArduPilot supports all of these and the documentation suggests they compose cleanly.

**What broke.** Every mode switch is a one-way handshake with non-trivial preconditions. `GUIDED` requires a continuous stream of `SET_POSITION_TARGET_*` messages, which interacts poorly with our radio link's intermittent latency. `QLOITER` requires explicit throttle hold via RC overrides — and the moment we stopped sending overrides, the autopilot considered itself uncommanded and engaged `FS_GCS_ENABL`, which on a QuadPlane defaults to RTL. We were essentially fighting the autopilot's failsafes by hand.

**Pivot.** Instead of switching modes, we now **upload micro-missions** at runtime. To navigate to the detected target, the FSM uploads `[NAV_WAYPOINT target + NAV_LOITER_UNLIM target]` and the aircraft already in AUTO simply consumes the new mission. The `LOITER_UNLIM` terminator is the key insight: it pins the aircraft in AUTO indefinitely rather than letting the mission complete (which would trigger an automatic RTL). The result is that we never need a `GUIDED` or `QLOITER` mode change mid-flight — we only leave AUTO once, to engage `QRTL` at the very end. This eliminated an entire class of failsafe interactions and made the FSM dramatically simpler.

### Pivot 2 — Remove the gimbal

**Initial hypothesis.** A 2-axis gimbal would let us decouple camera pointing from aircraft attitude, allowing a wider effective search swath while flying the hypodrome.

**What broke.** The gimbal added three new failure modes: (1) mechanical reliability of an additional moving subsystem; (2) MAVLink complexity — `GIMBAL_DEVICE_ATTITUDE_STATUS` returns a quaternion that must be converted to Euler angles, and we hit several gimbal-lock edge cases during testing; (3) the pixel-to-GPS conversion became a four-frame transformation (image → camera → gimbal → UAV → world), with unit mismatches between MAVLink fields delivered in radians and our internal calculations in degrees. We caught one bug where 30° of gimbal pitch was being added to radians of UAV attitude — the error pattern only became visible after several flights.

**Pivot.** Given a domain of only 100 × 50 m and a flight altitude of 30 m, a **fixed nadir-pointing camera** sees a ground footprint of approximately 30 × 30 m per frame at FOV 90° × 60°. By flying the hypodrome at low airspeed (10 m/s), the search pattern covers the full domain in a single pass without needing the camera to swivel. Removing the gimbal eliminates an entire reference frame from the conversion chain and removes the quaternion decoding entirely. The `gimbal.py` and `gimbal_test.py` files are retained in the repository for documentation but are no longer imported by the active code path.

### Pivot 3 — Remove the downward LiDAR

**Initial hypothesis.** A downward-facing LiDAR (`DISTANCE_SENSOR` messages) would give us the oblique distance from the aircraft to the ground target, which we needed because the gimballed camera was looking at an angle, not straight down.

**What broke.** Once we removed the gimbal (Pivot 2), the LiDAR became redundant. With a nadir-pointing camera, the geometry collapses into a right triangle whose vertical leg is simply the **AGL altitude** already published by ArduPilot in `GLOBAL_POSITION_INT.relative_alt`. The horizontal offset to a pixel detected at off-axis angle θ is `altitude × tan(θ)`. No additional sensor is needed.

**Pivot.** `conversion.py` now reads only `GLOBAL_POSITION_INT` (for altitude and current GPS) and `ATTITUDE` (for UAV yaw, used to rotate the body-frame offset into world North/East). The conversion is exactly:

```
offset_right_drone   = altitude × tan(angle_horizontal_camera)
offset_forward_drone = altitude × tan(angle_vertical_camera)
[ΔN, ΔE]             = rotation_matrix(uav_yaw) × [forward, right]
target_lat, target_lon = drone_position + [ΔN, ΔE] / earth_curvature_factors
```

The full transformation is now ~15 lines instead of ~60, and depends on two sensors that ArduPilot would refuse to fly without anyway (GPS and IMU). The LiDAR is removed from the airframe entirely.

### Pivot 4 — No braking anticipation

**Initial hypothesis.** A QuadPlane in fixed-wing mode flying at 18 m/s takes 100–150 m to brake to a vertical hover after a `MAV_CMD_DO_VTOL_TRANSITION`. To avoid overshooting the target, we initially anticipated the transition: when the aircraft was within 200 m of the detected GPS coordinate, we triggered the transition early.

**What broke.** Our actual flight domain is **100 × 50 m**. A 200 m anticipation buffer is geometrically impossible — by the time we are 200 m from the target, we are already well outside the domain.

**Pivot.** Two changes in tandem: (1) we **reduce cruise airspeed from 14 m/s to 10 m/s**, which roughly halves the braking distance; (2) we **trigger the FW→VTOL transition immediately upon detection** and rely on the post-transition VTOL navigation to bring the aircraft back to the exact GPS point. Because we already upload a `NAV_WAYPOINT` to the target after the transition, ArduPlane will return to it even if the aircraft overshoots during the FW→VTOL deceleration. We trade a few seconds of "fly past then come back" for a much shorter total mission duration and a design that actually fits the flight domain. The `ANTICIPATE_TRANSITION` state is retained in the FSM enum but is no longer entered.

### Pivot 5 — Dedicated radio per process

**Initial hypothesis.** Both Mission Planner (for human monitoring) and our Python script (for control) can share a single telemetry radio — they speak the same MAVLink protocol.

**What broke.** They cannot. MAVLink over a serial bus is half-duplex and stateful: the mission upload protocol uses a request/response handshake where the autopilot asks for waypoint N and the GCS replies with waypoint N. When two GCS instances are connected simultaneously, the autopilot's requests are answered by whichever GCS responds first, causing `INVALID_SEQUENCE` and `OPERATION_CANCELLED` errors mid-upload. We documented this thoroughly in `mission_upload.py`, which implements retry logic with up to 5 attempts and explicit detection of concurrent-session error codes — but the right fix is to eliminate the conflict at the physical layer.

**Pivot.** Two independent SiK radios on the ground station, paired to the same airborne module's network. Python uses one (`COM5`), Mission Planner uses the other (`COM6`). Each has its own serial port and its own MAVLink session. The robust upload logic in `mission_upload.py` is retained as a safety net but should rarely trigger in normal operation.

---

## Perception Layer

Detection runs on the ground station, not onboard. The video stream from the aircraft's VTX is digitized via an HDMI capture card (~50 ms latency) and consumed by a YOLOv8m model running in a background thread (`ml/NN.py`). ByteTrack provides frame-to-frame identity assignment so the FSM can lock onto a specific track ID even if multiple candidate objects enter the frame.

The detection thread writes to a module-level variable:

```python
latest_detection = (x_norm, y_norm, w_norm, h_norm, track_id)
```

The FSM reads this asynchronously. There is **no message queue, no callback, no synchronization primitive** — just shared memory and the Python GIL. This is intentional: it keeps the perception layer entirely decoupled from the flight-control layer. The neural network can be swapped (custom-trained model, different architecture, multi-class detector) without touching a single line of the FSM. Conversely, the FSM logic can be debugged in SITL without running YOLO at all (the `test_perception.py` script even provides a `MockMaster` for testing the conversion math without an autopilot).

For the current campaign we use a **pre-trained COCO model** filtered to a target class (`sports ball`, class 32, or similar). Training a custom model on aerial imagery requires an annotated dataset we have not yet collected — this is planned for next year. The architecture explicitly anticipates this: the only contract between perception and control is the `(x, y, w, h, id)` tuple.

---

## Flight Control Layer

### State machine

The mission is driven by a Finite State Machine in `controller_fsm.py`:

| State                   | Purpose                                                         |
|-------------------------|-----------------------------------------------------------------|
| `SEARCH_FW`             | Aircraft in AUTO on hypodrome, polling `latest_detection`       |
| `TRACK_DETECTED`        | Convert pixel → GPS, log the target                             |
| `TRANSITION_TO_VTOL`    | Send `MAV_CMD_DO_VTOL_TRANSITION`                               |
| `VTOL_HOLD_OVER_TARGET` | Upload `[NAV_WAYPOINT + LOITER_UNLIM]`, navigate, hold N seconds|
| `RETURN_HOME`           | Switch to `QRTL`, wait for landing and disarm                   |
| `FAILSAFE`              | Emergency fallback → `QRTL`                                     |

A separate daemon thread implements a software **kill switch**: pressing `K + Enter` in the terminal sends the magic ArduPilot disarm code `21196` over MAVLink, forcing an immediate disarm in flight. This is a last-resort backup to the hardware RC failsafe, not a substitute for it.

### Mission upload robustness

`mission_upload.py` implements the `MISSION_ITEM_INT` upload protocol with several layers of defense:

- It retries the upload up to 5 times.
- It deduplicates retransmitted `MISSION_REQUEST` messages (ArduPilot re-asks for the same item if it doesn't see our response quickly enough).
- It detects mid-session errors (`INVALID_SEQUENCE`, `OPERATION_CANCELLED`) and restarts cleanly.
- It maintains a GCS heartbeat at 2 Hz during the upload, even during the brief `MISSION_CLEAR_ALL` window, to prevent `FS_GCS_ENABL` from firing.

This file was written iteratively as we encountered each failure mode in SITL with MAVProxy in the loop. Once we adopted Pivot 5 (dedicated radio), most of this defense became theoretical — but it remains valuable in case of bus contention from any other MAVLink source.

### Why we never command the aircraft in real time

A natural design would be a 10 Hz loop sending `SET_POSITION_TARGET_LOCAL_NED` velocity setpoints to track the target. We chose not to, for one reason: **the radio link is the weakest part of the system**. A 433 MHz SiK radio over 100 m of clear LOS has ~10 % packet loss on a bad day and full dropouts of several seconds in noisy environments. A control loop that depends on real-time GCS commands fails the moment the link degrades. By contrast, our upload-and-monitor design degrades gracefully: if the link is lost after the micro-mission is uploaded, ArduPlane continues to execute it autonomously. The aircraft's behaviour is determined by what is on the autopilot at any given moment, not by what the GCS happens to be transmitting.

The `velocity.py` module exists for the future case where we want to track a moving target — it implements `SET_POSITION_TARGET_LOCAL_NED` setpoints — but it is not used by the current FSM, since this campaign's target is stationary.

---

## Simulation

The entire stack runs in **ArduPilot SITL** without any hardware:

```bash
./run_sitl.sh           # launches ArduPlane QuadPlane simulator with MAVProxy
python controller_fsm.py    # runs the FSM
```

SITL exposes two UDP endpoints, mirroring the dual-radio production setup:

- `udp:127.0.0.1:14550` → MAVProxy console + map (read-only monitoring)
- `udp:127.0.0.1:14551` → our Python script (dedicated port, no MAVProxy contention)

For real flight, the call to `connect_udp()` in `controller_fsm.py` is replaced with `connect_serial(port="COM5", baud=57600)`. No other code change is needed to move from simulation to hardware.

---

## What This Architecture Optimizes For

Reading the codebase back in its current form, the recurring theme is a preference for **simplicity at the cost of theoretical optimality**:

- **One mode (AUTO) instead of mode juggling**: simpler to reason about, robust to failsafe interactions.
- **Pre-uploaded missions instead of real-time setpoints**: robust to telemetry link degradation.
- **Fixed camera + altitude instead of gimbal + LiDAR**: fewer sensors, fewer reference frames, fewer unit-conversion bugs.
- **Shared memory between threads instead of message queues**: trivially debuggable, no scheduling concerns.
- **Retry-until-success instead of one-shot protocols**: resilient to the MAVLink protocol's eccentricities.

The trade-offs are real. We accept that the aircraft may overshoot the target during transition, that it cannot track a moving target without code changes, and that we depend on the GPS for ground-relative altitude rather than a more accurate LiDAR. For this campaign, those trade-offs are acceptable; for next year's iteration with a custom-trained NN and a larger flight domain, they should be revisited.

---

## Next Campaign — Integration Plan

The 2026–2027 edition of the Dassault UAV Challenge is the integration target for this stack. With the airframe now built and characterized, the work shifts from designing the platform to flying the autonomy on it. The plan, in order:

1. **Bench integration.** Mount the ground station and SiK radios next to the aircraft on the bench, verify MAVLink connectivity end-to-end on the real autopilot, run the FSM in `AUTO` with the aircraft disarmed, confirm that mission uploads and state transitions behave identically to SITL.
2. **Tethered hover and taxi tests.** Validate the FW↔VTOL transition commands and the QRTL behavior on the real aircraft, with the autonomy stack passive (monitoring only).
3. **First semi-autonomous flight.** Manual takeoff, FSM running in `SEARCH_FW` for the hypodrome, manual landing. Confirms that the perception thread and the GPS conversion produce correct target coordinates in real flight conditions.
4. **First fully autonomous flight.** Full mission end-to-end on a controlled test field, target placed at a known GPS coordinate to validate the conversion against ground truth.
5. **FOV calibration on the production camera.** The conversion math currently uses nominal manufacturer values; a one-time ground-target calibration is expected to significantly tighten the GPS estimate.
6. **Custom-trained detector.** Collect an annotated dataset of the competition target from drone-perspective imagery and fine-tune YOLOv8 on it. The current pre-trained COCO model is a placeholder.

---

## Open Problems / Future Work

Beyond the integration plan above, several directions remain open for the architecture itself:

- **Onboard inference** to remove the video-link latency from the perception loop. Currently every detection costs ~100 ms of capture + ~50 ms of inference + ~50 ms of GPS upload — at 10 m/s that is 2 m of aircraft displacement between "the target was here" and "go here."
- **Moving-target tracking** using `velocity.py` and a Kalman filter on successive detections.
- **Multi-target scenarios** where the FSM must choose between several candidates rather than locking onto the first detection.

---

## Technologies

Python 3 · pymavlink · ArduPilot (ArduPlane QuadPlane) · Ultralytics YOLOv8 · ByteTrack · OpenCV · SITL · SiK telemetry radios

---

## Author

**Maxime Jolliot**
ISAE-ENSMA — Engineering student, Autonomous Systems
