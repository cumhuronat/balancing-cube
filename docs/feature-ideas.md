# Feature & optimization roadmap (firmware-only, no hardware changes)

Status legend: [ ] backlog · [~] in progress · [x] done

## Quick wins

- [~] **1. Auto-trim balance point** — learn the true center-of-mass offset while
  balancing (steady wheel-angle offset ⇒ reference-tilt correction via slow
  integrator, Cubli-paper style). Replaces manual `phi_e` tuning; adapts to
  battery shifts. Persist learned trim in NVS.
- [~] **2. Tap-to-disarm** — double-tap the frame (sharp accel spikes) to
  gracefully spin down and return to ready; re-armable without power cycle.
- [~] **3. Soft spin-down on terminate** — actively brake wheels to rest after a
  safety cut instead of letting them freewheel for ~20 s.
- [ ] **4. Black-box flight recorder** — RAM ring buffer of last ~10 s of state at
  250 Hz, dumped over WiFi after terminate. Turns falls into data.

## Medium

- [ ] **5. Web dashboard + live tuning** — cube serves a page: live attitude,
  wheel speeds, editable gains persisted to NVS, remote kill. No reflash tuning.
- [ ] **6. Auto mode detection** — one firmware for corner AND edge balancing:
  detect the nearest equilibrium manifold at arming, run matching controller
  (edge mode currently requires code edits + reflash).
- [ ] **7. Online gyro-bias tracking** — boot-only calibration drifts with
  temperature on long runs; online estimator fixes marathon sessions.

## Ambitious

- [ ] **8. Stand-up from lying flat** — no mechanical brakes, so the impulsive
  Cubli jump is out; resonant swing-up (rock on a face edge, pump energy each
  cycle, catch the edge balance) looks torque-feasible for face→edge.
  Edge→corner is the boss fight.
- [ ] **9. Choreography mode** — scripted leans/wobbles/spin routines over WiFi
  through the existing reference-quaternion pipeline.

## Rejected / parked

- **Magnetometer (AK09916 inside the ICM20948)** for absolute heading: sits
  centimeters from three motors full of spinning magnets — expect garbage while
  armed. Yaw pinning already covers the need.
- **Higher control rate (500 Hz)**: ESCON current-setpoint path (1 kHz PWM)
  limits the benefit; not worth it.
