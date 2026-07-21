#ifndef parameters_h
#define parameters_h

// Math functions are used in the constant expressions below; include math.h
// directly so this header does not depend on Arduino.h being included first
#include <math.h>

// System frequencies and periods
const float f = 250;
const float dt = 1 / f;
const unsigned int dt_us = dt * 1e6;

// IMU gyroscope offsets (this unit, derived from six-pose telemetry 2026-07-21)
const float b_gyr_x = -1.04;
const float b_gyr_y = -1.20;
const float b_gyr_z = -0.81;

// IMU accelerator offsets and gains (this unit, AN-1057 six-pose procedure 2026-07-21)
const float b_acc_x = -60.4;
const float b_acc_y = -383.5;
const float b_acc_z = -132.1;
const float f_acc_x = 1674.5;
const float f_acc_y = 1671.4;
const float f_acc_z = 1681.5;

// Physical parameters
const float pi = 3.14159265359;
const float g = 9.80665;

// Surface parameters
const float b = 0.0;

// Electrical motor properties
const float Ra = 0.942; // Armature (winding) resistance (Ohm)
const float La = 0.363e-3; // Armature (winding) inductance (H)
const float Km = 36e-3; // Torque constant (Nm/A)
const float ia_max = 6; // Maximum current (A)
const float omega_nl = 6250 * pi / 30; // No load speed (rad/s)

// Mechanical motor properties
const float tau_c = 3.4e-3; // Coulomb friction torque (Nm)
const float bw = 1.513e-5; // Rotational viscous friction coefficient (Nms/rad)
const float omega_fric = 2.0; // Friction feedforward smoothing halfwidth (rad/s): a hard
                              // sign(omega) flips +-tau_c at every zero crossing, and with the
                              // wheels idling near zero it dithers the loop at its limit-cycle
                              // frequency (the ~2 Hz visible wobble)

// Structure properties
const float l = 0.15; // Structure side length (m)
const float m_s = 1.0; // Structure mass (kg)
const float I_s_xx = 3e-3; // Structure moment of inertia around xyz at center of mass (kgm^2)

// Reaction wheel properties
const float m_w = 0.225; // Reaction wheel mass (kg)
const float I_w_xx = 4.855e-4; // Reaction wheel moment of inertia around x axis at center of mass (kgm^2)
const float I_w_yy = 2.439e-4; // Reaction wheel moment of inertia around yz axis at center of mass (kgm^2)

// Combined properties of structure and reaction wheels
const float m_c = m_s + 3 * m_w; // Cubli total mass (kg)
const float I_c_xx_bar = I_s_xx + 2 * I_w_yy + (m_s + 2.0 * m_w) * l * l / 2.0; // In-plane moment of inertia (kgm^2)
const float I_c_xy_bar = -(m_s + m_w) * l * l / 4.0; // Out-of-plane moment of inertia (kgm^2)

// Auxiliary parameters
const float m_c_bar = m_c - m_w;
const float m_c_bar_g_l = m_c_bar * g * l;

// Estimator gains
const float lds = 1; // How much do you trust the accelerometer compared to the gyroscope? This gain determines that.
const float ldw = 150; // How much do you trust the hall sensor compared to the wheel model? This gain determines that.

// Arming qualifier and disarmed-estimator parameters. While disarmed the
// accelerometer correction runs at lds_disarmed for fast settling and the
// estimate's unobservable yaw is pinned to the reference; arming additionally
// requires the cube to be held still with a plausible gravity reading for
// arm_dwell consecutive cycles.
const float lds_disarmed = 10; // Accelerometer correction gain while disarmed
const float omega_still = 0.3; // Maximum gyro magnitude that counts as "held still" (rad/s)
const float acc_arm_lo = 0.9 * g; // Accelerometer magnitude window for arming (m/s^2)
const float acc_arm_hi = 1.1 * g;
const float acc_fuse_lo = 0.5 * g; // Accelerometer magnitude window for fusing the correction step (m/s^2).
const float acc_fuse_hi = 1.5 * g; // Broad on purpose: the Gibbs clamp already guards the singularity, and a
                                   // tight window starves tilt correction during hard balancing wobble.
const int arm_dwell = 50; // Consecutive qualifying cycles required to arm (50 cycles = 0.2 s)

// Armed-phase estimator safeguards. The yaw leak slowly absorbs secular gyro
// bias drift in the unobservable yaw (time constant yaw_leak_tau); the
// attitude watchdog force-terminates when a clean gravity measurement
// disagrees with the estimate for att_sane_cycles (a fallen cube must never
// keep its motors energized because the estimator went blind).
const float yaw_leak_tau = 2.0; // Yaw leak time constant while armed (s). Fast on purpose: the
                                // standing yaw error it leaves (bias * tau) sets the persistent yaw
                                // torque the controller injects into common-mode wheel momentum, and
                                // the corner-friction exit for that momentum is tiny. The leak pauses
                                // during commanded spins so it cannot eat the pirouette.
const float att_sane_ang = 45.0 * pi / 180.0; // Estimate-vs-gravity disagreement threshold (rad). Below the corner-to-face geometry (54.7 deg) so a face-fall with a blind estimator trips it; healthy estimator error is a few degrees.
const int att_sane_cycles = 125; // Sustained disagreement cycles before forced terminate (0.5 s)

// Wheel spin-down: after a disarm or terminate the wheels are braked to rest
// instead of freewheeling for tens of seconds
const float ia_brake = 3.0; // Braking current magnitude during spin-down (A)
const float omega_stop = 5.0; // Wheel speed below which spin-down completes (rad/s)
const float omega_brake_max = 330.0; // Above this wheel speed braking is ineffective (back-EMF
                                     // ~ Km*omega approaches the supply voltage and the ESCON
                                     // cannot regenerate): coast first, then brake
const float hall_rezero_rate = 0.002; // Disarmed-at-rest hall bias re-zeroing rate (fraction per cycle
                                     // of the residual reading); heals a hall calibration taken
                                     // before the ESCON outputs fully settled



// Network commands replace the retired tap gesture (UDP "STOP"/"LAND" on
// port 47270 and the web page); a landing may only start from quiet balancing
const float land_phi_max = 15.0 * pi / 180.0; // Maximum error angle to accept a LAND command (rad)

// Status beacon: broadcast while balancing too (one bounded UDP send per
// second on the control path, comparable to the existing armed-phase OTA
// polling; set to false after auto-trim has been validated if preferred)
const bool status_while_armed = true;

// Auto-trim of the balance point: a persistent wheel-angle offset is the
// signature of a center-of-mass mismatch and is slowly bled into a body-frame
// trim of the reference quaternion (learned value persisted in flash)
const float trim_rate = 3e-6; // Trim integrator rate (rad trim per rad wheel angle per s)
const float trim_quiet_acc = 0.5; // Adapt trim only when |a_mag - g| is below this (m/s^2):
                                  // wide-window fusion during wobble biases the estimate, and the
                                  // trim must never learn that bias into the reference
const float trim_quiet_omega = 0.5; // Adapt trim only below this body rate (rad/s)
const float trim_max = 3.0 * pi / 180.0; // Trim clamp (rad)
const float phi_quiet = 8.0 * pi / 180.0; // Adapt only below this error angle (rad)

// Soft landing
const float kg_land = 10.0; // Rate-governor gain (saturates the clamp at ~1 rad/s of rate error)
const float omega_d1 = 1.5; // Target descent rate, corner->edge phase (rad/s)
const float omega_d2 = 1.2; // Target descent rate, edge->face phase (rad/s)
const float lean_rate = 3.0 * pi / 180.0; // L0 reference lean ramp (rad/s)
const float lean_cap = 10.0 * pi / 180.0; // Maximum L0 reference lean (rad)
const float lean_engage = 6.0 * pi / 180.0; // Up-vector departure that starts the descent (rad)
const float ang_edge_hit = 8.0 * pi / 180.0; // Edge waypoint proximity ending L1 (rad)
const float ang_face_hit = 10.0 * pi / 180.0; // Face waypoint proximity ending L2 (rad)
const float ang_spike_gate = 15.0 * pi / 180.0; // Contact spikes only count this close to a waypoint (rad)
const float land_contact = 6.0; // Accel deviation registering as ground contact (m/s^2); descent dynamics alone reach 3-4
const float omega_land_abort = 8.0; // Body-rate abort, above the 6.4 rad/s free-fall touchdown (rad/s)
const float omega_w_abort = 600.0; // Defensive wheel-overspeed abort (rad/s)
const float tau_land_max = 0.9 * Km * ia_max; // Governor torque clamp (Nm)
const int land_t0_max = 375; // L0 timeout in cycles (1.5 s): hand over to the governor
const int land_t12_max = 500; // Descent-phase timeout in cycles (2 s): abort

// Edge catch and hold (landing increment 2). The capture region of the
// single-wheel edge balance is +-8 deg (saturation tilt of one wheel against
// gravity about the edge); the catch gate keeps entry speeds stoppable
// inside it.
const float kp_edge = 150.0; // Edge-hold proportional gain (threshold is ~76 s^-2)
const float kd_edge = 15.0; // Edge-hold damping gain
const float brake_zero_ang = 15.0 * pi / 180.0; // Below this edge distance L1 brakes toward zero rate (rad)
const float catch_ok = 1.2; // Maximum body rate to enter the hold (rad/s)
const float ang_hold_enter = 6.0 * pi / 180.0; // Maximum edge distance to enter the hold (rad)
const float ang_past_edge = 37.0 * pi / 180.0; // Face distance meaning the edge was overshot (rad)
const float hold_bail = 8.0 * pi / 180.0; // Forward tilt ending the hold into the face descent (rad)
const float hold_back_bail = 4.0 * pi / 180.0; // Backward tilt aborting to spin-down (rad)
const float omega_hold_max = 3.0; // Body rate aborting the hold (rad/s)
const int catch_max = 75; // Catch settle window in cycles (0.3 s)
const float lean_quiet = 0.4; // Body rate below which the pre-lean settle completes (rad/s)
const int lean_settle = 50; // Consecutive quiet cycles required before leaning (0.2 s)
const int progress_stall = 125; // Cycles without waypoint progress before bailing (0.5 s)
const int hold_cycles = 375; // Nominal balanced-pause duration in cycles (1.5 s)

// Common-mode wheel momentum drain: equal deceleration on all three wheels,
// reacted by the corner's static ground friction (the only path that can shed
// yaw-direction momentum). Time constant ~ I_w_xx / k_cm_drain ~ 3 s.
const float k_cm_drain = 1.5e-4; // Drain gain (Nm per rad/s of common-mode wheel speed)

// Controller gains. These must be re-tuned if your cube has different dynamics (weights, inertias, dimensions, etc.)
const float kp = 300;
const float kd = 40;
const float kpw = 0.009;
const float kdw = 0.02;

// One out of three of the following initial reference quaternions should be uncommented
// Quaternion reference (Cubli sitting on on corner, corrected for center of mass misalignment)
const float phi_e = 0 * pi / 180.0;
const float qu0 =                    cos(phi_e / 2.0 + acos(sqrt(3.0) / 3.0) / 2.0);
const float qu1 =  sqrt(2.0) / 2.0 * sin(phi_e / 2.0 + acos(sqrt(3.0) / 3.0) / 2.0);
const float qu2 = -sqrt(2.0) / 2.0 * sin(phi_e / 2.0 + acos(sqrt(3.0) / 3.0) / 2.0);
const float qu3 =  0.0;

// Quaternion reference (Cubli sitting on x axis edge, corrected for center of mass misalignment)
// const float phi_e = -3.0 * pi / 180.0;
// const float qu0 = cos(phi_e / 2.0 - pi / 8.0);
// const float qu1 = cos(phi_e / 2.0 + 3.0 * pi / 8.0);
// const float qu2 = 0.0;
// const float qu3 = 0.0;

// Quaternion reference (Cubli sitting on y axis edge, corrected for center of mass misalignment)
// const float phi_e = -3.0 * pi / 180.0;
// const float qu0 = cos(phi_e / 2.0 - pi / 8.0);
// const float qu1 = 0.0;
// const float qu2 = -cos(phi_e / 2.0 + 3.0 * pi / 8.0);
// const float qu3 = 0.0;

// Quaternion stuff
const float qu0_qu0 = qu0 * qu0;
const float qu0_qu1 = qu0 * qu1;
const float qu0_qu2 = qu0 * qu2;
const float qu0_qu3 = qu0 * qu3;
const float qu1_qu1 = qu1 * qu1;
const float qu1_qu2 = qu1 * qu2;
const float qu1_qu3 = qu1 * qu3;
const float qu2_qu2 = qu2 * qu2;
const float qu2_qu3 = qu2 * qu3;
const float qu3_qu3 = qu3 * qu3;

// Minimum and maximum error limits (for control safety)
// phi_min bounds TILT at arming (yaw is pinned away while disarmed). Do not
// widen it: per-wheel torque is Km * ia_max = 0.216 Nm against a gravity
// torque of ~2.13 * sin(tilt) Nm, which caps recoverable tilt at ~8-10 deg.
const float phi_min = 5.0 * pi / 180.0;
const float phi_max = 40.0 * pi / 180.0;

// Minimum jerk trajectory parameters
const float pos_traj = 2.0 * pi; // Trajectory path (rad)
const float t_rest = 10.0; // Rest time (s)
const float t_traj = 20.0; // Trajectory time (s)
const float cra_0 = 720.0 * pos_traj / pow(t_traj, 5);
const float sna_0 = 360.0 * pos_traj / pow(t_traj, 4);
const float jer_0 =  60.0 * pos_traj / pow(t_traj, 3);

#endif
