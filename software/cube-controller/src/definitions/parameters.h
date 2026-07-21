#ifndef parameters_h
#define parameters_h

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
const float acc_fuse_lo = 0.85 * g; // Accelerometer magnitude window for fusing the correction step (m/s^2)
const float acc_fuse_hi = 1.15 * g;
const int arm_dwell = 50; // Consecutive qualifying cycles required to arm (50 cycles = 0.2 s)

// Wheel spin-down: after a disarm or terminate the wheels are braked to rest
// instead of freewheeling for tens of seconds
const float ia_brake = 3.0; // Braking current magnitude during spin-down (A)
const float omega_stop = 5.0; // Wheel speed below which spin-down completes (rad/s)

// Tap-to-disarm: two sharp accelerometer spikes while balancing disarm the
// cube gracefully (times expressed in control cycles at f = 250 Hz)
const float tap_thresh = 1.5; // Accel-magnitude deviation registering as a tap (m/s^2). Bench-measured: armed taps reach 1.9-2.7 (the balancing cube yields, halving the peak vs at-rest taps at 3.7-6); balancing background stays below ~0.8.
const int tap_window = 175; // Maximum cycles between the two taps (0.7 s)
const int tap_refract = 20; // Cycles ignored after a registered spike (0.08 s)

// Status beacon: broadcast while balancing too (one bounded UDP send per
// second on the control path, comparable to the existing armed-phase OTA
// polling; set to false after auto-trim has been validated if preferred)
const bool status_while_armed = true;

// Auto-trim of the balance point: a persistent wheel-angle offset is the
// signature of a center-of-mass mismatch and is slowly bled into a body-frame
// trim of the reference quaternion (learned value persisted in flash)
const float trim_rate = 3e-6; // Trim integrator rate (rad trim per rad wheel angle per s)
const float trim_max = 3.0 * pi / 180.0; // Trim clamp (rad)
const float phi_quiet = 8.0 * pi / 180.0; // Adapt only below this error angle (rad)

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
