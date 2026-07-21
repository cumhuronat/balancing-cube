#include "estimator_attitude.h"
#include "../definitions/quat_utils.h"

// Constructor
AttitudeEstimator::AttitudeEstimator(int pin_sda, int pin_scl) : imu(pin_sda, pin_scl) {
    // Set initial rotation quaternion
    q0 = 1.0;
    q1 = 0.0;
    q2 = 0.0;
    q3 = 0.0;

    // Set initial angular velocity
    omega_x = 0.0;
    omega_y = 0.0;
    omega_z = 0.0;

    // Set initial angular velocity bias
    b_omega_x = 0.0;
    b_omega_y = 0.0;
    b_omega_z = 0.0;

    // Start with the disarmed (fast) correction gain; the .ino reverts it to
    // lds at the arming instant
    lds_gain = lds_disarmed;

    // Set initial accelerometer magnitude and fusion flag
    a_mag = 0.0;
    fused = false;
}

// Initializer
void AttitudeEstimator::init() {
    // Initialize IMU sensor object
    imu.init();

    // Angular velocity bias calibration
    calibrate();
}

// Angular velocity bias calibration and initial attitude
void AttitudeEstimator::calibrate() {
    // Average 500 samples (~1 s; IMU output data rate is ~1.1 kHz so 1 ms
    // spacing yields fresh samples) of gyroscope data for the bias, on top of
    // the one-time calibration that was done manually in advance. The
    // accelerometer is averaged in the same loop to seed the attitude.
    float a_x = 0, a_y = 0, a_z = 0;
    for(int i = 0; i < 500; i++) {
        // Read sensor values
        imu.read();

        // Add 1/500th part of the current readings to the averages
        b_omega_x += imu.gx / 500;
        b_omega_y += imu.gy / 500;
        b_omega_z += imu.gz / 500;
        a_x += imu.ax / 500;
        a_y += imu.ay / 500;
        a_z += imu.az / 500;

        delay(1);
    }

    // Seed the attitude from averaged gravity so the cube may boot in any
    // resting pose (yaw is arbitrary here; it is pinned while disarmed)
    quat_from_accel(a_x, a_y, a_z, q0, q1, q2, q3);
}

// Estimate step
void AttitudeEstimator::estimate() {
    // Read values from IMU
    imu.read();

    // Get angular velocity from IMU gyroscope data and apply second offset correction
    omega_x = imu.gx - b_omega_x;
    omega_y = imu.gy - b_omega_y;
    omega_z = imu.gz - b_omega_z;

    // Predict step
    predict(omega_x, omega_y, omega_z);

    // Get linear acceleration from IMU accelerometer data
    float ax = imu.ax;
    float ay = imu.ay;
    float az = imu.az;

    // Only fuse the accelerometer when it plausibly measures gravity; during
    // swings or free fall its direction lies and would corrupt the estimate
    a_mag = sqrt(ax * ax + ay * ay + az * az);
    fused = false;
    if(a_mag >= acc_fuse_lo && a_mag <= acc_fuse_hi) {
        fused = true;
        // Normalize linear acceleration
        ax /= a_mag;
        ay /= a_mag;
        az /= a_mag;

        // Correct step
        quat_accel_correct(lds_gain * dt, ax, ay, az, q0, q1, q2, q3);
    }

    // Normalize rotation quaternion
    float q_norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= q_norm;
    q1 /= q_norm;
    q2 /= q_norm;
    q3 /= q_norm;
}

// Pin the unobservable yaw onto the reference quaternion
void AttitudeEstimator::pin_yaw(float r0, float r1, float r2, float r3) {
    quat_pin_yaw(r0, r1, r2, r3, q0, q1, q2, q3);
}

// Partially pin the unobservable yaw toward the reference (armed leak)
void AttitudeEstimator::pin_yaw_partial(float r0, float r1, float r2, float r3, float frac) {
    quat_pin_yaw_partial(r0, r1, r2, r3, q0, q1, q2, q3, frac);
}

// Set the accelerometer correction gain
void AttitudeEstimator::set_correction_gain(float gain) {
    lds_gain = gain;
}

// Estimate step
void AttitudeEstimator::predict(float omega_x, float omega_y, float omega_z) {
    // Predict rotation quaternion time derivative
    float q0_dot = 0.5 * (-q1 * omega_x - q2 * omega_y - q3 * omega_z);
    float q1_dot = 0.5 * ( q0 * omega_x - q3 * omega_y + q2 * omega_z);
    float q2_dot = 0.5 * ( q3 * omega_x + q0 * omega_y - q1 * omega_z);
    float q3_dot = 0.5 * (-q2 * omega_x + q0 * omega_z + q1 * omega_y);

    // Predict rotation quaternion
    q0 += q0_dot * dt;
    q1 += q1_dot * dt;
    q2 += q2_dot * dt;
    q3 += q3_dot * dt;
}

float AttitudeEstimator::ax() {
    return imu.ax;
}

float AttitudeEstimator::ay() {
    return imu.ay;
}

float AttitudeEstimator::az() {
    return imu.az;
}

float AttitudeEstimator::gx() {
    return imu.gx;
}

float AttitudeEstimator::gy() {
    return imu.gy;
}

float AttitudeEstimator::gz() {
    return imu.gz;
}
