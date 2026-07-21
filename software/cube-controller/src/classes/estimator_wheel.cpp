#include "estimator_wheel.h"

// Constructor
WheelEstimator::WheelEstimator(int pin_speed) : hall(pin_speed) {
    // Set initial angular displacement and angular velocity
    theta_w = 0.0;
    omega_w = 0.0;
}

// Initializer
void WheelEstimator::init() {
    // Initialize and calibrate hall sensor
    hall.init();
}

// Re-zero the hall bias while the wheel is at rest
void WheelEstimator::rezero_hall() {
    hall.rezero();
}

// Zero the integrated wheel angle
void WheelEstimator::reset_theta() {
    theta_w = 0.0;
}

// Estimate step
void WheelEstimator::estimate(float tau) {
    // Predict step
    predict(tau);

    // Get angular velocity measurement from hall sensor
    hall.read();

    // Correct step
    correct(hall.omega);
}

// Predict step
void WheelEstimator::predict(float tau) {
    // Calculate friction torque (smoothed Coulomb term, matching the
    // controller's feedforward so model and plant stay consistent)
    float sf = omega_w / omega_fric;
    if(sf > 1.0) { sf = 1.0; } else if(sf < -1.0) { sf = -1.0; }
    float tau_f = sf * tau_c + bw * omega_w;

    // Calculate angular acceleration
    omega_w_dot = (1.0 / I_w_xx) * (-tau_f + tau);

    // Predict angular displacement and angular velocity
    theta_w += omega_w * dt + omega_w_dot * dt * dt / 2.0;
    omega_w += omega_w_dot * dt;
}

// Correct step
void WheelEstimator::correct(float omega_w_m) {
    // Correct angular velocity with measurement
    omega_w += ldw * dt * (omega_w_m - omega_w);
}

float WheelEstimator::omega() {
    // Get hall sensor reading
    return hall.omega;
}
