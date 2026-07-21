#ifndef estimator_attitude_h
#define estimator_attitude_h

#include "Arduino.h"

#include "../definitions/parameters.h"
#include "../classes/icm20948.h"

// Attitude estimator class
class AttitudeEstimator {
    public:
        // Constructor
        AttitudeEstimator(int pin_sda, int pin_scl);
        // Initializer
        void init();
        // Estimate step
        void estimate();
        // Pin the unobservable yaw onto the reference quaternion (r0..r3);
        // called while the controller is disarmed
        void pin_yaw(float r0, float r1, float r2, float r3);
        // Set the accelerometer correction gain (lds while armed,
        // lds_disarmed while disarmed)
        void set_correction_gain(float gain);
        // Rotation quaternion estimations
        float q0, q1, q2, q3;
        // Angular velocity (rad/s) estimations
        float omega_x, omega_y, omega_z;
        // Magnitude of the last accelerometer reading (m/s^2)
        float a_mag;
        // Whether the last estimate step fused the accelerometer
        bool fused;
        // Partially pin the unobservable yaw toward the reference (armed leak)
        void pin_yaw_partial(float r0, float r1, float r2, float r3, float frac);
        // Get acceleration and gyroscope values directly from IMU
        float ax(), ay(), az(), gx(), gy(), gz();

    private:
        // IMU sensor object
        ICM20948 imu;
        // Angular velocity bias calibration
        void calibrate();
        // Predict step
        void predict(float omega_x, float omega_y, float omega_z);
        // Accelerometer correction gain currently in use
        float lds_gain;
        // Angular velocity (rad/s) bias
        float b_omega_x, b_omega_y, b_omega_z;
};

#endif