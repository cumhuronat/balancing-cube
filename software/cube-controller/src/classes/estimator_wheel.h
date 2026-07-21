#ifndef estimator_wheel_h
#define estimator_wheel_h

#include "Arduino.h"

#include "../definitions/parameters.h"
#include "../classes/hall.h"

// Wheel estimator class
class WheelEstimator {
    public:
        // Constructor
        WheelEstimator(int pin_speed);
        // Initializer
        void init();
        // Estimate step
        void estimate(float tau = 0.0);
        // Zero the integrated wheel angle (called at the arming instant:
        // theta_w accumulated from hall bias while idle has no physical
        // meaning and would inject a spurious torque)
        void reset_theta();
        // Angular displacement (rad) and angular velocity (rad/s) estimations
        float theta_w, omega_w, omega_w_dot;
        // Get hall sensor reading directly from hall sensor
        float omega();

    private:
        // Motor hall sensor object
        Hall hall;
        // Predict step
        void predict(float tau);
        // Correct step
        void correct(float omega_w_m);
};

#endif
