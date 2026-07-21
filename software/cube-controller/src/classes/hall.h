#ifndef hall_h
#define hall_h

#include "Arduino.h"

#include "../definitions/parameters.h"

// Hall class
class Hall {
    public:
        // Class constructor
        Hall(int pin_speed);
        // Initialize and calibrate
        void init();
        // Read angular velocity
        void read();
        // Slowly re-zero the bias against the current reading (call only with
        // the wheel at rest and the motor disabled)
        void rezero();
        // Angular velocity (rad/s)
        float omega;

    private:
        // Input pin
        int pin_speed;
        // Bias (calibration offset)
        float bias;
};

#endif