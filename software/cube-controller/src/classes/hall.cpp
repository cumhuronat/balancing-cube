#include "hall.h"

// Class constructor
Hall::Hall(int pin_speed) : pin_speed(pin_speed) {
}

void Hall::init() {
    // Set analog read resolution
    analogReadResolution(12);

    // Indicate wheel calibration in progress using blue LED
    neopixelWrite(RGB_BUILTIN, 0, 0, 255);
    
    // Logging
    Serial.print("Starting hall sensor calibration (pin ");
    Serial.print(pin_speed);
    Serial.print(")... ");

    // Calibrate reading by averaging 100 samples over half a second
    for(int i = 0; i < 100; i++) {
        bias += ((float) analogReadMilliVolts(pin_speed) / 1000) / 100;
        delay(5);
    }

    // Remove nominal voltage (wheel at rest) from bias calculation
    bias -= 1.65;

    // Logging
    Serial.print("Done. Offset is ");
    Serial.print(bias);
    Serial.println(" V.");
}

// Slowly re-zero the bias against the current reading. The boot calibration
// samples the ESCON analog output ~1.5 s after power-up; if that output had
// not fully settled, the residual bias makes wheel angles ramp while idle and
// poisons everything that integrates them. This heals it continuously.
void Hall::rezero() {
    bias += hall_rezero_rate * omega * 30.0 / pi / 13000 * 2.7 * -1;
}

// Read angular velocity
void Hall::read() {
    // Read voltage using ADC (with 12-bit resolution)
    float voltage = (float) analogReadMilliVolts(pin_speed) / 1000;

    // Convert voltage reading to speed in rad/s. Speed is -6500 rpm at 0.3 V and 6500 rpm at 3.0 V. The range between
    // 0 and 0.3 V and 3.0 V and 3.3 V is not used due to slight nonlinearities in that domain. The sign of the output
    // is changed to align with the coordinate system and rotation direction definitions.
    omega = -1 * (-6500 + (voltage - bias - 0.3) * 13000 / 2.7) *  pi / 30;
}
