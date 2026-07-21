#include "icm20948.h"

// Class constructor 
ICM20948::ICM20948(int pin_sda, int pin_scl) : pin_sda(pin_sda), pin_scl(pin_scl), wirePort(0) {
}

// Initialize sensor
void ICM20948::init() {
    // Set up i2c communication at 400 kHz
    wirePort.begin(pin_sda, pin_scl, 400000);

    // Setup IMU
    imu = ICM20948_WE(&wirePort);

    // Verify that sensor works correctly
    if(imu.init()) {
        // Sensor is responding correctly
        Serial.println("ICM20948 is responding.");

        digitalWrite(RGB_BUILTIN, HIGH);
        delay(100);
        digitalWrite(RGB_BUILTIN, LOW);
    } else {
        // Sensor is not responding
        Serial.println("ICM20948 is not responding.");

        for(int i = 0; i < 10; i++) {
            neopixelWrite(RGB_BUILTIN, 255, 0, 0);
            delay(100);
            digitalWrite(RGB_BUILTIN, LOW);
            delay(100);
        }
    }

    // Set measurement ranges for accelerometer (2 m/s/s) and gyroscope (2000 dps)
    imu.setAccRange(ICM20948_ACC_RANGE_2G);
    imu.setGyrRange(ICM20948_GYRO_RANGE_2000);
    
    // Configure digital low-pass filters. The gyroscope keeps the narrow
    // filter (the controller's rate damping depends on it). The accelerometer
    // uses a ~50 Hz filter: wide enough that a finger tap on the frame
    // survives for the tap-to-disarm detector, while the estimator stays
    // protected by the fusion validity window and its slow correction gain.
    imu.setAccDLPF(ICM20948_DLPF_3);
    imu.setGyrDLPF(ICM20948_DLPF_6);
    
    // Disable divider to get maximum possible output data rate (approximately 1.1 kHz)
    imu.setAccSampleRateDivider(0);
    imu.setGyrSampleRateDivider(0);
}

// Read sensor data 
void ICM20948::read() {
    // Get values from sensor
    imu.readSensor();
    acc = imu.getAccRawValues();
    gyr = imu.getGyrValues();

    // Apply calibration offsets and gains and swap axes
    ax = -(acc.x - b_acc_x) / f_acc_x;
    ay = -(acc.z - b_acc_z) / f_acc_z;
    az = -(acc.y - b_acc_y) / f_acc_y;

    gx = -(gyr.x - b_gyr_x) * pi / 180;
    gy = -(gyr.z - b_gyr_z) * pi / 180;
    gz = -(gyr.y - b_gyr_y) * pi / 180;
}
