#ifndef pins_h
#define pins_h

// Motor controllers. M1/M3 are swapped relative to the motherboard schematic
// because the motor cables are plugged into swapped ESCON positions on this
// build (verified empirically 2026-07-21: firmware channel vs physical wheel
// axis via IMU, and drive vs hall channel via hand-spin telemetry).
const int M1_ENABLE = 5;
const int M1_CURRENT = 6;
const int M2_ENABLE = 17;
const int M2_CURRENT = 18;
const int M3_ENABLE = 13;
const int M3_CURRENT = 12;

// Motor speeds (same M1/M3 swap: each cable carries drive and hall together)
const int M1_SPEED = 7;
const int M2_SPEED = 9;
const int M3_SPEED = 10;

// IMU
const int IMU_SDA = 47;
const int IMU_SCL = 21;

#endif
