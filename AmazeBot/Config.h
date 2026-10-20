#pragma once
#include <Arduino.h>

// PROPOSED WIRING, NOT FACTS ABOUT YOUR ROBOT. Verify before enabling I/O.
constexpr bool WIRING_CONFIRMED = false;
constexpr uint8_t ULTRASONIC_PORT = 1; // RJ25 port 1, S2 = D12
constexpr uint8_t LINE_PORT = 2;       // RJ25 port 2
constexpr bool ULTRASONIC_ON_LEFT = true;
constexpr uint8_t DECODER_A = A0;      // port 4 S1, select bit 0
constexpr uint8_t DECODER_B = A1;      // port 4 S2, select bit 1
constexpr uint8_t LDR_PIN = A2;        // port 3 S1
constexpr uint8_t IR_PIN = A3;         // port 3 S2
// Decoder enable tied LOW in hardware. Output numbers, not MCU pin numbers.
constexpr uint8_t OUT_IR = 0, OUT_RED = 1, OUT_GREEN = 2, OUT_BLUE = 3;
constexpr uint8_t LEFT_MOTOR_PORT = 9;  // M1
constexpr uint8_t RIGHT_MOTOR_PORT = 10;// M2
constexpr int LEFT_FORWARD_SIGN = -1, RIGHT_FORWARD_SIGN = 1;

// Starting test settings, NOT measured motor calibrations. PWM range 0..255.
constexpr int DRIVE_PWM = 90, TURN_PWM = 90;
constexpr int LEFT_TRIM = 0, RIGHT_TRIM = 0;
constexpr int MAX_CORRECTION_PWM = 22;
constexpr uint16_t FORWARD_TEST_MS = 250;
// Zero means uncalibrated: named turns/grid tests refuse to run.
constexpr uint16_t LEFT_90_MS = 0, RIGHT_90_MS = 0, UTURN_MS = 0;
constexpr uint16_t GRID_TRAVEL_MS = 0;
// After reading a colour, signed travel to place the axle at your turning point.
// Positive = forward, negative = reverse. Zero = pivot where you stopped.
// Measure this with YOUR sensor mounting. The paper PDF cannot determine it.
constexpr int16_t WAYPOINT_TO_PIVOT_MS = 0;
constexpr bool MOTION_CALIBRATED = false;
constexpr bool WAYPOINT_GEOMETRY_CONFIRMED = false;
constexpr bool AUTO_START_AFTER_BOOT = false;
constexpr uint16_t START_DELAY_MS = 5000;
constexpr uint16_t MOTOR_SETTLE_MS = 120;
constexpr uint16_t LINE_CLEAR_MS = 80;
constexpr bool REQUIRE_BOTH_LINE_SENSORS = false; // stop on either sensor by default

// Trial timing/filter settings; check sensor logs on the assembled robot.
constexpr uint16_t COLOUR_SETTLE_MS = 100;
constexpr uint8_t ADC_SAMPLES = 8;
constexpr uint8_t COLOUR_TRAIN_SWEEPS = 8;
constexpr uint16_t IR_SETTLE_MS = 2; // brief says about 1 ms
constexpr uint16_t SENSOR_INTERVAL_MS = 35;
// 40 cm is a TEST range, not a supplied maze dimension. Echo round-trip is
// about 58 us/cm. Adjust to measured maximum useful side-wall distance.
constexpr uint16_t US_MAX_CM = 10;
constexpr unsigned long US_TIMEOUT_US = US_MAX_CM * 58UL + 300UL;
constexpr float US_MISSING_MARGIN_CM = 5.0f;
constexpr float US_CORRECTION_SPAN_CM = 2.0f;
constexpr float US_DEADBAND_CM = 0.5f;
// Increment after changing wiring, sensor height, shielding, resistors,
// LED mapping or colour measurement timing; then retrain and save.
constexpr uint16_t CALIBRATION_TAG = 1;
