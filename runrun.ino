/*
  Basic mBot / mCore movement and music test
  Assumes: left motor on M1, right motor on M2.
  Requires the Makeblock library providing MeMCore.h.

  Runs once after power-on, upload, or reset:
  wait 5 seconds -> forward -> left -> right -> U-turn -> victory tune.
  No sensors are used. Turn durations are initial guesses, not calibrated angles.
  Test in a clear floor area. Switch off power to stop the test early.
*/

#include <MeMCore.h>

MeDCMotor leftMotor(M1);
MeDCMotor rightMotor(M2);
MeBuzzer buzzer;

const int DRIVE_SPEED = 120;
const int TURN_SPEED = 120;

// Adjust these after testing your own robot.
const unsigned long LEFT_90_MS = 800;
const unsigned long RIGHT_90_MS = 800;
const unsigned long U_TURN_MS = 1600;

void stopRobot() {
  leftMotor.run(0);
  rightMotor.run(0);
}

void moveForward(unsigned long durationMs) {
  // Opposite signs because the motors are mounted facing opposite ways.
  leftMotor.run(-DRIVE_SPEED);
  rightMotor.run(DRIVE_SPEED);
  delay(durationMs);
  stopRobot();
}

void turnLeft(unsigned long durationMs) {
  // Left wheel backward, right wheel forward: rotate in place.
  leftMotor.run(TURN_SPEED);
  rightMotor.run(TURN_SPEED);
  delay(durationMs);
  stopRobot();
}

void turnRight(unsigned long durationMs) {
  // Left wheel forward, right wheel backward: rotate in place.
  leftMotor.run(-TURN_SPEED);
  rightMotor.run(-TURN_SPEED);
  delay(durationMs);
  stopRobot();
}

void playSuccessMusic() {
  // Stop the motors before celebrating!
  stopRobot();

  // A short C-E-G-C-G-C victory tune.
  // tone(frequency in Hz, duration in milliseconds)
  buzzer.tone(523, 150);
  delay(50);
  buzzer.tone(659, 150);
  delay(50);
  buzzer.tone(784, 150);
  delay(50);
  buzzer.tone(1047, 350);
  delay(100);
  buzzer.tone(784, 150);
  delay(50);
  buzzer.tone(1047, 500);
  buzzer.noTone();
}

void setup() {
  stopRobot();
  delay(5000);                 // Wait 5 seconds before starting.

  moveForward(1000);           // Forward for 1 second.
  delay(700);

  turnLeft(LEFT_90_MS);        // Aim for a 90-degree left turn.
  delay(700);

  turnRight(RIGHT_90_MS);      // Aim for a 90-degree right turn.
  delay(700);

  turnRight(U_TURN_MS);        // Aim for a 180-degree right turn.
  delay(700);

  playSuccessMusic();
}

void loop() {
  // Empty on purpose: the test runs only once.
  // Press RESET or switch power off/on to run it again.
}