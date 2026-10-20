#include <MeMCore.h>
#include <EEPROM.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "Config.h"
#include "Logic.h"

static_assert(ULTRASONIC_PORT >= 1 && ULTRASONIC_PORT <= 2, "Ultrasonic must use port 1 or 2");
static_assert(LINE_PORT >= 1 && LINE_PORT <= 2 && LINE_PORT != ULTRASONIC_PORT, "Line sensor needs other digital port");
static_assert(DECODER_A != DECODER_B && DECODER_A != LDR_PIN && DECODER_A != IR_PIN &&
              DECODER_B != LDR_PIN && DECODER_B != IR_PIN && LDR_PIN != IR_PIN, "Pin conflict");
static_assert(OUT_IR < 4 && OUT_RED < 4 && OUT_GREEN < 4 && OUT_BLUE < 4 &&
              OUT_IR != OUT_RED && OUT_IR != OUT_GREEN && OUT_IR != OUT_BLUE &&
              OUT_RED != OUT_GREEN && OUT_RED != OUT_BLUE && OUT_GREEN != OUT_BLUE, "Decoder mapping invalid");
static_assert(LEFT_FORWARD_SIGN * LEFT_FORWARD_SIGN == 1 && RIGHT_FORWARD_SIGN * RIGHT_FORWARD_SIGN == 1, "Motor signs must be +/-1");
static_assert(DRIVE_PWM > MAX_CORRECTION_PWM && DRIVE_PWM + MAX_CORRECTION_PWM <= 255, "PWM settings invalid");

MeDCMotor leftMotor(LEFT_MOTOR_PORT), rightMotor(RIGHT_MOTOR_PORT);
MeLineFollower lineSensor(LINE_PORT);
MePort ultrasonicPort(ULTRASONIC_PORT);
MeBuzzer buzzer;

struct Calibration {
  uint32_t magic;
  uint16_t tag;
  uint8_t colourMask, sensorMask;
  ColourModel colour[6];
  uint16_t irNear, irClear, irNone;
  uint16_t usCentreTenths;
  uint16_t checksum;
};
Calibration cal = {};
struct SensorData {
  uint8_t line;
  int irOff, irOn;
  uint16_t irSignal; // 1000*(off-on)/off: proximity proxy, NOT centimetres
  bool irValid;
  float usCm; // -1 = timeout/no echo in chosen range
};
SensorData sensed = {};
bool armed = false, running = false, aborted = false, streaming = false;
bool lineLatched = false, clearTiming = false;
unsigned long clearSince = 0, lastSensor = 0, lastLog = 0;
char commandBuffer[48];
uint8_t commandLength = 0;
bool commandOverflow = false;

void stopMotor() { leftMotor.stop(); rightMotor.stop(); }
void halt() { stopMotor(); running = false; armed = false; streaming = false; aborted = true; }
// The stop matcher is shared across idle/busy readers so "stop" still works
// if serial bytes arrive across a sensor read. ! is the shortest abort command.
bool stopRequested(char c) {
  static uint8_t matched = 0;
  const char word[] = "stop";
  if (c == '!' || c == 'x') { matched = 0; return true; }
  matched = c == word[matched] ? matched + 1 : (c == 's' ? 1 : 0);
  if (matched == 4) { matched = 0; return true; }
  return false;
}
// Busy operations accept stop/!/x; other input is discarded, preventing
// queued movement from executing after a test finishes.
bool pollStop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (stopRequested(c)) {
      halt(); commandLength = 0;
      Serial.println(F("STOPPED; send arm before further movement."));
    }
  }
  return !aborted;
}
bool waitMs(unsigned long duration) {
  unsigned long start = millis();
  while (millis() - start < duration) {
    if (!pollStop()) return false;
    delay(1);
  }
  return !aborted;
}
void selectEmitter(uint8_t output) {
  digitalWrite(DECODER_A, output & 1 ? HIGH : LOW);
  digitalWrite(DECODER_B, output & 2 ? HIGH : LOW);
}
void shineIR() { selectEmitter(OUT_IR); }
void shineRed() { selectEmitter(OUT_RED); }
void shineGreen() { selectEmitter(OUT_GREEN); }
void shineBlue() { selectEmitter(OUT_BLUE); }
// With G permanently LOW there is NO all-off selection. Parking on red keeps
// IR off. Brief intermediate selections during bit changes are allowed to settle.
void parkEmitter() { if (WIRING_CONFIRMED) shineRed(); }
int adcAverage(uint8_t pin) {
  analogRead(pin); // discard first conversion after ADC channel changes
  long sum = 0;
  for (uint8_t i = 0; i < ADC_SAMPLES; ++i) sum += analogRead(pin);
  return int(sum / ADC_SAMPLES);
}
float readUltrasonic() {
  // Direct single-wire measurement gives an explicit pulseIn timeout. Some
  // Makeblock library versions ignore their exposed timeout argument.
  uint8_t pin = ultrasonicPort.pin2();
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW); delayMicroseconds(2);
  digitalWrite(pin, HIGH); delayMicroseconds(10);
  digitalWrite(pin, LOW); pinMode(pin, INPUT);
  unsigned long echo = pulseIn(pin, HIGH, US_TIMEOUT_US);
  if (echo == 0 || echo > US_MAX_CM * 58UL) return -1;
  return echo / 58.0f;
}
bool readSensors() {
  sensed.line = lineSensor.readSensors();
  sensed.usCm = readUltrasonic();
  shineRed(); // IR off; visible red remains on
  if (!waitMs(IR_SETTLE_MS)) return false;
  sensed.irOff = adcAverage(IR_PIN);
  shineIR();
  if (!waitMs(IR_SETTLE_MS)) { parkEmitter(); return false; }
  sensed.irOn = adcAverage(IR_PIN);
  parkEmitter();
  int drop = sensed.irOff - sensed.irOn;
  sensed.irValid = sensed.irOff > 20 && sensed.irOn > 3 && drop >= -5;
  sensed.irSignal = (drop > 0 && sensed.irOff > 0) ? (1000L * drop / sensed.irOff) : 0;
  return true;
}
void printSensors() {
  Serial.print(F("line=")); Serial.print(sensed.line);
  Serial.print(F(" us_cm=")); Serial.print(sensed.usCm, 1);
  Serial.print(F(" ir_off=")); Serial.print(sensed.irOff);
  Serial.print(F(" ir_on=")); Serial.print(sensed.irOn);
  Serial.print(F(" ir_signal=")); Serial.print(sensed.irSignal);
  Serial.print(F(" ir_valid=")); Serial.println(sensed.irValid);
}
bool readRGB(uint16_t rgb[3]) {
  const uint8_t outputs[3] = { OUT_RED, OUT_GREEN, OUT_BLUE };
  for (int k = 0; k < 3; ++k) {
    selectEmitter(outputs[k]);
    if (!waitMs(COLOUR_SETTLE_MS)) { parkEmitter(); return false; }
    rgb[k] = adcAverage(LDR_PIN);
  }
  parkEmitter();
  return true;
}
bool rgbValid(const uint16_t rgb[3]) {
  for (int k = 0; k < 3; ++k) if (rgb[k] <= 3 || rgb[k] >= 1020) return false;
  return true;
}
void printRGB(const uint16_t rgb[3]) {
  Serial.print(F("RGB_ADC="));
  for (int k = 0; k < 3; ++k) { Serial.print(rgb[k]); Serial.print(' '); }
}
void printColour(int c) {
  switch (c) {
    case 0: Serial.print(F("red")); break;
    case 1: Serial.print(F("green")); break;
    case 2: Serial.print(F("orange")); break;
    case 3: Serial.print(F("pink")); break;
    case 4: Serial.print(F("blue")); break;
    case 5: Serial.print(F("white")); break;
    default: Serial.print(F("UNKNOWN"));
  }
}
int detectColour() {
  uint16_t rgb[3]; float best, second;
  if (!readRGB(rgb)) return -1;
  int result = rgbValid(rgb) ? classifyColour(rgb, cal.colour, cal.colourMask, best, second) : -1;
  printRGB(rgb); Serial.print(F(" result=")); printColour(result); Serial.println();
  return result;
}
uint16_t calibrationChecksum(const Calibration &data) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&data);
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < offsetof(Calibration, checksum); ++i) {
    crc ^= uint16_t(bytes[i]) << 8;
    for (uint8_t b = 0; b < 8; ++b) crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}
void loadCalibration() {
  Calibration temp;
  EEPROM.get(0, temp);
  if (temp.magic == 0x414D5A31UL && temp.tag == CALIBRATION_TAG &&
      temp.checksum == calibrationChecksum(temp)) {
    cal = temp; Serial.println(F("Calibration loaded."));
  } else {
    memset(&cal, 0, sizeof(cal)); Serial.println(F("No matching saved calibration; train before run."));
  }
}
void saveCalibration() {
  cal.magic = 0x414D5A31UL; cal.tag = CALIBRATION_TAG;
  cal.checksum = calibrationChecksum(cal);
  EEPROM.put(0, cal); Serial.println(F("Calibration saved in EEPROM."));
}
void dumpCalibration() {
  Serial.print(F("colourMask=")); Serial.print(cal.colourMask);
  Serial.print(F(" sensorMask=")); Serial.println(cal.sensorMask);
  for (int c = 0; c < 6; ++c) {
    printColour(c); Serial.print(F(" mean/spread: "));
    for (int k = 0; k < 3; ++k) {
      Serial.print(cal.colour[c].mean[k]); Serial.print('/');
      Serial.print(cal.colour[c].spread[k]); Serial.print(' ');
    }
    Serial.println();
  }
  Serial.print(F("IR near/clear/none=")); Serial.print(cal.irNear); Serial.print('/');
  Serial.print(cal.irClear); Serial.print('/'); Serial.println(cal.irNone);
  Serial.print(F("US centre cm=")); Serial.println(cal.usCentreTenths / 10.0f, 1);
}
void trainColour(int c) {
  uint32_t sum[3] = {};
  uint16_t low[3] = {1023,1023,1023}, high[3] = {};
  for (int n = 0; n < COLOUR_TRAIN_SWEEPS; ++n) {
    uint16_t rgb[3];
    if (!readRGB(rgb)) return;
    printRGB(rgb); Serial.println();
    if (!rgbValid(rgb)) { Serial.println(F("Not stored: ADC saturated; check circuit/brightness.")); return; }
    for (int k = 0; k < 3; ++k) {
      sum[k] += rgb[k]; if (rgb[k] < low[k]) low[k] = rgb[k];
      if (rgb[k] > high[k]) high[k] = rgb[k];
    }
  }
  for (int k = 0; k < 3; ++k) {
    cal.colour[c].mean[k] = sum[k] / COLOUR_TRAIN_SWEEPS;
    cal.colour[c].spread[k] = max(12, (high[k] - low[k]) / 2);
  }
  cal.colourMask |= 1 << c;
  Serial.println(F("Colour learned in RAM; use save to persist."));
}
void trainSensor(const char *name) {
  uint32_t total = 0;
  bool us = !strcmp(name, "uscenter");
  for (int n = 0; n < 16; ++n) {
    if (!readSensors()) return;
    if (us) {
      if (sensed.usCm < 3 || sensed.usCm >= US_MAX_CM) {
        Serial.println(F("US calibration failed: need a valid wall >=3 cm from sensor.")); return;
      }
      total += uint16_t(sensed.usCm * 10);
    } else {
      if (!sensed.irValid) { Serial.println(F("IR calibration failed: invalid/saturated ADC.")); return; }
      total += sensed.irSignal;
    }
    if (!waitMs(SENSOR_INTERVAL_MS)) return;
  }
  uint16_t value = total / 16;
  if (us) { cal.usCentreTenths = value; cal.sensorMask |= 8; }
  else if (!strcmp(name, "irnear")) { cal.irNear = value; cal.sensorMask |= 1; }
  else if (!strcmp(name, "irclear")) { cal.irClear = value; cal.sensorMask |= 2; }
  else { cal.irNone = value; cal.sensorMask |= 4; }
  Serial.print(F("Learned ")); Serial.print(name); Serial.print(' '); Serial.println(value);
  Serial.println(F("Use dump to inspect and save to persist."));
}
void setWheels(int left, int right) {
  if (!armed || aborted || !WIRING_CONFIRMED) { stopMotor(); return; }
  leftMotor.run(constrain(left, -255, 255) * LEFT_FORWARD_SIGN);
  rightMotor.run(constrain(right, -255, 255) * RIGHT_FORWARD_SIGN);
}
bool timedDrive(int left, int right, unsigned long ms) {
  if (!armed || aborted || ms == 0) { stopMotor(); return false; }
  setWheels(left, right);
  bool ok = waitMs(ms);
  stopMotor();
  return ok && waitMs(MOTOR_SETTLE_MS);
}
bool moveForward() { return timedDrive(DRIVE_PWM + LEFT_TRIM, DRIVE_PWM + RIGHT_TRIM, FORWARD_TEST_MS); }
bool turnLeft() { return timedDrive(-TURN_PWM, TURN_PWM, LEFT_90_MS); }
bool turnRight() { return timedDrive(TURN_PWM, -TURN_PWM, RIGHT_90_MS); }
bool uTurn() { return timedDrive(-TURN_PWM, TURN_PWM, UTURN_MS); }
void fault(const __FlashStringHelper *message) { halt(); parkEmitter(); Serial.println(message); }
bool wallStep() {
  if (!sensed.irValid) { fault(F("FAULT: invalid IR reading; inspect sensors.")); return false; }
  float usTarget = cal.usCentreTenths / 10.0f;
  float usPressure = 0;
  if (sensed.usCm > 0 && sensed.usCm <= usTarget + US_MISSING_MARGIN_CM)
    usPressure = clamp01((usTarget - sensed.usCm - US_DEADBAND_CM) / US_CORRECTION_SPAN_CM);
  float irPressure = clamp01((float(sensed.irSignal) - cal.irClear) / (cal.irNear - cal.irClear));
  if (usPressure >= 1 && irPressure >= 1) {
    fault(F("FAULT: both sides too close; check placement/calibration.")); return false;
  }
  int correction = ULTRASONIC_ON_LEFT ? correctionPWM(usPressure, irPressure, MAX_CORRECTION_PWM)
                                     : correctionPWM(irPressure, usPressure, MAX_CORRECTION_PWM);
  // Positive correction: left wheel faster, steer gently right.
  setWheels(DRIVE_PWM + LEFT_TRIM + correction, DRIVE_PWM + RIGHT_TRIM - correction);
  return true;
}
bool driveGrid() {
  if (!GRID_TRAVEL_MS) return false;
  unsigned long start = millis();
  while (millis() - start < GRID_TRAVEL_MS) {
    if (!pollStop() || !readSensors() || !wallStep() || !waitMs(SENSOR_INTERVAL_MS)) {
      stopMotor(); return false;
    }
  }
  stopMotor(); return waitMs(MOTOR_SETTLE_MS);
}
bool doubleLeftTurn() { return turnLeft() && driveGrid() && turnLeft(); }
bool doubleRightTurn() { return turnRight() && driveGrid() && turnRight(); }
bool sensorsReady() {
  return cal.sensorMask == 15 && cal.usCentreTenths >= 30 &&
         irCalibrationOrdered(cal.irNear, cal.irClear, cal.irNone);
}
bool raceReady() {
  return WIRING_CONFIRMED && MOTION_CALIBRATED && WAYPOINT_GEOMETRY_CONFIRMED &&
         LEFT_90_MS && RIGHT_90_MS && UTURN_MS && GRID_TRAVEL_MS &&
         cal.colourMask == 63 && sensorsReady();
}
void celebrate() {
  stopMotor();
  const uint16_t notes[] = {523,659,784,1047};
  for (int i = 0; i < 4 && !aborted; ++i) { buzzer.tone(notes[i], 140); waitMs(50); }
  buzzer.noTone();
}
bool onStrip(uint8_t line) { return REQUIRE_BOTH_LINE_SENSORS ? line == 0 : line != 3; }
void handleWaypoint() {
  stopMotor();
  if (!waitMs(MOTOR_SETTLE_MS)) return;
  // Require two consecutive matching, confident readings, with up to four reads.
  int previous = -1, colour = -1;
  for (int attempt = 0; attempt < 4 && !aborted; ++attempt) {
    int current = detectColour();
    if (current >= 0 && current == previous) { colour = current; break; }
    previous = current;
  }
  if (aborted) return;
  if (colour < 0) { fault(F("FAULT: colour uncertain. Stopped; inspect RGB/calibration.")); return; }
  if (colour == 5) {
    running = false; celebrate(); armed = false;
    Serial.println(F("FINISHED: motors remain stopped.")); return;
  }
  if (WAYPOINT_TO_PIVOT_MS != 0) {
    int sign = WAYPOINT_TO_PIVOT_MS > 0 ? 1 : -1;
    if (!timedDrive(sign * (DRIVE_PWM + LEFT_TRIM), sign * (DRIVE_PWM + RIGHT_TRIM),
                    abs(WAYPOINT_TO_PIVOT_MS))) return;
  }
  bool ok = false;
  switch (colour) {
    case 0: ok = turnLeft(); break;
    case 1: ok = turnRight(); break;
    case 2: ok = uTurn(); break;
    case 3: ok = doubleLeftTurn(); break;
    case 4: ok = doubleRightTurn(); break;
  }
  if (!ok) { fault(F("Turn incomplete; stopped.")); return; }
  // Rearm only after both line sensors have cleared the old strip continuously.
  lineLatched = true; clearTiming = false;
}
void raceStep() {
  uint8_t line = lineSensor.readSensors();
  if (!lineLatched && onStrip(line)) {
    stopMotor();
    if (!waitMs(20)) return;
    if (onStrip(lineSensor.readSensors())) handleWaypoint();
    return;
  }
  if (lineLatched) {
    if (line == 3) {
      if (!clearTiming) { clearTiming = true; clearSince = millis(); }
      else if (millis() - clearSince >= LINE_CLEAR_MS) lineLatched = false;
    } else clearTiming = false;
  }
  if (millis() - lastSensor >= SENSOR_INTERVAL_MS) {
    lastSensor = millis();
    if (readSensors()) wallStep();
  }
}
void help() {
  Serial.println(F("AmazeBot test firmware. Serial: 115200, Newline."));
  Serial.println(F("help | status | sensors | stream | quiet | rgb | detect"));
  Serial.println(F("cal red/green/orange/pink/blue/white"));
  Serial.println(F("cal irnear/irclear/irnone/uscenter | dump | save | load"));
  Serial.println(F("arm | pulse <leftPWM> <rightPWM> <ms> | forward"));
  Serial.println(F("left | right | uturn | grid | doubleleft | doubleright"));
  Serial.println(F("run (5s delay by default) | stop | ! (immediate abort)"));
  Serial.println(F("pulse uses physical wheel directions: positive = forward."));
}
void status() {
  Serial.print(F("wiring=")); Serial.print(WIRING_CONFIRMED);
  Serial.print(F(" motion=")); Serial.print(MOTION_CALIBRATED);
  Serial.print(F(" geometry=")); Serial.print(WAYPOINT_GEOMETRY_CONFIRMED);
  Serial.print(F(" sensorsReady=")); Serial.print(sensorsReady());
  Serial.print(F(" raceReady=")); Serial.print(raceReady());
  Serial.print(F(" armed=")); Serial.print(armed);
  Serial.print(F(" running=")); Serial.println(running);
  dumpCalibration();
}
void startRace() {
  if (!armed || !raceReady()) {
    Serial.println(F("Run refused: arm and complete wiring, motion, geometry and all calibrations."));
    status(); return;
  }
  streaming = false;
  Serial.println(F("Starting after configured delay. Send ! to cancel."));
  if (!waitMs(START_DELAY_MS)) return;
  lineLatched = false; clearTiming = false; lastSensor = millis() - SENSOR_INTERVAL_MS;
  running = true;
}
bool parseNumber(char *token, long &value) {
  if (!token || !*token) return false;
  char *end; value = strtol(token, &end, 10); return *end == '\0';
}
void executeCommand(char *text) {
  char *cmd = strtok(text, " \t");
  if (!cmd) return;
  if (!strcmp(cmd, "stop")) { halt(); Serial.println(F("STOPPED")); return; }
  if (!strcmp(cmd, "help")) { help(); return; }
  // While driving, reject everything except stop/help: no blocking sensor
  // diagnostics, retraining or EEPROM writes with motors still running.
  if (running) { Serial.println(F("Send stop before other commands.")); return; }
  if (!strcmp(cmd, "status")) { status(); return; }
  if (!strcmp(cmd, "dump")) { dumpCalibration(); return; }
  if (!strcmp(cmd, "load")) { loadCalibration(); return; }
  if (!strcmp(cmd, "save")) { saveCalibration(); return; }
  if (!strcmp(cmd, "quiet")) { streaming = false; return; }
  if (!WIRING_CONFIRMED) { Serial.println(F("Verify Config.h wiring, set WIRING_CONFIRMED=true, upload again.")); return; }
  aborted = false;
  if (!strcmp(cmd, "arm")) { armed = true; Serial.println(F("Motors armed for explicit commands.")); return; }
  if (!strcmp(cmd, "sensors")) { if (readSensors()) printSensors(); return; }
  if (!strcmp(cmd, "stream")) { streaming = true; return; }
  if (!strcmp(cmd, "rgb")) { uint16_t rgb[3]; if (readRGB(rgb)) { printRGB(rgb); Serial.println(); } return; }
  if (!strcmp(cmd, "detect")) { detectColour(); return; }
  if (!strcmp(cmd, "cal")) {
    streaming = false;
    char *name = strtok(NULL, " \t");
    if (!name) { help(); return; }
    const char *names[] = {"red","green","orange","pink","blue","white"};
    for (int c = 0; c < 6; ++c) if (!strcmp(name, names[c])) { trainColour(c); return; }
    if (!strcmp(name,"irnear") || !strcmp(name,"irclear") || !strcmp(name,"irnone") || !strcmp(name,"uscenter"))
      trainSensor(name);
    else Serial.println(F("Unknown calibration name."));
    return;
  }
  if (!armed) { Serial.println(F("Send arm first; lift wheels for initial direction tests.")); return; }
  streaming = false;
  if (!strcmp(cmd, "pulse")) {
    long left, right, duration;
    if (!parseNumber(strtok(NULL," \t"),left) || !parseNumber(strtok(NULL," \t"),right) ||
        !parseNumber(strtok(NULL," \t"),duration) || strtok(NULL," \t") ||
        left < -255 || left > 255 || right < -255 || right > 255 || duration < 1 || duration > 3000) {
      Serial.println(F("Usage: pulse left[-255..255] right[-255..255] ms[1..3000]")); return;
    }
    timedDrive(left, right, duration); return;
  }
  if (!strcmp(cmd, "forward")) { moveForward(); return; }
  if (!strcmp(cmd, "left") && LEFT_90_MS) { turnLeft(); return; }
  if (!strcmp(cmd, "right") && RIGHT_90_MS) { turnRight(); return; }
  if (!strcmp(cmd, "uturn") && UTURN_MS) { uTurn(); return; }
  if (!strcmp(cmd, "run")) { startRace(); return; }
  if (sensorsReady() && GRID_TRAVEL_MS) {
    if (!strcmp(cmd, "grid")) { driveGrid(); return; }
    if (!strcmp(cmd, "doubleleft") && LEFT_90_MS) { doubleLeftTurn(); return; }
    if (!strcmp(cmd, "doubleright") && RIGHT_90_MS) { doubleRightTurn(); return; }
  }
  Serial.println(F("Unknown command or missing timing/sensor calibration. Use help/status."));
}
void serviceSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (stopRequested(c)) { halt(); commandLength = 0; commandOverflow = false; Serial.println(F("STOPPED")); }
    else if (c == '\n' || c == '\r') {
      if (!commandOverflow && commandLength) {
        commandBuffer[commandLength] = '\0'; executeCommand(commandBuffer);
      } else if (commandOverflow) Serial.println(F("Command too long; discarded."));
      commandLength = 0; commandOverflow = false;
    } else if (commandLength < sizeof(commandBuffer)-1) commandBuffer[commandLength++] = c;
    else commandOverflow = true;
  }
}
void robotSetup() {
  stopMotor(); Serial.begin(115200);
  if (WIRING_CONFIRMED) {
    digitalWrite(DECODER_A, OUT_RED & 1 ? HIGH : LOW);
    digitalWrite(DECODER_B, OUT_RED & 2 ? HIGH : LOW);
    pinMode(DECODER_A, OUTPUT); pinMode(DECODER_B, OUTPUT);
    pinMode(LDR_PIN, INPUT); pinMode(IR_PIN, INPUT); parkEmitter();
  }
  loadCalibration(); help(); status();
  if (AUTO_START_AFTER_BOOT && raceReady()) { armed = true; startRace(); }
}
void robotLoop() {
  serviceSerial();
  if (running) raceStep();
  else if (streaming && !aborted && millis() - lastLog >= 250) {
    lastLog = millis(); if (readSensors()) printSensors();
  }
}
