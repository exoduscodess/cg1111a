#include <MeMCore.h>
#include <EEPROM.h>

const uint8_t DEC_A_PIN = A0;
const uint8_t DEC_B_PIN = A1;
const uint8_t LDR_PIN = A2;
const uint8_t IR_PIN = A3;
const uint8_t Y_IR_EMITTER = 0;
const uint8_t Y_GREEN = 1;
const uint8_t Y_BLUE = 2;
const uint8_t Y_RED = 3;
const uint8_t IR_OFF_OUTPUT = Y_BLUE;   // "emitter off" state (some LED is always on)
const uint8_t C_RED = 0, C_GREEN = 1, C_ORANGE = 2, C_PINK = 3, C_LBLUE = 4, C_WHITE = 5;
const uint8_t NUM_COLOURS = 6;
const char* const COLOUR_NAMES[NUM_COLOURS] = {"RED", "GREEN", "ORANGE", "PINK", "LIGHT BLUE", "WHITE"};
const unsigned long LDR_SETTLE_MS = 80;   // LDR is slow: wait after switching LED
const uint8_t CAL_SAMPLES = 5;
const uint8_t EEPROM_MAGIC = 0xC3;
int refs[NUM_COLOURS][3];                 // calibrated {R,G,B} LDR readings
bool haveCal = false;
const bool IR_ON_LEFT = true;
const int LEFT_SIGN = -1;                 
const int RIGHT_SIGN = 1;                
const int BASE_SPEED = 240;              
const int LEFT_TRIM = 0;
const int RIGHT_TRIM = 0;
const int NUDGE_PWM = 55;                 
const int US_NUDGE_PWM = 55;              
const int STEER_STEP = 55;                
const int TURN_SPEED = 240;               
const unsigned long TURN_LEFT_90_MS = 339;   
const unsigned long TURN_RIGHT_90_MS = 345;  
const long SECOND_TURN_TRIM_MS_blue = 40;          
const long SECOND_TURN_TRIM_MS_pink = 5;
const unsigned long TURN_180_MS = 623; 
const unsigned long ONE_GRID_MS_pink = 750;    
const unsigned long ONE_GRID_MS_blue = 720;
const unsigned long CREEP_MS = 0;         
const int CREEP_SPEED = 160;             
const unsigned long BACKUP_MS = 0;       
const unsigned long LINE_IGNORE_MS = 400; 
const int IR_DIP_TRIGGER = 40;           
const float US_NEAR_CM = 7.0f;
const uint8_t IR_CAL_N = 7;
const float IR_CAL_POINTS_CM[IR_CAL_N] = {2, 3, 4, 5, 6, 8, 10};  
int   irCalDip[IR_CAL_N] = {317, 219, 158, 102, 72, 31, 16};
float irCalCm[IR_CAL_N]  = {  2,   3,   4,   5,  6,  8, 10};
uint8_t irCalCount = IR_CAL_N;
float usOffsetCm = 0.0f;                
const uint8_t EEPROM_MAGIC_SIDE = 0xC5;
const int EEPROM_SIDE_ADDR = 40;          // colour table uses addresses 0..36
const float SPACE_FAR_CM = 15.0f;         // anything farther counts as "open"
const float SPACE_TIE_CM = 1.5f;          // closer than this = tie
const bool DEFAULT_180_LEFT = false;      // tie / both sides open: pivot right
const unsigned long CONTROL_MS = 25;
const unsigned long SONAR_MS = 60;
const unsigned long SONAR_TIMEOUT_US = 5000;

MeDCMotor leftMotor(M1), rightMotor(M2);
MePort ultrasonicPort(PORT_1);
MeLineFollower lineFinder(PORT_2);
MeBuzzer buzzer;

bool running = false;
bool abortRequested = false;
bool colourTest = false;
int8_t pendingTest = -1;
bool buttonWasDown = false;
bool irTurnRequested = false, ultrasonicTooClose = false;
unsigned long lastButtonMs = 0, lastControlMs = 0, lastSonarMs = 0, lastTestMs = 0;
unsigned long ignoreLineUntil = 0;
int irBaselineRaw = 0, irOnRaw = 0, irRaw = 0;
int steering = 0;                         // positive = steer right
float ultrasonicCm = -1.0f;

void calibrateSideSpace();                // defined with the side-space functions below
void printSideCal();

void selectOutput(uint8_t y) {
  digitalWrite(DEC_A_PIN, (y & 1) ? HIGH : LOW);
  digitalWrite(DEC_B_PIN, (y & 2) ? HIGH : LOW);
}

int readAvg(uint8_t pin, uint8_t n) {
  analogRead(pin);                        // discard first read after channel switch
  long total = 0;
  for (uint8_t i = 0; i < n; ++i) total += analogRead(pin);
  return total / n;
}

void readColourRaw(int out[3]) {
  const uint8_t order[3] = {Y_RED, Y_GREEN, Y_BLUE};
  for (uint8_t i = 0; i < 3; ++i) {
    selectOutput(order[i]);
    delay(LDR_SETTLE_MS);
    out[i] = readAvg(LDR_PIN, 16);
  }
  selectOutput(IR_OFF_OUTPUT);            // back to the IR "emitter off" state
}

uint8_t classifyColour(const int v[3], long &bestDist) {
  uint8_t best = 0;
  bestDist = 0x7FFFFFFF;
  for (uint8_t c = 0; c < NUM_COLOURS; ++c) {
    long d = 0;
    for (uint8_t k = 0; k < 3; ++k) {
      long diff = (long)v[k] - refs[c][k];
      d += diff * diff;
    }
    if (d < bestDist) { bestDist = d; best = c; }
  }
  return best;
}

void printTable() {
  Serial.println(F("Calibration (R,G,B LDR readings):"));
  for (uint8_t c = 0; c < NUM_COLOURS; ++c) {
    Serial.print(COLOUR_NAMES[c]); Serial.print(F(": "));
    Serial.print(refs[c][0]); Serial.print(F(", "));
    Serial.print(refs[c][1]); Serial.print(F(", "));
    Serial.println(refs[c][2]);
  }
}

void saveRefs() {
  EEPROM.update(0, EEPROM_MAGIC);
  EEPROM.put(1, refs);
}

bool loadRefs() {
  if (EEPROM.read(0) != EEPROM_MAGIC) return false;
  EEPROM.get(1, refs);
  return true;
}

void calibrateColours() {
  Serial.println(F("CALIBRATION: chimney + skirt ON. Place paper, send any key."));
  for (uint8_t c = 0; c < NUM_COLOURS; ++c) {
    Serial.print(F("Place ")); Serial.print(COLOUR_NAMES[c]);
    Serial.println(F(" under the sensor, then send any key..."));
    while (Serial.available()) Serial.read();
    while (!Serial.available()) { }
    delay(100);
    while (Serial.available()) Serial.read();   // flush newline etc.

    long sum[3] = {0, 0, 0};
    for (uint8_t s = 0; s < CAL_SAMPLES; ++s) {
      int v[3];
      readColourRaw(v);
      for (uint8_t k = 0; k < 3; ++k) sum[k] += v[k];
      delay(50);
    }
    for (uint8_t k = 0; k < 3; ++k) refs[c][k] = sum[k] / CAL_SAMPLES;
  }
  saveRefs();
  haveCal = true;
  printTable();
  Serial.println(F("Saved to EEPROM."));
}

void setMotors(int l, int r) {            // signed: positive = forward
  leftMotor.run(LEFT_SIGN * constrain(l, -255, 255));
  rightMotor.run(RIGHT_SIGN * constrain(r, -255, 255));
}

void stopMotors() { setMotors(0, 0); }

// Pivot in place: left = left wheel backward + right wheel forward (mirror for right).
void turnLeftFor(unsigned long ms)  { setMotors(-TURN_SPEED,  TURN_SPEED); delay(ms); stopMotors(); delay(60); }
void turnRightFor(unsigned long ms) { setMotors( TURN_SPEED, -TURN_SPEED); delay(ms); stopMotors(); delay(60); }

void turnLeft90()  { turnLeftFor(TURN_LEFT_90_MS); }
void turnRight90() { turnRightFor(TURN_RIGHT_90_MS); }

void readIR() {
  selectOutput(IR_OFF_OUTPUT);            // emitter off -> ambient baseline
  delay(2);
  irBaselineRaw = readAvg(IR_PIN, 8);
  selectOutput(Y_IR_EMITTER);             // emitter on -> ambient + reflection
  delay(2);
  irOnRaw = readAvg(IR_PIN, 8);
  selectOutput(IR_OFF_OUTPUT);
  irRaw = irBaselineRaw - irOnRaw;        // dip = reflected IR
}

float readUltrasonic() {
  uint8_t pin = ultrasonicPort.pin2();
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);  delayMicroseconds(3);
  digitalWrite(pin, HIGH); delayMicroseconds(10);
  digitalWrite(pin, LOW);
  pinMode(pin, INPUT);
  unsigned long echo = pulseIn(pin, HIGH, SONAR_TIMEOUT_US);
  return echo == 0 ? -1.0f : echo / 58.0f;
}

void driveBetweenWalls() {
  if (ultrasonicTooClose) {
    steering = IR_ON_LEFT ? -US_NUDGE_PWM : US_NUDGE_PWM;   // ultrasonic has priority
  } else {
    int desired = 0;
    if (irTurnRequested) desired = IR_ON_LEFT ? NUDGE_PWM : -NUDGE_PWM;
    steering += constrain(desired - steering, -STEER_STEP, STEER_STEP);
  }
  int l = BASE_SPEED + LEFT_TRIM + steering;
  int r = BASE_SPEED + RIGHT_TRIM - steering;
 
  if (l > 255) { r -= l - 255; l = 255; }
  if (r > 255) { l -= r - 255; r = 255; }
  setMotors(constrain(l, 0, 255), constrain(r, 0, 255));
}

void controlStep() {
  readIR();
  if (millis() - lastSonarMs >= SONAR_MS) {
    lastSonarMs = millis();
    ultrasonicCm = readUltrasonic();
  }
  irTurnRequested = (irRaw > IR_DIP_TRIGGER);
  ultrasonicTooClose = (ultrasonicCm > 0 && ultrasonicCm <= US_NEAR_CM);
  driveBetweenWalls();
}

void stopRobot() {
  running = false;
  abortRequested = true;
  steering = 0;
  stopMotors();
}

void startRobot() {
  if (!haveCal) { Serial.println(F("No calibration saved. Send k first.")); return; }
  if (running) return;
  abortRequested = false;
  steering = 0;
  irTurnRequested = ultrasonicTooClose = false;
  lastControlMs = millis() - CONTROL_MS;
  lastSonarMs = millis() - SONAR_MS;
  ignoreLineUntil = 0;
  running = true;
  Serial.println(F("Running. x/button stops."));
}

void handleInputs() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'x' || c == 'X') { stopRobot(); Serial.println(F("Stopped.")); }
    else if (c == 's' || c == 'S') startRobot();
    else if (c == 'k' || c == 'K') { if (running) Serial.println(F("Robot is running: send x first.")); else calibrateColours(); }
    else if (c == 'm' || c == 'M') { if (running) Serial.println(F("Robot is running: send x first.")); else calibrateSideSpace(); }
    else if (c == 'p' || c == 'P') { printTable(); printSideCal(); }
    else if (c == 't' || c == 'T') { colourTest = !colourTest; Serial.println(colourTest ? F("Colour test ON") : F("Colour test OFF")); }
    else if (c >= '1' && c <= '5') pendingTest = c - '1';
  }
  bool down = analogRead(A7) < 100;       
  if (down && !buttonWasDown && millis() - lastButtonMs > 250) {
    lastButtonMs = millis();
    if (running) { stopRobot(); Serial.println(F("Stopped.")); }
    else startRobot();
  }
  buttonWasDown = down;
}

void driveForMs(unsigned long ms) {       
  unsigned long end = millis() + ms;
  steering = 0;
  lastSonarMs = millis() - SONAR_MS;
  while (!abortRequested && (long)(millis() - end) < 0) {
    handleInputs();
    if (millis() - lastControlMs >= CONTROL_MS) {
      lastControlMs = millis();
      controlStep();
    }
  }
  stopMotors();
  delay(60);
}

void playTune() {
  const int notes[] = {523, 659, 784, 1047, 784, 1047};
  const int durs[]  = {150, 150, 150, 300, 150, 500};
  for (uint8_t i = 0; i < 6; ++i) {
    buzzer.tone(notes[i], durs[i]);
    delay(durs[i] + 40);
  }
  buzzer.noTone();
}

void finishMaze() {
  stopRobot();
  Serial.println(F("END OF MAZE: stopped."));
  playTune();
}

float irDipToCm(float dip) {
  if (dip >= irCalDip[0]) return irCalCm[0];
  for (uint8_t i = 0; i + 1 < irCalCount; ++i) {
    if (dip >= irCalDip[i + 1]) {           // between entry i and i+1: interpolate
      return irCalCm[i] + (irCalDip[i] - dip) * (irCalCm[i + 1] - irCalCm[i])
                          / (float)(irCalDip[i] - irCalDip[i + 1]);
    }
  }
  return SPACE_FAR_CM;                      // weaker than the last entry: open
}

float capFar(float cm) { return (cm < 0 || cm > SPACE_FAR_CM) ? SPACE_FAR_CM : cm; }

float measureUltrasonicCm() {
  float v[3];
  for (uint8_t i = 0; i < 3; ++i) {
    float r = readUltrasonic();
    v[i] = (r < 0) ? 999.0f : r;
    delay(30);
  }
  for (uint8_t i = 0; i < 2; ++i)           // tiny sort
    for (uint8_t j = 0; j < 2 - i; ++j)
      if (v[j] > v[j + 1]) { float t = v[j]; v[j] = v[j + 1]; v[j + 1] = t; }
  return v[1];
}

float measureIrDip() {                      // average of four ambient-corrected dips
  long total = 0;
  for (uint8_t i = 0; i < 4; ++i) { readIR(); total += irRaw; }
  return total / 4.0f;
}

void waitForKey() {
  while (Serial.available()) Serial.read();
  while (!Serial.available()) { }
  delay(100);
  while (Serial.available()) Serial.read();   // flush newline etc.
}

void saveSideCal() {
  EEPROM.update(EEPROM_SIDE_ADDR, EEPROM_MAGIC_SIDE);
  int a = EEPROM_SIDE_ADDR + 1;
  EEPROM.put(a, irCalDip);       a += (int)sizeof(irCalDip);
  EEPROM.put(a, irCalCm);        a += (int)sizeof(irCalCm);
  EEPROM.update(a, irCalCount);  a += 1;
  EEPROM.put(a, usOffsetCm);
}

bool loadSideCal() {
  if (EEPROM.read(EEPROM_SIDE_ADDR) != EEPROM_MAGIC_SIDE) return false;
  int a = EEPROM_SIDE_ADDR + 1;
  EEPROM.get(a, irCalDip);       a += (int)sizeof(irCalDip);
  EEPROM.get(a, irCalCm);        a += (int)sizeof(irCalCm);
  irCalCount = EEPROM.read(a);   a += 1;
  EEPROM.get(a, usOffsetCm);
  if (irCalCount < 2 || irCalCount > IR_CAL_N) irCalCount = 2;   // corrupted: stay safe
  return true;
}

void printSideCal() {
  Serial.println(F("Side-space calibration (IR dip -> cm):"));
  for (uint8_t i = 0; i < irCalCount; ++i) {
    Serial.print(irCalCm[i], 0); Serial.print(F(" cm: dip ")); Serial.println(irCalDip[i]);
  }
  Serial.print(F("Ultrasonic offset: ")); Serial.print(usOffsetCm, 1); Serial.println(F(" cm"));
}


void calibrateSideSpace() {
  Serial.println(F("SIDE-SPACE CALIBRATION. Use a flat maze wall, same lighting as the maze."));
  int newDip[IR_CAL_N];
  for (uint8_t i = 0; i < IR_CAL_N; ++i) {
    Serial.print(F("Hold the wall ")); Serial.print(IR_CAL_POINTS_CM[i], 0);
    Serial.println(F(" cm from the IR sensor face, then send any key..."));
    waitForKey();
    newDip[i] = (int)(measureIrDip() + 0.5f);
    Serial.print(F("  dip = ")); Serial.println(newDip[i]);
  }
  int keptDip[IR_CAL_N];
  float keptCm[IR_CAL_N];
  uint8_t kept = 0;
  for (uint8_t i = 0; i < IR_CAL_N; ++i) {
    if (kept == 0 || newDip[i] < keptDip[kept - 1]) {
      keptDip[kept] = newDip[i];
      keptCm[kept] = IR_CAL_POINTS_CM[i];
      ++kept;
    } else {
      Serial.print(F("Ignored ")); Serial.print(IR_CAL_POINTS_CM[i], 0);
      Serial.print(F(" cm (dip ")); Serial.print(newDip[i]);
      Serial.println(F(" did not drop)."));
    }
  }
  if (kept < 2) {
    Serial.println(F("Too few usable readings. Check IR wiring/aim, then send m again. Nothing saved."));
    return;
  }
  for (uint8_t i = 0; i < kept; ++i) { irCalDip[i] = keptDip[i]; irCalCm[i] = keptCm[i]; }
  irCalCount = kept;

  Serial.println(F("Now place the mBot between two walls with EQUAL clearance each side"));
  Serial.println(F("(both walls within ~8 cm, robot straight), then send any key..."));
  waitForKey();
  float irCm = irDipToCm(measureIrDip());
  float usCm = measureUltrasonicCm();
  if (usCm > 100.0f || irCm >= SPACE_FAR_CM) {
    Serial.println(F("A wall was too far to read. Offset NOT changed; IR table saved."));
  } else {
    usOffsetCm = usCm - irCm;
    Serial.print(F("IR=")); Serial.print(irCm, 1);
    Serial.print(F(" cm, ultrasonic=")); Serial.print(usCm, 1);
    Serial.print(F(" cm -> offset ")); Serial.print(usOffsetCm, 1); Serial.println(F(" cm"));
  }
  saveSideCal();
  printSideCal();
  Serial.println(F("Saved to EEPROM."));
}

bool leftHasMoreSpace() {
  float irCm = capFar(irDipToCm(measureIrDip()));
  float usCm = capFar(measureUltrasonicCm() - usOffsetCm);
  float leftCm  = IR_ON_LEFT ? irCm : usCm;
  float rightCm = IR_ON_LEFT ? usCm : irCm;
  Serial.print(F("Side space: left=")); Serial.print(leftCm, 1);
  Serial.print(F(" cm, right=")); Serial.print(rightCm, 1); Serial.print(F(" cm"));
  bool left;
  if (fabs(leftCm - rightCm) < SPACE_TIE_CM) { left = DEFAULT_180_LEFT; Serial.print(F(" (tie, default)")); }
  else left = leftCm > rightCm;
  Serial.println(left ? F(" -> pivot LEFT") : F(" -> pivot RIGHT"));
  return left;
}

void executeColour(uint8_t colour) {
  switch (colour) {
    case C_RED:    turnLeft90();  break;
    case C_GREEN:  turnRight90(); break;
    case C_ORANGE:                            // 180: pivot toward the side with more space
      if (leftHasMoreSpace()) turnLeftFor(TURN_180_MS + 30);
      else                    turnRightFor(TURN_180_MS - 5);
      break;
    case C_PINK:                              // two successive left turns, two grids
      turnLeft90();
      driveForMs(ONE_GRID_MS_pink);                // no black strip in grid 2: hard-coded
      if (!abortRequested) turnLeftFor(TURN_LEFT_90_MS + SECOND_TURN_TRIM_MS_pink);
     break;
    case C_LBLUE:                             // two successive right turns, two grids
      turnRight90();
      driveForMs(ONE_GRID_MS_blue);
      if (!abortRequested) turnRightFor(TURN_RIGHT_90_MS + SECOND_TURN_TRIM_MS_blue);
      break;
    case C_WHITE:  finishMaze(); break;
  }
  steering = 0;
  lastSonarMs = millis() - SONAR_MS;
  ignoreLineUntil = millis() + LINE_IGNORE_MS;
}

void handleWaypoint() {
  stopMotors();
  if (CREEP_MS > 0) {
    setMotors(CREEP_SPEED, CREEP_SPEED);
    delay(CREEP_MS);
    stopMotors();
  }
  delay(150);                             // let the robot settle before sensing
  int v[3];
  long dist;
  readColourRaw(v);
  uint8_t colour = classifyColour(v, dist);
  if (BACKUP_MS > 0 && colour != C_WHITE) {   // make room to pivot (paper already read)
    setMotors(-CREEP_SPEED, -CREEP_SPEED);
    delay(BACKUP_MS);
    stopMotors();
    delay(80);
  }
  executeColour(colour);
}

bool onBlackStrip() {
  return lineFinder.readSensors() != S1_OUT_S2_OUT;   // either sensor over black
}

void setup() {
  Serial.begin(115200);
  pinMode(DEC_A_PIN, OUTPUT);
  pinMode(DEC_B_PIN, OUTPUT);
  pinMode(LDR_PIN, INPUT);
  pinMode(IR_PIN, INPUT);
  selectOutput(IR_OFF_OUTPUT);
  stopRobot();
  buttonWasDown = analogRead(A7) < 100;
  haveCal = loadRefs();
  bool haveSide = loadSideCal();
  Serial.println(F("Checkpoint 3. k=colours m=side space p=print t=colour test 1-5=test turn s=start x=stop"));
  if (haveCal) printTable();
  else Serial.println(F("No calibration found. Send k."));
  if (haveSide) printSideCal();
  else Serial.println(F("No side-space calibration (using defaults). Send m for the orange turn."));
}

void loop() {
  handleInputs();

  if (pendingTest >= 0 && !running) {     // test a single turn action, no driving
    uint8_t which = pendingTest;
    pendingTest = -1;
    abortRequested = false;
    lastControlMs = millis() - CONTROL_MS;
    Serial.print(F("Test: ")); Serial.println(COLOUR_NAMES[which]);
    executeColour(which);
    stopMotors();
  }

  if (colourTest && !running && haveCal && millis() - lastTestMs >= 500) {
    lastTestMs = millis();
    int v[3];
    long dist;
    readColourRaw(v);
    uint8_t c = classifyColour(v, dist);
    Serial.print(F("RGB=")); Serial.print(v[0]); Serial.print(',');
    Serial.print(v[1]); Serial.print(','); Serial.print(v[2]);
    Serial.print(F(" -> ")); Serial.print(COLOUR_NAMES[c]);
    Serial.print(F("  (dist^2=")); Serial.print(dist); Serial.println(')');
  }

  if (running) {
    if (millis() > ignoreLineUntil && onBlackStrip()) {
      handleWaypoint();
    } else if (millis() - lastControlMs >= CONTROL_MS) {
      lastControlMs = millis();
      controlStep();
    }
  }
}
