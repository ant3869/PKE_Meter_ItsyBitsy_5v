#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <Ramp.h>
#include <QMC5883LCompass.h>
#include <DFRobotDFPlayerMini.h>

//#define DEBUG_MODE

#ifdef DEBUG_MODE
#define DEBUG_PRINTLN(x, d) \
  { \
    Serial.println(x); \
    delay(d); \
  }
#else
#define DEBUG_PRINTLN(x, d)
#endif

//--------------------------------------------------------------- DEFINITIONS

#define EMF_MIN 0
#define EMF_MAX 1100
#define NUM_READINGS 10
#define CALIBRATION_TIME 13000
#define NUM_LEDS 7
#define LED_SLOW 1000
#define LED_FAST 30
#define SERVO_PIN 4
// #define SERVO_MIN_POS 0
// #define SERVO_MAX_POS 120
#define SERVO_MIN_POS 750
#define SERVO_MAX_POS 1735
#define SERVO_MIN_PULSE 750 // Minimum pulse width in microseconds
#define SERVO_MAX_PULSE 1735 // Maximum pulse width in microseconds

//--------------------------------------------------------------- OBJECTS

Servo myServo;
rampUnsignedInt wingRamp;
DFRobotDFPlayerMini myDFPlayer;
QMC5883LCompass compass;

//--------------------------------------------------------------- VARIABLES

constexpr int LED_PINS[] = { 13, 12, 11, 10, 9, 7, 5 };
unsigned long ledInterval = (CALIBRATION_TIME / NUM_LEDS);
int currentLED = 0;

bool imuInitialized = false;
bool dfPlayerInitialized = false;

float EMF_MIN_uT = 0.0;
float EMF_MAX_uT = 1100.0;
float readings[NUM_READINGS];
unsigned long lastMagReadTime = 0;
float lastMagData = 0.0;
unsigned long magReadInterval = 500;
float changeThreshold = 5.0;
float processedValue = 0.0;
int readIndex = 0;
float total = 0.0;
int lastRangeIndex = 0;
float baseline = 0.0;
float sensitivity = 1.2;

int currentVolume = 0;
float prevTargetPos = 0.0;
float prevTarget = 0.0;
unsigned long lastPulseTime = 0; // Last pulse time
int pulseWidth = 750; // Pulse width in microseconds (1500 us is typically the center position for servos)

const unsigned int parkPos = 750;  // servo position when parked
const unsigned int midPos = 1125;  // servo position when buttonA pressed and held down
const unsigned int topPos = 1735;  // servo position when buttonB pressed and held down

//--------------------------------------------------------------- FUNCTIONS

void calibrate_baseline();
void playCalibrationSFX();
void playBeepSFX();
float calculateMagnitude();
void moveServoToPosition(unsigned long elapsed);
bool shouldLightLED(unsigned long elapsed);
void lightCurrentLED(int ledIndex);
void finalizeCalibration();
void calculateAndSetBaseline(float totalReadings, int count);
void initializeIMU();
float calculateSensitivity(float val);
float process_mag_data();
float updateEMFReading(float newEMF);
void adjustedReading();
void initializeLeds();
void controlLedsDynamic(float reading);
void initializeDFPlayer();
void setVolume(int volume);
void PlayTrack(int TrackToPlay);
void soundControl(float value);
void initializeServo();
void writeServoPulse(unsigned int pulseWidth);
void parkServo();
void MoveWings(int val);
void UpdateWings(float moveval);
void lightAllLEDs();
void nixAllLEDs();

//--------------------------------------------------------------- SETUP

void setup() {
  delay(2000);
  Serial.begin(115200);
  while (!Serial) { delay(1); }
  DEBUG_PRINTLN("Debug mode enabled.", 1000);

  Wire.begin();
  DEBUG_PRINTLN("I2C initialized.", 0);

  initializeIMU();
  initializeLeds();
  initializeServo();
  initializeDFPlayer();
  calibrate_baseline();

  DEBUG_PRINTLN("Intial setup complete.", 2000);
}

//--------------------------------------------------------------- MAIN

void loop() {
  adjustedReading();
  updateIndicators(processedValue);

  // String debugMessage = "Baseline: " + String(baseline) + ", Processed value: " + String(processedValue);
  // DEBUG_PRINTLN(debugMessage.c_str(), 200);

  delay(30);
}

void updateIndicators(float data) {
  controlLedsDynamic(data);
  soundControl(data);
  UpdateWings(data);
}

//--------------------------------------------------------------- CALIBRATION

void calibrate_baseline() {
  unsigned long start_time = millis();
  float totalReadings = 0.0;
  int readingCount = 0;

  DEBUG_PRINTLN("Starting calibration sequence.", 1000);
  playCalibrationSFX();

  while (millis() - start_time < CALIBRATION_TIME) {
    totalReadings += calculateMagnitude();
    readingCount++;

    // String message = "totalReadings: " + String(totalReadings) + ", readingCount: " + String(readingCount);
    // DEBUG_PRINTLN(message, 0);

    unsigned long elapsed = millis() - start_time;

    if (shouldLightLED(elapsed)) {
      lightCurrentLED(currentLED);
    }

    moveServoToPosition(elapsed);
    delay(30);
  }

  calculateAndSetBaseline(totalReadings, readingCount);
  finalizeCalibration();

  DEBUG_PRINTLN("Calibration sequence complete.", 1500);
}

// void calculateAndSetBaseline(float totalReadings, int count) {
//   if (count > 0) {
//     baseline = totalReadings / count;
//   } else {
//     baseline = 0.0f;
//     DEBUG_PRINTLN("No valid magnetometer readings for baseline calculation.", 2000);
//   }
// }

void calculateAndSetBaseline(float totalReadings, int count) {
  baseline = count > 0 ? totalReadings / count : 0.0;
}

void finalizeCalibration() {
  lightAllLEDs();
  parkServo();
  nixAllLEDs();
  playBeepSFX();
  soundControl(250.0);
}

//--------------------------------------------------------------- IMU

void initializeIMU() {
  compass.init();
  delay(1000);
  
  bool validData = false;
  for (int i = 0; i < 5; i++) {
    compass.read();
    int x = compass.getX();
    int y = compass.getY();
    int z = compass.getZ();

    String message = "Readings - x: " + String(x) + ", y: " + String(y) + ", z: " + String(z);
    DEBUG_PRINTLN(message, 0);
    
    if (x != 0 && y != 0 && z != 0) {
      validData = true;
      break;
    }

    delay(200);
  }

  if (!validData) {
    DEBUG_PRINTLN("QMC5883L not detected. Check connections.", 1000);
    imuInitialized = false;
  } else { 
    imuInitialized = true;
    compass.setSmoothing(10,true);
    DEBUG_PRINTLN("QMC5883L initialized successfully.", 200);
  }
}

float calculateMagnitude() {
  compass.read();
  int x = compass.getX();
  int y = compass.getY();
  int z = compass.getZ();

  if (x == 0 && y == 0 && z == 0) {
    return 0;
  }

  long xs = (long)x * x;
  long ys = (long)y * y;
  long zs = (long)z * z;
  long sumSquares = xs + ys + zs;

  return sqrt(sumSquares);
}

float calculateSensitivity(float val) {
   if (val > 0) {
    return pow(val, sensitivity);
  } 

  return 0;
}

float process_mag_data() {
  float emf_strength = calculateMagnitude() - baseline;
  float emf_sensitity = calculateSensitivity(emf_strength);
  float mapped_emf = map(emf_sensitity, EMF_MIN_uT, EMF_MAX_uT, EMF_MIN, EMF_MAX);
  float result = constrain(mapped_emf, EMF_MIN_uT, EMF_MAX_uT);

  if (result < 15.0 || result < 0) { result = 0.0; }

  return result;
}

float updateEMFReading(float newEMF) {
  total -= readings[readIndex];
  readings[readIndex] = newEMF;
  total += readings[readIndex];
  readIndex = (readIndex + 1) % NUM_READINGS;

  return total / NUM_READINGS;
}

void adjustedReading() {
  unsigned long currentTime = millis();

  if (currentTime - lastMagReadTime >= magReadInterval) {
    float val = process_mag_data();
    processedValue = updateEMFReading(val);

    if (abs(processedValue - lastMagData) > changeThreshold) {
      magReadInterval = 250;
    } else {
      magReadInterval = 500;
    }

    lastMagReadTime = currentTime;
    lastMagData = processedValue;
  }
}

//--------------------------------------------------------------- LEDS

void initializeLeds() {
  for (int i = 0; i < NUM_LEDS; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
  }
}

void lightAllLEDs() {
  for (int i = 0; i < NUM_LEDS; i++) {
    digitalWrite(LED_PINS[i], HIGH);
  }

  delay(1000);
}

void nixAllLEDs() {
  for (int i = 0; i < NUM_LEDS; i++) {
    digitalWrite(LED_PINS[i], LOW);  // Turn on each LED
  }

  delay(1000);
}

bool shouldLightLED(unsigned long elapsed) {
  if (elapsed > currentLED * ledInterval && currentLED < NUM_LEDS) {
    currentLED++;
    return true;
  }

  return false;
}

void lightCurrentLED(int ledIndex) {
  digitalWrite(LED_PINS[ledIndex], HIGH);
}

void controlLedsDynamic(float reading) {
  float ledSpeed = map(reading, EMF_MIN_uT, EMF_MAX_uT, LED_SLOW, LED_FAST);
  float interval = constrain(ledSpeed, LED_FAST, LED_SLOW);

  static uint32_t previousMillis = 0;
  unsigned long currentMillis = millis();
  static byte currentLed = 0;

  if (currentMillis - previousMillis >= interval) {
    digitalWrite(LED_PINS[currentLed], LOW);
    currentLed = (currentLed + 1) % NUM_LEDS;
    digitalWrite(LED_PINS[currentLed], HIGH);
    previousMillis = millis();
  }
}

//--------------------------------------------------------------- SOUND

void initializeDFPlayer() {
  Serial1.begin(9600);

  while (!Serial1) {
    DEBUG_PRINTLN("Serial1 failed to start.", 0);
    delay(1);
  }

  for (int attempts = 0; attempts < 5; attempts++) {
    if (myDFPlayer.begin(Serial1)) {
      dfPlayerInitialized = true;
      DEBUG_PRINTLN("DFPlayer initialized successfully.", 200);
      playBeepSFX();

      break;
    } else {
      DEBUG_PRINTLN("DFPlayer initialization failed. Retrying...", 500);
    }
  }

  if (!dfPlayerInitialized) {
    DEBUG_PRINTLN("DFPlayer initialization failed after multiple attempts. Check connections.", 500);
  }
}

void playCalibrationSFX() {
  setVolume(50);

  if (dfPlayerInitialized) {
    myDFPlayer.playFolder(1, 4);
    delay(2000);
  }
}

void playBeepSFX() {
  setVolume(100);

  if (dfPlayerInitialized) {
    myDFPlayer.playFolder(1, 1);
    delay(500);
  }
}

void setVolume(int volume) {
  if (dfPlayerInitialized) {
    #ifdef DEBUG_MODE
    volume = 10;  // Set volume to 10 if DEBUG_MODE is enabled
    #endif

    if (volume <= 2) {
      volume = 5;
    }

    myDFPlayer.volume(volume);
    currentVolume = volume;
    delay(100);
  }
}

void PlayTrack(int TrackToPlay) {
  static int TrackPlaying = -1;

  if (TrackPlaying != TrackToPlay) {
    myDFPlayer.stop();
    myDFPlayer.playFolder(2, TrackToPlay);
    TrackPlaying = TrackToPlay;
  }
}

void soundControl(float value) {
  int currentRangeIndex = 0;
  int emfValue = static_cast<int>(value);

  if (emfValue <= 466) 
    currentRangeIndex = 1;
  else if (emfValue <= 933)
    currentRangeIndex = 2;
  else
    currentRangeIndex = 3;

  if (currentRangeIndex != lastRangeIndex && dfPlayerInitialized) {
    PlayTrack(currentRangeIndex);
    lastRangeIndex = currentRangeIndex;
  }
}

//--------------------------------------------------------------- SERVO

void initializeServo() {
  pinMode(SERVO_PIN, OUTPUT);
  myServo.attach(SERVO_PIN);
  delay(500);
  parkServo();
  DEBUG_PRINTLN("Servo initialization complete.", 500);
}

void writeServoPulse(unsigned int pulseWidth) {
  unsigned long currentTime = micros();
  
  if (currentTime - lastPulseTime >= 20000) { // 20 ms period for standard servo signal
    digitalWrite(SERVO_PIN, HIGH); // Start a new pulse
    delayMicroseconds(pulseWidth); // Wait for the pulse width duration
    digitalWrite(SERVO_PIN, LOW); // End the pulse
    lastPulseTime = currentTime;
  }
}

void moveServoToPosition(unsigned long elapsed) {
  float position = map(elapsed, 0, CALIBRATION_TIME, SERVO_MIN_POS, SERVO_MAX_POS);
  MoveWings(position);
}

void parkServo() {
  static float prevTarget = -1;

  if (myServo.attached()) {
    DEBUG_PRINTLN("Moving servo to parked posistion.", 10);

    wingRamp.go(SERVO_MIN_POS, 1000, LINEAR, ONCEFORWARD); 
    myServo.write(wingRamp.update());
    prevTarget = SERVO_MIN_POS;

    DEBUG_PRINTLN("Finished parking servo.", 0);
  }
}

void UpdateWings(float target) {
  static float prevTarget = -1;

  if (target != prevTarget) {
    int mappos = static_cast<int>(map(target, EMF_MIN_uT, EMF_MAX_uT, SERVO_MIN_POS, SERVO_MAX_POS));
    MoveWings(mappos);
    prevTarget = target;
  }
}

void MoveWings(int moveval) {
  if (!wingRamp.isRunning()) {
    int clamped = constrain(moveval, SERVO_MIN_POS, SERVO_MAX_POS);

    if (clamped <= SERVO_MIN_POS)
      clamped = SERVO_MIN_POS;
    else if (clamped >= SERVO_MAX_POS)
      clamped = SERVO_MAX_POS;

    wingRamp.go(clamped, 1000, LINEAR, ONCEFORWARD);
  }

  if (wingRamp.isRunning()) {
    myServo.write(wingRamp.update());
  }
}