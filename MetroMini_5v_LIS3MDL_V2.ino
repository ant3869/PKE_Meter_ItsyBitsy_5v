#include <Wire.h>
#include <Servo.h>
#include <Ramp.h>
#include <TimerOne.h>
#include <Arduino.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPR121.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

#define DEBUG_MODE     // Comment this line to not use debug mode
#define USE_PWM_SERVO  // Comment this line to use the Servo library

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

#define SERVO_MIN_POS 0
#define SERVO_MAX_POS 100
#define SERVO_MIN_PULSE (SERVO_MIN_POS + 750) // Minimum pulse width in microseconds
#define SERVO_MAX_PULSE (SERVO_MAX_POS + 1700) // Maximum pulse width in microseconds

#define SERVO_PIN 11
#define SOUND_POT_PIN 7

//--------------------------------------------------------------- OBJECTS

Adafruit_MPR121 touchSensor = Adafruit_MPR121(); // (A4, A5) (i2c)
SoftwareSerial mySoftwareSerial(12, 13); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
Adafruit_LIS3MDL lis3mdl; // (STEMMA QT / Qwiic) (i2c)
rampUnsignedInt wingRamp;
Servo myServo;

//--------------------------------------------------------------- VARIABLES
// constexpr int BUTTON_PINS[] = {2, 12, A2, A1}; 12, 2, A0, A1, A2
// constexpr int BUTTON_PINS[] = {A3, A2, A1, A0};
constexpr int LED_PINS[] = {6, 5, 4, 3, 2, 1, 0}; 
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
int smoothPotValue();
void ControlVolume();

//--------------------------------------------------------------- SETUP

void setup() {
  delay(1000);
  Serial.begin(9600);
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
  ControlVolume();
  adjustedReading();
  updateIndicators(processedValue);

  String debugMessage = "Baseline: " + String(baseline) + ", Processed value: " + String(processedValue);
  DEBUG_PRINTLN(debugMessage.c_str(), 500);

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
    // DEBUG_PRINTLN(message, 500);

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

void setPerformanceMode(){
 lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);

  switch (lis3mdl.getPerformanceMode()) {
    case LIS3MDL_LOWPOWERMODE: DEBUG_PRINTLN("Performance mode set to: Low", 300); break;
    case LIS3MDL_MEDIUMMODE: DEBUG_PRINTLN("Performance mode set to: Medium", 300); break;
    case LIS3MDL_HIGHMODE: DEBUG_PRINTLN("Performance mode set to: High", 300); break;
    case LIS3MDL_ULTRAHIGHMODE: DEBUG_PRINTLN("Performance mode set to: Ultra-High", 300); break;
  }
}

void setOperationMode(){
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

  // Single shot mode will complete conversion and go into power down
  switch (lis3mdl.getOperationMode()) {
    case LIS3MDL_CONTINUOUSMODE: DEBUG_PRINTLN("Operation mode set to: Continuous", 300); break;
    case LIS3MDL_SINGLEMODE: DEBUG_PRINTLN("Operation mode set to: Single mode", 300); break;
    case LIS3MDL_POWERDOWNMODE: DEBUG_PRINTLN("Operation mode set to: Power-down", 300); break;
  }
}

void setDataRate(){
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);

  switch (lis3mdl.getDataRate()) {
    case LIS3MDL_DATARATE_0_625_HZ: DEBUG_PRINTLN("Data rate set to: 0.625 Hz", 300); break;
    case LIS3MDL_DATARATE_1_25_HZ: DEBUG_PRINTLN("Data rate set to: 1.25 Hz", 300); break;
    case LIS3MDL_DATARATE_2_5_HZ: DEBUG_PRINTLN("Data rate set to: 2.5 Hz", 300); break;
    case LIS3MDL_DATARATE_5_HZ: DEBUG_PRINTLN("Data rate set to: 5 Hz", 300); break;
    case LIS3MDL_DATARATE_10_HZ: DEBUG_PRINTLN("Data rate set to: 10 Hz", 300); break;
    case LIS3MDL_DATARATE_20_HZ: DEBUG_PRINTLN("Data rate set to: 20 Hz", 300); break;
    case LIS3MDL_DATARATE_40_HZ: DEBUG_PRINTLN("Data rate set to: 40 Hz", 300); break;
    case LIS3MDL_DATARATE_80_HZ: DEBUG_PRINTLN("Data rate set to: 80 Hz", 300); break;
    case LIS3MDL_DATARATE_155_HZ: DEBUG_PRINTLN("Data rate set to: 155 Hz", 300); break;
    case LIS3MDL_DATARATE_300_HZ: DEBUG_PRINTLN("Data rate set to: 300 Hz", 300); break;
    case LIS3MDL_DATARATE_560_HZ: DEBUG_PRINTLN("Data rate set to: 560 Hz", 300); break;
    case LIS3MDL_DATARATE_1000_HZ: DEBUG_PRINTLN("Data rate set to: 1000 Hz", 300); break;
  }
}

void setRange(){
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

  switch (lis3mdl.getRange()) {
    case LIS3MDL_RANGE_4_GAUSS: DEBUG_PRINTLN("Range set to: +-4 gauss" , 300); break;
    case LIS3MDL_RANGE_8_GAUSS: DEBUG_PRINTLN("Range set to: +-8 gauss", 300); break;
    case LIS3MDL_RANGE_12_GAUSS: DEBUG_PRINTLN("Range set to: +-12 gauss", 300); break;
    case LIS3MDL_RANGE_16_GAUSS: DEBUG_PRINTLN("Range set to: +-16 gauss", 300); break;
  }
}

void setThreshold(){
  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, true, false, true); // enabled!
}

void setSettings(){
  setPerformanceMode();
  setOperationMode();
  setDataRate();
  setRange();
  setThreshold();
}

void initializeIMU() {
    for (int attempts = 0; attempts < 5; attempts++) {
      if (lis3mdl.begin_I2C()) {
        break;
      } else {
        DEBUG_PRINTLN("LIS3MDL initialization failed. Retrying...", 500);
      }
    }

    if (!imuInitialized) {
      DEBUG_PRINTLN("LIS3MDL initialization failed after multiple attempts. Check connections.", 1000);
    } else {
      DEBUG_PRINTLN("LIS3MDL Found!", 0);
      imuInitialized = true;
      setSettings();
    }
}

float calculateMagnitude() {
  sensors_event_t event; 
  lis3mdl.getEvent(&event);

  float x = event.magnetic.x;
  float y = event.magnetic.y;
  float z = event.magnetic.z;

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
  if (result < 0) { result = 0; }

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
   mySoftwareSerial.begin(9600);

  while (!mySoftwareSerial) {
    DEBUG_PRINTLN("mySoftwareSerial failed to start.", 0);
    delay(1);
  }

  for (int attempts = 0; attempts < 5; attempts++) {
    if (myDFPlayer.begin(mySoftwareSerial)) {
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
  setVolume(50);

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

    myDFPlayer.volume(volume);
    currentVolume = volume;
    delay(100);
  }
}

int smoothPotValue() {
   const int numSamples = 10;
   long total = 0;

   for (int i = 0; i < numSamples; i++) {
      total += analogRead(SOUND_POT_PIN);
      delay(10);
   }

   return total / numSamples;
}


void ControlVolume() {
  int potValue = smoothPotValue(); // Smooth reading
  int volume = map(potValue, 0, 1023, 0, 100); // Map to a range suitable for volume control

  if (volume != currentVolume && dfPlayerInitialized) {
  DEBUG_PRINTLN("Volume level: ", 0);
  DEBUG_PRINTLN(volume, 0);

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

  #ifdef USE_PWM_SERVO

    Timer1.initialize(20000);
    Timer1.pwm(SERVO_PIN, map(90, 0, 180, SERVO_MIN_PULSE, SERVO_MIN_PULSE));

  #else
    myServo.attach(SERVO_PIN);
  #endif

  delay(500);
  parkServo();
  DEBUG_PRINTLN("Servo initialization complete.", 0);
  delay(500);
}

// void writeServoPulse(unsigned int pulseWidth) {
//   unsigned long currentTime = micros();
  
//   if (currentTime - lastPulseTime >= 20000) { // 20 ms period for standard servo signal
//     digitalWrite(SERVO_PIN, HIGH); // Start a new pulse
//     delayMicroseconds(pulseWidth); // Wait for the pulse width duration
//     digitalWrite(SERVO_PIN, LOW);  // End the pulse
//     lastPulseTime = currentTime;
//   }
// }

void moveServoToPosition(unsigned long elapsed) {
  #ifdef USE_PWM_SERVO
  int position = map(elapsed, 0, CALIBRATION_TIME, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  #else
  int position = map(elapsed, 0, CALIBRATION_TIME, SERVO_MIN_POS, SERVO_MAX_POS);
  #endif
  MoveWings(position);
}

void parkServo() {
  static float prevTarget = -1;

  DEBUG_PRINTLN("Moving servo to parked posistion.", 10);

  #ifdef USE_PWM_SERVO
    wingRamp.go(SERVO_MIN_PULSE, 1000, LINEAR, ONCEFORWARD); 
    Timer1.setPwmDuty(SERVO_PIN, wingRamp.update());
    prevTarget = SERVO_MIN_PULSE;
  #else
    wingRamp.go(SERVO_MIN_POS, 1000, LINEAR, ONCEFORWARD); 
    myServo.write(wingRamp.update());
    prevTarget = SERVO_MIN_POS;
  #endif

  DEBUG_PRINTLN("Finished parking servo.", 0);
}

void UpdateWings(float target) {
  static float prevTarget = -1;

  if (target != prevTarget) {
    MoveWings(static_cast<int>(target));
    prevTarget = target;
  }
}

void MoveWings(int target) {
if (!wingRamp.isRunning()) {

  #ifdef USE_PWM_SERVO
      int mappos = static_cast<int>(map(target, EMF_MIN, EMF_MAX, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      int clamped = constrain(mappos, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    #else
      int mappos = static_cast<int>(map(target, EMF_MIN, EMF_MAX, SERVO_MIN_POS, SERVO_MAX_POS));
      int clamped = constrain(mappos, SERVO_MIN_POS, SERVO_MAX_POS);
    #endif

  wingRamp.go(clamped, 1000, LINEAR, ONCEFORWARD);
  } else {
    #ifdef USE_PWM_SERVO
      Timer1.setPwmDuty(SERVO_PIN, wingRamp.update());
    #else
      myServo.write(wingRamp.update());
    #endif  
  }
}

//--------------------------------------------------------------- BUTTONS

enum ButtonAction {
  BUTTON_IDLE,
  BUTTON_PRESSED,
  BUTTON_RELEASED
};

void adjustDataRate() {
  static int rateIndex = 0;
  rateIndex = (rateIndex + 1) % 5; // Cycle through 5 different rates as an example

  switch(rateIndex) {
    case 0: lis3mdl.setDataRate(LIS3MDL_DATARATE_10_HZ); break;
    case 1: lis3mdl.setDataRate(LIS3MDL_DATARATE_20_HZ); break;
    case 2: lis3mdl.setDataRate(LIS3MDL_DATARATE_40_HZ); break;
    case 3: lis3mdl.setDataRate(LIS3MDL_DATARATE_80_HZ); break;
    case 4: lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ); break;
  }
}

void adjustSensitivity() {
  static float sensitivity = 1.0;
  sensitivity += 0.2;
  if (sensitivity > 2.0) sensitivity = 1.0;
}

void checkButton(){
  static int lastButtonState = HIGH;
  // int currentButtonState = digitalRead(];

  // if (currentButtonState != lastButtonState) {
  //   if (currentButtonState == LOW) {
  //     adjustDataRate(); // Change data rate on button press
  //     adjustSensitivity(); // Adjust sensitivity on button press
  //   }
  //   lastButtonState = currentButtonState;
  // }
}
