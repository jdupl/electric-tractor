/*
    Electric tractor AKA Tesla killer

    ALL BTS7960 ctrls Pin 3 (R_EN), 4 (L_EN), 7 (VCC) to Arduino 5V pin
    ALL BTS7960 ctrls Pin 8 (GND) to Arduino GND

    hydraulics: actuators must be extended to RAISE hyd -> HYD_UP = RRPM
    hydraulics: actuators must be retracted to LOWER hyd -> HYD_DOWN = LRPM

    Steering: joystick left gives more power to the right wheel
*/

#include "RoboClaw.h"
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>

#include <OneWire.h>
#include <DallasTemperature.h>

// Define pins for the two DS18B20 sensors
#define M1_TEMP_PIN 50
#define M2_TEMP_PIN 51

// Setup OneWire and DallasTemperature for both sensors
OneWire oneWire1(M1_TEMP_PIN);
DallasTemperature sensorM1(&oneWire1);

OneWire oneWire2(M2_TEMP_PIN);
DallasTemperature sensorM2(&oneWire2);


#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32


//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);


#define DEBUG_EN 0 // set to 1 or 0
#define TICK_MS 2

const int NUM_READINGS = 10;

// Analog pins
#define THROTTLE_PIN  2
#define HYD_PIN  1
#define STEERING_PIN  0

// Digital pins
#define REVERSE_PIN 48

// limit switch wiring
// LOW when untouched, HIGH when touched
// 'NO' pin GND  'C' digital pin pulled up
#define HYD_LIMIT_SWITCH 23 // green INPUT PULLUP

#define FAN_PIN 22

// hydraulics motors M4
#define HYD_RPWM  6 // BTS7960 M3 Pin 1 (RPWM)
#define HYD_LPWM  8 // BTS7960 M3 Pin 2 (LPWM)

#define THROTTLE_MIN_VAL 0 // analog value at idle
#define THROTTLE_MAX_VAL 1023 // analog value at max input

const float FAN_ON_TEMP = 35.0;
const float FAN_OFF_TEMP = 30.0;


#define FORWARD 1
#define REVERSE -1

int g_hydAnalogCenter;
int g_steeringAnalogCenter;


struct SensorData {
  float voltage;
  float totalAmps;
  float ctrlTemp;
  float m1Temp;
  float m2Temp;
  bool isValid;

  bool errorCurrent;
  bool errorTemp;
  bool errorVoltage;

};

struct DriveInfo {
  int pwm;
  float leftFactor;
  float rightFactor;
  int joystickValue;
};



RoboClaw roboclaw = RoboClaw(&Serial3, 10000);

void printTx(String chars) {
  if (DEBUG_EN == 1) {
    Serial.println(chars);
  }
}

bool limitSwitchTouched() {
  return digitalRead(HYD_LIMIT_SWITCH) == HIGH;
}


int convertAnalogThrottleToPWM(int val) {
  printTx("analog throttle val" + String(val));
  if (val <= THROTTLE_MAX_VAL && val > THROTTLE_MIN_VAL) {
    return map(val, THROTTLE_MIN_VAL, THROTTLE_MAX_VAL, 0, 126);
  }
  return 0;
}

void readThrottleInputs(int &userThrottleAnalogVal, int &userDirectionState) {
  int userReverseDigitalVal = digitalRead(REVERSE_PIN);
  userThrottleAnalogVal = analogRead(THROTTLE_PIN);

  if (userReverseDigitalVal == LOW) {
    userDirectionState = FORWARD;
  } else if (userReverseDigitalVal == HIGH){
    userDirectionState = REVERSE;
  }

  printTx("throttle input from user %: " + String(userThrottleAnalogVal));
  printTx("direction input from user:" + String(userDirectionState));
}

void getWheelPowerDistribution(float &rightWheelFactor, float &leftWheelFactor) {
    int val = analogRead(STEERING_PIN);
    printTx("STEERING analog value: " + String(val));

    const int deadZone = 20;  // +/- around center
    const int center = g_steeringAnalogCenter;
    const int maxVal = 1023;

    // Normalize to range [-1.0, 1.0] — Right = positive, Left = negative
    float steerInput;
    if (val < center) {
        // Right turn
        steerInput = float(center - val) / center;
    } else {
        // Left turn
        steerInput = -float(val - center) / (maxVal - center);
    }

    steerInput = constrain(steerInput, -1.0, 1.0);

    if (abs(steerInput) < float(deadZone) / (maxVal - center)) {
        leftWheelFactor = 1.0;
        rightWheelFactor = 1.0;
    } else if (steerInput > 0) {
        // Turning right — reduce right wheel
        float strength = pow(1.0 - steerInput, 1.25);
        rightWheelFactor = strength;
        leftWheelFactor = 1.0;
    } else {
        // Turning left — reduce left wheel
        float strength = pow(1.0 + steerInput, 1.25);
        leftWheelFactor = strength;
        rightWheelFactor = 1.0;
    }

    printTx("Wheel power factors R: " + String(rightWheelFactor, 2) + " L: " + String(leftWheelFactor, 2));
}


void updateMotorsPWM(int scaledPWMRight, int scaledPWMLeft, int direction) {
  if (direction == FORWARD) {
    roboclaw.ForwardM1(0x80, scaledPWMRight);
    roboclaw.ForwardM2(0x80, scaledPWMLeft);
  } else if (direction == REVERSE) {
    roboclaw.BackwardM1(0x80, scaledPWMRight);
    roboclaw.BackwardM2(0x80, scaledPWMLeft);
  } else {
    roboclaw.ForwardM1(0x80, 0);
    roboclaw.ForwardM2(0x80, 0);
  }
}

DriveInfo readThrottleAndUpdateWheelMotors() {
    int userThrottleAnalogVal = 0;
    int userDirectionState = 0;
    float rightWheelFactor = 0.0;
    float leftWheelFactor = 0.0;

    printTx("reading throttle inputs");
    readThrottleInputs(userThrottleAnalogVal, userDirectionState);
    
    printTx("calculating power dist");
    getWheelPowerDistribution(rightWheelFactor, leftWheelFactor);

    int targetPWM = convertAnalogThrottleToPWM(userThrottleAnalogVal);
    int scaledPWMRight = targetPWM * rightWheelFactor;
    int scaledPWMLeft = targetPWM * leftWheelFactor;
    
    printTx("Right pwm: " + String(scaledPWMRight));
    printTx("Left pwm: " + String(scaledPWMLeft));

    printTx("updating motors");
    updateMotorsPWM(scaledPWMRight, scaledPWMLeft, userDirectionState);

    return {targetPWM, leftWheelFactor, rightWheelFactor, analogRead(STEERING_PIN)};
}

void waitForIdleThrottle() {
    int userThrottleAnalogVal = 0;
    int userDirectionState = 0;

    readThrottleInputs(userThrottleAnalogVal, userDirectionState);

    while(userThrottleAnalogVal > 20 || userDirectionState != FORWARD) {
        printTx("waiting for idle before starting");
        readThrottleInputs(userThrottleAnalogVal, userDirectionState);
    }
}

void updateHydraulics() {
  // HYD_DOWN = LRPM
  // HYD_UP = RRPM
  int lpwm = 0;
  int rpwm = 0;

  int val = analogRead(HYD_PIN);
  printTx("HYDRAULICS: " + String(val));
  Serial.println("HYDRAULICS: " + String(val));

   if (val < g_hydAnalogCenter - 250) {
    // down
    lpwm = 255;
   } else if (val > g_hydAnalogCenter + 250) {
        // up
        if (limitSwitchTouched()) {
            printTx("Limit switch touched");
        } else {
            rpwm = 255;
        }
   }
  if (rpwm > 0) {
    printTx("HYD up: " + String(rpwm));
  }
  if (lpwm > 0) {
    printTx("HYD down: " + String(lpwm));  
  }
  
  analogWrite(HYD_LPWM, lpwm);
  analogWrite(HYD_RPWM, rpwm);
}

void calibrateJoysticks() {
  int hydSum = 0;
  int steeringSum = 0;

  // Read analog values and calculate the sum for hydraulic joystick pin
  for (int i = 0; i < NUM_READINGS; i++) {
    hydSum += analogRead(HYD_PIN);
    delay(25);
  }

  g_hydAnalogCenter = hydSum / NUM_READINGS;

  // Read analog values and calculate the sum for steering joystick pin
  for (int i = 0; i < NUM_READINGS; i++) {
    steeringSum += analogRead(STEERING_PIN);
    delay(25);
  }

  g_steeringAnalogCenter = steeringSum / NUM_READINGS;

  printTx("Hydraulic Joystick Center: " + String(g_hydAnalogCenter));
  printTx("Hydraulic Steering Center: " + String(g_steeringAnalogCenter));
}

void setup() {
  Serial.begin(115200);
  delay(100);
  printTx("booting");
  
  printTx("setting wire speed");
  Wire.setClock(5000);
  
  printTx("starting oled");
  u8g2.begin();
  printTx("pin setup");
  loadingScreen("pin setup");
  
  pinMode(HYD_RPWM, OUTPUT);
  analogWrite(HYD_RPWM, 0);

  pinMode(HYD_LPWM, OUTPUT);
  analogWrite(HYD_LPWM, 0);

  pinMode(REVERSE_PIN, INPUT_PULLUP);
  pinMode(HYD_LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(STEERING_PIN, INPUT);
  pinMode(HYD_PIN, INPUT);

  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW); // Start with fan off

  pinMode(M1_TEMP_PIN, INPUT_PULLUP);
  pinMode(M2_TEMP_PIN, INPUT_PULLUP);

  sensorM1.setResolution(9); // 9-bit = 93.75ms
  sensorM2.setResolution(9);

  sensorM1.setWaitForConversion(false);
  sensorM2.setWaitForConversion(false);

  sensorM1.begin();
  sensorM2.begin();

  loadingScreen("roboclaw");
  delay(500);

  printTx("init roboclaw");
  roboclaw.begin(38400);

  printTx("getting roboclaw version");
  char version[48];
  while (!roboclaw.ReadVersion(0x80, version)) {
    delay(500);
    printTx("failed roboclaw version");
  }

  loadingScreen("wait for idle");
  printTx("waiting for idle");
  waitForIdleThrottle();
  
  loadingScreen("calibration");
  printTx("joystick calibration");
  calibrateJoysticks();
  
  loadingScreen("GOBLIN READY");
  
  delay(500);
}

void readMotorsTemps(float &temp1C, float &temp2C) {
  static unsigned long lastRequestTime = 0;
  static float lastM1Temp = 0;
  static float lastM2Temp = 0;
  static bool requested = false;

  unsigned long now = millis();

  temp1C = lastM1Temp;
  temp2C = lastM2Temp;
  if (!requested) {
    sensorM1.requestTemperatures();
    sensorM2.requestTemperatures();
    lastRequestTime = now;
    requested = true;
  } else if (now - lastRequestTime >= 1000) {
    float t1 = sensorM1.getTempCByIndex(0);
    float t2 = sensorM2.getTempCByIndex(0);
    
    if (t1 > -10 || t2 > -10) {
      temp1C = t1;
      temp2C = t2;
      lastM1Temp = temp1C;
      lastM2Temp = temp2C;
    } else if (t1 <= -10) {
      printTx("Error with temp M1");
      temp1C = -1;
      temp2C = t2;
      lastM2Temp = temp2C;
    } else if (t2 <= -10) {
      printTx("Error with temp M2");
      temp1C = t1;
      temp2C = -1;
      lastM1Temp = temp1C;
    }
    unsigned long elapsed = millis() - now;

    requested = false;
  }
}
                                                                                                                                                                                                                                                                              
void loadingScreen(String step) {
  printTx("Loading " + step);
  
  u8g2.clearDisplay();

  u8g2.setFont(u8g2_font_fub11_tr);  // Bigger font
  u8g2.setCursor(0, 0);
  u8g2.print("GOBLIN");

  u8g2.setCursor(0, 20);
  u8g2.print(step);
  
  u8g2.sendBuffer();
}

SensorData updateSensorData() {
  SensorData data;
  data.isValid = true;
  data.errorCurrent = false;
  data.errorTemp = false;
  data.errorVoltage = false;
  data.m1Temp = 0;
  data.m2Temp = 0;

  int16_t current1 = 0;
  int16_t current2 = 0;
  uint16_t ctrlTempRaw = 0;
  uint16_t voltageRaw = 0;
  
  readMotorsTemps(data.m1Temp, data.m2Temp);
  
  if (!roboclaw.ReadCurrents(0x80, current1, current2)) {
    printTx("FAILED to read current");
    data.isValid = false;
    data.errorCurrent = true;
  }

  if (!roboclaw.ReadTemp(0x80, ctrlTempRaw)) {
    printTx("FAILED to read temp");
    data.isValid = false;
    data.errorTemp = true;
  }

  voltageRaw = roboclaw.ReadMainBatteryVoltage(0x80);
  if (voltageRaw == 0 || voltageRaw == 65535) {
    printTx("FAILED to read voltage");
    data.isValid = false;
    data.errorVoltage = true;
  }

  data.totalAmps = (current1 + current2) / 100.0;
  data.ctrlTemp = ctrlTempRaw / 10.0;
  data.voltage = voltageRaw / 10.0;

  return data;
}


void updateFan(const SensorData& data) {
  // Static to preserve fan state between calls
  static bool fanOn = false;

  // Check if any temp exceeds the ON threshold
  bool shouldTurnOn = (data.ctrlTemp > FAN_ON_TEMP) ||
                      (data.m1Temp > FAN_ON_TEMP) ||
                      (data.m2Temp > FAN_ON_TEMP);

  // Check if all temps are below the OFF threshold
  bool shouldTurnOff = (data.ctrlTemp < FAN_OFF_TEMP) &&
                       (data.m1Temp < FAN_OFF_TEMP) &&
                       (data.m2Temp < FAN_OFF_TEMP);

  if (!fanOn && shouldTurnOn) {
    digitalWrite(FAN_PIN, HIGH);
    fanOn = true;
  } else if (fanOn && shouldTurnOff) {
    digitalWrite(FAN_PIN, LOW);
    fanOn = false;
  }
}

void updateDisplay(const SensorData& data, DriveInfo& driveInfo, unsigned long lastTickMs ) {
  printTx("clearing");
  u8g2.clearBuffer();  // Start drawing

  printTx("set font");
  u8g2.setFont(u8g2_font_6x10_tr);  // Use a readable font

  if (!data.isValid) {
    printTx("display error");
    u8g2.setCursor(0, 10);
    u8g2.print("Sensor Error:");

    int y = 25;
    if (data.errorCurrent) {
      u8g2.setCursor(0, y); u8g2.print("- Current Read"); y += 10;
    }
    if (data.errorTemp) {
      u8g2.setCursor(0, y); u8g2.print("- Ctrl Temp Read"); y += 10;
    }
    if (data.errorVoltage) {
      u8g2.setCursor(0, y); u8g2.print("- Voltage Read");
    }

    printTx("displaying error now");
    u8g2.sendBuffer();  // Push to display
    return;
  }

  // ===== Line 1: Voltage and Current =====
  u8g2.setFont(u8g2_font_fub11_tr);  // Bigger font
  u8g2.setCursor(0, 14);
  u8g2.print(data.voltage, 1);
  u8g2.print("|");
  u8g2.print(data.totalAmps, 1);

  // ===== Line 2: Temperatures =====
  u8g2.setCursor(0, 32);
  u8g2.print((int)data.ctrlTemp);
  u8g2.print("/");
  u8g2.print((int)data.m2Temp);
  u8g2.print("/");
  u8g2.print((int)data.m1Temp);
  u8g2.print((char)0xB0);  // Degree symbol

    // ===== Line 3: PWM =====
  u8g2.setFont(u8g2_font_6x10_tr);  // Smaller font
  u8g2.setCursor(0, 50);
  u8g2.print("PWM: ");
  u8g2.print(driveInfo.pwm);
  
  // ===== Optional: Show Power Factors Only if They Differ =====
  if (abs(driveInfo.leftFactor - driveInfo.rightFactor) > 0.01) {
    u8g2.print(" L:");
    u8g2.print(driveInfo.leftFactor, 2);
    u8g2.print(" R:");
    u8g2.print(driveInfo.rightFactor, 2);
  }
  //u8g2.print(driveInfo.joystickValue);

  // ===== Line 4: delay =====
  u8g2.setCursor(0, 60);
  u8g2.print("MS: ");
  u8g2.print(lastTickMs);

  u8g2.sendBuffer();  // Draw everything at once
}


void loop() {
  unsigned long tick_start = millis();
  unsigned static long lastTickMs = 0;

  printTx("starting throttle/motor update");
  DriveInfo driveInfo = readThrottleAndUpdateWheelMotors();

  printTx("hydraulic update");
  updateHydraulics();

  printTx("reading sensor data");
  SensorData sensorData = updateSensorData();

  printTx("update fan");
  updateFan(sensorData);

  printTx("update display");
  updateDisplay(sensorData, driveInfo, lastTickMs);
  printTx("display updated");

  lastTickMs = millis() - tick_start;
  printTx("COMPLETED tick took " + String(lastTickMs));

  delay(TICK_MS);
}