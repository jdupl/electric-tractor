/*
    Electric tractor AKA Tesla killer

    ALL BTS7960 ctrls Pin 3 (R_EN), 4 (L_EN), 7 (VCC) to Arduino 5V pin
    ALL BTS7960 ctrls Pin 8 (GND) to Arduino GND

    hydraulics: actuators must be extended to RAISE hyd -> HYD_UP = RRPM
    hydraulics: actuators must be retracted to LOWER hyd -> HYD_DOWN = LRPM

    Steering: joystick left gives more power to the right wheel
*/

#define DEBUG_EN 0 // set to 1 or 0
#define TICK_MS 20

const int NUM_READINGS = 10;

// Analog pins
#define THROTTLE_PIN  2
#define HYD_PIN  1
#define STEERING_PIN  0

// Digital pins
#define REVERSE_PIN 12

// limit switch wiring
// LOW when untouched, HIGH when touched
// 'NO' pin GND  'C' digital pin pulled up
#define HYD_LIMIT_SWITCH 13 // green INPUT PULLUP

// Left wheel motors M1
#define WL_RPWM  3 // BTS7960 M1 Pin 1 (RPWM)
#define WL_LPWM  5 // BTS7960 M1 Pin 2 (LPWM)

// Right wheel motors M2
#define WR_RPWM  10 // BTS7960 M2 Pin 1 (RPWM)
#define WR_LPWM  11 // BTS7960 M2 Pin 2 (LPWM)

// hydraulics motors M3
#define HYD_RPWM  6 // BTS7960 M3 Pin 1 (RPWM)
#define HYD_LPWM  9 // BTS7960 M3 Pin 2 (LPWM)


// min and max PWM delta whitin a tick
#define MAX_PWM_CHANGE_PER_TICK_ACCEL 128
#define MAX_PWM_CHANGE_PER_TICK_DECCEL 80
#define MAX_PWM 255


#define THROTTLE_MIN_VAL 0 // analog value at idle
#define THROTTLE_MAX_VAL 1024 // analog value at max input


#define FORWARD 1
#define REVERSE -1


int g_previousPWMRight = 0;
int g_previousPWMLeft = 0;
int g_lastThrottlePWM = 0;
int g_lastDirectionState = 0;
int g_hydAnalogCenter;
int g_steeringAnalogCenter;


void printTx(String chars) {
  if (DEBUG_EN == 1) {
    Serial.println(chars);
  }
}

bool limitSwitchTouched() {
  return digitalRead(HYD_LIMIT_SWITCH) == HIGH;
}

int smoothAcceleration(int targetPWM, int previousPWM) {
    int scaledPWM = 0;

    // no change in PWM
    if (previousPWM == targetPWM) {
      return targetPWM;
    }

    int pwmDelta = targetPWM - previousPWM;

    if (pwmDelta > 0 && pwmDelta > MAX_PWM_CHANGE_PER_TICK_ACCEL) {
      // acceleration too fast
      printTx("Slowing down acceleration...");
      scaledPWM = previousPWM + MAX_PWM_CHANGE_PER_TICK_ACCEL;

    } else if (pwmDelta < 0 && abs(pwmDelta) > MAX_PWM_CHANGE_PER_TICK_DECCEL) {
      // deacceleration too fast
      printTx("Slowing down deacceleration...");
      scaledPWM = previousPWM - MAX_PWM_CHANGE_PER_TICK_DECCEL;

    } else {
      // pwm change is legit
      scaledPWM = targetPWM;
    }
    return scaledPWM;
}

int convertAnalogThrottleToPercent(int val) {
  printTx("analog throttle val" + String(val));
  if (val < THROTTLE_MAX_VAL && val > THROTTLE_MIN_VAL) {
    return map(val, THROTTLE_MIN_VAL, THROTTLE_MAX_VAL, 0, 100);
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

int convertPercentToPWM(int percent) {
  int pwm = int(percent * MAX_PWM / 100);

  if (pwm > MAX_PWM) {
    printTx("FATAL ERROR: Wheel pwm exceeds max pwm set: " + String(pwm));
    pwm = 0;
  }

  printTx("Wheel pwm: " + String(pwm));

  return pwm;
}

boolean hasDirectionConflict(int userDirectionState) {
  // avoid switching from FORWARD to REVERSE when motors are moving
  return g_lastDirectionState != userDirectionState \
      && g_lastThrottlePWM != 0;
}


void updateMotorsPWM(int scaledPWMRight, int scaledPWMLeft, int direction) {
    if (direction == REVERSE) {
        analogWrite(WL_RPWM, scaledPWMLeft);
        analogWrite(WL_LPWM, 0);
        analogWrite(WR_RPWM, scaledPWMRight);
        analogWrite(WR_LPWM, 0);
    } else if (direction == FORWARD) {
        analogWrite(WL_RPWM, 0);
        analogWrite(WL_LPWM, scaledPWMLeft);
        analogWrite(WR_RPWM, 0);
        analogWrite(WR_LPWM, scaledPWMRight);
    }
}

int doDirectionConflictHandling(int userDirectionState) {
  if (hasDirectionConflict(userDirectionState)) {
    printTx("WARNING: refusing to change direction state. Motors not halted");
    return g_lastDirectionState;
  } else {
    return userDirectionState;
  }
}

void getWheelPowerDistribution(float &rightWheelFactor, float &leftWheelFactor) {
    int val = analogRead(STEERING_PIN);

    printTx("STEERING analog value: " + String(val));

    int startSteerLeft = g_steeringAnalogCenter + 50;
    int startSteerRight = g_steeringAnalogCenter - 50;

    if (val >= startSteerLeft) {
        int percent = map(val, startSteerLeft, 1023, 100, 0);
        leftWheelFactor = percent / 100.0;
        rightWheelFactor = 1;
    } else if (val <= startSteerRight) {
        leftWheelFactor = 1;
        int percent = map(val, 0, startSteerRight, 0, 100);
        rightWheelFactor = percent / 100.0;
    } else {
        leftWheelFactor = 1;
        rightWheelFactor = 1;
    }
    printTx("Wheel power factors R: " + String(rightWheelFactor) + " L: " + String(leftWheelFactor));
}

void readThrottleAndUpdateWheelMotors() {
    int userThrottleAnalogVal = 0;
    int userDirectionState = 0;
    float rightWheelFactor = 0.0;
    float leftWheelFactor = 0.0;

    readThrottleInputs(userThrottleAnalogVal, userDirectionState);
    getWheelPowerDistribution(rightWheelFactor, leftWheelFactor);

    int throttlePercent = convertAnalogThrottleToPercent(userThrottleAnalogVal);
    int targetPWM = convertPercentToPWM(throttlePercent);

    int scaledPWMRight = smoothAcceleration(targetPWM * rightWheelFactor, g_previousPWMRight);
    int scaledPWMLeft = smoothAcceleration(targetPWM * leftWheelFactor, g_previousPWMLeft);

    int protectedDirectionState = doDirectionConflictHandling(userDirectionState);

    g_previousPWMRight = scaledPWMRight;
    g_previousPWMLeft = scaledPWMLeft;
    g_lastDirectionState = protectedDirectionState;

    updateMotorsPWM(scaledPWMRight, scaledPWMLeft, protectedDirectionState);
}

void updateHydraulics() {
  // HYD_DOWN = LRPM
  // HYD_UP = RRPM
  int lpwm = 0;
  int rpwm = 0;

  int val = analogRead(HYD_PIN);
  printTx("HYDRAULICS: " + String(val));

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

  printTx("HYD up: " + String(lpwm));
  printTx("HYD down: " + String(rpwm));

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
  Serial.begin(9600);

  pinMode(WL_RPWM, OUTPUT);
  analogWrite(WL_RPWM, 0);

  pinMode(WL_LPWM, OUTPUT);
  analogWrite(WL_LPWM, 0);

  pinMode(WR_RPWM, OUTPUT);
  analogWrite(WR_RPWM, 0);

  pinMode(WR_LPWM, OUTPUT);
  analogWrite(WR_LPWM, 0);

  pinMode(HYD_RPWM, OUTPUT);
  analogWrite(HYD_RPWM, 0);

  pinMode(HYD_LPWM, OUTPUT);
  analogWrite(HYD_LPWM, 0);

  pinMode(REVERSE_PIN, INPUT_PULLUP);
  pinMode(HYD_LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(STEERING_PIN, INPUT);
  pinMode(HYD_PIN, INPUT);

  g_lastDirectionState = FORWARD;
  g_lastThrottlePWM = 0;

  calibrateJoysticks();
}

void loop() {

  readThrottleAndUpdateWheelMotors();
  updateHydraulics();

  delay(TICK_MS);
}
