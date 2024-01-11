/*
// Electric tractor AKA Tesla killer
//
// ALL BTS7960 ctrls Pin 3 (R_EN), 4 (L_EN), 7 (VCC) to Arduino 5V pin
// ALL BTS7960 ctrls Pin 8 (GND) to Arduino GND

// actuator notes
// to extend: left RPM
// to retract: right RPM

// hydraulics: actuators must be extended to RAISE hyd -> HYD_UP = LRPM
// hydraulics: actuators must be retracted to LOWER hyd -> HYD_DOWN = RRPM

// Steering: actuator must be extended to steer LEFT -> STEER_LEFT = LRPM
// Steering: actuator must be retracrted to steer RIGHT  b -> STEER_RIGHT = RRPM
*/

#define DEBUG_EN 0 // set to 1 or 0
#define TICK_MS 50

const int NUM_READINGS = 10;

// Analog pins
#define THROTTLE_PIN  2
#define HYD_PIN  1
#define STEERING_PIN  0

// Digital pins
#define SEL_PIN 12

// Wheel motors M1/M2
#define W_RPWM  3 // BTS7960 M1 M2 Pin 1 (RPWM)
#define W_LPWM  5 // BTS7960 M1 M2 Pin 2 (LPWM)

// hydraulics motors M4
#define HYD_RPWM  6 // BTS7960 M4 Pin 1 (RPWM)
#define HYD_LPWM  9 // BTS7960 M4 Pin 2 (LPWM)

// Steering motor M3
#define STEER_RPWM  10 // BTS7960 M3 Pin 1 (RPWM)
#define STEER_LPWM  11 // BTS7960 M3 Pin 2 (LPWM)

// min and max PWM delta whitin a tick
#define MAX_PWM_CHANGE_PER_TICK_ACCEL 128
#define MAX_PWM_CHANGE_PER_TICK_DECCEL 80
#define MAX_PWM 255


#define THROTTLE_MIN_VAL 200 // analog value at idle
#define THROTTLE_MAX_VAL 890 // analog value at max input


#define FORWARD 1
#define REVERSE -1


int g_lastThrottlePWM = 0;
int g_lastDirectionState = 0;
int g_hydAnalogCenter;
int g_steeringAnalogCenter;


void printTx(String chars) {
  if (DEBUG_EN == 1) {
    Serial.println(chars);
  }
}

int smoothAcceleration(int targetPWM) {
    int scaledPWM = 0;

    // no change in PWM
    if (g_lastThrottlePWM == targetPWM) {
      return targetPWM;
    }

    int pwmDelta = targetPWM - g_lastThrottlePWM;

    if (pwmDelta > 0 && pwmDelta > MAX_PWM_CHANGE_PER_TICK_ACCEL) {
      // acceleration too fast
      printTx("Slowing down acceleration...");
      scaledPWM = g_lastThrottlePWM + MAX_PWM_CHANGE_PER_TICK_ACCEL;

    } else if (pwmDelta < 0 && abs(pwmDelta) > MAX_PWM_CHANGE_PER_TICK_DECCEL) {
      // deacceleration too fast
      printTx("Slowing down deacceleration...");
      scaledPWM = g_lastThrottlePWM - MAX_PWM_CHANGE_PER_TICK_DECCEL;

    } else {
      // pwm change is legit
      scaledPWM = targetPWM;
    }
    return scaledPWM;
}

int convertAnalogThrottleToPercent(int val) {
  if (val < THROTTLE_MAX_VAL && val > THROTTLE_MIN_VAL) {
    return map(val, THROTTLE_MIN_VAL, THROTTLE_MAX_VAL, 0, 100);
  }
  return 0;
}

void readThrottleInputs(int &userThrottleAnalogVal, int &userDirectionState) {
  int userReverseDigitalVal = digitalRead(SEL_PIN);
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

void saveCurrentThrottleInfos(int scaledPWM, int protectedDirectionState) {
  g_lastThrottlePWM = scaledPWM;
  g_lastDirectionState = protectedDirectionState;
}

void doThrottleUpdate(int targetPWM, int protectedDirectionState) {
  int scaledPWM = smoothAcceleration(targetPWM);

  // Write the PWM value to the respective pins
  if (protectedDirectionState == FORWARD) {
    analogWrite(W_LPWM, scaledPWM);
    analogWrite(W_RPWM, 0);
  } else if (protectedDirectionState == REVERSE) {
    analogWrite(W_LPWM, 0);
    analogWrite(W_RPWM, scaledPWM);
  }

  saveCurrentThrottleInfos(scaledPWM, protectedDirectionState);
}

int doDirectionConflictHandling(int userDirectionState) {
  if (hasDirectionConflict(userDirectionState)) {
    printTx("WARNING: refusing to change direction state. Motors not halted");
    return g_lastDirectionState;
  } else {
    return userDirectionState;
  }
}

void updateThrottle() {
  int userThrottleAnalogVal = 0;
  int userDirectionState = 0;

  readThrottleInputs(userThrottleAnalogVal, userDirectionState);

  int throttlePercent = convertAnalogThrottleToPercent(userThrottleAnalogVal);

  doThrottleUpdate(convertPercentToPWM(throttlePercent),
                   doDirectionConflictHandling(userDirectionState));

}

void updateHydraulics() {
  // HYD_DOWN = RRPM
  // HYD_UP = LRPM
  int lpwm = 0;
  int rpwm = 0;

  int val = analogRead(HYD_PIN);
  printTx("HYDRAULICS: " + String(val));

   if (val > g_hydAnalogCenter + 50) {
    // down
    rpwm = 255;
   } else if (val < g_hydAnalogCenter - 50) {
    // up
    lpwm = 255;
   }

  printTx("HYD up: " + String(lpwm));
  printTx("HYD down: " + String(rpwm));

  analogWrite(HYD_LPWM, lpwm);
  analogWrite(HYD_RPWM, rpwm);
}

void updateSteering() {
  // STEER_LEFT = LRPM
  // STEER_RIGHT = RRPM
  int lpwm = 0;
  int rpwm = 0;
  int val = analogRead(STEERING_PIN);

  printTx("STEERING analog value: " + String(val));

  if (val > 520) {
    lpwm = 255;
  } else if (val < 490) {
    rpwm = 255;
  }

  printTx("STEER Left pwm: " + String(lpwm));
  printTx("STEER right pwm: " + String(rpwm));

  analogWrite(STEER_LPWM, lpwm);
  analogWrite(STEER_RPWM, rpwm);
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

  pinMode(W_RPWM, OUTPUT);
  analogWrite(W_RPWM, 0);

  pinMode(W_LPWM, OUTPUT);
  analogWrite(W_LPWM, 0);

  pinMode(HYD_RPWM, OUTPUT);
  analogWrite(HYD_RPWM, 0);

  pinMode(HYD_LPWM, OUTPUT);
  analogWrite(HYD_LPWM, 0);

  pinMode(STEER_RPWM, OUTPUT);
  analogWrite(STEER_RPWM, 0);

  pinMode(STEER_LPWM, OUTPUT);
  analogWrite(STEER_LPWM, 0);

  pinMode(SEL_PIN, INPUT_PULLUP);
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(STEERING_PIN, INPUT);
  pinMode(HYD_PIN, INPUT);

  g_lastDirectionState = FORWARD;
  g_lastThrottlePWM = 0;

  calibrateJoysticks();
}

void loop() {

  updateThrottle();
  updateSteering();
  updateHydraulics();

  delay(TICK_MS);
}
