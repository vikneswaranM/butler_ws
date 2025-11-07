// ===== 4-Wheel Skid-Steer Drive (NO ENCODERS) =====
// Serial 57600. Commands from ROS2 bridge:
//   VEL,<linear_x>,<angular_z>\n   e.g. VEL,0.25,0.00
//   STOP\n
//
// linear_x, angular_z are already clamped by ROS; here we map to PWM.

// -------- L298N pin map (INx) --------
  #define M1_IN1 2
  #define M1_IN2 3
  #define M2_IN3 4
  #define M2_IN4 5
  #define M3_IN1 6
  #define M3_IN2 7
  #define M4_IN3 8
  #define M4_IN4 9

// -------- OPTIONAL: enable pins --------
// If your L298N boards have ENA/ENB jumpers installed, you can leave USE_ENABLE_PINS 0.
// If jumpers are missing, set to 1 and wire these pins to ENA/ENB on your drivers.
#define USE_ENABLE_PINS 0
#if USE_ENABLE_PINS
  #define EN_LEFT  10   // ENA for left driver (M1+M3)
  #define EN_RIGHT 11   // ENB for right driver (M2+M4)
#endif

// -------- Safety / behavior --------
const unsigned long CMD_TIMEOUT_MS = 300;   // watchdog: stop if no command
const float LOCAL_MAX_ABS = 1.0f;           // local clamp (ROS already clamps)
const uint8_t MIN_PWM = 60;                 // overcome deadband (tune 40..90)

// Status LED
#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

float linear_x = 0.0f, angular_z = 0.0f;
unsigned long lastCmdMs = 0;
bool have_valid_cmd = false;

void setup() {
  Serial.begin(57600);

  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN3, OUTPUT); pinMode(M2_IN4, OUTPUT);
  pinMode(M3_IN1, OUTPUT); pinMode(M3_IN2, OUTPUT);
  pinMode(M4_IN3, OUTPUT); pinMode(M4_IN4, OUTPUT);

#if USE_ENABLE_PINS
  pinMode(EN_LEFT, OUTPUT);
  pinMode(EN_RIGHT, OUTPUT);
  analogWrite(EN_LEFT, 255);   // fully enable
  analogWrite(EN_RIGHT, 255);
#endif

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  stopAllMotors();
  lastCmdMs = millis();
}

void loop() {
  // ---- Read a full line if available ----
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    line.replace("\r", "");
    if (line.length() > 0) {
      parseLine(line);
      lastCmdMs = millis();
      have_valid_cmd = true;
      // blink on valid VEL line
      digitalWrite(LED_BUILTIN, HIGH);
      delay(5);
      digitalWrite(LED_BUILTIN, LOW);
    }
    while (Serial.peek() == '\r' || Serial.peek() == '\n') Serial.read();
  }

  // ---- Deadman watchdog ----
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    linear_x = 0.0f; angular_z = 0.0f;
    stopAllMotors();
    have_valid_cmd = false;
  } else if (have_valid_cmd) {
    driveFromCmd(linear_x, angular_z);
  } else {
    stopAllMotors();
  }
}

// ===== Helpers =====

void parseLine(const String& s) {
  if (s.startsWith("VEL")) {
    int c1 = s.indexOf(',');
    int c2 = s.indexOf(',', c1 + 1);
    if (c1 > 0 && c2 > c1) {
      float lx = s.substring(c1 + 1, c2).toFloat();
      float az = s.substring(c2 + 1).toFloat();
      lx = constrain(lx, -LOCAL_MAX_ABS, LOCAL_MAX_ABS);
      az = constrain(az, -LOCAL_MAX_ABS, LOCAL_MAX_ABS);
      linear_x = lx;
      angular_z = az;
      return;
    }
  } else if (s == "STOP") {
    linear_x = 0.0f; angular_z = 0.0f;
    stopAllMotors();
    return;
  }
  // malformed => stop
  linear_x = 0.0f; angular_z = 0.0f;
  stopAllMotors();
}

void driveFromCmd(float lx, float az) {
  // Skid-steer mix
  float left  = lx - az;
  float right = lx + az;

  int leftPWM  = (int)(constrain(left,  -1.0f, 1.0f) * 255.0f);
  int rightPWM = (int)(constrain(right, -1.0f, 1.0f) * 255.0f);

  // Apply deadband: if nonzero and small, bump to MIN_PWM
  if (leftPWM > 0 && leftPWM < MIN_PWM)  leftPWM = MIN_PWM;
  if (leftPWM < 0 && leftPWM > -MIN_PWM) leftPWM = -MIN_PWM;
  if (rightPWM > 0 && rightPWM < MIN_PWM)  rightPWM = MIN_PWM;
  if (rightPWM < 0 && rightPWM > -MIN_PWM) rightPWM = -MIN_PWM;

  // Left side: M1 & M3
  setMotor(M1_IN1, M1_IN2, leftPWM);
  setMotor(M3_IN1, M3_IN2, leftPWM);
  // Right side: M2 & M4
  setMotor(M2_IN3, M2_IN4, rightPWM);
  setMotor(M4_IN3, M4_IN4, rightPWM);
}

void setMotor(int pinFwd, int pinRev, int pwm) {
  if (pwm > 0) {
    analogWrite(pinFwd, pwm);  analogWrite(pinRev, 0);
  } else if (pwm < 0) {
    analogWrite(pinFwd, 0);    analogWrite(pinRev, -pwm);
  } else {
    analogWrite(pinFwd, 0);    analogWrite(pinRev, 0);
  }
}

void stopAllMotors() {
  analogWrite(M1_IN1, 0); analogWrite(M1_IN2, 0);
  analogWrite(M2_IN3, 0); analogWrite(M2_IN4, 0);
  analogWrite(M3_IN1, 0); analogWrite(M3_IN2, 0);
  analogWrite(M4_IN3, 0); analogWrite(M4_IN4, 0);
}
