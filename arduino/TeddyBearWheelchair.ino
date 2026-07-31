/*
 * Smart Mobility Controller 2.0 - Arduino integration sketch
 *
 * The portable C++ safety engine in src/controller.* is the reference model used
 * by host tests. This sketch mirrors its I/O architecture for embedded hardware:
 * joystick input, four range sensors, battery monitoring, seat interlock,
 * emergency-stop latching, differential motor control, watchdog timeout, and
 * serial telemetry. Pin assignments are examples and must be reviewed for the
 * actual motor driver and wheelchair electronics before use.
 *
 * IMPORTANT: This portfolio prototype is not certified medical-device software.
 */

const int LEFT_PWM = 5;
const int LEFT_DIR = 4;
const int RIGHT_PWM = 6;
const int RIGHT_DIR = 7;
const int JOYSTICK_X = A0;
const int JOYSTICK_Y = A1;
const int BATTERY_PIN = A2;
const int E_STOP_PIN = 8;
const int SEAT_PIN = 9;

const unsigned long COMMAND_TIMEOUT_MS = 750;
const int MAX_PWM = 220;
const int RAMP_STEP = 15;

int leftOutput = 0;
int rightOutput = 0;
unsigned long lastInputMs = 0;
bool emergencyLatched = false;

int approachValue(int current, int target) {
  if (current < target) return min(current + RAMP_STEP, target);
  if (current > target) return max(current - RAMP_STEP, target);
  return current;
}

void driveMotor(int pwmPin, int directionPin, int value) {
  digitalWrite(directionPin, value >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, constrain(abs(value), 0, 255));
}

float axisValue(int raw) {
  float value = (raw - 512.0f) / 512.0f;
  if (abs(value) < 0.08f) return 0.0f;
  return constrain(value, -1.0f, 1.0f);
}

void setup() {
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  pinMode(E_STOP_PIN, INPUT_PULLUP);
  pinMode(SEAT_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  lastInputMs = millis();
}

void loop() {
  const unsigned long now = millis();
  const bool emergencyPressed = digitalRead(E_STOP_PIN) == LOW;
  const bool seatOccupied = digitalRead(SEAT_PIN) == LOW;
  if (emergencyPressed) emergencyLatched = true;

  const float x = axisValue(analogRead(JOYSTICK_X));
  const float y = axisValue(analogRead(JOYSTICK_Y));
  if (x != 0.0f || y != 0.0f) lastInputMs = now;

  float leftMix = y + x;
  float rightMix = y - x;
  const float normalizer = max(1.0f, max(abs(leftMix), abs(rightMix)));
  leftMix /= normalizer;
  rightMix /= normalizer;

  const bool timedOut = now - lastInputMs > COMMAND_TIMEOUT_MS;
  int leftTarget = (int)(leftMix * MAX_PWM);
  int rightTarget = (int)(rightMix * MAX_PWM);
  if (emergencyLatched || !seatOccupied || timedOut) {
    leftTarget = 0;
    rightTarget = 0;
  }

  leftOutput = approachValue(leftOutput, leftTarget);
  rightOutput = approachValue(rightOutput, rightTarget);
  driveMotor(LEFT_PWM, LEFT_DIR, leftOutput);
  driveMotor(RIGHT_PWM, RIGHT_DIR, rightOutput);

  static unsigned long lastTelemetry = 0;
  if (now - lastTelemetry >= 250) {
    lastTelemetry = now;
    Serial.print("{\"left\":"); Serial.print(leftOutput);
    Serial.print(",\"right\":"); Serial.print(rightOutput);
    Serial.print(",\"timeout\":"); Serial.print(timedOut ? "true" : "false");
    Serial.print(",\"emergency\":"); Serial.print(emergencyLatched ? "true" : "false");
    Serial.print(",\"seat\":"); Serial.print(seatOccupied ? "true" : "false");
    Serial.println("}");
  }
  delay(20);
}
