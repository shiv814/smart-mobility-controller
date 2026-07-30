#include "../src/controller.hpp"

using mobility::Command;
using mobility::Controller;
using mobility::MotorOutput;

constexpr int LEFT_PWM = 5;
constexpr int LEFT_DIR = 4;
constexpr int RIGHT_PWM = 6;
constexpr int RIGHT_DIR = 7;

Controller controller;

void driveMotor(int pwmPin, int directionPin, int value) {
  digitalWrite(directionPin, value >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, constrain(abs(value), 0, 255));
}

Command parseCommand(char value) {
  switch (value) {
    case 'F': return Command::Forward;
    case 'B': return Command::Reverse;
    case 'L': return Command::Left;
    case 'R': return Command::Right;
    default: return Command::Stop;
  }
}

void setup() {
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  Serial.begin(9600);
  controller.command(Command::Stop, millis());
}

void loop() {
  if (Serial.available() > 0) {
    controller.command(parseCommand(Serial.read()), millis());
  }
  MotorOutput output = controller.update(millis());
  driveMotor(LEFT_PWM, LEFT_DIR, output.left_pwm);
  driveMotor(RIGHT_PWM, RIGHT_DIR, output.right_pwm);
  delay(20);
}
