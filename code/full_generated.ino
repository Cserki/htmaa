/*
#include <AccelStepper.h>
#include <Servo.h>
#include <ezButton.h>

// Stepper motor setup
AccelStepper baseMotor(AccelStepper::DRIVER, 2, 5);
AccelStepper lowerArmMotor(AccelStepper::DRIVER, 3, 6);
AccelStepper upperArmMotor(AccelStepper::DRIVER, 4, 7);

// Servo setup
Servo gripperServo;

// Button setup (connect to digital pin 8)
ezButton modeButton(8);

// Joystick analog pins
const int joy1X = A0;  // base
const int joy1Y = A1;  // lower arm
const int joy2X = A2;  // gripper
const int joy2Y = A3;  // upper arm

// Gripper
int gripperAngle = 90;  // default closed
int gripperStep = 5;
int gripperMin = 60;
int gripperMax = 120;

// Deadzone
const int deadzone = 30;

// Tuning constants (easily tweak!)
const int maxSpeedBase      = 600;
const int maxSpeedLowerArm  = 600;
const int maxSpeedUpperArm  = 400;

void setup() {
  Serial.begin(9600);

  // Motor setup
  baseMotor.setMaxSpeed(maxSpeedBase);
  baseMotor.setAcceleration(400);

  lowerArmMotor.setMaxSpeed(maxSpeedLowerArm);
  lowerArmMotor.setAcceleration(400);

  upperArmMotor.setMaxSpeed(maxSpeedUpperArm);
  upperArmMotor.setAcceleration(300);

  // Servo
  gripperServo.attach(9);  // attach to pin 9
  gripperServo.write(gripperAngle);  // initialize position

  // Button
  modeButton.setDebounceTime(50);  // optional
}

void loop() {
  modeButton.loop();  // necessary for ezButton

  // Read joysticks
  int baseVal      = analogRead(joy1X) - 512;
  int lowerArmVal  = analogRead(joy1Y) - 512;
  int gripperVal   = analogRead(joy2X) - 512;
  int upperArmVal  = analogRead(joy2Y) - 512;

  // Deadzone handling
  baseVal      = (abs(baseVal) > deadzone) ? baseVal : 0;
  lowerArmVal  = (abs(lowerArmVal) > deadzone) ? lowerArmVal : 0;
  gripperVal   = (abs(gripperVal) > deadzone) ? gripperVal : 0;
  upperArmVal  = (abs(upperArmVal) > deadzone) ? upperArmVal : 0;

  // Map analog input to stepper speed
  baseMotor.setSpeed(map(baseVal, -512, 512, -maxSpeedBase, maxSpeedBase));
  lowerArmMotor.setSpeed(map(lowerArmVal, -512, 512, -maxSpeedLowerArm, maxSpeedLowerArm));
  upperArmMotor.setSpeed(map(upperArmVal, -512, 512, -maxSpeedUpperArm, maxSpeedUpperArm));

  // Run steppers
  baseMotor.runSpeed();
  lowerArmMotor.runSpeed();
  upperArmMotor.runSpeed();

  // Gripper control (servo)
  if (gripperVal > deadzone && gripperAngle < gripperMax) {
    gripperAngle += gripperStep;
    gripperServo.write(gripperAngle);
    delay(10);
  } else if (gripperVal < -deadzone && gripperAngle > gripperMin) {
    gripperAngle -= gripperStep;
    gripperServo.write(gripperAngle);
    delay(10);
  }

  // Example button usage: reset gripper
  if (modeButton.isPressed()) {
    gripperAngle = 90;
    gripperServo.write(gripperAngle);
  }
}
*/