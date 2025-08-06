/*
  Arduino Robotic Arm Control

  Controls three stepper motors (base, lower arm, upper arm) and a servo (gripper) using joysticks and buttons.
  - Joystick 1 Button (pin 2): Move all steppers to their neutral positions.
  - Joystick 2 Button (pin 3): Move servo to neutral position and enable gripper closing.
  - Reset Button (pin 12): Capture current stepper positions as new neutral positions.
  - Proximity sensor: Automatically closes gripper if obstacle detected.
  - Serial output provides debug info about joystick values, stepper positions, and neutral positions.
*/

#include <AccelStepper.h>
#include <Servo.h>
#include <ezButton.h>

// --- Pin Definitions ---
constexpr int joy1X = A0; // Joystick 1 X-axis (base rotation)
constexpr int joy1Y = A1; // Joystick 1 Y-axis (lower arm)
constexpr int joy2X = A2; // Joystick 2 X-axis (gripper)
constexpr int joy2Y = A3; // Joystick 2 Y-axis (upper arm)
constexpr int joy1B = 2;  // Joystick 1 button
constexpr int joy2B = 3;  // Joystick 2 button
constexpr int servoPin = 10;    // Servo pin for gripper control
constexpr int servoNeutral = 0; // Neutral position for servo
constexpr int baseStepPin = 6;  // Base stepper step pin
constexpr int baseDirPin = 7;   // Base stepper direction pin
constexpr int lowerArmStepPin = 4; // Lower arm stepper step pin
constexpr int lowerArmDirPin = 5;  // Lower arm stepper direction pin
constexpr int upperArmStepPin = 8; // Upper arm stepper step pin
constexpr int upperArmDirPin = 9;  // Upper arm stepper direction pin
constexpr int proximitySensorPin = 11; // Proximity sensor pin
constexpr int ledPin = LED_BUILTIN;    // Built-in LED pin
constexpr int resetButtonPin = 12;     // Reset button pin

// --- Other Constants ---
constexpr int deadzone = 100;   // Deadzone for joystick input
constexpr int max_input = 512;  // Maximum analog value for joystick

// --- Button and Motor Instances ---
ezButton joy1Button(joy1B);           // Button for moving to neutral positions
ezButton joy2Button(joy2B);           // Button for moving servo to neutral
ezButton resetButton(resetButtonPin); // Button for capturing neutral positions
Servo servo;                          // Servo for gripper control
AccelStepper baseStepper(AccelStepper::DRIVER, baseStepPin, baseDirPin);         // Base stepper motor
AccelStepper lowerArmStepper(AccelStepper::DRIVER, lowerArmStepPin, lowerArmDirPin); // Lower arm stepper motor
AccelStepper upperArmStepper(AccelStepper::DRIVER, upperArmStepPin, upperArmDirPin); // Upper arm stepper motor

// --- Servo Configuration ---
const int servo_step = 1; // Amount to move per update (smaller = slower)
const unsigned long servoInterval = 10; // ms between servo moves
unsigned long lastServoUpdate = 0;
int servo_lower_bound = 20; // Lower bound for servo position
int servo_upper_bound = 80; // Upper bound for servo position
int servo_target = servo_lower_bound;   // Target position for the gripper servo
int servo_position = servo_lower_bound; // Current position for the gripper servo
bool can_close = true; // Flag for gripper closing

// --- Stepper Motor Configuration ---
int max_speed = 100;   // Maximum speed for stepper motors
int acceleration = 30; // Acceleration for stepper motors

// --- Neutral Position Variables ---
long baseNeutralPos = 0;      // Neutral position for base stepper
long lowerArmNeutralPos = 0;  // Neutral position for lower arm stepper
long upperArmNeutralPos = 0;  // Neutral position for upper arm stepper

void setup() {
  Serial.begin(9600);

  pinMode(ledPin, OUTPUT); // Initialize LED
  pinMode(proximitySensorPin, INPUT); // Initialize proximity sensor

  // Initialize buttons with debounce
  joy1Button.setDebounceTime(deadzone);
  joy2Button.setDebounceTime(deadzone);
  resetButton.setDebounceTime(deadzone);

  // Initialize servo
  servo.attach(servoPin);
  servo.write(servo_position);

  // Initialize stepper motors
  baseStepper.setMaxSpeed(max_speed);
  baseStepper.setAcceleration(acceleration);
  lowerArmStepper.setMaxSpeed(max_speed);
  lowerArmStepper.setAcceleration(acceleration);
  upperArmStepper.setMaxSpeed(max_speed);
  upperArmStepper.setAcceleration(acceleration);

  // Capture initial positions as neutral for steppers
  baseNeutralPos = baseStepper.currentPosition();
  lowerArmNeutralPos = lowerArmStepper.currentPosition();
  upperArmNeutralPos = upperArmStepper.currentPosition();
}

bool debug = true; // Enable debug output

// Non-blocking timing variables for debug output
unsigned long lastLoopTime = 0;
const unsigned long loopInterval = 500; // ms between debug prints

void loop() {
  // Update button states
  joy1Button.loop();
  joy2Button.loop();
  resetButton.loop();

  // --- Read joystick values ---
  int upper_arm_val = analogRead(joy2Y) - max_input;
  int base_val = analogRead(joy1X) - max_input;
  int lower_arm_val = analogRead(joy1Y) - max_input;
  int servo_val = analogRead(joy2X) - max_input;

  // --- Apply deadzone ---
  servo_val = (abs(servo_val) > deadzone) ? servo_val : 0;
  base_val = (abs(base_val) > deadzone) ? base_val : 0;
  lower_arm_val = (abs(lower_arm_val) > deadzone) ? lower_arm_val : 0;
  upper_arm_val = (abs(upper_arm_val) > deadzone) ? upper_arm_val : 0;

  // --- Servo control based on joystick input ---
  if (servo_val != 0) {
    int mapped_servo_val = map(servo_val, -max_input, max_input, 0, 180);
    servo_target = constrain(mapped_servo_val, servo_lower_bound, servo_upper_bound);
  } else {
    servo_target = servo_position; // Stop moving if joystick is in deadzone
  }

  // Smoothly move servo toward target position at intervals
  unsigned long now = millis();
  if (now - lastServoUpdate >= servoInterval) {
    lastServoUpdate = now;
    if (servo_position < servo_target) {
      servo_position += servo_step;
      if (servo_position > servo_target)
        servo_position = servo_target;
      servo.write(servo_position);
    } else if (servo_position > servo_target) {
      servo_position -= servo_step;
      if (servo_position < servo_target)
        servo_position = servo_target;
      servo.write(servo_position);
    }
  }

  // --- Stepper motor control based on joystick input ---
  if (base_val != 0) {
    int mapped_base_val = map(base_val, -max_input, max_input, -max_speed, max_speed);
    baseStepper.setSpeed(mapped_base_val);
    baseStepper.runSpeed();
  }

  if (lower_arm_val != 0) {
    int mapped_lower_arm_val = map(lower_arm_val, -max_input, max_input, -max_speed, max_speed);
    lowerArmStepper.setSpeed(mapped_lower_arm_val);
    lowerArmStepper.runSpeed();
  }

  if (upper_arm_val != 0) {
    int mapped_upper_arm_val = map(upper_arm_val, -max_input, max_input, -max_speed, max_speed);
    upperArmStepper.setSpeed(mapped_upper_arm_val);
    upperArmStepper.runSpeed();
  }

  // --- Gripper obstacle detection ---
  if (digitalRead(proximitySensorPin) == LOW && can_close) {
    servo.write(servoNeutral);     // If obstacle detected, close gripper
    servo_position = servoNeutral;
    servo_target = servo_position;
    Serial.println("Obstacle detected! Closing gripper.");
    can_close = false;
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }

  // --- Button actions ---
  // Joystick 1 Button: Move all steppers to neutral positions
  if (joy1Button.isPressed()) {
    Serial.println("Joystick 1 Button Pressed");
    baseStepper.moveTo(baseNeutralPos);
    lowerArmStepper.moveTo(lowerArmNeutralPos);
    upperArmStepper.moveTo(upperArmNeutralPos);
    baseStepper.runToPosition();
    lowerArmStepper.runToPosition();
    upperArmStepper.runToPosition();
  }

  // Joystick 2 Button: Move servo to neutral position and enable gripper closing
  if (joy2Button.isPressed()) {
    Serial.println("Joystick 2 Button Pressed");
    servo_target = servo_upper_bound;
    servo_position = servo_target;
    servo.write(servo_position);
    can_close = true;
  }

  // Reset Button: Capture current stepper positions as new neutral positions
  if (resetButton.isPressed()) {
    Serial.println("Reset Button Pressed");
    baseNeutralPos = baseStepper.currentPosition();
    lowerArmNeutralPos = lowerArmStepper.currentPosition();
    upperArmNeutralPos = upperArmStepper.currentPosition();
  }

  // --- Debug output ---
  now = millis();
  if (debug && now - lastLoopTime >= loopInterval) {
    lastLoopTime = now;

    Serial.print("Base: ");
    Serial.print(base_val);
    Serial.print(", Lower Arm: ");
    Serial.print(lower_arm_val);
    Serial.print(", Upper Arm: ");
    Serial.print(upper_arm_val);
    Serial.print(", Servo: ");
    Serial.println(servo_position);

    // Print joystick values for debugging
    Serial.print("Joy1 X: ");
    Serial.print(base_val);
    Serial.print(", Y: ");
    Serial.println(lower_arm_val);

    Serial.print("Joy2 X: ");
    Serial.print(servo_val);
    Serial.print(", Y: ");
    Serial.println(upper_arm_val);

    // Print stepper positions
    Serial.print("Base current Position: ");
    Serial.print(baseStepper.currentPosition());
    Serial.print(", Lower current Position: ");
    Serial.print(lowerArmStepper.currentPosition());
    Serial.print(", Upper current Position: ");
    Serial.println(upperArmStepper.currentPosition());

    // Print stepper neutral positions
    Serial.print("Base neutral Position: ");
    Serial.print(baseNeutralPos);
    Serial.print(", Lower neutral Position: ");
    Serial.print(lowerArmNeutralPos);
    Serial.print(", Upper neutral Position: ");
    Serial.println(upperArmNeutralPos);
  }
}
