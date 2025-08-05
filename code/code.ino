#include <AccelStepper.h>
#include <Servo.h>
#include <ezButton.h>

// PIN DEFINITIONS
// Joystick pin assignments
constexpr int joy1X = A0; // joystick 1 X-axis (base rotation)
constexpr int joy1Y = A1; // joystick 1 Y-axis (lower arm)
constexpr int joy2X = A2; // joystick 2 X-axis (gripper)
constexpr int joy2Y = A3; // joystick 2 Y-axis (upper arm)
constexpr int joy1B = 2;  // joystick 1 button (optional)
constexpr int joy2B = 3;  // joystick 2 button (optional)

// Servo
constexpr int servoPin = 10;    // servo pin for gripper control
constexpr int servoNeutral = 0; // neutral position for servo

// Stepper motor pins
// base rotation
constexpr int baseStepPin = 6; // step pin for base rotation stepper
constexpr int baseDirPin = 7;  // direction pin for base rotation stepper

// lower arm
constexpr int lowerArmStepPin = 4; // step pin for lower arm stepper
constexpr int lowerArmDirPin = 5;  // direction pin for lower arm stepper

// upper arm
constexpr int upperArmStepPin = 8; // step pin for upper arm stepper
constexpr int upperArmDirPin = 9;  // direction pin for upper arm stepper

// proximity sensor pins
constexpr int proximitySensorPin = 11; // pin for proximity sensor

// led pin (built-in LED)
constexpr int ledPin = LED_BUILTIN; // built-in LED pin

// OTHER CONSTANTS
constexpr int deadzone = 50;   // deadzone for joystick input
constexpr int max_input = 512; // maximum analog value for joystick

// CLASS INSTANTIATIONS
// Create ezButton instances for joystick buttons
ezButton joy1Button(joy1B);
ezButton joy2Button(joy2B);

// Create Servo instance for gripper control
Servo servo;

// AccelStepper instances for stepper motors
AccelStepper baseStepper(AccelStepper::DRIVER, baseStepPin, baseDirPin);
AccelStepper lowerArmStepper(AccelStepper::DRIVER, lowerArmStepPin,
                             lowerArmDirPin);
AccelStepper upperArmStepper(AccelStepper::DRIVER, upperArmStepPin,
                             upperArmDirPin);

// CONFIGURATION
// Servo configuration
int servo_position = servoNeutral; // Current position for the gripper servo
int servo_target = servoNeutral;   // Target position for the gripper servo
const int servo_step = 1; // Amount to move per update (smaller = slower)
const unsigned long servoInterval =
    20; // ms between servo moves (higher = slower)
unsigned long lastServoUpdate = 0;
int servo_lower_bound = 0;
int servo_upper_bound = 80;

// Stepper motor configuration
int max_speed = 100;   // Maximum speed for stepper motors
int acceleration = 30; // Acceleration for stepper motors

// Add these global variables to store neutral positions for steppers
long baseNeutralPos = 0;
long lowerArmNeutralPos = 0;
long upperArmNeutralPos = 0;

void setup() {
  Serial.begin(9600);

  // Initialize led
  pinMode(ledPin, OUTPUT);

  // Initialize proximity sensor
  pinMode(proximitySensorPin, INPUT);

  // Initialize buttons
  joy1Button.setDebounceTime(deadzone);
  joy2Button.setDebounceTime(deadzone);

  // Initialize servo
  servo.attach(servoPin);      // Attach servo
  servo.write(servo_position); // Set initial position to neutral (90 degrees)

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

bool debug = true; // Set to true to enable debugging output

// Non-blocking timing variables
unsigned long lastLoopTime = 0;
const unsigned long loopInterval = 500; // ms

void loop() {
  // Update button states
  joy1Button.loop();
  joy2Button.loop();
  
  // Read joystick values - analogRead returns values from 0 to 1023
  // Center values
  int upper_arm_val = analogRead(joy2Y) - max_input;
  int base_val = analogRead(joy1X) - max_input; //
  int lower_arm_val = analogRead(joy1Y) - max_input;
  int servo_val = analogRead(joy2X) - max_input;
  
  // Apply deadzone
  servo_val = (abs(servo_val) > deadzone) ? servo_val : 0;
  base_val = (abs(base_val) > deadzone) ? base_val : 0;
  lower_arm_val = (abs(lower_arm_val) > deadzone) ? lower_arm_val : 0;
  upper_arm_val = (abs(upper_arm_val) > deadzone) ? upper_arm_val : 0;

  // Calculate target position based on joystick input
  if (servo_val != 0) {
    int mapped_servo_val = map(servo_val, -max_input, max_input, 0, 180);
    servo_target =
    constrain(mapped_servo_val, servo_lower_bound, servo_upper_bound);
  } else {
    // If joystick is in deadzone, stop moving by setting target to current
    // position
    servo_target = servo_position;
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

  // Move stepper motors based on joystick input
  if (base_val != 0) {
    int mapped_base_val =
    map(base_val, -max_input, max_input, -max_speed, max_speed);
    baseStepper.setSpeed(mapped_base_val); // Set speed for base stepper
    baseStepper.runSpeed();                // Run the stepper motor
  }

  if (lower_arm_val != 0) {
    int mapped_lower_arm_val =
    map(lower_arm_val, -max_input, max_input, -max_speed, max_speed);
    lowerArmStepper.setSpeed(
        mapped_lower_arm_val);  // Set speed for lower arm stepper
        lowerArmStepper.runSpeed(); // Run the stepper motor
  }
  
  if (upper_arm_val != 0) {
    int mapped_upper_arm_val =
    map(upper_arm_val, -max_input, max_input, -max_speed, max_speed);
    upperArmStepper.setSpeed(
      mapped_upper_arm_val);  // Set speed for upper arm stepper
      upperArmStepper.runSpeed(); // Run the stepper motor
  }

  // gripper obstacle detection
  if (digitalRead(proximitySensorPin) == LOW) {
    servo.write(servoNeutral); // If obstacle detected, close gripper
    servo_position = servoNeutral; // Update servo position
    servo_target = servo_position;
    Serial.println("Obstacle detected! Closing gripper.");
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }

  // move servo back to neutral position
  if (joy1Button.isPressed()) {
    Serial.println("Joystick 1 Button Pressed");
    baseStepper.moveTo(baseNeutralPos);
    lowerArmStepper.moveTo(lowerArmNeutralPos);
    upperArmStepper.moveTo(upperArmNeutralPos);
    baseStepper.runToPosition(); // Move base stepper to neutral position
    lowerArmStepper.runToPosition(); // Move lower arm stepper to neutral position
    upperArmStepper.runToPosition(); // Move upper arm stepper to neutral position
  }

  if (joy2Button.isPressed()) {
    Serial.println("Joystick 2 Button Pressed");
    servo_target = servo_upper_bound;   // Move servo back to neutral position
    servo_position = servo_target; // Reset servo position immediately
    servo.write(servo_position);   // Update servo position
  }

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
    Serial.print("Base Stepper Position: ");
    Serial.println(baseStepper.currentPosition());
    Serial.print("Lower Arm Stepper Position: ");
    Serial.println(lowerArmStepper.currentPosition());
    Serial.print("Upper Arm Stepper Position: ");
    Serial.println(upperArmStepper.currentPosition());
  }
}
