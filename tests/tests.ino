#include <AccelStepper.h>
#include <Servo.h>
#include <ezButton.h>

// PIN DEFINITIONS
constexpr int joy1B = 2;  // joystick 1 button
constexpr int joy2B = 3;  // joystick 2 button
constexpr int baseStepPin = 8;
constexpr int baseDirPin = 9;
constexpr int lowerArmStepPin = 6;
constexpr int lowerArmDirPin = 7;
constexpr int upperArmStepPin = 4;
constexpr int upperArmDirPin = 5;
constexpr int resetButtonPin = 12;  // New button pin

// OTHER CONSTANTS
constexpr int acceleration = 80;  // Acceleration for stepper motors
constexpr int max_speed = 50;     // Maximum speed for stepper motors
constexpr bool debug = true;      // Enable debug mode
constexpr int debounce_time = 30;

// CLASS INSTANTIATIONS
ezButton joy1Button(joy1B);
ezButton joy2Button(joy2B);
ezButton resetButton(resetButtonPin);  // New reset button
AccelStepper baseStepper(AccelStepper::DRIVER, baseStepPin, baseDirPin);
AccelStepper lowerArmStepper(AccelStepper::DRIVER, lowerArmStepPin,
                             lowerArmDirPin);
AccelStepper upperArmStepper(AccelStepper::DRIVER, upperArmStepPin,
                             upperArmDirPin);

// NEUTRAL POSITION VARIABLES
long baseNeutralPos = 0;
long lowerArmNeutralPos = 0;
long upperArmNeutralPos = 0;

// PREDEFINED MOTION FUNCTION
void runPredefinedMotion() {
  // Move each stepper +100 steps, then back to neutral
  baseStepper.moveTo(baseStepper.currentPosition() + 10);
  lowerArmStepper.moveTo(lowerArmStepper.currentPosition() + 20);
  // upperArmStepper.moveTo(upperArmStepper.currentPosition() + 40);

  while (baseStepper.distanceToGo() != 0 || lowerArmStepper.distanceToGo() != 0 || upperArmStepper.distanceToGo() != 0) {
    baseStepper.run();
    lowerArmStepper.run();
    upperArmStepper.run();
  }
}

void testSimpleLinesWithIncreasingSpeed(int rounds) {
  int speed = 50;  // Initial speed
  int accel = 80;  // Initial acceleration
  int lower_step_between_rounds = 10;

  for (int i = 1; i <= rounds; i++) {
    Serial.print("Round: ");
    Serial.println(i + 1);

    // base drawing motion
    baseStepper.setMaxSpeed(speed * i);
    baseStepper.setAcceleration(accel * i);
    lowerArmStepper.setMaxSpeed(speed * i);
    lowerArmStepper.setAcceleration(accel * i);
    upperArmStepper.setMaxSpeed(speed * i);
    upperArmStepper.setAcceleration(accel * i);

    baseStepper.moveTo(80);
    baseStepper.runToPosition();

    lowerArmStepper.moveTo(20);
    lowerArmStepper.runToPosition();

    upperArmStepper.moveTo(20);
    upperArmStepper.runToPosition();

    baseStepper.moveTo(baseNeutralPos);
    baseStepper.runToPosition();

    lowerArmStepper.moveTo(lowerArmNeutralPos);
    lowerArmStepper.runToPosition();

    upperArmStepper.moveTo(upperArmNeutralPos);
    upperArmStepper.runToPosition();

    // movement between rounds
    lowerArmStepper.moveTo(lowerArmNeutralPos + 1);
  }
}

void testSimpleLinesAccuracy(int rounds) {
  int lower_step_between_rounds = 20;

  //lowerArmStepper.move(-2);

  for (int i = 1; i <= rounds; i++) {
    Serial.print("Round: ");
    Serial.println(i + 1);

    // base drawing motion
    int reps = 2;
    for (int j = 0; j < reps; j++) {
      baseStepper.moveTo(baseNeutralPos + 20);
      baseStepper.runToPosition();

      lowerArmStepper.move(lowerArmNeutralPos + 20);
      lowerArmStepper.runToPosition();

      baseStepper.moveTo(baseNeutralPos);
      baseStepper.runToPosition();

      if (j == reps) {
        lowerArmStepper.moveTo(lowerArmNeutralPos);
        lowerArmStepper.runToPosition();
      }
    }

    // movement between rounds
    if (i != rounds) {
      lowerArmStepper.moveTo(baseNeutralPos + 20);
      lowerArmStepper.runToPosition();

      upperArmStepper.moveTo(upperArmNeutralPos + lower_step_between_rounds * i);
      upperArmStepper.runToPosition();

      lowerArmStepper.moveTo(lowerArmNeutralPos - 20);
      lowerArmStepper.runToPosition();
    }

    // Reset to neutral position
    lowerArmNeutralPos = lowerArmStepper.currentPosition();
    upperArmNeutralPos = upperArmStepper.currentPosition();
  }

  Serial.println("Test Motion Completed");
}

void setup() {
  Serial.begin(9600);

  joy1Button.setDebounceTime(debounce_time);
  joy2Button.setDebounceTime(debounce_time);
  resetButton.setDebounceTime(debounce_time);

  baseStepper.setMaxSpeed(max_speed);
  baseStepper.setAcceleration(acceleration);
  lowerArmStepper.setMaxSpeed(max_speed);
  lowerArmStepper.setAcceleration(acceleration);
  upperArmStepper.setMaxSpeed(max_speed);
  upperArmStepper.setAcceleration(acceleration);

  // Capture initial positions as neutral
  baseNeutralPos = baseStepper.currentPosition();
  lowerArmNeutralPos = lowerArmStepper.currentPosition();
  upperArmNeutralPos = upperArmStepper.currentPosition();
}

void loop() {
  joy1Button.loop();
  joy2Button.loop();
  resetButton.loop();  // Poll new button

  // Button 1: Capture neutral positions
  if (joy1Button.isPressed()) {
    Serial.println("Joystick 1 Button Pressed - Capturing Neutral Positions");
    baseNeutralPos = baseStepper.currentPosition();
    lowerArmNeutralPos = lowerArmStepper.currentPosition();
    upperArmNeutralPos = upperArmStepper.currentPosition();
  }

  // Button 2: Run predefined motion
  // if (joy2Button.isPressed()) {
  if (resetButton.isPressed()) {
    Serial.println("Joystick 2 Button Pressed - Running Predefined Motion");
    testSimpleLinesAccuracy(3);  // Run the test with 3 rounds
  }

  // Reset Button: Move motors back to captured neutral positions
  // if (resetButton.isPressed()) {
  if (joy2Button.isPressed()) {
    Serial.println("Reset Button Pressed - Returning to Neutral Positions");
    baseStepper.moveTo(baseNeutralPos);
    lowerArmStepper.moveTo(lowerArmNeutralPos);
    upperArmStepper.moveTo(upperArmNeutralPos);
  }

  // Steppers need to be run continuously for non-blocking movement
  baseStepper.run();
  lowerArmStepper.run();
  upperArmStepper.run();

  // non-blocking delay
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();
  if (debug && currentTime - lastTime >= 300) {  // 100 ms delay
    lastTime = currentTime;

    Serial.print("Base Position: ");
    Serial.print(baseStepper.currentPosition());
    Serial.print(" | Lower Arm Position: ");
    Serial.print(lowerArmStepper.currentPosition());
    Serial.print(" | Upper Arm Position: ");
    Serial.println(upperArmStepper.currentPosition());
  }
}
