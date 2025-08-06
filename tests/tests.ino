#include <AccelStepper.h>
#include <Servo.h>
#include <ezButton.h>

// PIN DEFINITIONS
constexpr int joy1B = 2; // joystick 1 button (captures neutral positions)
constexpr int joy2B = 3; // joystick 2 button (returns to neutral positions)
constexpr int baseStepPin = 8;      // base stepper step pin
constexpr int baseDirPin = 9;       // base stepper direction pin
constexpr int lowerArmStepPin = 6;  // lower arm stepper step pin
constexpr int lowerArmDirPin = 7;   // lower arm stepper direction pin
constexpr int upperArmStepPin = 4;  // upper arm stepper step pin
constexpr int upperArmDirPin = 5;   // upper arm stepper direction pin
constexpr int resetButtonPin = 12;  // reset button (runs predefined motion)

// OTHER CONSTANTS
constexpr int acceleration = 80; // Acceleration for stepper motors
constexpr int max_speed = 50;    // Maximum speed for stepper motors
constexpr bool debug = true;     // Enable debug mode (serial output)
constexpr int debounce_time = 30; // Button debounce time in ms

// CLASS INSTANTIATIONS
ezButton joy1Button(joy1B);           // Button for capturing neutral positions
ezButton joy2Button(joy2B);           // Button for returning to neutral positions
ezButton resetButton(resetButtonPin); // Button for running predefined motion
AccelStepper baseStepper(AccelStepper::DRIVER, baseStepPin, baseDirPin);         // Base stepper motor
AccelStepper lowerArmStepper(AccelStepper::DRIVER, lowerArmStepPin, lowerArmDirPin); // Lower arm stepper motor
AccelStepper upperArmStepper(AccelStepper::DRIVER, upperArmStepPin, upperArmDirPin); // Upper arm stepper motor

// NEUTRAL POSITION VARIABLES
long baseNeutralPos = 0;      // Neutral position for base stepper
long lowerArmNeutralPos = 0;  // Neutral position for lower arm stepper
long upperArmNeutralPos = 0;  // Neutral position for upper arm stepper

// PREDEFINED MOTION FUNCTION
void runPredefinedMotion() {
  // Example: Move lower arm and upper arm by fixed offsets, then return to neutral
  lowerArmStepper.moveTo(lowerArmStepper.currentPosition() - 10);
  upperArmStepper.moveTo(upperArmStepper.currentPosition() + 40);
  
  while (baseStepper.distanceToGo() != 0 ||
        lowerArmStepper.distanceToGo() != 0 ||
        upperArmStepper.distanceToGo() != 0) {
    baseStepper.run();
    lowerArmStepper.run();
    upperArmStepper.run();
  }
}

int r = 0; // Counter for test rounds

void testSimpleLinesWithIncreasingSpeed(int rounds) {
  int speed = 50; // Initial speed for test
  int accel = 80; // Initial acceleration for test
  int lower_step_between_rounds = 10; // Step offset between rounds for lower arm

  for (int i = 1; i <= rounds; i++) {
    Serial.print("Round: ");
    Serial.println(i + 1);

    // Set speed and acceleration for each round
    baseStepper.setMaxSpeed(speed * i);
    baseStepper.setAcceleration(accel * i);
    lowerArmStepper.setMaxSpeed(speed * i);
    lowerArmStepper.setAcceleration(accel * i);
    upperArmStepper.setMaxSpeed(speed * i);
    upperArmStepper.setAcceleration(accel * i);

    // Move steppers to test positions and back to neutral
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

    // Small movement between rounds for lower arm
    lowerArmStepper.moveTo(lowerArmNeutralPos + 1);
  }
}

int initial_speed = 25;     // Starting speed for accuracy test
int speed_increment = 25;   // Speed increment per round

void testSimpleLinesAccuracy(int round) {
  int current_speed = initial_speed + speed_increment * round; // Speed for this round

  baseStepper.setMaxSpeed(current_speed);
  lowerArmStepper.setMaxSpeed(current_speed);
  upperArmStepper.setMaxSpeed(current_speed);

  Serial.print("Round: ");
  Serial.print(round + 1);
  Serial.print(" | Speed: ");
  Serial.println(current_speed);

  // base drawing motion
  int reps = 5; // Number of repetitions per round
  for (int j = 0; j < reps; j++) {
    Serial.print("Repetition: ");
    Serial.println(j + 1);
    lowerArmStepper.moveTo(lowerArmNeutralPos - 24);
    lowerArmStepper.runToPosition();

    baseStepper.moveTo(baseNeutralPos + 80);
    baseStepper.runToPosition();

    lowerArmStepper.moveTo(lowerArmNeutralPos);
    lowerArmStepper.runToPosition();

    baseStepper.moveTo(baseNeutralPos);
    baseStepper.runToPosition();
  }
}

// Predefined motion for weight test (executed step-by-step)
void testWithWeight() {
  upperArmStepper.moveTo(220); upperArmStepper.runToPosition();
  upperArmStepper.moveTo(150); upperArmStepper.runToPosition();
  lowerArmStepper.moveTo(-113); lowerArmStepper.runToPosition();
  lowerArmStepper.moveTo(0); lowerArmStepper.runToPosition();
  upperArmStepper.moveTo(220); upperArmStepper.runToPosition();
  upperArmStepper.moveTo(0); upperArmStepper.runToPosition();
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
  resetButton.loop(); // Poll new button

  // Button 1: Capture neutral positions
  if (joy1Button.isPressed()) {
    Serial.println("Joystick 1 Button Pressed - Capturing Neutral Positions");
    baseNeutralPos = baseStepper.currentPosition();
    lowerArmNeutralPos = lowerArmStepper.currentPosition();
    upperArmNeutralPos = upperArmStepper.currentPosition();
  }
  
  // Run the prerecorded motion with increasing speed when resetButton is pressed
  /*
  if (resetButton.isPressed()) {
    Serial.println("Reset Button Pressed - Running Predefined Motion with Increasing Speed");
    testSimpleLinesAccuracy(r); // Run the test with 3 rounds
    r += 1; // Increment round for next test
  }
  */
  ///*
  if (resetButton.isPressed()) {
    testWithWeight();
  }
  //*/

  // Move motors back to captured neutral positions when joy2Button is pressed
  if (joy2Button.isPressed()) {
    Serial.println("Joystick 2 Button Pressed - Returning to Neutral Positions");
    baseStepper.moveTo(baseNeutralPos);
    lowerArmStepper.moveTo(lowerArmNeutralPos);
    upperArmStepper.moveTo(upperArmNeutralPos);
  }

  // Steppers need to be run continuously for non-blocking movement
  baseStepper.run();
  lowerArmStepper.run();
  upperArmStepper.run();

  // non-blocking delay for debug output
  static unsigned long lastTime = 0; // Last time debug was printed
  unsigned long currentTime = millis();
  if (debug && currentTime - lastTime >= 300) { // 300 ms delay
    lastTime = currentTime;

    Serial.print("Base Position: ");
    Serial.print(baseStepper.currentPosition());
    Serial.print(" | Lower Arm Position: ");
    Serial.print(lowerArmStepper.currentPosition());
    Serial.print(" | Upper Arm Position: ");
    Serial.println(upperArmStepper.currentPosition());
  }
}
