#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <Arduino.h>
#include <ezButton.h>

class Joystick {
  int pinX, pinY;
  int deadzone;
  bool hasButton;
  ezButton *button = nullptr;

public:
  Joystick(int xPin, int yPin, int dz = 30)
      : pinX(xPin), pinY(yPin), deadzone(dz), hasButton(false) {}

  Joystick(int xPin, int yPin, int btnPin, int dz = 30)
      : pinX(xPin), pinY(yPin), deadzone(dz), hasButton(true) {
    button = new ezButton(btnPin);
    button->setDebounceTime(50);
  }

  void begin() {
    if (hasButton) {
      button->loop();  // pre-initialize
    }
  }

  void update() {
    if (hasButton) {
      button->loop();
    }
  }

  int getX() {
    int val = analogRead(pinX) - 512;
    return (abs(val) > deadzone) ? val : 0;
  }

  int getY() {
    int val = analogRead(pinY) - 512;
    return (abs(val) > deadzone) ? val : 0;
  }

  bool isPressed() {
    return hasButton ? button->isPressed() : false;
  }

  bool isReleased() {
    return hasButton ? button->isReleased() : false;
  }

  bool getState() {
    return hasButton ? button->getState() == LOW : false;
  }

  ~Joystick() {
    if (button != nullptr) delete button;
  }
};

#endif