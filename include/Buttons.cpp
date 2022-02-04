#pragma once
#include "Buttons.h"

Buttons::Buttons(vex::controller* c) {

  rController = c;

}

inline bool Buttons::pressing(Button b) {
  return getObject(b)->pressing();
}
inline bool Buttons::pressing(int index) {
  return pressing(static_cast<Button>(index));
}
inline bool Buttons::pressed(Button b) {
  return pressing(b) && !prevButtonState[b];
}
inline bool Buttons::released(Button b) {
  return !pressing(b) && prevButtonState[b];
}

inline Buttons::Button Buttons::get() {

  for (int i = 0; i < NUM_BUTTONS; i++) {
    Button b = static_cast<Button>(i);
    if (pressed(b)) {
      return b;
    }
  }
  return NONE;
}



// Run at every tick. Read all button inputs and update states
void Buttons::updateButtonState() {

  for (int i = 0; i < NUM_BUTTONS; i++) {
    prevButtonState[i] = pressing(i); 
  }
}

vex::controller::button* Buttons::getObject(Button b) {
  switch (b) {
    case LEFT:
      return &rController->ButtonLeft;
    case RIGHT:
      return &rController->ButtonRight;
    case UP:
      return &rController->ButtonUp;
    case DOWN:
      return &rController->ButtonDown;
    case X:
      return &rController->ButtonX;
    case Y:
      return &rController->ButtonY;
    case A:
      return &rController->ButtonA;
    case B:
      return &rController->ButtonB;
    case L1:
      return &rController->ButtonL1;
    case R1:
      return &rController->ButtonR1;
    case L2:
      return &rController->ButtonL2;
    case R2:
      return &rController->ButtonR2;
    default:
      return nullptr;
  }
}