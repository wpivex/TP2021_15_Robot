#pragma once
#include "Buttons.h"
#include "constants.h"

Buttons::Buttons() {

  AXES[0] = &Controller1.Axis1;
  AXES[1] = &Controller1.Axis2;
  AXES[2] = &Controller1.Axis3;
  AXES[3] = &Controller1.Axis4;

}

inline float Buttons::axis(Axis a) {
  float pos = AXES[a]->position();

  // deadzone
  if (fabs(pos) <= 5) {
    return 0;
  }
  // normalize between 1-100, and cube to give more sensitivity to small inputs
  return pow(pos / 100.0, 3);
}

inline bool Buttons::pressing(Button b) {
  if (b == NONE || b == INVALID) return false;
  return getObject(b)->pressing();
}
inline bool Buttons::pressing(int index) {
  if (index == NONE || index == INVALID) return false;
  return pressing(static_cast<Button>(index));
}
inline bool Buttons::pressed(Button b) {
  if (b == NONE || b == INVALID) return false;
  return pressing(b) && !prevButtonState[b];
}
inline bool Buttons::released(Button b) {
  if (b == NONE || b == INVALID) return false;
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

const vex::controller::button* Buttons::getObject(Button b) {
  switch (b) {
    case LEFT:
      return &Controller1.ButtonLeft;
    case RIGHT:
      return &Controller1.ButtonRight;
    case UP:
      return &Controller1.ButtonUp;
    case DOWN:
      return &Controller1.ButtonDown;
    case X:
      return &Controller1.ButtonX;
    case Y:
      return &Controller1.ButtonY;
    case A:
      return &Controller1.ButtonA;
    case B:
      return &Controller1.ButtonB;
    case L1:
      return &Controller1.ButtonL1;
    case R1:
      return &Controller1.ButtonR1;
    case L2:
      return &Controller1.ButtonL2;
    case R2:
      return &Controller1.ButtonR2;
    default:
      return nullptr;
  }
}