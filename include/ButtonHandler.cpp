#include "../include/ButtonHandler.h"

ButtonHandler::ButtonHandler(vex::controller* c) {

  rController = c;

}

inline bool ButtonHandler::currentlyPressing(Button b) {
  return get(b)->pressing();
}
inline bool ButtonHandler::currentlyPressing(int index) {
  return currentlyPressing(static_cast<Button>(index));
}
inline bool ButtonHandler::startPressed(Button b) {
  return currentlyPressing(b) && !prevButtonState[b];
}
inline bool ButtonHandler::startReleased(Button b) {
  return !currentlyPressing(b) && prevButtonState[b];
}

inline ButtonHandler::Button ButtonHandler::getButtonPressed() {

  for (int i = 0; i < NUM_BUTTONS; i++) {
    Button b = static_cast<Button>(i);
    if (startPressed(b)) {
      return b;
    }
  }
  return NONE;
}



// Run at every tick. Read all button inputs and update states
void ButtonHandler::updateButtonState() {

  for (int i = 0; i < NUM_BUTTONS; i++) {
    prevButtonState[i] = currentlyPressing(i); 
  }
}

vex::controller::button* ButtonHandler::get(Button b) {
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