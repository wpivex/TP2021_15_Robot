#include "vex.h"
#include <functional>
#include <vector>
#include <string>
#include "../../include/Buttons.cpp"
#include "constants.h"

using namespace vex;

const int BUTTON_FRAMES_NEEDED = 75;

static const char* getName(BTN::Button b) {
  switch (b) {
    case BTN::LEFT:
      return "Left";
    case BTN::RIGHT:
      return "Right";
    case BTN::UP:
      return "Up";
    case BTN::DOWN:
      return "Down";
    case BTN::X:
      return "X";
    case BTN::Y:
      return "Y";
    case BTN::A:
      return "A";
    case BTN::B:
      return "B";
    case BTN::L1:
      return "L1";
    case BTN::R1:
      return "R1";
    case BTN::L2:
      return "L2";
    case BTN::R2:
      return "R2";
    default:
      return "invalid";
  }
}

class Auton {

  public:

  Auton(int (*func)(), const char* programName, BTN::Button hotkey) {
    function = func;
    name = programName;
    name += " (";
    name += getName(key);
    name += ")";
    key = hotkey;
  }

  int (*function)();
  std::string name;
  BTN::Button key;
};

class AutonSelector {

public:
  
  void setAuton(int (*func)(), const char* name, BTN::Button key);
  int runSelectedAuton();

  void tick();

private:

  void drawButtons();

  std::vector<Auton> autons;
  int selectedAuton = 0;
  int lengthHeld = 0;
  int heldAuton = -1;

};