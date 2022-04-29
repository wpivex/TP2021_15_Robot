// INCLUDE RIGHT VERSION OF ROBOT (15 OR 24)
// #include "main15.cpp"
//#include "main24.cpp"
#include "testAutoSelector.cpp"
//#include "constants.h"

using namespace vex;

int main() {

  Competition.bStopAllTasksBetweenModes = true;
  //Controller1->ButtonA.pressed(callback);
  //return mainFunc();
  wouldBeCalledMainFunctionButIsJustForTestingAutonSelector();
  while (true) {
    wait(20, msec);
    Controller1->Screen.clearScreen();
    Controller1->Screen.setCursor(1, 1);
    Controller1->Screen.print("fuck %d", Controller1->Axis2.value());
    
  }
  return 0;
}