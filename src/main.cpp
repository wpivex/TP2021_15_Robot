// INCLUDE RIGHT VERSION OF ROBOT (15 OR 24)
// #include "main15.cpp"
//#include "main24.cpp"
#include "testAutoSelector.cpp"
int main() {

  Competition.bStopAllTasksBetweenModes = true;
  //return mainFunc();
  //wouldBeCalledMainFunctionButIsJustForTestingAutonSelector();
  while (true) {
    wait(20, msec);
    logController("thing: %d", Controller1->ButtonA.pressing() ? 1 : 0);
  }
  return 0;


}