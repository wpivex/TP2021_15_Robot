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
    logController("teat %d", Controller1.Axis2.position());
  }
}