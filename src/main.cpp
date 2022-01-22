// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision6              vision        6               
// DigitalInE           digital_in    E               
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "../include/robot.cpp"

// CODE FOR 15" ROBOT

competition Competition;
controller Controller1(controllerType::primary);

Robot fifteen = Robot(&Controller1);

int mainTeleop() {
  while (true) {
    fifteen.teleop();
    wait(100, msec);
  }
  return 0;
}

void userControl(void) { task controlLoop1(mainTeleop); }

void mainAuto(void) {
  //Potentially fifteen.raiseLeftLift(); to get ring on team goal

  fifteen.driveCurved(forward, 54/*approx two squares, plus a little to the right*/, 15);
  //fifteen.grabGoal(); //?

  //If not using raiseLeftList, do this instead:
  fifteen.driveCurved(reverse, 54, 15);
  //Something something get ring on goal
}

int tetherAuto(void) { return 0; }

void autonomous() { thread auto1(mainAuto); }


int main() {


  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  while (true) {
    wait(100, msec);
  }
}
