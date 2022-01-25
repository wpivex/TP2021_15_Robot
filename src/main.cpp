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
  fifteen.setFrontClamp(false);
  fifteen.setTransmission(false);
  fifteen.driveStraight(100, 20);
  // fifteen.driveCurved(forward, 54, 30);
  fifteen.setFrontClamp(true);
  // fifteen.setTransmission(true);
  // fifteen.driveCurved(reverse, 60, 15);
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
