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

Robot mainBot = Robot(&Controller1);

int mainTeleop() {
  while (true) {
    mainBot.teleop();
    wait(100, msec);
  }
  return 0;
}

void userControl(void) { task controlLoop1(mainTeleop); }

void mainAuto(void) {
  
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
