#include "../include/robot.cpp"

competition Competition;
controller Controller1(controllerType::primary);

Robot* mainBotP;

int mainTeleop() {
  while(true) {
    mainBotP->teleop();
  }
  return 0;
}

void userControl(void) {
  task controlLoop1(mainTeleop);
}

void mainAuto(void) {
  for(int i=0; i<4; i++) {
    mainBotP->driveStraight(30, 24 + 14.5);
    mainBotP->turnToAngle(30, 90);
  }
}

int tetherAuto(void) {
  return 0;
}

void autonomous() {
  thread auto1(mainAuto);
}

int main() {
  Robot mainBot = Robot(&Controller1);
  mainBotP = &mainBot;
  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
