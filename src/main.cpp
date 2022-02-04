// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision6              vision        6               
// DigitalInE           digital_in    E               
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "robot.cpp"

// CODE FOR 15" ROBOT

competition Competition;
controller Controller1(controllerType::primary);

Robot fifteen = Robot(&Controller1);

int mainTeleop() {
  //fifteen.setTransmission(true);
  while (true) {
    fifteen.teleop();
    wait(20, msec);
  }
  return 0;
}

void userControl(void) { task controlLoop1(mainTeleop); }


void mainAuto() {

  fifteen.waitGyroCallibrate();

}

void autonomous() { thread auto1(mainAuto); }

/*

// https://discord.com/channels/863826887067435029/869718734409957387/937578714324140053
void skillsNonClimbing() {

  fifteen.waitGyroCallibrate();  

  // Start the robot facing towards the home goal on platform.
  wait(500, msec);

  // Pick up home goal
  fifteen.driveStraight(70, 42);
  wait(500, msec);
  fifteen.setFrontClamp(true);
  wait(2000, msec);

  // Orient to align to left yellow goal
  fifteen.driveCurved(reverse, 10, -40, 70);
  fifteen.raiseFrontArm(150, 70, true);
  
  fifteen.gyroTurn(reverse, 90);

  // Drive to left yellow
  fifteen.driveStraight(70, -30);
  fifteen.setBackClamp(true);
  wait(500, msec);
  fifteen.raiseFrontArm(150, 70, true);

  // Go to opposite platform
  fifteen.gyroTurn(forward, 20);
  fifteen.driveStraight(70, -50);
  fifteen.gyroTurn(reverse, 30);

}
*/

int main() {

  fifteen.gyroSensor.calibrate();
  
  // DRIVER SKILLS TRUE, OTHERWISE FALSE
  //fifteen.setTransmission(false);

  //Competition.autonomous(autonomous);
  //Competition.drivercontrol(userControl);

  mainTeleop();

  while (true) {
    wait(100, msec);
  }
}
