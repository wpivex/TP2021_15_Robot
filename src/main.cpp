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



// https://discord.com/channels/863826887067435029/869718734409957387/937578714324140053
void skillsNonClimbing() {

  fifteen.waitGyroCallibrate();  

  // only on old robot
  fifteen.setTransmission(true);
  fifteen.setFrontClamp(false);
  fifteen.setBackClamp(false);
  fifteen.clampArmsDown();

  // Start the robot facing towards the home goal on platform.
  wait(500, msec);

  // Pick up home goal
  fifteen.driveStraight(42, 70, reverse, 10, 5);
  wait(500, msec);
  fifteen.setFrontClamp(true);
  wait(2000, msec);

  // Orient to align to left yellow goal
  fifteen.driveCurved(10, 70, forward, 10, 5, 0.5);
  fifteen.raiseFrontArm(150, 70, true);
  fifteen.turnToAngleGyro(true, 60, 100, 20, 10);

  // Drive to left yellow
  fifteen.driveStraightGyro(15, 70, forward, 10, 5);
  fifteen.goForwardVision(YELLOW, 40, forward, 30, 10, &fifteen.limitSwitchFront);
  fifteen.setBackClamp(true);
  wait(500, msec);
  fifteen.raiseFrontArm(150, 70, true);

  // Go to opposite platform
  fifteen.turnToAngleGyro(true, 20, 70, 10, 10);
  fifteen.driveStraightGyro(50, 70, forward, 10, 10);


}


int main() {

  fifteen.gyroSensor.calibrate();
  
  // DRIVER SKILLS TRUE, OTHERWISE FALSE
  //fifteen.setTransmission(false);

  //Competition.autonomous(autonomous);
  //Competition.drivercontrol(userControl);

  skillsNonClimbing();

  while (true) {
    wait(100, msec);
  }
}
