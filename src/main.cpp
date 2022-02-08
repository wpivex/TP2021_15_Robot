// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision6              vision        6               
// DigitalInE           digital_in    E               
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "robot.cpp"

// CODE FOR 15" ROBOT

competition Competition;


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
  fifteen.setTransmission(false);

  fifteen.waitGyroCallibrate();
  // fifteen.alignToGoalVision(YELLOW, true, 1000000);
  fifteen.goForwardVision(YELLOW, 15, forward, 1000, 10000, &fifteen.limitSwitchFront);
  fifteen.turnToAngleGyro(true, 180, 30, 10, 10);
  // fifteen.setFrontClamp(true);
}




// https://discord.com/channels/863826887067435029/869718734409957387/937578714324140053
void skillsNonClimbing() {

  fifteen.waitGyroCallibrate();  

  // only on old robot
  fifteen.setTransmission(false);
  fifteen.setFrontClamp(false);
  fifteen.setBackClamp(false);
  fifteen.clampArmsDown();


  // Pick up home goal
  fifteen.driveStraight(8, 30, reverse, 10, 5);
  wait(500, msec);
  fifteen.setBackClamp(true);
  wait(1000, msec);

  // Orient to align to left yellow goal
  fifteen.driveCurved(10, 30, forward, 10, 5, 0.5);
  fifteen.raiseBackArm(150, 70, true);
  fifteen.turnToAngleGyro(true, 43, 20, 20, 10);

  wait(1000, msec);

  // Drive to left yellow
  //fifteen.driveStraightGyro(10, 20, forward, 10, 5);
  fifteen.goForwardVision(YELLOW, 25, forward, 45, 10, &fifteen.limitSwitchFront);
  fifteen.setFrontClamp(true);
  wait(300, msec);
  fifteen.raiseFrontArm(600, 70, true);

  // Go to opposite platform
  fifteen.turnToAngleGyro(true, 15, 20, 10, 10);
  wait(1000, msec);
  fifteen.driveStraightGyro(35, 20, forward, 10, 10);


}


void autonomous() { thread auto1(skillsNonClimbing); }


int main() {

  fifteen.gyroSensor.calibrate();
  
  // DRIVER SKILLS TRUE, OTHERWISE FALSE
  // fifteen.setTransmission(false);

  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  //skillsNonClimbing();

  while (true) {
    wait(100, msec);
  }
}
