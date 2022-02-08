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


float torqueMultiplier = 2.77777777;

// https://discord.com/channels/863826887067435029/869718734409957387/937578714324140053
void skillsNonClimbing() {

  fifteen.waitGyroCallibrate();  

  // only on old robot
  fifteen.setTransmission(true);
  fifteen.setFrontClamp(false);
  fifteen.setBackClamp(false);
  fifteen.clampArmsDown();


  // Pick up home goal
  fifteen.driveStraight(15*torqueMultiplier, 50, reverse, 10*torqueMultiplier, 5);
  fifteen.setBackClamp(true);

  // Orient to align to left yellow goal
  fifteen.driveCurved(10*torqueMultiplier, 100, forward, 10, 5, 0.5);
  fifteen.raiseBackArm(150, 70, true);
  fifteen.turnToAngleGyro(true, 43, 20, 20, 10);

  wait(1000, msec);

  // Drive to left yellow
  //fifteen.driveStraightGyro(10, 20, forward, 10, 5);
  fifteen.goForwardVision(YELLOW, 100, forward, 55*torqueMultiplier, 10*torqueMultiplier, &fifteen.limitSwitchFront);
  fifteen.driveStraight(3*torqueMultiplier, 20, forward, 10, -1);
  fifteen.setFrontClamp(true);
  wait(1000, msec);
  fifteen.raiseFrontArm(600, 70, true);

  // Go to opposite platform
  fifteen.turnToAngleGyro(true, 20, 20, 10, 10);
  wait(500, msec);
  fifteen.driveStraightGyro(35*torqueMultiplier, 100, forward, 10, 10);
  // Place neutral goal
  fifteen.setFrontClamp(false);

  // Lower arm while driving back
  fifteen.raiseFrontArm(15, 70, false);
  fifteen.driveStraight(20, 100, reverse, 5, 10);
  //Lower back goal
  // fifteen.setBackLift(false);
  // fifteen.setBackClamp(false);
  //Drive forward to get back goal off
  fifteen.driveStraight(5,50,forward,10,0,true);
  //Turn 180 to pick up goal
  fifteen.turnToAngleGyro(true, 180, 20, 10, 10);
  
  wait(500, msec);
  fifteen.goForwardVision(BLUE, 50, forward, 8*torqueMultiplier, 10, &fifteen.limitSwitchFront);
  fifteen.setFrontClamp(true);
  
  wait(500, msec);
  // Turn and drive back torwards goal
  // raise arm while turning
  fifteen.raiseFrontArm(600, 70, false);
  fifteen.turnToAngleGyro(true, 180, 20, 10, 10);
  
  wait(500, msec);
  //Drive up to platform and drop goal
  fifteen.driveStraightGyro(20*torqueMultiplier, 75, forward, 10, 10);
  fifteen.setFrontClamp(false);
  
  //Turn 90 degrees, torwards red goal, drive and grab it
  fifteen.turnToAngleGyro(true, 43, 20, 20, 10);
  fifteen.goForwardVision(RED, 50, forward, 8*torqueMultiplier, 10, &fifteen.limitSwitchFront);
  fifteen.setFrontClamp(true);

  //turn to face red side
  fifteen.turnToAngleGyro(true, 43, 20, 20, 10);
  wait(500,msec);

  fifteen.driveStraight(70*torqueMultiplier, 100, forward, 10, -1);
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
