// CODE FOR 24" ROBOT
#include "r24/AI24.cpp"

Robot24 twentyFour = Robot24();

int tickOdom() {
  while (true) {
    twentyFour.activeLocation();
    log("%f\n%f\n%f\n%f\n%f\n%f",twentyFour.recordedL,twentyFour.recordedR,twentyFour.absoluteX,twentyFour.absoluteY,twentyFour.gyroSensor.heading(),twentyFour.recordedTheta); 
    wait(20, msec);
  }
  return 0;
}

int mainTeleop24() {
  twentyFour.setBackClamp(false);
  twentyFour.setFrontClamp(false);
  task odom(tickOdom);

  while (true) {
    twentyFour.teleop();
    wait(20, msec);
  }
  return 0;
}

/*
void middleFirst(void) {
  int color = 1; //red is 1, blue is 2
  robot.setBackClamp(true);
  robot.driveCurved(reverse, 20, 55);
  wait(2000, msec);
  robot.goForwardVision(true, 100, 30, 25, 0);
  robot.driveStraight(100, -10);
  robot.setBackClamp(false);
  
  //Go back 2 ft
  robot.driveStraight(100, 24);
  robot.blindAndVisionTurn(80, 0);
  
  robot.setFrontClamp(true);
  robot.goForwardVision(false, 100, 24, 40, 0);
  robot.setFrontClamp(false);
  wait(250, msec);

  //Go back 2 ft
  robot.driveStraight(100, -15);  
  robot.intakeOverGoal(color);
}
*/

int raiseArmFunc() {
  twentyFour.setArmDegrees(215);
  return 0;
}

int matchAuto() {
  twentyFour.setArmBrakeType(hold);
  twentyFour.resetEncoderDistance();
  task odom(tickOdom);
  int matchStartTime = timer::system();
  Goal allianceColor = RED;
  twentyFour.setBrakeType(hold);

  // ~~~~~~~~~~~~~ Box Rush Right ~~~~~~~~~~~~~~~
  twentyFour.openClaw();
  // Drive forwards at full speed (while adjusting towards goal if needed)
  twentyFour.setArmDegrees(5, 50, false);
  twentyFour.goForwardUntilSensor(36, 20, 3, 5);
  twentyFour.closeClaw();
  wait(200, msec);
  // Raise arm a bit (so that other team cannot grab it)
  // twentyFour.setArmDegrees(215);
  task raiseArm(raiseArmFunc);
  twentyFour.goFightOdom(10);
  wait(3000, msec);

  // twentyFour.goForwardU(5, 100, twentyFour.getAngle(), 0, 5); // slow down to a stop after fighting backwards
  twentyFour.goToPoint(-2, 3, 100, true);

  // ~~~~~~~~~~~ Middle Goal Check ~~~~~~~~~~~~~~
  // twentyFour.goTurnU(120);
  twentyFour.setBackClamp(true);
  twentyFour.goVision(-40, 100, YELLOW, reverse, 0, 0);
  twentyFour.setBackClamp(false);
  wait(200, msec);

  twentyFour.goToPoint(0, 2, 100);

  twentyFour.goTurnU(275);
  twentyFour.goForwardTimed(3, -30);
  runAI(&twentyFour, PORT2, matchStartTime);

  return 0;
}

int testCurrent() {

  twentyFour.setMaxArmTorque(CURRENT::MID);
  twentyFour.setLeftVelocity(reverse, 100);
  twentyFour.setRightVelocity(reverse, 100);
  int startTime = timer::system();
  VisualGraph g(0, 3, 10, 200);
  int a = 0;
  while (!isTimeout(startTime, 2.5)) {
    
    float curr = (twentyFour.leftMotorA.current() + twentyFour.rightMotorA.current()) / 2;
    g.push(curr);

    a = (a + 1) % 10;
    if (a == 0) g.display();

    wait(20, msec);

  }
  twentyFour.stopLeft();
  twentyFour.stopRight();

  return 0;
}

int testTurn() {
  twentyFour.goTurnU(90);
  wait(1000, msec);
  twentyFour.goTurnU(0, 1, true, 10, 100);
  return 0;
}

int testForward() {
  twentyFour.goForwardU(24, 100,0, 20, 5, 5);
  return 0;
}

int testGoPoint() {
  twentyFour.goToPoint(-12, 0, 100);
  return 0;
}

void autonomous24() { twentyFour.setBrakeType(coast); task auto1(matchAuto); }
void userControl24(void) { twentyFour.setBrakeType(coast); task controlLoop1(mainTeleop24); }


int mainFunc() {

  twentyFour.setBackClamp(false);
  twentyFour.setFrontClamp(false);
  twentyFour.setArmBrakeType(coast);

  twentyFour.calibrateGyroBlocking();
  twentyFour.resetArmRotation();

  Competition.autonomous(autonomous24);
  Competition.drivercontrol(userControl24);

  while (true) {
    wait(20, msec);
  }

}
