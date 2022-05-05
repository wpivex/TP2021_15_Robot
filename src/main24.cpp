// CODE FOR 24" ROBOT
#include "r24/AI24.cpp"

Robot24 twentyFour = Robot24();

int tickOdom() {
  while (true) {
    twentyFour.activeLocation();
    log("Left Encoder: %f\nRight Encoder: %f\nAbsolute X: %f\nAbsolute Y: %f\nGyro Readout: %f\nPrevious Angle:%f",
      twentyFour.recordedL,twentyFour.recordedR,twentyFour.absoluteX,twentyFour.absoluteY,twentyFour.gyroSensor.heading(),twentyFour.recordedTheta); 
    wait(10, msec);
  }
  return 0;
}

int mainTeleop24() {
  twentyFour.setBackClamp(false);
  twentyFour.setFrontClamp(false);
  task odom(tickOdom);
  twentyFour.absoluteY = -9;

  while (true) {
    twentyFour.teleop();
    wait(20, msec);
  }
  return 0;
}


int middleFirst(void) {
  twentyFour.setArmBrakeType(hold);
  twentyFour.resetOdom();
  twentyFour.absoluteY = -9;
  task odom(tickOdom);
  twentyFour.setBrakeType(hold);
  twentyFour.setArmDegrees(500, 100);
  twentyFour.goForwardU(10, 100, twentyFour.gyroSensor.heading(), 5, 5);
  twentyFour.goTurnU(130, -1);

  // ~~~~~~~~~~~ Middle Goal Check ~~~~~~~~~~~~~~
  twentyFour.setBackClamp(true);
  twentyFour.goVisionUntilSensor(35, 100, twentyFour.frontSlideSensor, 3);
  twentyFour.setBackClamp(false);
  twentyFour.goForwardU(3, 50, twentyFour.gyroSensor.heading(), 0, 3);
  wait(250, msec);

  // disable below for gyro wack

  twentyFour.goForwardU(24, 100, twentyFour.gyroSensor.heading(), 5, 5);
  twentyFour.goTurnU(300);
  return 0;
}


int matchAuto() {
  twentyFour.setArmBrakeType(hold);
  twentyFour.resetOdom();
  twentyFour.absoluteY = -9;
  task odom(tickOdom);
  twentyFour.setBrakeType(hold);
  int matchStartTime = timer::system();
  Goal allianceColor = RED;

  // ~~~~~~~~~~~~~ Box Rush Right ~~~~~~~~~~~~~~~
  twentyFour.openClaw();
  // Drive forwards at full speed (while adjusting towards goal if needed)
  twentyFour.setArmDegrees(30, 100, false);
  twentyFour.goForwardUntilSensor(35, 100, twentyFour.clawSensor, 0);
  twentyFour.closeClaw();
  // twentyFour.goForwardU(3, 100, twentyFour.getAngle(), 0, 3); // slow down to a stop
  twentyFour.setArmDegrees(100, 100, false); // raise arm concurrently, just enough to clear ground
  twentyFour.goFightOdom(10, 3); // 25 for AI, 10 normally
  wait(250, msec);
  twentyFour.setArmDegrees(500, 100);

  // delete past this point for no middle and AI

  // twentyFour.setMaxArmTorque(CURRENT::OFF);
  // twentyFour.goTurnU(135);

  // // ~~~~~~~~~~~ Middle Goal Check ~~~~~~~~~~~~~~
  // twentyFour.setBackClamp(true);
  // twentyFour.goVisionUntilSensor(35, 100, twentyFour.frontSlideSensor, 3);
  // twentyFour.stopLeft();
  // twentyFour.stopRight();
  // twentyFour.setBackClamp(false);
  // twentyFour.goForwardU(-3, 100, twentyFour.getAngle(), 0, 3); // slow down to a stop

  // // disable below for gyro wack

  // twentyFour.gotToY(31, 100);
  // twentyFour.goTurnU(270);

  // runAI(&twentyFour, PORT19, matchStartTime);

  // twentyFour.goTurnU(45);
  // twentyFour.goAlignVision(allianceColor);
  // twentyFour.goTurnU(fmod((twentyFour.gyroSensor.heading() + 180), 360.0));
  // twentyFour.setFrontClamp(true);


  // twentyFour.goForwardTimed(2, 50);
  // twentyFour.setFrontClamp(false);
  // twentyFour.goForwardU(2, 50, twentyFour.gyroSensor.heading(), 5, 2);

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
  twentyFour.setArmDegrees(500);
  twentyFour.goTurnU(90);
  wait(2000, msec);
  twentyFour.goTurnU(180);
  wait(2000, msec);
  twentyFour.goTurnU(0);
  return 0;
}

int testStrafe() {
  twentyFour.testStrafePID();
  log("Done");
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
