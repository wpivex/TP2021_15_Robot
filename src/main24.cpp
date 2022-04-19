// CODE FOR 24" ROBOT
#include "AI24.cpp"

Robot24 twentyFour = Robot24();

int mainTeleop24() {
  twentyFour.setBackClamp(false);
  twentyFour.setFrontClamp(false);

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

int matchAuto() {
  int matchStartTime = timer::system();
  Goal allianceColor = RED;
  twentyFour.setBrakeType(hold);

  // ~~~~~~~~~~~~~ Box Rush Right ~~~~~~~~~~~~~~~
  twentyFour.openClaw();
  // Drive forwards at full speed (while adjusting towards goal if needed)
  twentyFour.setArmDegrees(5, 50, false);
  twentyFour.goForwardUntilSensor(36, 100, 3, 5);
  twentyFour.closeClaw();
  wait(200, msec);
  // Raise arm a bit (so that other team cannot grab it)
  twentyFour.setArmDegrees(215);
  twentyFour.goFightBackwards(1.3);
  // RETREAT
  // twentyFour.goForward(-8, 100, 0, 5);

  // // ~~~~~~~~~~~ Middle Goal Check ~~~~~~~~~~~~~~
  //robot.cursedTurn(150,70);

  twentyFour.goTurnU(120);
  twentyFour.setBackClamp(true);
  twentyFour.goVision(50, 65, YELLOW, reverse, 0, 0);
  twentyFour.setBackClamp(false);
  wait(200, msec);

  twentyFour.goForward(40, 100, 5, 0, 5, {}, false);
  twentyFour.goTurnU(270);
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
  twentyFour.goForwardU(24, 100, 0, 20, 20);
  return 0;
}

void autonomous24() { twentyFour.setBrakeType(hold); task auto1(testForward); }
void userControl24(void) { twentyFour.setBrakeType(coast); task controlLoop1(mainTeleop24); }


int main24() {

  twentyFour.setBackClamp(false);
  twentyFour.setFrontClamp(false);

  twentyFour.calibrateGyroBlocking();
  twentyFour.resetArmRotation();

  Competition.autonomous(autonomous24);
  Competition.drivercontrol(userControl24);

  while (true) {
    wait(20, msec);
  }

}
