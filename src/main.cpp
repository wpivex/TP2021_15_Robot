// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision6              vision        6               
// DigitalInE           digital_in    E               
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "robot.cpp"
#include <string>
#include <sstream>

// CODE FOR 15" ROBOT


Robot fifteen = Robot(&Controller1);

int mainTeleop() {
  fifteen.setSecondaryCurrent(true);
  //fifteen.setTransmission(true);
  while (true) {
    fifteen.teleop();
    wait(20, msec);
  }
  return 0;
}

int leftAuto() {

  AI_Direction = -1;

  int matchStartTime = timer::system();

  float lowArmAngle = -20;
  float highArmAngle = 680;
  int rampUp = 15;

  fifteen.setBrakeType(hold);
  fifteen.setSecondaryCurrent(false); // disable non-drive motors for box rush

  // Initial go rush
  fifteen.clawUp();
  float ang = fifteen.getAngle();
  fifteen.goForwardU(35.5, 100, ang, 0, 0, false);
  fifteen.clawDown(); // start claw down motion early
  fifteen.goForwardU(0.5, 100, ang, 0, 0, false);
  //float slowDown = 2; fifteen.goForwardU(slowDown, 100, ang, 0, slowDown, true, -1, 50);
  fifteen.goFightBackwards();
  fifteen.setSecondaryCurrent(true);

  // Get back to wall align but avoiding platform
  fifteen.moveArmTo(200, 100, false);
  fifteen.goTurnU(50);
  fifteen.goForwardU(-10, 70, 50, rampUp, 7, true, 20, 20, 2);
  fifteen.goTurnU(0);
  fifteen.goForwardU(-13, 70, 0, rampUp, 1, true, 20, 35, 2);
  fifteen.moveArmTo(highArmAngle, 100, false);
  fifteen.goForwardTimed(1, -35); // wall align back
  //fifteen.gyroSensor.setHeading(0, deg);

  // Align with left wall
  fifteen.goForwardU(4, 40, 0, 0, 0);
  wait(150, msec);
  fifteen.goTurnU(270);
  fifteen.goForwardU(8, 50, 270, rampUp, 5, false, 20, 30, 2);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, false);
  fifteen.goForwardTimed(1.3, 30);

  // Get alliance goal
  
  fifteen.goForwardU(-20, 70, 270, rampUp, 5, false, 20, 40, 3);
  fifteen.goForwardU(-9, 40, 270, 0, 0, true, 10, 20, 1.5);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);

  // do match load rings
  fifteen.startIntake();
  fifteen.goForwardU(30, 35, 270, rampUp, 5, true, 20, 15, 3);
  fifteen.backDown();
  fifteen.goForwardU(-17, 40, 270, rampUp, 5, true, 20, 15, 2.5); // go three passes to pick up rings
  fifteen.goForwardU(16, 35, 270, rampUp, 0, false);
  fifteen.goForwardTimed(0.7, 30);

  // Get into AI strafe position
  fifteen.goForwardU(-1.5, 30, 270, 0, 0, false);
  fifteen.goCurve(-10, 50, 0.365, rampUp, 0, false);
  fifteen.clawUp();
  fifteen.moveArmTo(200, 100, false);
  fifteen.goCurve(-15.5, 50, 0.365, 0, 0, false);
  fifteen.goForward(-7.5, 50, 0, 3, true);
  fifteen.goTurnU(270);

  fifteen.runAI(matchStartTime);

  return 0;
}

int rightAuto() {

  AI_Direction = 1;

  int matchStartTime = timer::system();

  float lowArmAngle = -20;
  float highArmAngle = 560;

  fifteen.setBrakeType(hold);

  // Initial go rush
  float ang = fifteen.getAngle();
  fifteen.clawUp();
  fifteen.goForwardU(44, 100, ang, 0, 3, false, 40, 75);
  fifteen.clawDown(); // start claw down motion early
  fifteen.goForwardU(3, 75, ang, 0, 3, true, 20, 50);
  fifteen.goFightBackwards();

  // Get back to wall align but avoiding platform
  fifteen.moveArmTo(200, 100, false);
  fifteen.goForwardU(-23, 70, 350, 3, 1, false, 20, 35, 3);
  fifteen.moveArmTo(highArmAngle, 100, false);
  fifteen.goForwardTimed(1, -35); // wall align back
  //fifteen.gyroSensor.setHeading(0, deg);

  // Get alliance goal  
  fifteen.goForwardU(30, 80, 355, 5, 10);
  wait(150, msec);
  fifteen.goTurnU(270);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, true);

  fifteen.goForwardU(-16, 40, 270, 5, 4, true);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);
  fifteen.backDown();
  fifteen.goForwardU(5, 40, 270, 1, 1);
  fifteen.goTurnU(180);

  // do match load rings
  fifteen.startIntake();
  fifteen.goForwardU(20, 30, 180, 2, 0, false);
  fifteen.goForwardTimed(1, 30);
  fifteen.goForwardU(-27, 60, 180, 4, 10, true, 20, 15, 2.5); // go three passes to pick up rings
  fifteen.goForwardU(24, 30, 180, 2, 0, false);
  fifteen.goForwardTimed(0.5, 30);

  // Get into AI strafe position
  fifteen.goForwardU(-12, 80, 180, 3, 0, false);
  fifteen.stopIntake();
  fifteen.clawUp();
  fifteen.moveArmTo(200, 100, false);
  fifteen.goForwardU(-14, 80, 180, 0, 7, true);
  fifteen.goTurnU(270);

  fifteen.runAI(matchStartTime);

  return 0;
}

int midAuto() {

  AI_Direction = 1;

  int matchStartTime = timer::system();

  float lowArmAngle = -20;
  float highArmAngle = 560;

  fifteen.setBrakeType(hold);

  // Initial go rush
  float ang = fifteen.getAngle();
  fifteen.clawUp();
  fifteen.goForwardU(55, 100, ang, 0, 3, false, 40, 75);
  fifteen.clawDown(); // start claw down motion early
  fifteen.goForwardU(3, 75, ang, 0, 3, true, 20, 50);
  fifteen.goFightBackwards();

  // Wall aligns to localize
  fifteen.goTurnU(270);
  fifteen.goForwardU(-15, 60, 270, 3, 4, false, 20, 30, 3);
  fifteen.moveArmTo(highArmAngle, 100);
  fifteen.goForwardTimed(2, -30);
  fifteen.goForwardU(28, 60, 270, 3, 8);
  fifteen.goTurnU(180);
  fifteen.goForwardU(15, 60, 180, 3, 4, false, 20, 30, 3);
  fifteen.goForwardTimed(2, 30);

  // Get alliance goal  
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, false);
  fifteen.goForwardU(-3, 30, 180, 1, 1);
  fifteen.goTurnU(225);
  fifteen.goForwardU(24, 60, 225, 3, 10);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);
  fifteen.backDown();
  fifteen.goForwardU(5, 40, 270, 1, 1);
  fifteen.goTurnU(180);

  // do match load rings
  fifteen.startIntake();
  fifteen.goForwardU(20, 30, 180, 2, 0, false);
  fifteen.goForwardTimed(1, 30);
  fifteen.goForwardU(-27, 60, 180, 4, 10, true, 20, 15, 2.5); // go three passes to pick up rings
  fifteen.goForwardU(24, 30, 180, 2, 0, false);
  fifteen.goForwardTimed(0.5, 30);

  // Get into AI strafe position
  fifteen.goForwardU(-12, 80, 180, 3, 0, false);
  fifteen.stopIntake();
  fifteen.clawUp();
  fifteen.moveArmTo(200, 100, false);
  fifteen.goForwardU(-14, 80, 180, 0, 7, true);
  fifteen.goTurnU(270);

  fifteen.runAI(matchStartTime);

  return 0;
}

int midSimpleAuto() {

  int matchStartTime = timer::system();

  float lowArmAngle = -20;
  float highArmAngle = 560;

  fifteen.setBrakeType(hold);

  // Initial go rush
  float ang = fifteen.getAngle();
  fifteen.clawUp();
  fifteen.goForwardU(50, 100, ang, 0, 3, false, 40, 75);
  fifteen.clawDown(); // start claw down motion early
  fifteen.goForwardU(3, 75, ang, 0, 3, true, 20, 50);
  fifteen.goFightBackwards();
  fifteen.goCurve(-20, 30, 0.3, 3, 5);

  return 0;
}

int test() {
  fifteen.clawUp();
  wait(1000, msec);
  fifteen.clawDown();
  wait(200,msec);
  fifteen.goFightBackwards();
  return 0;
}


void autonomous() { fifteen.setBrakeType(hold); task auto1(leftAuto); }
//void autonomous() { fifteen.setBrakeType(hold); task auto1(rightAuto); }
//void autonomous() { fifteen.setBrakeType(hold); task auto1(midAuto); }
//void autonomous() { fifteen.setBrakeType(hold); task auto1(midSimpleAuto); }
//void autonomous() { fifteen.setBrakeType(hold); task auto1(test); }

void userControl(void) { fifteen.setBrakeType(coast); task controlLoop1(mainTeleop); }
//void userControl(void) { task controlLoop1(logDistance); }


int main() {

  fifteen.backUp();
  fifteen.clawDown();

  // If no competition switch, immediately start teleop
  if (!Competition.isCompetitionSwitch()) {
    userControl();
    return 0;
  }

  wait(500, msec);
  fifteen.gyroSensor.calibrate();
  fifteen.waitGyroCallibrate();
  logController("yes");

  Competition.bStopAllTasksBetweenModes = true;

  fifteen.resetEncoderDistance();
  
  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  //platformClimb2();


  while (true) {
    wait(20, msec);
  }

}