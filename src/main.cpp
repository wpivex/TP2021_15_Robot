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

int displayTime() {
  Brain.resetTimer();

  while (true) {
    logController("Time: %f", Brain.timer(sec));
    wait(100, msec);
  }
}

int mainTeleop() {
  fifteen.setSecondaryCurrent(true);
  //fifteen.setTransmission(true);
  while (true) {
    fifteen.teleop();
    wait(20, msec);
  }
  return 0;
}

int boxRushNoGyro() {

  fifteen.setBrakeType(hold);
  fifteen.setSecondaryCurrent(false); // disable non-drive motors for box rush
  
   // Initial go rush
  fifteen.clawUp();
  fifteen.goForwardU(37.5, 100, -1, 0, 0, false);
  fifteen.clawDown(); // start claw down motion early
  fifteen.goForwardU(0.5, 100, -1, 0, 0, false);
  float slowDown = 0.75; fifteen.goForwardU(slowDown, 100, -1, 0, slowDown, true, -1, 30);
  fifteen.goFightBackwards();
  fifteen.setSecondaryCurrent(true);
  fifteen.goCurve(-15, 50, -0.3, 10, 5);

  return 0;
}

bool boxRush() {
  float lowArmAngle = -20;
  float highArmAngle = 680;
  int rampUp = 15;

  fifteen.setBrakeType(hold);
  fifteen.setSecondaryCurrent(false); // disable non-drive motors for box rush

  // Initial go rush
  fifteen.clawUp();
  float ang = fifteen.getAngle();
  fifteen.goForwardU(37.5, 100, ang, 0, 0, false);
  fifteen.clawDown(); // start claw down motion early
  fifteen.goForwardU(0.5, 100, ang, 0, 0, false);
  float slowDown = 0.75; fifteen.goForwardU(slowDown, 100, ang, 0, slowDown, true, -1, 30);
  fifteen.goFightBackwards();
  fifteen.setSecondaryCurrent(true);

  // handle gyro disconnect failsafe
  if (!fifteen.gyroSensor.installed()) {
    fifteen.goCurve(-15, 50, -0.3, 10, 5);
    return false;
  }

  // Get back to wall align but avoiding platform
  fifteen.moveArmTo(200, 100, false);
  fifteen.goTurnU(50);
  fifteen.goForwardU(-12, 70, 50, rampUp, 7, true, 20, 20, 2);
  fifteen.goTurnU(0);
  fifteen.goForwardU(-12, 70, 0, rampUp, 1, true, 20, 35, 2);
  fifteen.goForwardTimed(1, -35); // wall align back
  return true;
}

int leftAuto() {

  AI_Direction = -1;

  int matchStartTime = timer::system();

  float lowArmAngle = -20;
  float highArmAngle = 680;
  int rampUp = 15;

  if (!boxRush()) return 0;

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
  fifteen.goForwardU(30, 25, 270, rampUp, 5, true, 20, 15, 3);
  fifteen.backDown();
  fifteen.goForwardU(-17, 40, 265, rampUp, 5, true, 20, 15, 2.5); // go three passes to pick up rings
  fifteen.goForwardU(16, 25, 270, rampUp, 0, false);
  fifteen.goForwardTimed(0.7, 30);

  // Get into AI strafe position
  fifteen.goForwardU(-1.5, 30, 270, 0, 0, false);
  fifteen.goCurve(-10, 50, 0.365, rampUp, 0, false);
  fifteen.clawUp();
  fifteen.moveArmTo(200, 100, false);
  fifteen.goCurve(-15.5, 50, 0.365, 0, 0, false);
  fifteen.goForward(-5, 50, 0, 3, true);
  fifteen.goTurnU(270);

  fifteen.runAI(matchStartTime);

  return 0;
}

// int unclogRings() {
//   fifteen.startIntake(reverse);
//   wait(300, msec);
//   fifteen.stopIntake();
//   return 0;
// }

int twoRingAuton() {

  int matchStartTime = timer::system();
  float highArmAngle = 680;
  float lowArmAngle = -20;
  float fwdMinSpeed = 18;
  int rampUp = 15;
  fifteen.setBrakeType(hold);

  if (!boxRush()) return 0;

  //fifteen.moveArmTo(600, 100);
  bool obtainedGoal = fifteen.moveArmToManual(600, 100);

  // Align with left wall
  fifteen.goForwardU(4.5, 40, 0, 0, 0);
  wait(150, msec);
  fifteen.goTurnU(270);
  fifteen.goForwardU(8, 70, 270, rampUp, 5, false, 20, 30, 2);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, false);
  fifteen.goForwardTimed(1.3, 30);

  // Get alliance goal
  
  fifteen.goForwardU(-20, 70, 270, rampUp, 5, false, 20, 40, 3);
  fifteen.goForwardU(-6, 50, 270, 0, 0, true, 10, 20, 1.5);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);

  // do match load rings
  fifteen.startIntake();
  fifteen.goForwardU(28.5, 30, 270, rampUp, 5, true, 20, 15, 2.75);
  fifteen.backDown();
  fifteen.goForwardU(-20, 50, 270, rampUp, 5, true, 20, 15, 2.5); // go two passes to pick up rings
  fifteen.goForwardU(16, 30, 270, rampUp, 0, false);
  fifteen.goForwardTimed(0.5, 25);

  // exit early and do not go for second goal if past amt. of seconds
  if (false/*isTimeout(matchStartTime, obtainedGoal ? 32 : 25)*/) {
    logController("exit early");
    fifteen.goForwardU(-10, 60, 270, rampUp, 5);
    fifteen.goTurnU(0);
    return 0;
    }


  /*
  Robot needs to get to other alliance goal through platform. Behavior is dependent on whether yellow goal was obtained.
  If yellow goal obtained, turn 180 to face platform, release alliance goal from 1dof, climb
  Otherwise, back up, release alliance goal from 1dof, turn 180 to face platform, pick up alliance goal with front arm, climb
  */

  if (obtainedGoal) {

    fifteen.goForwardU(-14, 60, 270, rampUp, 5);
    fifteen.goTurnU(90, -1); // face platform
    fifteen.backUp();
    fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, true);
    fifteen.goForwardU(6, 50, 90, rampUp, 3);
    fifteen.setBackLift(fifteen.BACK_LIFT_UP, false);

  } else {

    // Exchange blue goal from back to front
    fifteen.backUp();
    fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, false); // drop alliance goal
    fifteen.goForwardU(-4.5, 70, 270, rampUp, 0, false);
    fifteen.moveArmTo(lowArmAngle, 100, false); // lower arm to pick up alliance goal
    fifteen.goForwardU(-5, 70, 270, 0, 4);
    fifteen.goForwardU(5.5, 70, 270, rampUp, 5);
    fifteen.setBackLift(fifteen.BACK_LIFT_UP, false);
    wait(500, msec); // wait for back lift to fold sufficiently
    fifteen.goTurnU(90, 1);
    fifteen.clawUp();
    fifteen.goForwardU(12, 70, 90, rampUp, 6);
    fifteen.clawDown();
    wait(200, msec);

    // Get to platfom ready position
    fifteen.moveArmTo(500, 100, false);
    fifteen.goForwardU(-2.5, 50, 90, 3, 1);
    fifteen.moveArmTo(500, 100, true);
    fifteen.goForwardU(7.5, 40, 90, rampUp, 3);
  }
  // At this point, back wheel is aligned between tiles, ready to lower arm and climb
  float startPitch = fifteen.gyroSensor.roll();
  fifteen.moveArmTo(100, 100); // lower platform
  fifteen.goForwardU(18, 65, 90, rampUp, 0);
  fifteen.climbPlatform(startPitch, 50);
  fifteen.moveArmTo(highArmAngle, 100, false);
  wait(400, msec);
  fifteen.goForwardTimed(2.8, 37);

  // Grab blue goal
  fifteen.goForwardU(-3, 30, 90, 0, 1);
  fifteen.goTurnU(180);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, false);
  fifteen.goForwardU(5, 60, 180, 10, 2, true, 20, 16, 1.5);
  wait(300, msec);
  fifteen.goForwardU(-22, 40, 180, rampUp, 5);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);
  fifteen.backDown();

  // Do rings
  fifteen.startIntake();
  fifteen.goForwardU(28, 25, 180, rampUp, 5, true, 20, 16, 2.25);
  fifteen.goForwardU(-26, 40, 180, rampUp, 5, true, 20, 16, 2.25);
  fifteen.goForwardU(25.5, 25, 180, rampUp, 5, true, 20, 16, 2.25);

  fifteen.goCurve(-20, 50, 0.35, 15, 5);

  return 0;
}

int armTest() {
  float highArmAngle = 600;
  fifteen.clawUp();
  wait(500, msec);
  fifteen.clawDown();
  wait(500, msec);
  fifteen.moveArmTo(200, 100);
  wait(1000, msec);
  bool obtainedGoal = fifteen.moveArmToManual(600, 100);
  return 0;
}


//void autonomous() { fifteen.setBrakeType(hold); task auto1(leftAuto); }
void autonomous() { fifteen.setBrakeType(hold); /*task t(displayTime); */task auto1(twoRingAuton); }
//void autonomous() { fifteen.setBrakeType(hold); task t(displayTime); task auto1(armTest); }
//void autonomous() { fifteen.setBrakeType(hold); task auto1(boxRushNoGyro); }s

void userControl(void) { fifteen.setBrakeType(coast); task controlLoop1(mainTeleop); }
//void userControl(void) { task controlLoop1(logDistance); }


int main() {

  fifteen.backUp();
  fifteen.clawDown();

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