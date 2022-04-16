// CODE FOR 15" ROBOT
#include "r15/robot15.cpp"

Robot15 fifteen = Robot15();

int mainTeleop() {

  while (true) {
    fifteen.teleop();
    wait(20, msec);
  }
  return 0;
}

int test() {
  //fifteen.goForwardPID(48, 100, fifteen.getAngle(), 20, 10);
  // fifteen.goForward(24, 80, 20, 12);
  // wait(1000, msec);
  // fifteen.goForward(-24, 80, 20, 12);
  fifteen.goTurnU(180);

  // wait(1000, msec);

  // fifteen.goFightBackwards();

  return 0;
}

int autonAI() {

  int matchStartTime = timer::system();
  float highArmAngle = 680;
  float lowArmAngle = -20;
  fifteen.setBrakeType(hold);

  // Initial go rush
  fifteen.clawUp();
  fifteen.goForward(43, 100, 4, 6, false, 20, 50);
  fifteen.goForward(3, 50, 0, 2, false, -1, 40);
  fifteen.clawDown(); // start claw down motion early
  fifteen.goForward(3, 40, 0, 3, true);
  fifteen.goFightBackwards();

  // Get back to wall align but avoiding platform
  fifteen.moveArmTo(200, 100, false);
  fifteen.goTurnU(50);
  fifteen.goForwardU(-13, 70, 50, 10, 7, false);
  fifteen.goTurnU(0);
  fifteen.goForwardU(-11, 70, 0, 10, 1, false, 20, 35);
  fifteen.goForwardTimed(1.5, -35); // wall align back
  bool obtainedGoal = fifteen.moveArmToManual(highArmAngle, 100); // raise arm and use current thresholds to determine whether obtained yellow goal

  // Align with left wall
  fifteen.goForwardU(1, 30, 0, 0, 2);
  wait(150, msec);
  fifteen.goTurnU(270);
  fifteen.goForwardU(11, 50, 270, 10, 5, false, 20, 35);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, false);
  fifteen.goForwardTimed(1.0, 35);

  // Get alliance goal
  
  fifteen.goForwardU(-26, 70, 270, 10, 5, false, 20, 40);
  fifteen.goForwardU(-9, 40, 270, 10, 4, true);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);

  // do match load rings
  fifteen.startIntake();
  fifteen.goForwardU(27, 30, 270, 10, 5, true, 20, 10, 3);
  fifteen.goForwardU(-17, 35, 270, 10, 5, true, 20, 10, 2.5); // go three passes to pick up rings
  fifteen.goForwardU(17, 35, 270, 10, 5, true, 20, 10, 2.5); // go three passes to pick up rings
  fifteen.goForwardU(-17, 35, 270, 10, 5, true, 20, 10, 2.5); // go three passes to pick up rings
  fifteen.goForwardU(16, 30, 270, 10, 0, false);
  fifteen.goForwardTimed(0.7, 30); // another wall align at left goal

  /*
  Robot needs to get to other alliance goal through platform. Behavior is dependent on whether yellow goal was obtained.

  If yellow goal obtained, turn 180 to face platform, release alliance goal from 1dof, climb
  Otherwise, back up, release alliance goal from 1dof, turn 180 to face platform, pick up alliance goal with front arm, climb
  */

  if (obtainedGoal) {

    fifteen.goForwardU(-18, 60, 270, 10, 5);
    
    // Force turn counterclockwise because am lazy to do this properly
    fifteen.setLeftVelocity(reverse, 100);
    fifteen.setRightVelocity(forward, 100);
    wait(300, msec);
    fifteen.goTurnU(90); // face platform
    
    fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, true);
    fifteen.goForwardU(12, 50, 90, 10, 5);
    fifteen.setBackLift(fifteen.BACK_LIFT_UP, false);

  } else {

    // Exchange blue goal from back to front
    fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, false); // drop alliance goal
    fifteen.goForwardU(-12, 50, 270, 10, 0, false);
    fifteen.moveArmTo(lowArmAngle, 100); // lower arm to pick up alliance goal
    fifteen.goForwardU(-5, 50, 270, 0, 4);
    fifteen.goForwardU(10, 50, 270, 10, 4);
    fifteen.setBackLift(fifteen.BACK_LIFT_UP, false);
    wait(500, msec); // wait for back lift to fold sufficiently
    fifteen.goTurnU(90);
    fifteen.goForwardU(10, 50, 90, 10, 5);
    fifteen.clawDown();

    // Get to platfom ready position
    fifteen.moveArmTo(500, 100);
    fifteen.goForwardU(7, 40, 90, 10, 3);
  }
  // At this point, back wheel is aligned between tiles, ready to lower arm and climb

  // Climb platform
    fifteen.moveArmTo(100, 100); // lower platform
    fifteen.goForwardU(70, 50, 90, 10, 5);

  return 0;
}

int testArm() {
  float highArmAngle = 680;
  fifteen.clawUp();
  wait(2000, msec);
  fifteen.clawDown();
  wait(500, msec);
  fifteen.moveArmToManual(highArmAngle, 100);
  return 0;
}

void autonomous() { fifteen.setBrakeType(hold); task auto1(testArm); }
//void autonomous() { thread auto1(mainAuto); }

void userControl(void) { fifteen.setBrakeType(coast); task controlLoop1(mainTeleop); }
//void userControl(void) { task controlLoop1(logDistance); }


int main15() {

  
  fifteen.clawDown();
  wait(500, msec);
  fifteen.gyroSensor.calibrate();
  fifteen.waitGyroCallibrate();

  Competition.bStopAllTasksBetweenModes = true;

  fifteen.resetEncoderDistance();
  
  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  while (true) {
    wait(20, msec);
  }

}

/*
int worldSkills() {

  int autonStart = vex::timer::system();

  float lowArmAngle = -20;
  float highArmAngle = 680;

  fifteen.clawUp();
  fifteen.backLiftL.resetRotation();
  fifteen.backLiftR.resetRotation();
  fifteen.frontArmL.resetRotation();
  fifteen.frontArmR.resetRotation();

  // grab home goal
  fifteen.moveArmTo(highArmAngle, 100, false);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, true);
  fifteen.goForwardU(-10.5, 40, 270, 3, 5);
  fifteen.startIntake();
  wait(200, msec);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);
  fifteen.goForwardU(23, 30, 270, 2, 5, true, 20, 10, 3);
  fifteen.goForwardU(-19, 35, 270, 2, 5, true, 20, 10, 3); // go three passes to pick up rings
  fifteen.goForwardU(19, 30, 270, 2, 5);

  // Go to blue
  fifteen.goCurve(-28, 85, 0.33, 3, 4, false);
  fifteen.stopIntake();
  float ang = 88;
  fifteen.moveArmTo(lowArmAngle, 100, false);
  fifteen.goTurnU(ang);
  fifteen.goForwardU(76, 95, ang, 0, 10, false, 20, 40);
  fifteen.goForwardU(10, 40, ang, 0, 3);
  fifteen.clawDown(); // clamp blue

  // Turn to blue zone
  fifteen.goTurnU(45);
  fifteen.moveArmTo(highArmAngle, 100, false);
  fifteen.goCurve(10, 100, -0.4, 3, 0, false);

  // Head to blue platform with curvy path
  fifteen.goForwardU(40, 95, 0, 0, 0, false);
  fifteen.goCurve(20, 100, -0.35, 0, 0, false);
  fifteen.goForwardU(48, 95, 270, 0, 15);
  fifteen.goTurnU(0);

  // Do sweep maneuver
  fifteen.moveArmTo(450, 50);
  fifteen.goTurnU(330, true, 5, true); // "sweep" the goal left
  fifteen.moveArmTo(highArmAngle, 100, false);
  fifteen.goTurnU(0);
  fifteen.moveArmTo(highArmAngle, 100, true); // finish arm movement
  fifteen.goForwardU(5, 40, 0, 1, 2, true, 20, 10, 2);
  fifteen.clawUp();
  fifteen.stopIntake();
  wait(100, msec);

  // Swap goal from back to front
  fifteen.goForwardU(-3, 30, 0, 0.5, 0.5);
  float angle = 330;
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, false);
  fifteen.goTurnU(angle);
  fifteen.moveArmTo(lowArmAngle, 100, false);
  fifteen.goForwardU(-6, 60, angle, 1, 3);
  fifteen.goForwardU(7, 60, angle, 1, 3);
  fifteen.setBackLift(fifteen.BACK_LIFT_UP, false);

  // Grab dropped goal
  fifteen.goTurnU(angle - 180);
  fifteen.goForwardU(10, 50, angle - 180, 1, 0, false);
  fifteen.clawDown(); // slam into goal and claw down so momentum will lessen chance of rings blocking claw
  fifteen.moveArmTo(highArmAngle, 100, false);
  fifteen.goForwardU(3, 50, angle - 180, 0, 3);

  // Elevate other goal
  fifteen.goCurve(10, 100, 0.45, 5, 0, false);
  fifteen.goTurnU(0);
  fifteen.clawUp();
  fifteen.goForwardU(-5, 40, 0, 1, 2);

  // Get to red
  fifteen.goTurnU(90);
  fifteen.stopIntake();
  fifteen.moveArmTo(lowArmAngle, 100, false);
  fifteen.goForwardU(48, 95, 90, 5, 15);

  // Get red with 1dof
  angle = 160;
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, true);
  fifteen.goTurnU(angle);
  fifteen.goForwardU(-15, 50, angle, 2, 5, true, 20, 10, 3);
  fifteen.setBackLift(fifteen.BACK_LIFT_SLIGHT, true);
  fifteen.goForwardU(18, 50, angle, 1, 3);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);
  fifteen.goTurnU(200);

  // Ge yellow
  fifteen.goVision(35, 60, YELLOW, 2, 10);
  fifteen.clawDown();
  fifteen.goTurnU(200);
  fifteen.moveArmTo(highArmAngle, 100, false);
  fifteen.goForwardU(-12, 60, 200, 2, 5);
  fifteen.goTurnU(180);

  // Localize robot position with GPS to determine the distances for the rest of the run
  float *x, *y, *headingAngle;
  fifteen.getGPSData(x, y, headingAngle);
  fifteen.gyroSensor.setHeading(*headingAngle, degrees);
  fifteen.goTurnU(180); // redo angle to 180 after gps localization

  // Use known current position to get to y = 0
  fifteen.goForwardU(75, 60, 180, 3, 12, false, 20, 30); // get field then match rings
  fifteen.goForwardTimed(1.5, 30); // wall align
  fifteen.possiblyResetGyro(180);
  fifteen.goForwardU(-26, 80, 180, 3, 10);
  fifteen.goTurnU(270);
  fifteen.goForwardU(0 - *y, 80, 270, 3, 20);

  return 0;

} */