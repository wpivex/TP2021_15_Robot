// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision6              vision        6               
// DigitalInE           digital_in    E               
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "robot.cpp"

// CODE FOR 15" ROBOT


Robot fifteen = Robot(&Controller1);

int mainTeleop() {
  //fifteen.setTransmission(true);
  while (true) {
    fifteen.teleop();
    wait(20, msec);
  }
  return 0;
}


int armStartup() {

  fifteen.frontArmL.spin(forward, 100, percentUnits::pct);
  fifteen.frontArmR.spin(forward, 100, percentUnits::pct);
  task::sleep(200);
  fifteen.frontArmL.spin(reverse, 100, percentUnits::pct);
  fifteen.frontArmR.spin(reverse, 100, percentUnits::pct);
  task::sleep(350);
  fifteen.frontArmL.stop();
  fifteen.frontArmR.stop();
  fifteen.frontArmL.resetRotation();
  fifteen.frontArmL.resetRotation();

  return 0;
}

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
  float startAngle = fifteen.gyroSensor.heading();
  fifteen.moveArmTo(highArmAngle, 100, false);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, true);
  fifteen.goForward(-10.5, 40, 5, 5);
  fifteen.startIntake();
  wait(200, msec);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);
  fifteen.goForwardU(21.5, 22, startAngle, 5, 5);
  fifteen.goForwardU(-15, 35, startAngle, 5, 5); // go three passes to pick up rings
  fifteen.goForwardU(15, 22, startAngle, 5, 5);
  fifteen.goForwardU(-8, 35, startAngle, 5, 5);
  
  // Wall align
  fifteen.goTurnU(180);
  fifteen.goForwardU(3, 50, 180, 1, 3, false, 20, 30);
  fifteen.goForwardTimed(0.5, 30);
  fifteen.possiblyResetGyro(180);

  // Go to blue
  fifteen.goForwardU(-22.5, 80, 180, 3, 15);
  fifteen.goTurnU(90);
  fifteen.moveArmTo(lowArmAngle, 100, false);
  fifteen.goForwardU(60, 90, 90, 3, 5, false, 20, 50);
  fifteen.stopIntake();
  fifteen.goVision(30, 60, BLUE, 2, 10);
  fifteen.clawDown(); // clamp blue
  wait(100, msec);

  // Wall align
  fifteen.goForwardU(-7, 50, 90, 2, 4);
  fifteen.moveArmTo(highArmAngle, 100, false); // start the arm movement
  fifteen.goTurnU(90);
  fifteen.moveArmTo(highArmAngle, 100, true); // finish arm movement
  fifteen.startIntake();
  fifteen.goForwardU(13, 60, 90, 2, 5, false, 20, 30);
  fifteen.goForwardTimed(0.7, 30);
  fifteen.possiblyResetGyro(90);

  // Head to blue zone
  fifteen.goForwardU(-4.5, 50, 90, 2, 4);
  fifteen.goTurnU(0);
  fifteen.goForwardU(65, 90, 0, 5, 15);

  // Move front blue goal to platform
  fifteen.goTurnU(270);
  fifteen.goForwardU(46, 95, 270, 5, 15);
  wait(100, msec);
  fifteen.goTurnU(0);

  // Do sweep maneuver
  fifteen.moveArmTo(450, 50);
  fifteen.goTurnU(330, true, 5, true); // "sweep" the goal left
  fifteen.moveArmTo(highArmAngle, 100, false);
  fifteen.goTurnU(0);
  fifteen.moveArmTo(highArmAngle, 100, true); // finish arm movement
  fifteen.goForwardU(4, 40, 0, 1, 2, true, 20, 10, 2);
  fifteen.clawUp();
  wait(100, msec);

  // Swap goal from back to front
  fifteen.goForwardU(-4, 40, 0, 1, 1);
  fifteen.goTurnU(90);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, true);
  fifteen.goForwardU(7, 60, 90, 1, 3);
  fifteen.setBackLift(fifteen.BACK_LIFT_UP, false);
  fifteen.stopIntake();
  fifteen.moveArmTo(lowArmAngle, 100, false);
  wait(1000, msec);
  fifteen.goTurnU(270);
  fifteen.goForwardU(10, 50, 270, 1, 0, false);
  fifteen.clawDown(); // slam into goal and claw down so momentum will lessen chance of rings blocking claw
  fifteen.goForwardU(4, 50, 270, 0, 3);
  wait(100, msec);

  // Elevate other goal
  fifteen.moveArmTo(highArmAngle, 100, false); // start movement upwards, concurrency
  fifteen.goForwardU(-16, 90, 270, 4, 12);
  fifteen.moveArmTo(highArmAngle, 100, true);
  fifteen.startIntake();
  fifteen.goTurnU(0);
  fifteen.goForwardU(5, 40, 0, 1, 1);
  fifteen.clawUp();
  wait(100, msec);

  // Wall align right side
  fifteen.goForwardU(-5, 40, 0, 1, 2);
  fifteen.goTurnU(270);
  fifteen.stopIntake();
  fifteen.moveArmTo(lowArmAngle, 100, false);
  fifteen.goForwardU(-52, 95, 270, 2, 5, false, 20, 30);
  fifteen.goForwardTimed(0.8, -30);
  fifteen.possiblyResetGyro(270);

  // Get red by first aiming with vision then 180, pick up with 1dof
  fifteen.goCurve(10, 50, 0.3, 2, 5);
  fifteen.goAlignVision(RED, 2);
  fifteen.goVision(5, 30, RED, 1, 1);
  float head = fmod((fifteen.getAngle() + 180),360.0);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, false);
  fifteen.goTurnU(head);
  fifteen.goForwardU(-21, 50, head, 2, 5, true, 20, 10, 3);
  fifteen.setBackLift(fifteen.BACK_LIFT_SLIGHT, true);
  fifteen.goForwardU(15, 50, head, 1, 3);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);
  fifteen.goTurnU(200);

  // Ge yellow
  fifteen.goVision(35, 60, YELLOW, 2, 10);
  fifteen.clawDown();
  wait(100, msec);
  fifteen.moveArmTo(highArmAngle, 60, false);
  fifteen.goTurnU(180);
  fifteen.startIntake();

  // Strafe into ring lane
  fifteen.goCurve(-18, 60, -0.3, 2, 5); // strafe left to align for rings
  fifteen.goTurnU(180);

  // Localize robot position with GPS to determine the distances for the rest of the run
  float *x, *y, *headingAngle;
  fifteen.getGPSData(x, y, headingAngle);
  fifteen.gyroSensor.setHeading(*headingAngle, degrees);
  fifteen.goTurnU(180);

  // Use known current position to get to (x = -33, y = 0)
  fifteen.goForwardU(*x + 33, 60, 180, 3, 12);
  fifteen.goTurnU(270);
  fifteen.goForwardU(0 - *y, 80, 270, 3, 20);

  return 0;

}

int testForward() {
  fifteen.goForward(24, 80, 5, 15);
  return 0;
}



void autonomous() { fifteen.setBrakeType(hold); task auto1(testForward); }
//void autonomous() { thread auto1(mainAuto); }

void userControl(void) { fifteen.setBrakeType(coast); task controlLoop1(mainTeleop); }
//void userControl(void) { task controlLoop1(logDistance); }


int main() {

  wait(500, msec);
  fifteen.gyroSensor.calibrate();
  fifteen.waitGyroCallibrate();

  Competition.bStopAllTasksBetweenModes = true;
  fifteen.clawDown();

  
  fifteen.leftMotorA.resetRotation();
  fifteen.rightMotorA.resetRotation();
  fifteen.backLiftL.resetRotation();
  fifteen.backLiftR.resetRotation();
  
  
  // DRIVER SKILLS TRUE, OTHERWISE FALSE
  //fifteen.setTransmission(false);

  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);


  //platformClimb2();

  while (true) {
    wait(20, msec);
  }
}