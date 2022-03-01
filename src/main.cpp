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

int logDistance() {

  fifteen.leftMotorA.resetRotation();
  fifteen.rightMotorA.resetRotation();

  while (true) {

    if (fifteen.buttons.pressed(Buttons::A)) {
      fifteen.leftMotorA.resetRotation();
      fifteen.rightMotorA.resetRotation();
    }

    float left = degreesToDistance(fifteen.leftMotorA.rotation(degrees));
    float right = degreesToDistance(fifteen.rightMotorA.rotation(degrees));
    log("%f %f %f", left, right, fifteen.getAngle());
    wait(20, msec);
  }

  return 0;
}


int testGPS() {
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);
  wait(2000, msec);
  fifteen.goTurnU(112);
  wait(2000, msec);
  fifteen.goTurnU(140);
  return 0;
}

int ringSkills() {

  const float INTAKE_RING = 200;
  const float LOW_ARM = 0;

  fifteen.frontArmL.resetRotation();
  fifteen.frontArmR.resetRotation();
  fifteen.backLiftL.resetRotation();
  fifteen.backLiftR.resetPosition();
  fifteen.frontArmL.setBrake(hold);
  fifteen.frontArmR.setBrake(hold);
  fifteen.backLiftL.setBrake(hold);
  fifteen.backLiftR.setBrake(hold);

  fifteen.clawUp();
  fifteen.moveArmTo(INTAKE_RING, 100, false); // move arm in position to take rings

  // Grab home goal
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, true);
  fifteen.goForward(-10, 40, 1, 2);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);

  // Grab match rings
  fifteen.startIntake();
  fifteen.goForward(20, 40, 2, 4);
  fifteen.goForward(-17, 60, 2, 4);
  fifteen.goForward(17, 40, 2, 4);

  // Head to left yellow goal
  fifteen.goTurnU(112);
  fifteen.stopIntake();
  fifteen.moveArmTo(LOW_ARM, 100, false);
  fifteen.goForward(35, 70, 3, 4, false, 20, 30);
  fifteen.goForward(15, 30, 0, 5);
  fifteen.clawDown();
  fifteen.moveArmTo(300, 100, false);

  // Drop yellow goal on blue side
  fifteen.goTurnU(90);
  fifteen.goForwardU(41 - fifteen.getX(), 100, 90, 5, 10, true, 50); // go x = 41
  fifteen.clawUp(); // drop yellow goal

  // Drop blue goal from back
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, false);
  fifteen.goForward(-15, 50, 1, 2);

  // Pick up red with 1dof
  fifteen.goForward(8, 50, 1, 2);
  fifteen.goTurnU(180);
  fifteen.goForwardU(-10, 50, 180, 1, 2);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);

  // Pick up blue goal in front
  fifteen.goForwardU(10, 50, 180, 1, 2);
  fifteen.goTurnU(270);
  fifteen.goForward(8, 50, 1, 2);
  fifteen.clawDown();
  fifteen.moveArmTo(500, 100, false);

  // Pick up rings for red
  fifteen.goCurve(-10, 70, 0.3, 2, 3); // strafe to be collinear with rings
  fifteen.goTurnU(270);
  fifteen.startIntake();
  fifteen.goForward(20, 40, 1, 0, false);
  fifteen.goCurve(5, 40, 0.8, 0, 0, false); // curve to keep picking up rings in L shape
  fifteen.goForward(10, 40, 0, 1, true);

  // Drop red off
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, false);
  fifteen.moveArmTo(150, 100, false);
  fifteen.goCurve(-30, 80, -0.5, 2, 5);
  fifteen.startIntake(reverse);

  // Head to other blue
  fifteen.goCurve(5, 50, 0.45, 1, 1);
  fifteen.setBackLift(fifteen.BACK_LIFT_SLIGHT, false);
  fifteen.stopIntake();
  fifteen.goPointGPS(-38, -25, reverse);
  fifteen.goTurnU(0);
  fifteen.goForwardU(-22, 60, 0, 1, 3);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);

  // Collect rings in L shape
  fifteen.moveArmTo(500, 100, false);
  fifteen.startIntake();
  fifteen.goCurve(10, 50, 0.5, 1, 0, false);
  fifteen.goForwardU(18, 50, 90, 0, 0, false);
  fifteen.goCurve(10, 50, 0.5, 0, 0, false);
  fifteen.goForwardU(13, 50, 180, 0, 2, true);
  fifteen.goForwardTimed(1.5, 30); // align with side wall
  fifteen.startIntake(reverse);

  // Go to blue platform area
  fifteen.goForwardU(-20, 70, 180, 2, 4);
  fifteen.stopIntake();
  fifteen.goTurnU(90);
  fifteen.goForwardU(50, 100, 90, 5, 5, false, 20, 30);
  fifteen.goForwardTimed(2, 30); // align with back wall
  fifteen.goForward(-2.5, 30, 0, 0.5);
  fifteen.goTurnU(0);

  // climb
  fifteen.moveArmTo(100, 100);
  fifteen.goForwardU(39, 30, 0, 1, 1);
  wait(350, msec);
  fifteen.goForwardU(2.8, 18, 0, 0, 0.5);
  
  


  return 0;

}

int logGPS() {
  while (true) {
    log("Quality: %d \nX: %f \nY: %f \nAngle: %f", GPS11.quality(), fifteen.getX(), fifteen.getY(), fifteen.getAngle());
    wait(20, msec);
  }
  return 0;
}

void autonomous() { fifteen.setBrakeType(hold); task auto1(testGPS); }
//void autonomous() { thread auto1(mainAuto); }

void userControl(void) { fifteen.setBrakeType(coast); task controlLoop1(mainTeleop); }
//void userControl(void) { task controlLoop1(logDistance); }


int main() {

  wait(500, msec);
  fifteen.waitGpsCallibrate();

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
