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

int logGyro() {
  while (true) {
    log("%f", fifteen.gyroSensor.heading());
    task::sleep(100);
  }
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
  fifteen.goToAxis(xaxis, false, -23, 80);
  return 0;
}

int worldSkills() {

  int autonStart = vex::timer::system();

  float lowArmAngle = -20;
  float highArmAngle = 670;

  fifteen.clawUp();
  fifteen.backLiftL.resetRotation();
  fifteen.backLiftR.resetRotation();
  fifteen.frontArmL.resetRotation();
  fifteen.frontArmR.resetRotation();

  // grab home goal
  float startAngle = fifteen.gyroSensor.heading();
  fifteen.moveArmTo(highArmAngle, 100, false);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, true);
  fifteen.goForward(-10.5, 40, 1, 2);
  fifteen.startIntake();
  wait(200, msec);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);
  fifteen.goForwardU(21.5, 22, startAngle, 2, 2);
  fifteen.goForwardU(-15, 35, startAngle, 2, 2.5); // go three passes to pick up rings
  fifteen.goForwardU(15, 22, startAngle, 2, 2);
  fifteen.goForwardU(-8, 35, startAngle, 2, 2.5);
  
  // Wall align
  fifteen.goTurnU(180);
  fifteen.goForwardU(8, 50, 180, 1, 2, false, 20, 30);
  fifteen.goForwardTimed(1, 30);
  fifteen.possiblyResetGyro(180);

  // Go to blue
  fifteen.goForwardU(-22, 80, 180, 2, 5);
  fifteen.goTurnU(90);
  fifteen.moveArmTo(lowArmAngle, 100, false);
  fifteen.goForwardU(47, 90, 90, 3, 4, false, 20, 50);
  fifteen.goVision(38.5, 50, BLUE, 2, 3);
  fifteen.clawDown(); // clamp blue
  wait(100, msec);

  // Wall align
  fifteen.goForwardU(-7, 60, 90, 1, 1);
  fifteen.moveArmTo(highArmAngle, 100, false); // start the arm movement
  fifteen.goTurnU(90);
  fifteen.moveArmTo(highArmAngle, 100, true); // finish arm movement
  fifteen.goForwardU(13, 60, 90, 1, 2, false, 20, 30);
  fifteen.goForwardTimed(0.7, 30);
  fifteen.possiblyResetGyro(90);

  // Head to blue zone
  fifteen.goForwardU(-6, 50, 90, 1, 3);
  fifteen.moveArmTo(200, 100);
  fifteen.goTurnU(0);
  fifteen.goForwardU(52, 90, 0, 2, 10);

  // Move front blue goal to platform
  fifteen.goTurnU(294);
  fifteen.goForwardU(36, 90, 294, 2, 0, false);
  fifteen.moveArmTo(highArmAngle, 100);
  fifteen.goForwardU(20, 90, 294, 0, 10);

  // Do sweep maneuver
  fifteen.moveArmTo(500, 50);
  fifteen.moveArmTo(highArmAngle, 100);
  fifteen.goTurnU(310);
  fifteen.goForwardU(3, 40, 310, 1, 1);
  fifteen.clawUp();
  wait(100, msec);

  // Swap goal from back to front
  fifteen.goForwardU(-4, 40, 310, 1, 1);
  fifteen.goTurnU(270);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, true);
  fifteen.goForwardU(7, 60, 270, 1, 2);
  fifteen.setBackLift(fifteen.BACK_LIFT_UP, false);
  fifteen.moveArmTo(lowArmAngle, 100);
  fifteen.goTurnU(90);
  fifteen.goForwardU(11, 40, 90, 1, 1);
  fifteen.clawDown();
  wait(100, msec);

  // Elevate other goal
  fifteen.moveArmTo(highArmAngle, 100);
  fifteen.goTurnU(0);
  fifteen.goForwardU(5, 40, 0, 1, 1);
  fifteen.clawUp();
  wait(100, msec);

  // Wall align right side
  fifteen.goForwardU(-5, 40, 0, 1, 1);
  fifteen.goTurnU(270);
  fifteen.goForwardU(50, 100, 270, 2, 5, false, 20, 30);
  fifteen.goForwardTimed(1, -30);
  fifteen.possiblyResetGyro(270);

  // Get red by first aiming with vision then 180, pick up with 1dof
  fifteen.goCurve(15, 50, 0.3, 2, 5);
  fifteen.goAlignVision(RED, 3);
  float head = fmod((fifteen.getAngle() + 180),360.0);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, false);
  fifteen.goTurnU(head);
  fifteen.goForwardU(-20, 40, head, 1, 3);
  fifteen.setBackLift(fifteen.BACK_LIFT_SLIGHT, true);
  fifteen.goForwardU(15, 50, head, 1, 3);
  fifteen.goTurnU(180);

  // Ge yellow
  fifteen.goVision(35, 50, YELLOW, 1, 5);
  fifteen.clawDown();
  wait(100, msec);
  fifteen.moveArmTo(200, 100, false);
  fifteen.goTurnU(180);

  // Get  rings
  fifteen.goCurve(15, 60, -0.3, 2, 5);
  fifteen.goTurnU(180);
  fifteen.startIntake();
  fifteen.goForwardU(72, 95, 180, 3, 5, false, 20, 30);
  fifteen.goForwardTimed(1.5, 30);
  fifteen.stopIntake();
  fifteen.possiblyResetGyro(180);
  
  // localize on other wall
  fifteen.goForwardU(30, 70, 180, 2, 5);
  fifteen.goTurnU(90);
  fifteen.goForwardU(15, 80, 90, 2, 4, false, 20, 30);
  fifteen.goForwardTimed(1, 30);
  fifteen.possiblyResetGyro(90);
  
  // Go to 24 pullup position
  fifteen.goForwardU(64, 80, 90, 3, 15);

  



  return 0;

}

int middleSchoolSkills() {


  int autonStart = vex::timer::system();

  float lowArmAngle = -20;
  float highArmAngle = 670;

  //fifteen.waitGyroCallibrate();
  fifteen.clawUp();

  fifteen.backLiftL.resetRotation();
  fifteen.backLiftR.resetRotation();
  fifteen.frontArmL.resetRotation();
  fifteen.frontArmR.resetRotation();



  //fifteen.backLiftL.spin(forward, 0, pct);
  //fifteen.backLiftR.spin(forward, 0, pct);

  // grab home goal
  fifteen.moveArmTo(highArmAngle, 100, false);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, true);
  fifteen.goForward(-10.5, 40, 1, 2);
  fifteen.startIntake();
  wait(200, msec);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);
  fifteen.goForward(21.5, 22, 2, 2);
  fifteen.goForward(-15, 35, 2, 2.5); // go three passes to pick up rings
  fifteen.goForward(15, 22, 2, 2);
  fifteen.goForward(-15, 35, 2, 2.5);
  fifteen.goForward(11, 22, 2, 2);
  
  // get yellow
  float turnAngle = 17;
  fifteen.stopIntake();
  fifteen.moveArmTo(lowArmAngle, 80, false);
  fifteen.goTurnU(turnAngle);
  fifteen.clawUp();
  wait(300, msec);
  fifteen.goForwardU(7, 50, turnAngle, 2, 0, false); // go forward a little to make sure vision sees left yellow goal as biggest object
  fifteen.goVision(33, 40, YELLOW, 0, 0, false);
  fifteen.goForwardU(12, 40, turnAngle, 0, 4);
  fifteen.clawDown(); // grab yellow
  fifteen.moveArmTo(250, 100, false);
  fifteen.goTurnU(0);

  // drop yellow off
  fifteen.goForwardU(46, 100, 0, 3, 10);
  fifteen.clawUp();
  fifteen.goTurnU(330);
  fifteen.moveArmTo(lowArmAngle, 100, false);
  fifteen.goForwardU(-5, 40, 0, 0.5, 0.5);

  // get red
  fifteen.goTurnU(270, false, 2, true);
  fifteen.goAlignVision(RED, 2);
  fifteen.goVision(15, 40, RED, 1, 3, true, 3);
  fifteen.clawDown(); // clamp red
  wait(100, msec);
  fifteen.goForward(-3, 30, 1, 0, false); // start going backward
  fifteen.moveArmTo(highArmAngle, 100, false); // halfway going backward, start raising arm
  fifteen.goForward(-4, 30, 0, 1, true); // finish going backward
  fifteen.goTurnU(270, 1); // make sure aligned
  fifteen.moveArmTo(highArmAngle, 100, true); // make sure arm movement is completed
  fifteen.goForwardU(13, 60, 270, 1, 2, false, 20, 30);
  fifteen.goForwardTimed(1, 30);
  fifteen.possiblyResetGyro(270); // recallibrate initial heading since squared with wall
  fifteen.goForwardU(-26.5, 50, 270, 1, 5); // back away from wall
  fifteen.goTurnU(180); // turn to red zone

  // Go back to red zone
  fifteen.startIntake();
  fifteen.goForwardU(36, 100, 180, 3, 10);
  fifteen.goTurnU(190);
  fifteen.goForwardU(24, 50, 190, 1, 2);
  fifteen.goTurnU(180);
  fifteen.goForwardU(16, 80, 180, 2, 3, false, 20, 30); // slow to 30 speed for wall align
  fifteen.goForwardTimed(1.5, 30); // align with back wall
  fifteen.possiblyResetGyro(180);
  fifteen.stopIntake();

  // Drop off red
  fifteen.goForwardU(-5, 50, 180, 2, 0, false);
  fifteen.moveArmTo(300, 100, false);
  fifteen.goForwardU(-13, 50, 180, 0, 0, false);
  fifteen.clawUp(); // drop off red
  fifteen.goForwardU(-4, 50, 180, 0, 5, true); // back up for clearance from red goal

  // Go to blue across field
  fifteen.moveArmTo(lowArmAngle, 60, false);
  fifteen.goTurnU(90);
  fifteen.goForwardU(40, 100, 90, 3, 10);

  // Wait for 24 robot to pass with universal wait until time delta
  while (!isTimeout(autonStart, 36.0)) wait(20, msec);
  
  // Grab blue
  fifteen.goForwardU(7, 100, 90, 3, 4, false, 20, 50);
  fifteen.goVision(38.5, 50, BLUE, 2, 3);
  fifteen.clawDown(); // clamp blue
  wait(100, msec);

  // Wall align
  fifteen.goForwardU(-7, 60, 90, 1, 1);
  fifteen.moveArmTo(highArmAngle, 100, false); // start the arm movement
  fifteen.goTurnU(90);
  fifteen.moveArmTo(highArmAngle, 100, true); // finish arm movement
  fifteen.goForwardU(13, 60, 90, 1, 2, false, 20, 30);
  fifteen.goForwardTimed(0.7, 30);
  fifteen.possiblyResetGyro(90);

  // multi step turn
  fifteen.startIntake();
  fifteen.goForwardU(-20, 50, 90, 2, 4);
  fifteen.goTurnU(70);
  fifteen.goForwardU(10, 40, 70, 1, 2);

  // Head to blue platform area
  fifteen.goTurnU(357); // slight angle to left to avoid brushing against right wall when turning
  fifteen.goForwardU(80, 90, 357, 3, 8, false, 20, 30);
  fifteen.goForwardTimed(1.5, 30);
  fifteen.possiblyResetGyro(0);

  // align to platform
  fifteen.goForwardU(-6, 30, 0, 0, 0.5);
  fifteen.goTurnU(300); // aim platform side
  fifteen.goForwardU(5, 40, 300, 1, 2);
  fifteen.goTurnU(270);
  fifteen.goForwardU(-5, 50, 270, 0.5, 1, false, 20, 30);
  fifteen.goForwardTimed(1, -30);
  fifteen.goForwardU(15, 60, 270, 1, 3);
  fifteen.stopIntake();

  // climb
  fifteen.moveArmTo(100, 100, true);
  fifteen.goForwardU(41.5, 40, 270, 0, 0);
  wait(350, msec);
  fifteen.goForwardU(2.5, 40, 270, 0, 0);

  return 0;

}

int logGPS() {
  while (true) {
    log("Quality: %d \nX: %f \nY: %f \nAngle: %f", GPS11.quality(), fifteen.getX(), fifteen.getY(), fifteen.getAngle());
    wait(20, msec);
  }
  return 0;
}

int logAngle() {
  while (true) {
    fifteen.getAngle();
    wait(20, msec);
  }
}

int visionTest() {
  fifteen.goToAxis(xaxis, true, -44, 50, 6); // localize in platform-platform direction
  return 0;
}

int testRadius(){
  wait(500, msec);
  fifteen.gyroSensor.calibrate();
  fifteen.waitGyroCallibrate();
  fifteen.goRadiusCurve(48, 96*atan(1), true, 30, 0, 0, 5);
  return 0;
}

void autonomous() { fifteen.setBrakeType(hold); task auto1(testRadius); }
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