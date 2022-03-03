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
  fifteen.goToAxis(xaxis, false, -23, 80);
  return 0;
}

int vcat300Skills2() {

  float lowArmAngle = -10;

  //fifteen.waitGyroCallibrate();
  fifteen.clawUp();

  fifteen.backLiftL.resetRotation();
  fifteen.backLiftR.resetRotation();
  fifteen.frontArmL.resetRotation();
  fifteen.frontArmR.resetRotation();


  //fifteen.backLiftL.spin(forward, 0, pct);
  //fifteen.backLiftR.spin(forward, 0, pct);

  // grab home goal
  fifteen.moveArmTo(200, 100, false);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, true);
  fifteen.goForward(-10, 30, 5, 5);
  fifteen.startIntake();
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);
  fifteen.goForward(17, 30, 10, 5);
  

  // get yellow
  fifteen.goTurnU(25);
  fifteen.stopIntake();
  fifteen.moveArmTo(lowArmAngle, 100, false);
  logController("forward");
  wait(1000, msec);
  logController("vision");
  fifteen.goVision(40, 60, YELLOW, 2, 0, true);
  wait(1000, msec);
  logController("forward 2");
  fifteen.goForwardU(5, 30, 25, 0, 5);
  fifteen.clawDown(); // grab yellow
  fifteen.moveArmTo(200, 100, false);
  logController("turn to 0");
  fifteen.goTurnU(0);

  // drop yellow off
  fifteen.goForwardU(40, 100, 0, 3, 10);
  fifteen.clawUp();
  fifteen.goForwardU(1, 30, 0, 0, 0);
  fifteen.goTurnU(330);
  fifteen.goForwardU(-10, 50, 330, 1, 3);
  fifteen.moveArmTo(lowArmAngle, 100, false);

  // get red
  fifteen.goTurnU(270);
  //wait(300, msec);
  //fifteen.driveStraight(7, 50, reverse, 5, 5); // go back a little for better vision alignment
  fifteen.goAlignVision(RED, 5);
  fifteen.goVision(20, 60, RED, 1, 3);
  fifteen.clawDown(); // clamp red
  wait(100, msec);
  fifteen.goForward(-4, 30, 1, 1);
  fifteen.moveArmTo(600, 100, true);
  fifteen.goForwardTimed(1.75, 30);
  fifteen.gyroSensor.setHeading(270, deg); // recallibrate initial heading since squared with wall
  //wait(300, msec);

  fifteen.goForward(-33, 50, 1, 5);
  fifteen.moveArmTo(200, 60, false);
  fifteen.goTurnU(180);
  fifteen.goForwardU(40, 100, 180, 3, 10);
  fifteen.goTurnU(190);
  fifteen.goToAxis(xaxis, false, -30, 80);
  fifteen.clawUp(); // drop off red
  wait(100, msec);

  // get blue across field
  fifteen.moveArmTo(lowArmAngle, 100, false);
  fifteen.goForward(-4, 35, 1, 1);
  fifteen.goTurnU(90);
  fifteen.goForwardU(40, 100, 90, 3, 5, false, 20, 40);
  fifteen.goVision(50.05, 60, BLUE, 0, 4);
  fifteen.clawDown(); // clamp blue
  wait(100, msec);

  // Wall align
  fifteen.goForwardU(-4, 30, 90, 1, 1);
  fifteen.moveArmTo(600, 100, true);
  fifteen.goForwardTimed(1.5, 30);

  // multi step turn
  fifteen.goForwardU(-20, 30, 90, 1, 2);
  fifteen.goTurnU(70);
  fifteen.goForwardU(10, 30, 70, 1, 2);

  // Head to blue platform areas
  fifteen.goTurnU(0);
  fifteen.goForwardU(80, 100, 0, 3, 10);
  fifteen.goForwardTimed(1.5, 20);

  // align to platform
  fifteen.goForwardU(-4, 20, 0, 1, 1);
  fifteen.goTurnU(285);
  fifteen.goForwardU(7, 30, 285, 1, 2);
  fifteen.goTurnU(270); // aim platform side



  // climb
  fifteen.moveArmTo(100, 100);

  fifteen.goForwardU(39, 30, 270, 1, 2);
  wait(350, msec);
  fifteen.goForwardU(2.8, 18, 270, 0, 0);

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
  fifteen.goAlignVision(YELLOW, 5);
  log("yes");
  wait(2000, msec);
  fifteen.goVision(50, 60, YELLOW, 1, 5);
  return 0;
}

void autonomous() { fifteen.setBrakeType(hold); task auto1(vcat300Skills2); }
//void autonomous() { thread auto1(mainAuto); }

void userControl(void) { fifteen.setBrakeType(coast); task controlLoop1(mainTeleop); }
//void userControl(void) { task controlLoop1(logDistance); }


int main() {

  wait(500, msec);
  GPS11.calibrate();
  fifteen.gyroSensor.calibrate();
  fifteen.waitGpsCallibrate();
  fifteen.gyroSensor.setHeading(0, degrees);
  log("calibrated gyro");

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
