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

  float lowArmAngle = -20;

  //fifteen.waitGyroCallibrate();
  fifteen.clawUp();

  fifteen.backLiftL.resetRotation();
  fifteen.backLiftR.resetRotation();
  fifteen.frontArmL.resetRotation();
  fifteen.frontArmR.resetRotation();


  //fifteen.backLiftL.spin(forward, 0, pct);
  //fifteen.backLiftR.spin(forward, 0, pct);

  // grab home goal
  fifteen.moveArmTo(250, 100, false);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, true);
  fifteen.goForward(-10, 40, 1, 2);
  fifteen.startIntake();
  wait(500, msec);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);
  fifteen.goForward(17, 20, 1, 2);
  

  // get yellow
  float turnAngle = 28;
  fifteen.goTurnU(turnAngle);
  fifteen.stopIntake();
  fifteen.moveArmTo(lowArmAngle, 100, false);
  fifteen.clawUp();
  fifteen.goVision(40, 60, YELLOW, 2, 0, false);
  fifteen.goForwardU(12, 40, turnAngle, 0, 4);
  fifteen.clawDown(); // grab yellow
  fifteen.moveArmTo(250, 100, false);
  logController("turn to 0");
  fifteen.goTurnU(0);

  // drop yellow off
  fifteen.goForwardU(34, 100, 0, 3, 10, true);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, true);
  fifteen.goForwardU(11, 50, 0, 2, 3, true, 20, 12, 3);
  fifteen.setBackLift(fifteen.BACK_LIFT_UP, false);
  
  // Move blue goal to front claw from back
  fifteen.clawUp();
  fifteen.goForwardU(-1, 30, 0, 0, 0, false);
  fifteen.moveArmTo(lowArmAngle, 100, false);
  fifteen.goTurnU(180);
  fifteen.goForwardU(15, 50, 180, 1, 3);
  fifteen.clawDown();
  fifteen.moveArmTo(250, 100, false);

  // get red
  fifteen.goForwardU(-9, 40, 180, 0, 1);
  fifteen.goTurnU(90);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, false);
  fifteen.goForwardU(2, 40, 90, 0, 0.5);
  wait(300, msec);
  fifteen.goForwardU(-22, 50, 90, 1, 1);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);

  // Bring red to home zone
  fifteen.goTurnU(180);
  if (GPS11.quality() == 100) fifteen.gyroSensor.setHeading(GPS11.heading(), degrees);
  fifteen.startIntake();
  fifteen.goForwardU(30, 40, 180, 1, 0, false);
  fifteen.goToAxis(xaxis, true, -44, 80, 6); // localize in platform-platform direction
  fifteen.stopIntake();
  float y = fifteen.getY(); // localize and store robot's position on the platform-to-platform axis to be able to calculate distance from blue goal
  fifteen.goTurnU(0);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, true); // drop off red

  // get blue across field
  fifteen.goForwardU(6, 50, 0, 1, 3);
  fifteen.setBackLift(fifteen.BACK_LIFT_SLIGHT, false);
  fifteen.goTurnU(270);
  fifteen.goForwardU(-(44 + y), 100, 270, 5, 0, false); // Go to blue goal with previous localization information
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, false);
  fifteen.goForwardU(-5, 100, 270, 0, 5, false, 20, 40); // slow down on approach to blue goal
  fifteen.goForwardU(-5, 40, 270, 0, 2, true); // slow final approach to blue goal
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true); // pick up blue

  // Pick up rings
  fifteen.startIntake();
  fifteen.moveArmTo(600, 100, false);
  fifteen.goCurve(12, 40, 0.5, 1, 1);
  fifteen.goTurnU(0);
  fifteen.goForwardU(15, 40, 0, 1, 0, false);
  fifteen.goCurve(10, 40, 0.8, 0, false);
  fifteen.goForwardU(-12, 40, 90, 2, 5); // back away from rings
  fifteen.stopIntake();
  fifteen.goTurnU(0); // aim at platform

  // Go to platform
  float x = fifteen.getX(); // Localize (x,y) because gps will be terrible later in the run
  y = fifteen.getY();
  fifteen.goForwardU(55 - x, 80, 0, 3, 7);
  fifteen.goTurnU(270);
  fifteen.goForwardU(-41 - y, 30, 270, 1, 1);

  // climb
  fifteen.moveArmTo(100, 100);

  fifteen.goForwardU(40, 40, 270, 1, 1);
  wait(350, msec);
  fifteen.goForwardU(3, 18, 270, 0, 0);

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
