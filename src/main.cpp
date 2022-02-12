// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision6              vision        6               
// DigitalInE           digital_in    E               
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "robot.cpp"

// CODE FOR 15" ROBOT

competition Competition;

Robot fifteen = Robot(&Controller1);

int mainTeleop() {
  //fifteen.setTransmission(true);
  while (true) {
    fifteen.teleop();
    wait(20, msec);
  }
  return 0;
}




void mainAuto() {

  fifteen.waitGyroCallibrate();
  
  fifteen.gyroSensor.setHeading(5, degrees);

  fifteen.driveStraight(45.5, 100, forward, 5, 10, true, {}, 5);
  fifteen.clawDown();
  fifteen.setBrakeType(brake);
  fifteen.driveStraightFighting(1, 100, reverse);
  fifteen.stopLeft();
  fifteen.stopRight();
  
  log("stop");
  wait(1000, msec);

  fifteen.gyroTurnU(0);

  fifteen.driveStraightTimed(20, reverse, 3);

  fifteen.driveStraight(15, 30, forward, 5, 5);

  fifteen.clawUp();
  wait(100, msec);
  fifteen.setBrakeType(coast);
  fifteen.driveStraightGyro(8, 30, reverse, 5, 5);
  
  fifteen.gyroTurn(true, 180);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, false);
  wait(1000, msec);
  fifteen.driveStraight(11.5, 30, reverse, 5, 10);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, false);
  wait(1000, msec);
  fifteen.driveStraightGyro(12, 30, forward, 5, 5);
  fifteen.gyroTurn(true, 180);


}

int vcat300Skills() {


  float lowArmAngle = 0;

  fifteen.waitGyroCallibrate();
  fifteen.gyroSensor.setHeading(285, deg);

  fifteen.frontArmL.resetRotation();
  fifteen.frontArmR.resetRotation();
  fifteen.backLiftL.resetRotation();
  fifteen.backLiftR.resetPosition();

  //fifteen.backLiftL.spin(forward, 0, pct);
  //fifteen.backLiftR.spin(forward, 0, pct);

  // grab home goal
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, true);
  fifteen.driveStraight(10, 30, reverse, 5, 5);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);
  fifteen.driveStraight(17, 30, forward, 10, 5);
  //wait(300, msec);

  // get yellow
  fifteen.gyroTurn(true, 93);
  fifteen.driveStraightGyro(20, 55, forward, 5, 0);
  fifteen.goForwardVision(YELLOW, 30, forward, 32, 10, nullptr);
  fifteen.driveStraightGyro(5, 30, forward, 5, 5);
  fifteen.clawDown(); // grab yellow
  wait(100, msec);
  fifteen.moveArmTo(200, 100, false);
  fifteen.gyroTurnU(0);

  // drop yellow off
  fifteen.driveStraightGyro(40, 100, forward, 5, 10, true, {}, 3);
  fifteen.clawUp();
  fifteen.driveStraightGyro(1, 30, reverse, 5, 0);
  fifteen.gyroTurnU(330);
  fifteen.driveStraightGyro(10, 50, reverse, 10, 10, true, {}, 3);
  fifteen.moveArmTo(lowArmAngle, 100, false);

  // get red
  fifteen.gyroTurnU(270);
  //wait(300, msec);
  //fifteen.driveStraight(7, 50, reverse, 5, 5); // go back a little for better vision alignment
  fifteen.alignToGoalVision(RED, true, forward, 5);
  wait(100, msec);
  fifteen.goForwardVision(RED, 25, forward, 20, 5);
  fifteen.clawDown(); // clamp red
  wait(100, msec);
  fifteen.driveStraightGyro(4, 30, reverse, 3, 3);
  fifteen.moveArmTo(600, 100);
  fifteen.driveStraightTimed(30, forward, 1.75); // align with wall
  fifteen.gyroSensor.setHeading(270, deg); // recallibrate initial heading since squared with wall
  //wait(300, msec);

  fifteen.driveStraightGyro(33, 50, reverse, 10, 10);
  fifteen.moveArmTo(200, 60, false);
  fifteen.gyroTurnU(180);
  fifteen.driveStraightGyroHeading(40, 100, 180, forward, 10, 10, {}, 5);
  fifteen.driveTurn(2, 15, true, 2); // fast slight turn
  wait(200, msec);
  fifteen.driveStraightGyroHeading(23, 100, 180, forward, 10, 10, {}, 5);
  fifteen.clawUp(); // drop off red
  wait(100, msec);

  // get blue across field
  fifteen.moveArmTo(lowArmAngle, 100, false);
  fifteen.driveStraightGyro(6.5, 35, reverse, 10, 5);
  fifteen.gyroTurnU(90);
  fifteen.driveStraightGyroHeading(40, 100, 90, forward, 10, 5, {}, 5);
  fifteen.goForwardVision(BLUE, 40, forward, 53, 8, nullptr);
  fifteen.clawDown(); // clamp blue
  wait(100, msec);

  // Wall align
  fifteen.driveStraightGyro(4, 20, reverse, 5, 0);
  fifteen.moveArmTo(600, 100, true);
  fifteen.driveStraightTimed(30, forward, 1.5); // align with wall

  // multi step turn
  fifteen.driveStraightGyro(20, 20, reverse, 10, 5);
  fifteen.gyroTurnU(70);
  fifteen.driveStraightGyro(10, 20, forward, 10, 5);

  // Head to blue platform areas
  fifteen.gyroTurnU(0);
  fifteen.driveStraightGyroHeading(80, 70, 0, forward, 10, 15, {}, 5); // drive to platform side
  fifteen.driveStraightTimed(20, forward, 1.5);

  // align to platform
  fifteen.driveStraightGyro(4, 10, reverse, 2, 2);
  fifteen.gyroTurnU(285);
  fifteen.driveStraightGyroHeading(7, 30, 285, forward, 5, 5);
  fifteen.gyroTurnU(270); // aim platform side


  // Wall align to the back
  fifteen.driveStraightTimed(20, reverse, 1.5);
  
  fifteen.driveStraightGyroHeading(20, 25, 270, forward, 5, 5);

  // climb
  fifteen.moveArmTo(100, 100);


  fifteen.driveStraightGyroHeading(38, 30, 270, forward, 50, 5);
  wait(350, msec);
  fifteen.driveStraightGyroHeading(3.8, 20, 270, forward, 50, 4);

  fifteen.setBrakeType(hold);


  return 0;

}

int logDistance() {

  fifteen.leftMotorA.resetRotation();
  fifteen.rightMotorA.resetRotation();
  fifteen.gyroSensor.resetRotation();

  while (true) {

    if (fifteen.buttons.pressed(Buttons::A)) {
      fifteen.leftMotorA.resetRotation();
      fifteen.rightMotorA.resetRotation();
      fifteen.gyroSensor.resetRotation();
    }

    float left = degreesToDistance(fifteen.leftMotorA.rotation(degrees));
    float right = degreesToDistance(fifteen.rightMotorA.rotation(degrees));
    log("%f %f %f", left, right, fifteen.gyroSensor.rotation());
    log("%f %f %f", left, right, fifteen.gyroSensor.rotation());
    wait(20, msec);
  }

  return 0;
}

int testTurn() {


  fifteen.waitGyroCallibrate();
  fifteen.gyroTurn(true, 90);
  logController("done");
  return 0;

}

int testTurn2() {
  fifteen.waitGyroCallibrate();
  fifteen.driveTurn(0.75, 15, true, 2); // fast slight turn
  wait(200, msec);
  return 0;
}


void autonomous() { task auto1(vcat300Skills); }
//void autonomous() { thread auto1(logDistance); }

void userControl(void) { task controlLoop1(mainTeleop); }
//void userControl(void) { task controlLoop1(logDistance); }

int main() {

  fifteen.gyroSensor.calibrate();
  fifteen.leftMotorA.resetRotation();
  fifteen.rightMotorA.resetRotation();
  fifteen.gyroSensor.resetRotation();
  fifteen.backLiftL.resetRotation();
  fifteen.backLiftR.resetRotation();
  
  
  // DRIVER SKILLS TRUE, OTHERWISE FALSE
  //fifteen.setTransmission(false);

  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  //platformClimb2();

  while (true) {
    wait(100, msec);
  }
}
