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

  fifteen.turnToUniversalAngleGyro(0, 4, 0, 5);
  fifteen.dumbUniversalGyroTurn(0, 4, 10);

  fifteen.driveStraightTimed(20, reverse, 3);

  fifteen.driveStraight(15, 30, forward, 5, 5);

  fifteen.clawUp();
  wait(100, msec);
  fifteen.setBrakeType(coast);
  fifteen.driveStraightGyro(8, 30, reverse, 5, 5);
  
  fifteen.dumbGyroTurn(true, 180, 4, 10);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, false);
  wait(1000, msec);
  fifteen.driveStraight(11.5, 30, reverse, 5, 10);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, false);
  wait(1000, msec);
  fifteen.driveStraightGyro(12, 30, forward, 5, 5);
  fifteen.dumbGyroTurn(true, 180, 8, 10);


}

int vcat300Skills() {


  float lowArmAngle = 0;
  float turnSpeed = 5;

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
  fifteen.dumbGyroTurn(true, 90, turnSpeed, 5); // rotate to yellow
  wait(300, msec);
  fifteen.driveStraightGyro(20, 55, forward, 5, 0);
  fifteen.goForwardVision(YELLOW, 30, forward, 32, 10, nullptr);
  fifteen.driveStraightGyro(5, 30, forward, 5, 5);
  fifteen.clawDown(); // grab yellow
  wait(300, msec);
  fifteen.moveArmTo(200, 100, false);
  fifteen.dumbUniversalGyroTurn(2, 4, 5);
  wait(300, msec);

  // drop yellow off
  fifteen.driveStraightGyro(40, 90, forward, 5, 10);
  fifteen.clawUp();
  fifteen.driveStraightGyro(1, 30, reverse, 5, 0);
  fifteen.dumbUniversalGyroTurn(330, 3, 10);
  wait(300, msec);
  fifteen.driveStraightGyro(10, 50, reverse, 10, 10);
  fifteen.moveArmTo(lowArmAngle, 100, false);

  // get red
  fifteen.dumbUniversalGyroTurn(270, turnSpeed, 5); // turn to red
  //wait(300, msec);
  //fifteen.driveStraight(7, 50, reverse, 5, 5); // go back a little for better vision alignment
  fifteen.alignToGoalVision(RED, true, forward, 5);

  wait(300, msec);
  fifteen.goForwardVision(RED, 30, forward, 13, 5);
  fifteen.clawDown(); // clamp red
  wait(300, msec);
  fifteen.driveStraightGyro(4, 30, reverse, 3, 3);
  fifteen.moveArmTo(600, 100);
  fifteen.driveStraightTimed(30, forward, 2); // align with wall
  fifteen.gyroSensor.setHeading(270, deg); // recallibrate initial heading since squared with wall
  //wait(300, msec);

  fifteen.driveStraightGyro(23.5, 30, reverse, 10, 10);
  fifteen.dumbUniversalGyroTurn(180, 7, 5);
  wait(300, msec);
  fifteen.driveStraightGyroHeading(40, 100, 180, forward, 10, 10);
  fifteen.dumbGyroTurn(true, 10, 5, 5);
  fifteen.driveStraightGyroHeading(23, 100, 180, forward, 10, 10);
  fifteen.clawUp(); // drop off red
  wait(300, msec);

  // get blue across field
  fifteen.moveArmTo(lowArmAngle, 100, false);
  fifteen.driveStraightGyro(6, 40, reverse, 10, 5);
  fifteen.dumbUniversalGyroTurn(90, turnSpeed, 5);
  fifteen.driveStraightGyroHeading(30, 100, 90, forward, 10, 5, {}, 5);
  fifteen.goForwardVision(BLUE, 40, forward, 46, 5, nullptr);
  fifteen.clawDown(); // clamp blue
  wait(300, msec);

  // Wall align
  fifteen.driveStraightGyro(4, 20, reverse, 5, 0);
  fifteen.moveArmTo(600, 100, true);
  fifteen.driveStraightTimed(30, forward, 2); // align with wall

  // multi step turn
  fifteen.driveStraightGyro(18, 20, reverse, 10, 5);
  fifteen.dumbUniversalGyroTurn(70, turnSpeed, 5);
  fifteen.driveStraightGyro(14, 20, forward, 10, 5);

  // Head to blue platform area
  fifteen.dumbUniversalGyroTurn(0, turnSpeed, 7); // aim to platform side
  fifteen.driveStraightGyroHeading(80, 70, 0, forward, 10, 15); // drive to platform side
  fifteen.driveStraightTimed(20, forward, 1.5);

  // align to platform
  fifteen.driveStraightGyro(2, 10, reverse, 2, 2);
  fifteen.dumbUniversalGyroTurn(285, turnSpeed, 5); // aim to platform side
  fifteen.driveStraightGyroHeading(7, 30, 285, forward, 5, 5);
  fifteen.dumbUniversalGyroTurn(270, turnSpeed, 5); // aim to platform side

  
  wait(300, msec);
  fifteen.driveStraightGyroHeading(7, 25, 270, forward, 5, 5);

  // climb
  fifteen.moveArmTo(100, 100);

  wait(500, msec);

  fifteen.driveStraightGyroHeading(16, 30, 270, forward, 50, 5);
  wait(350, msec);
  fifteen.driveStraightGyroHeading(2, 30, 270, forward, 50, 4);

  fifteen.leftMotorA.setBrake(hold);
  fifteen.leftMotorB.setBrake(hold);
  fifteen.leftMotorC.setBrake(hold);
  fifteen.leftMotorD.setBrake(hold);
  fifteen.rightMotorA.setBrake(hold);
  fifteen.rightMotorB.setBrake(hold);
  fifteen.rightMotorC.setBrake(hold);
  fifteen.rightMotorD.setBrake(hold);

  return 0;

}

void platformClimb3() {
  fifteen.waitGyroCallibrate();

  fifteen.clawDown();
  wait(1000, msec);
  fifteen.moveArmTo(600, 100);
  fifteen.driveStraightGyro(11, 30, forward, 50, 5);
  fifteen.moveArmTo(100, 100);

  fifteen.driveStraightGyro(31.5, 30, forward, 50, 5);
  wait(350, msec);
  fifteen.driveStraightGyro(4, 30, forward, 50, 4);

  fifteen.setBrakeType(hold);
}



void platformClimb2() {
  fifteen.waitGyroCallibrate();

  fifteen.clawDown();
  wait(1000, msec);

  bool climbing = false;
  double pitch = 0;
  double angle = 18;
  double turn = 0;

  float SPEED = 15;

  while (!climbing || pitch > angle) {

    pitch = fifteen.gyroSensor.roll() - initialPitch;

    if (pitch > angle + 2) climbing = true;
    logController("%d %f", climbing ? 1 : 0, pitch);

    fifteen.setLeftVelocity(forward, SPEED - turn);
    fifteen.setRightVelocity(forward, SPEED + turn);

    wait(20, msec);
  }
  fifteen.driveStraight(25, 60, reverse, 10, 5);


  while (true) {
    pitch = fifteen.gyroSensor.roll() - initialPitch;
    logController("%d %f", climbing ? 1 : 0, pitch);
    wait(20, msec);
  }
}

void platformClimb() {

  fifteen.waitGyroCallibrate();
  

  double PITCH_SCALE = 70;
  double YAW_SCALE = 50;
  double speed, turn, left, right;

  while (true) {

    double pitch = fifteen.gyroSensor.roll() - initialPitch;
    double yaw = fifteen.gyroSensor.yaw();

    // normalized pitch is between -1 and 1, bounded to 30 degrees
    double pitchN = fmin(1, fmax(-1, (pitch / 30.0)));
    double yawN = fmin(1, fmax(-1, (yaw / 30.0)));

    pitch = pow(pitch, 5);


    speed = pitchN * PITCH_SCALE;
    turn = yawN * YAW_SCALE;

    left = fmax(-100, fmin(speed - turn, 100));
    right = fmax(-100, fmin(speed + turn, 100));


    fifteen.setLeftVelocity(forward, left);
    fifteen.setRightVelocity(forward, right);

    logController("%f,%f,%f", yaw, left, right);
    log("%f, %f", left, right);

    wait(20, msec);
  }

  fifteen.stopLeft();
  fifteen.stopRight();

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
