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

void userControl(void) { task controlLoop1(mainTeleop); }


void mainAuto() {

  fifteen.waitGyroCallibrate();

}

void vcat300Skills() {


  float lowArmAngle = 10;
  float turnSpeed = 10;

  fifteen.waitGyroCallibrate();
  fifteen.gyroSensor.setHeading(284, deg);

  fifteen.frontArmL.resetRotation();
  fifteen.frontArmR.resetRotation();

  // grab home goal
  fifteen.driveStraight(10, 30, forward, 10, 5);
  //wait(300, msec);
  fifteen.turnToAngleGyro(true, 90, turnSpeed, 20, 5); // rotate to yellow
  wait(300, msec);
  fifteen.goForwardVision(YELLOW, 45, forward, 40, 10, nullptr);
  fifteen.clawDown(); // grab yellow
  wait(300, msec);
  fifteen.moveArmTo(200, 100);
  fifteen.turnToUniversalAngleGyro(3, 4, 3, 10);
  wait(300, msec);

  // drop yellow off
  fifteen.driveStraightGyro(37, 90, forward, 10, 10);
  fifteen.clawUp();
  fifteen.turnToUniversalAngleGyro(330, 4, 5, 10);
  wait(300, msec);
  fifteen.driveStraightGyro(13, 50, reverse, 10, 10);
  fifteen.moveArmTo(lowArmAngle, 100);

  // get red
  fifteen.turnToUniversalAngleGyro(270, turnSpeed, 5, 10); // turn to red
  //wait(300, msec);
  //fifteen.driveStraight(7, 50, reverse, 5, 5); // go back a little for better vision alignment
  wait(1000, msec);
  fifteen.alignToGoalVision(RED, true, forward, 5);

  wait(300, msec);
  fifteen.driveStraightGyro(13, 30, forward, 5, 5);
  fifteen.clawDown(); // clamp red
  wait(300, msec);
  fifteen.driveStraightGyro(3, 30, reverse, 3, 3);
  fifteen.moveArmTo(600, 100);
  fifteen.driveStraightTimed(30, forward, 2); // align with wall
  fifteen.gyroSensor.setHeading(270, deg); // recallibrate initial heading since squared with wall
  //wait(300, msec);

  fifteen.driveStraightGyro(18, 40, reverse, 10, 10);
  fifteen.turnToAngleGyro(false, 86, 7, 0, 10);
  wait(300, msec);
  fifteen.driveStraightGyro(65, 100, forward, 10, 10);
  fifteen.clawUp(); // drop off red
  wait(300, msec);

  // get blue across field
  fifteen.driveStraightGyro(6.5, 40, reverse, 10, 5);
  fifteen.turnToUniversalAngleGyro(90, turnSpeed, 10, 5);
  fifteen.moveArmTo(lowArmAngle, 100);
  fifteen.driveStraightGyro(32, 100, forward, 10, 10);
  fifteen.goForwardVision(BLUE, 45, forward, 39, 10, nullptr);
  fifteen.clawDown(); // clamp blue
  wait(300, msec);

  // Wall align
  fifteen.moveArmTo(600, 100);
  fifteen.driveStraightTimed(30, forward, 2); // align with wall
  fifteen.driveStraightGyro(5, 30, reverse, 10, 5);

  // Head to blue platform area
  fifteen.turnToUniversalAngleGyro(0, turnSpeed, 15, 10); // aim to platform side
  fifteen.driveStraightGyro(64, 70, forward, 10, 15);
  fifteen.driveStraightTimed(30, forward, 1.5);

  // align to platform
  fifteen.driveStraightGyro(3, 40, reverse, 5, 5);
  fifteen.turnToAngleGyro(false, 90, turnSpeed, 15, 5);
  wait(300, msec);
  fifteen.driveStraightTimed(30, forward, 2); // square with platform
  fifteen.driveStraightGyro(7, 30, reverse, 5, 5);

  // climb
  fifteen.moveArmTo(100, 100);

  wait(500, msec);

  fifteen.driveStraightGyro(31.5, 30, forward, 50, 5);
  wait(350, msec);
  fifteen.driveStraightGyro(4, 30, forward, 50, 4);

  fifteen.leftMotorA.setBrake(hold);
  fifteen.leftMotorB.setBrake(hold);
  fifteen.leftMotorC.setBrake(hold);
  fifteen.leftMotorD.setBrake(hold);
  fifteen.rightMotorA.setBrake(hold);
  fifteen.rightMotorB.setBrake(hold);
  fifteen.rightMotorC.setBrake(hold);
  fifteen.rightMotorD.setBrake(hold);





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

  fifteen.leftMotorA.setBrake(hold);
  fifteen.leftMotorB.setBrake(hold);
  fifteen.leftMotorC.setBrake(hold);
  fifteen.leftMotorD.setBrake(hold);
  fifteen.rightMotorA.setBrake(hold);
  fifteen.rightMotorB.setBrake(hold);
  fifteen.rightMotorC.setBrake(hold);
  fifteen.rightMotorD.setBrake(hold);

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

void autonomous() { thread auto1(vcat300Skills); }

int main() {

  fifteen.gyroSensor.calibrate();
  
  // DRIVER SKILLS TRUE, OTHERWISE FALSE
  //fifteen.setTransmission(false);

  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  //platformClimb2();

  while (true) {
    wait(100, msec);
  }
}
