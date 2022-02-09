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

void autonomous() { thread auto1(platformClimb3); }

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
