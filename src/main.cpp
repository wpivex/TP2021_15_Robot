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

void autonomous() { thread auto1(mainAuto); }

void platformClimb() {

  fifteen.waitGyroCallibrate();

  fifteen.gyroSensor.resetRotation();

  double PITCH_SCALE = 3.5;
  double YAW_SCALE = 2;
  double speed, turn, left, right;

  while (true) {

    double pitch = -fifteen.gyroSensor.roll(); // gyro sensor is rotated
    double yaw = fifteen.gyroSensor.yaw();

    // normalized pitch is between -1 and 1, bounded to 30 degrees
    double pitchN = fmin(1, fmax(-1, (pitch / 30.0)));



    speed = 0 - pitch * PITCH_SCALE;
    turn = yaw * YAW_SCALE;
    turn = 0;

    left = fmax(-100, fmin(speed - turn, 100));
    right = fmax(-100, fmin(speed + turn, 100));

    //fifteen.setLeftVelocity(forward, left);
    //fifteen.setRightVelocity(forward, right);

    //logController("%f,%f,%f,%f", pitch, yaw, left, right);
    logController("%f", pitchN);

    wait(100, msec);
  }

  fifteen.stopLeft();
  fifteen.stopRight();

}

int main() {

  fifteen.gyroSensor.calibrate();
  
  // DRIVER SKILLS TRUE, OTHERWISE FALSE
  //fifteen.setTransmission(false);

  //Competition.autonomous(autonomous);
  //Competition.drivercontrol(userControl);

  platformClimb();

  while (true) {
    wait(100, msec);
  }
}
