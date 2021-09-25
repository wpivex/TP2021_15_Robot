#include "robot.h"
#include <math.h>

// Motor ports Left: 12T, 14F,  20T Right: 2F, 6T, 10F
// gear ratio is 60/36
Robot::Robot(controller* c) : leftMotorA(0), leftMotorB(0), leftMotorC(0), leftMotorD(0), rightMotorA(0), rightMotorB(0), rightMotorC(0), 
  rightMotorD(0) {
  leftMotorA = motor(PORT1, ratio18_1, true);
  leftMotorB = motor(PORT2, ratio18_1, true);
  leftMotorC = motor(PORT3, ratio18_1, true);
  leftMotorD = motor(PORT4, ratio18_1, true);
  rightMotorA = motor(PORT5, ratio18_1, false);
  rightMotorB = motor(PORT6, ratio18_1, false);
  rightMotorC = motor(PORT7, ratio18_1, false);
  rightMotorD = motor(PORT8, ratio18_1, false);
  driveType = ARCADE;
  robotController = c; 
}

void Robot::teleop() {
  robotController->Screen.clearScreen();

  digital_out drivePiston1 = digital_out(triportPorts.A);
  digital_out drivePiston2 = digital_out(triportPorts.B);

  float leftJoystick = (driveType == ARCADE) ? robotController->Axis3.position()^3 + robotController->Axis1.position()^3: robotController->Axis3.position()^3;
  float rightJoystick = (driveType == ARCADE) ? robotController->Axis3.position()^3 + robotController->Axis1.position()^3: robotController->Axis2.position()^3;

  if (fabs(leftJoystick) > 5) {
    float percent = (driveType == ARCADE) ? (pow((robotController->Axis3.position()/100.00f), 3.00f) + pow((robotController->Axis1.position()/100.00f), 5.00f))*100.00f : leftJoystick;
    setLeftVelocity(forward, percent);
  } else {
    stopLeft();
  }

  if (fabs(rightJoystick) > 5) {
    float percent = (driveType == ARCADE) ? (pow((robotController->Axis3.position()/100.00f), 3.00f) - pow((robotController->Axis1.position()/100.00f), 5.00f))*100.00f : rightJoystick;
    setRightVelocity(forward, percent);
  } else {
    stopRight();
  }

  if(robotController->ButtonA.pressing()) {
    drivePiston1.set(drivePiston1.value());
    drivePiston2.set(drivePiston2.value());
  }
}

void Robot::driveStraight(float percent, float dist, float accPercent) {

}

void Robot::driveTimed(float percent, float driveTime) {
  int milliseconds = vex::timer::system();
  while (vex::timer::system() < milliseconds + driveTime) {
    setLeftVelocity(forward, percent);
    setRightVelocity(forward, percent);
  }
  stopLeft();
  stopRight();
}

void Robot::driveStraight(float percent, float dist) {
  driveStraight(percent, dist, 0.1); //acc is from 0 to 1
}

void Robot::turnToAngle(float percent, float turnAngle) {

}

void Robot::setLeftVelocity(directionType d, double percent) {
  leftMotorA.spin(d, percent, percentUnits::pct);
  leftMotorB.spin(d, percent, percentUnits::pct);
  leftMotorC.spin(d, percent, percentUnits::pct);
  leftMotorD.spin(d, percent, percentUnits::pct);
}

void Robot::setRightVelocity(directionType d, double percent) {
  rightMotorA.spin(d, percent, percentUnits::pct);
  rightMotorB.spin(d, percent, percentUnits::pct);
  rightMotorC.spin(d, percent, percentUnits::pct);
  rightMotorD.spin(d, percent, percentUnits::pct);
}

void Robot::stopLeft() {
  leftMotorA.stop();
  leftMotorB.stop();
  leftMotorC.stop();
  leftMotorD.stop();
}

void Robot::stopRight() {
  rightMotorA.stop();
  rightMotorB.stop();
  rightMotorC.stop();
  rightMotorD.stop();
}