#include "BaseRobot.h"

BaseRobot::BaseRobot(int32_t gyroPort): gyroSensor(gyroPort) {
  
}

float BaseRobot::getAngle() {
  return gyroSensor.heading();
}

void BaseRobot::waitGyroCallibrate() {
  if (gyroSensor.isCalibrating()) {
    int i = 0;
    while (gyroSensor.isCalibrating()) {
      wait(20, msec);
      i++;
    }
    gyroSensor.resetRotation();
    wait(1000, msec);
  }
  
  wait(500, msec);
  log("done calibration");
}

// If the robot is known to have a given heading (i.e. from wall align) and the gyro heading is close enough to heading, recalibrate gyro heading
void BaseRobot::possiblyResetGyro(float targetAngle, float angleTolerance) {

  if (fabs(getAngleDiff(targetAngle, getAngle())) < angleTolerance) {
    logController("YES set heading\nfrom:%f\nto:%f", getAngle(), targetAngle);
    gyroSensor.setHeading(targetAngle, degrees);
  } else {
    logController("NO set heading\nfrom:%f\nto:%f", getAngle(), targetAngle);
  }
}

void BaseRobot::setMotorVelocity(motor m, directionType d, double percent) {
  if (percent < 0) {
    d = (d == forward) ? reverse : forward;
    percent = -percent;
  }
  m.spin(d, percent / 100.0 * 12.0, voltageUnits::volt);
}

void BaseRobot::goCurve(float distInches, float maxSpeed, float turnPercent, float rampUpInches, float slowDownInches, 
bool stopAfter, float rampMinSpeed, float slowMinSpeed) {
  float timeout = 5;

  Trapezoid trap(fabs(distInches), maxSpeed, slowMinSpeed, rampUpInches, slowDownInches, rampMinSpeed);

  int startTime = vex::timer::system();
  resetEncoderDistance();
  directionType dir = distInches > 0 ? forward : reverse;

  while (!trap.isCompleted() && !isTimeout(startTime, timeout)) {
  
    float speed = trap.tick(fabs(getEncoderDistance()));

    // turnPercent bounded between -1 (counterclockwise point turn) and 1 (clockwise point turn)
    float lspeed, rspeed;
    if (turnPercent >= 0) {
      lspeed = 1;
      rspeed = 1 - 2*turnPercent;
    } else {
      rspeed = 1;
      lspeed = 1 + 2*turnPercent;
    }

    setLeftVelocity(dir, lspeed * speed);
    setRightVelocity(dir, rspeed * speed);

    wait(20, msec);
  }

  if (stopAfter) {
    stopLeft();
    stopRight();
  }
}

void BaseRobot::goForwardTimed(float duration, float speed) {

  int startTime = vex::timer::system();

  while (!isTimeout(startTime, duration)) {
    setLeftVelocity(forward, speed);
    setRightVelocity(forward, speed);
    wait(20, msec);
  }
  stopLeft();
  stopRight();
}

// Go forward a number of inches, maintaining a specific heading
void BaseRobot::goForwardU_Abstract(float K_P, float distInches, float maxSpeed, float universalAngle, float rampUpInches, float slowDownInches, 
bool stopAfter, float rampMinSpeed, float slowDownMinSpeed, float timeout) {

  Trapezoid trap(distInches, maxSpeed, slowDownMinSpeed, rampUpInches, slowDownInches, rampMinSpeed);
  PID turnPID(K_P, 0.00, 0);

  float correction = 0;
  float currDist;
  int startTime = vex::timer::system();
  
  resetEncoderDistance();

  //log("start forward %f %f %f", distInches, startX, startY);

  while (!trap.isCompleted() && !isTimeout(startTime, timeout)) {

    currDist = getEncoderDistance();
    
    float speed = trap.tick(currDist);
    float ang = getAngleDiff(universalAngle, getAngle());
    correction = (universalAngle == -1) ? 0 : turnPID.tick(ang);
 
    setLeftVelocity(forward, speed + correction);
    setRightVelocity(forward, speed - correction);

    //log("Target: %f\nActual:%f\nLeft:%f\nRight:%f\n", universalAngle, getAngle(), speed+correction, speed-correction);
    //log("%f", gyroSensor.heading());

    wait(20, msec);
  }
  if (stopAfter) {
    stopLeft();
    stopRight();
  }
  //log("straight done");
}

void BaseRobot::goForward(float distInches, float maxSpeed, float rampUpInches, float slowDownInches, bool stopAfter, 
        float rampMinSpeed, float slowDownMinSpeed, float timeout) {
  goForwardU(distInches, maxSpeed, getAngle(), rampUpInches, slowDownInches, stopAfter, rampMinSpeed, slowDownMinSpeed, timeout);
}

// Turn to some universal angle based on starting point. Turn direction is determined by smallest angle to universal angle
void BaseRobot::goTurnU_Abstract(float KP, float KI, float KD, float TOLERANCE, float REPEATED, float MINUMUM,
float universalAngleDegrees, bool stopAfter, float timeout, float maxSpeed) {

  PID anglePID(KP, KI, KD, TOLERANCE, REPEATED, MINUMUM, maxSpeed);

  float speed;

  log("initing");
  int startTime = vex::timer::system();
  log("about to loop");

  while (!anglePID.isCompleted() && !isTimeout(startTime, timeout)) {

    float ang = getAngleDiff(universalAngleDegrees, getAngle());
    speed = anglePID.tick(ang);

    //log("Turn \nTarget: %f \nCurrent: %f \nDiff: %f\nSpeed: %f \nGPS: %f", universalAngleDegrees, getAngle(), ang, speed, GPS11.heading());
    //log("heading: %f", GPS11.heading());
    log("%f", getAngle());

    setLeftVelocity(forward, speed);
    setRightVelocity(reverse, speed);

    wait(20, msec);
  }
  //log("wtf done");

  if (stopAfter) {
    stopLeft();
    stopRight();
  }  
}