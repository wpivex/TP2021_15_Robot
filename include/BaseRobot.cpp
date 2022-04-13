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

void BaseRobot::goCurve(float distInches, float maxSpeed, float turnPercent, float rampUpFrames, float slowDownInches, 
bool stopAfter, float rampMinSpeed, float slowMinSpeed) {
  float timeout = 5;

  Trapezoid trap(fabs(distInches), maxSpeed, slowMinSpeed, rampUpFrames, slowDownInches, rampMinSpeed);

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
void BaseRobot::goForwardU_Abstract(float K_P, float distInches, float maxSpeed, float universalAngle, float rampUpFrames, float slowDownInches, 
bool stopAfter, float rampMinSpeed, float slowDownMinSpeed, float timeout) {

  Trapezoid trap(distInches, maxSpeed, slowDownMinSpeed, rampUpFrames, slowDownInches, rampMinSpeed);
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

void BaseRobot::goForward(float distInches, float maxSpeed, float rampUpFrames, float slowDownInches, bool stopAfter, 
        float rampMinSpeed, float slowDownMinSpeed, float timeout) {
  goForwardU(distInches, maxSpeed, getAngle(), rampUpFrames, slowDownInches, stopAfter, rampMinSpeed, slowDownMinSpeed, timeout);
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

// Go forward until the maximum distance is hit or the timeout is reached
// for indefinite timeout, set to -1
void BaseRobot::goVision_Abstract(float K_P, float MIN_SPEED, int32_t CAMERA_PORT, float distInches, float speed, Goal goal,
  float rampUpFrames, float slowDownInches, bool stopAfter, float timeout) {

  Trapezoid trapDist(distInches, speed, MIN_SPEED, rampUpFrames, slowDownInches);
  PID pidTurn(K_P, 0, 0);

  vision camera(CAMERA_PORT, goal.bright, goal.sig);

  int startTime = vex::timer::system();
  resetEncoderDistance();

  // forward until the maximum distance is hit, the timeout is reached, or limitSwitch is turned on
  while (!trapDist.isCompleted() && !isTimeout(startTime, timeout)) {

    camera.takeSnapshot(goal.sig);
    
    float correction = camera.largestObject.exists ? pidTurn.tick((VISION_CENTER_X-camera.largestObject.centerX) / VISION_CENTER_X) : 0;
    float distDegrees = fmin(getLeftEncoderDistance(), getRightEncoderDistance()); // take smaller of two distances because arcs
    float speed = trapDist.tick(degreesToDistance(distDegrees));

    setLeftVelocity(forward, speed - correction);
    setRightVelocity(forward, speed + correction);

    wait(20, msec);
  }
  if (stopAfter) {
    stopLeft();
    stopRight();
  }
}

// Align to the goal of specified color with PID
void BaseRobot::goAlignVision_Abstract(float K_P, float K_I, float K_D, float TOLERANCE, float REPEATED, float MINIMUM, int32_t CAMERA_PORT, 
    Goal goal, float timeout, bool stopAfter) {

  vision camera(CAMERA_PORT, goal.bright, goal.sig);

  int startTime = vex::timer::system();
  float speed = 0;

  PID vTurnPID(K_P, K_I, K_D, TOLERANCE, REPEATED, MINIMUM);

  while (!vTurnPID.isCompleted() && !isTimeout(startTime, timeout)) {

    camera.takeSnapshot(goal.sig);

    if (camera.largestObject.exists) speed = vTurnPID.tick((VISION_CENTER_X-camera.largestObject.centerX) / VISION_CENTER_X);

    setLeftVelocity(reverse, speed);
    setRightVelocity(forward, speed);

    wait(20, msec);
  }

  if (stopAfter) {
    stopLeft();
    stopRight();
  }
}

void BaseRobot::basicDriveTeleop() {

  float drive = buttons.axis(Buttons::LEFT_VERTICAL);
  float turn = buttons.axis(Buttons::RIGHT_HORIZONTAL);
  float max = std::max(1.0, std::max(fabs(drive+turn), fabs(drive-turn)));
  setLeftVelocity(forward,100 * (drive+turn)/max);
  setRightVelocity(forward,100 * (drive-turn)/max);
  
}