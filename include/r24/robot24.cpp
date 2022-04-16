#include "robot24.h"

// Motor ports Left: 1R, 2F, 3F,  20T Right: 12R, 11F, 13F
// gear ratio is 60/36
Robot24::Robot24() : BaseRobot(PORT2), leftMotorA(0), leftMotorB(0), leftMotorC(0), leftMotorD(0), leftMotorE(0), 
  rightMotorA(0), rightMotorB(0), rightMotorC(0), rightMotorD(0), rightMotorE(0), rightArm1(0), rightArm2(0), 
  leftArm1(0), leftArm2(0) {

  leftMotorA = motor(PORT7, ratio18_1, true); 
  leftMotorB = motor(PORT6, ratio18_1, true);
  leftMotorC = motor(PORT3, ratio18_1, true);
  leftMotorD = motor(PORT4, ratio18_1, true);
  leftMotorE = motor(PORT5, ratio18_1, true);

  rightMotorA = motor(PORT16, ratio18_1, false);
  rightMotorB = motor(PORT11, ratio18_1, false);
  rightMotorC = motor(PORT12, ratio18_1, false);
  rightMotorD = motor(PORT15, ratio18_1, false);
  rightMotorE = motor(PORT14, ratio18_1, false);

  rightArm1 = motor(PORT10, ratio36_1, true);
  rightArm2 = motor(PORT20, ratio36_1, true);
  leftArm1 = motor(PORT8, ratio36_1, false);
  leftArm2 = motor(PORT18, ratio36_1, false);  

  rightArm1.setBrake(hold);
  rightArm2.setBrake(hold);
  leftArm1.setBrake(hold);
  leftArm2.setBrake(hold);


  FRONT_CAMERA_PORT = PORT10;
  BACK_CAMERA_PORT = PORT9;

  driveType = TWO_STICK_ARCADE;

  setControllerMapping(DEFAULT_MAPPING);
}

void Robot24::setControllerMapping(ControllerMapping mapping) {

  cMapping = mapping;

  if (mapping == DEFAULT_MAPPING) {
    driveType = TWO_STICK_ARCADE;

    FRONT_CLAMP_TOGGLE = Buttons::L1;
    BACK_CLAMP_TOGGLE = Buttons::R1;
    CLAW_TOGGLE = Buttons::A;
  } 
}

void Robot24::driveTeleop() {

  if (driveHold) setBrakeType(hold);
  else setBrakeType(coast);

  if(driveType == TANK) {
    setLeftVelocity(forward,buttons.axis(Buttons::LEFT_VERTICAL));
    setRightVelocity(forward,buttons.axis(Buttons::RIGHT_VERTICAL));
  }else{
    float drive = driveType == ONE_STICK_ARCADE ? buttons.axis(Buttons::RIGHT_VERTICAL) : buttons.axis(Buttons::LEFT_VERTICAL);
    float turn = buttons.axis(Buttons::RIGHT_HORIZONTAL);
    float max = std::max(1.0, std::max(fabs(drive+turn), fabs(drive-turn)));
    setLeftVelocity(forward,100 * (drive+turn)/max);
    setRightVelocity(forward,100 * (drive-turn)/max);
  }
}

void Robot24::goalClamp() {

  if (buttons.pressed(FRONT_CLAMP_TOGGLE)) {
    frontGoal.set(!frontGoal.value());
  } else if (buttons.pressed(BACK_CLAMP_TOGGLE)) {
      backGoal.set(!backGoal.value());
  }
}

void Robot24::clawMovement() {
  if (buttons.pressed(CLAW_TOGGLE)) {
    clawPiston.set(!clawPiston.value());
  }
}

void Robot24::setFrontClamp(bool intaking) {
  frontGoal.set(intaking);
}

void Robot24::setBackClamp(bool intaking) {
  backGoal.set(intaking);
}

void Robot24::armTeleop() {
  if (buttons.pressing(buttons.R2)) {
    setArmPercent(forward, 50);
  } else if (buttons.pressing(buttons.L2)) {
    setArmPercent(reverse, 40);
  } else {
    stopArm();
  }

  // Toggle limiting arm current
  if (buttons.pressed(buttons.X)) {
    teleopArmLimited = !teleopArmLimited;
    if (teleopArmLimited) setMaxArmTorque(CURRENT::MID);
    else setMaxArmTorque(CURRENT::HIGH);
  }
}

// Run every tick
void Robot24::teleop() {
  driveTeleop();
  armTeleop();
  clawMovement();
  goalClamp();
  buttons.updateButtonState();
}

void Robot24::goForwardUntilSensor(float maxDistance, float speed, float rampUpInches, int timeout, bool stopAfter) {

  Trapezoid trap(maxDistance, speed, speed, rampUpInches, 0);

  int startTime = vex::timer::system();
  resetEncoderDistance();
  gyroSensor.resetRotation();

  // finalDist is 0 if we want driveTimed instead of drive some distance
  while (!trap.isCompleted() && !isTimeout(startTime, timeout) && clawSensor.value()) {

    float speed = trap.tick(getEncoderDistance());

    setLeftVelocity(forward, speed);
    setRightVelocity(forward, speed);
    
    wait(20, msec);
  }
  if (stopAfter) {
    stopLeft();
    stopRight();
  }
}

// Go forward a number of inches, maintaining a specific heading
// Calling general function with 24-specifc params

void Robot24::goForwardU(float distInches, float maxSpeed, float universalAngle, float rampUpFrames, float slowDownInches, 
bool stopAfter, float rampMinSpeed, float slowDownMinSpeed, float timeout) {
  BaseRobot::goForwardU_Abstract(20, distInches, maxSpeed, universalAngle, rampUpFrames, slowDownInches, stopAfter, rampMinSpeed, slowDownMinSpeed, timeout);
}

// Turn to some universal angle based on starting point. Turn direction is determined by smallest angle to universal angle
// Calling general function with 24-specifc params
void Robot24::goTurnU(float universalAngleDegrees, bool stopAfter, float timeout, float maxSpeed) {
  BaseRobot::goTurnU_Abstract(2, 0.00, 0.05, 5, 1, 35, universalAngleDegrees, stopAfter, timeout, maxSpeed);
}

// Trapezoidal motion profiling
// Will use gyro sensor *doesn't rn
// distAlongCirc is positive if forward, negative if reverse
// curveDirection is true for right, false for left
void Robot24::goRadiusCurve(float radius, float numRotations, bool curveDirection, float maxSpeed, float rampUp, float slowDown, bool stopAfter, float timeout) {

  float distAlongCircum = numRotations * 2 * M_PI;

  Trapezoid trap(distAlongCircum, maxSpeed, 12, rampUp,slowDown);
  PID anglepid(0.025, 0, 0);


  int startTime = vex::timer::system();
  resetEncoderDistance();

  // Repeat until either arrived at target or timed out
  while (!trap.isCompleted() && !isTimeout(startTime, timeout)) {

    float distSoFar = getEncoderDistance();

    float v_avg = trap.tick(distSoFar); 
    float v_ratio = fabs((radius+DIST_BETWEEN_WHEELS)/(radius-DIST_BETWEEN_WHEELS));

    log("V_avg: %f\nV_diff: %f", v_avg, v_ratio);

    float lPower = v_avg * sqrt(curveDirection ? v_ratio:1/v_ratio);
    float rPower =  v_avg * sqrt(curveDirection ? 1/v_ratio:v_ratio);

    setLeftVelocity(forward, lPower);
    setRightVelocity(forward, rPower);

    wait(20, msec);
  }
  if (stopAfter) {
    stopLeft();
    stopRight();
  }
  log("done");

}

// PID gyro sensor-based curving 
// distInches is positive if forward, negative if reverse
void Robot24::gyroCurve(float distInches, float maxSpeed, float turnAngle, int timeout, bool stopAfter) {

  // Needs tuning desperately
  float kp = 0.015;
  float ki = 0;
  float kd = 0;


  // We have these values somewhere but I'm not sure where
  float distanceInDegrees = distanceToDegrees(distInches);
  
  Trapezoid trap(distInches, maxSpeed, maxSpeed, 0, 0);
  PID anglePID(kp, ki, kd, -1, -1, 0, 0.5);
  float targetAngle = 0;

  int startTime = vex::timer::system();
  resetEncoderDistance();
  gyroSensor.resetRotation();


  // Repeat until either arrived at target or timed out
  while (!trap.isCompleted() && !isTimeout(startTime, timeout)) {

    // Not sure if linear distance is correct / it seems relavtively arbitrary for this function. Approximate me!
    float distanceError = (leftMotorA.position(degrees) + rightMotorA.position(degrees)) / 2;
    targetAngle = turnAngle * fabs(fmax(0.3, fmin(1, (distanceError + 0.3*distanceInDegrees) / (distanceInDegrees))));

    float speed = fmin(100, fmax(-100, trap.tick(distanceError))); 
    float turnDifference = anglePID.tick(targetAngle - gyroSensor.rotation());

    setLeftVelocity(forward, speed * (0.5+turnDifference));
    setRightVelocity(forward, speed * (0.5-turnDifference));

    wait(20, msec);

  }

  logController("done");
  if(stopAfter) {
    stopLeft();
    stopRight();
  }

}

// Go forward until the maximum distance is hit or the timeout is reached
// for indefinite timeout, set to -1
void Robot24::goVision(float distInches, float speed, Goal goal, directionType cameraDir, float rampUpFrames, 
    float slowDownInches, bool stopAfter, float timeout) {
    
  int32_t port = cameraDir == forward ? FRONT_CAMERA_PORT : BACK_CAMERA_PORT;
  BaseRobot::goVision_Abstract(50, FORWARD_MIN_SPEED, port, distInches, speed, goal, rampUpFrames, slowDownInches, stopAfter, timeout);
}

// Align to the goal of specified color with PID
void Robot24::goAlignVision(Goal goal, directionType cameraDir, float timeout, bool stopAfter) {
  int32_t port = cameraDir == forward ? FRONT_CAMERA_PORT : BACK_CAMERA_PORT;
  BaseRobot::goAlignVision_Abstract(70, 0, 0, 0.05, 3, 25, port, goal, timeout, stopAfter);
}

// Same as goAlignVision, but use Trapezoidal instead of PID
void Robot24::goAlignVisionTrap(Goal goal, directionType cameraDir, float timeout, bool stopAfter) {

  vision camera(cameraDir == forward ? FRONT_CAMERA_PORT : BACK_CAMERA_PORT, goal.bright, goal.sig);

  int startTime = vex::timer::system();
  float speed = 0;

  Trapezoid trap(VISION_CENTER_X, 100, 35, 0, 50);


  while (!trap.isCompleted() && !isTimeout(startTime, timeout)) {

    camera.takeSnapshot(goal.sig);

    if (camera.largestObject.exists) speed = trap.tick(camera.largestObject.centerX);

    setLeftVelocity(reverse, speed);
    setRightVelocity(forward, speed);

    wait(20, msec);
  }

  if (stopAfter) {
    stopLeft();
    stopRight();
  }
}

void Robot24::openClaw() {
  clawPiston.set(true);
}

void Robot24::closeClaw() {
  clawPiston.set(false);
}


void Robot24::setLeftVelocity(directionType d, double percent) {
  setMotorVelocity(leftMotorA, d, percent);
  setMotorVelocity(leftMotorB, d, percent);
  setMotorVelocity(leftMotorC, d, percent);
  setMotorVelocity(leftMotorD, d, percent);
  setMotorVelocity(leftMotorE, d, percent);
}

void Robot24::setRightVelocity(directionType d, double percent) {
  setMotorVelocity(rightMotorA, d, percent);
  setMotorVelocity(rightMotorB, d, percent);
  setMotorVelocity(rightMotorC, d, percent);
  setMotorVelocity(rightMotorD, d, percent);
  setMotorVelocity(rightMotorE, d, percent);
}

void Robot24::stopLeft() {
  leftMotorA.stop();
  leftMotorB.stop();
  leftMotorC.stop();
  leftMotorD.stop();
  leftMotorE.stop();
}

void Robot24::stopRight() {
  rightMotorA.stop();
  rightMotorB.stop();
  rightMotorC.stop();
  rightMotorD.stop();
  rightMotorE.stop();
}

void Robot24::stopArm() {
  rightArm1.stop(hold);
  rightArm2.stop(hold);
  leftArm1.stop(hold);
  leftArm2.stop(hold);
}

void Robot24::resetArmRotation() {
  rightArm1.resetRotation();
  rightArm2.resetRotation();
  leftArm1.resetRotation();
  leftArm2.resetRotation();
}


void Robot24::setArmDegrees(float degrees, float speed, bool blocking) {
  rightArm1.spinTo(degrees, deg, speed, velocityUnits::pct, false);
  rightArm2.spinTo(degrees, deg, speed, velocityUnits::pct, false);
  leftArm1.spinTo(degrees, deg, speed, velocityUnits::pct, false);
  leftArm2.spinTo(degrees, deg, speed, velocityUnits::pct, blocking);
}

void Robot24::setArmPercent(directionType d, double percent) {
  if (percent < 0) {
    d = (d == forward) ? reverse : forward;
    percent = -percent;
  }
  rightArm1.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
  rightArm2.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
  leftArm1.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
  leftArm2.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
}

void Robot24::setBrakeType(brakeType b) {
  leftMotorA.setBrake(b);
  leftMotorB.setBrake(b);
  leftMotorC.setBrake(b);
  leftMotorD.setBrake(b);
  leftMotorE.setBrake(b);

  rightMotorA.setBrake(b);
  rightMotorB.setBrake(b);
  rightMotorC.setBrake(b);
  rightMotorD.setBrake(b);
  rightMotorE.setBrake(b);
}

void Robot24::setMaxArmTorque(float c) {
  leftArm1.setMaxTorque(c, currentUnits::amp);
  leftArm2.setMaxTorque(c, currentUnits::amp);
  rightArm1.setMaxTorque(c, currentUnits::amp);
  rightArm2.setMaxTorque(c, currentUnits::amp);
}

void Robot24::setMaxDriveTorque(float c) {
  leftMotorA.setMaxTorque(c, currentUnits::amp);
  leftMotorB.setMaxTorque(c, currentUnits::amp);
  leftMotorC.setMaxTorque(c, currentUnits::amp);
  leftMotorD.setMaxTorque(c, currentUnits::amp);
  leftMotorE.setMaxTorque(c, currentUnits::amp);

  rightMotorA.setMaxTorque(c, currentUnits::amp);
  rightMotorB.setMaxTorque(c, currentUnits::amp);
  rightMotorC.setMaxTorque(c, currentUnits::amp);
  rightMotorD.setMaxTorque(c, currentUnits::amp);
  rightMotorE.setMaxTorque(c, currentUnits::amp);
}

// return in inches
float Robot24::getLeftEncoderDistance() {
  float sum = leftMotorA.rotation(deg) + leftMotorB.rotation(deg) + leftMotorC.rotation(deg) + leftMotorD.rotation(deg) + leftMotorE.rotation(deg);
  return degreesToDistance(sum / 5.0);
}

// return in inches
float Robot24::getRightEncoderDistance() {
  float sum = rightMotorA.rotation(deg) + rightMotorB.rotation(deg) + rightMotorC.rotation(deg) + rightMotorD.rotation(deg) + rightMotorE.rotation(deg);
  return degreesToDistance(sum / 5.0);
}

void Robot24::resetEncoderDistance() {
  leftMotorA.resetRotation();
  rightMotorA.resetRotation();
  leftMotorB.resetRotation();
  rightMotorB.resetRotation();
  leftMotorC.resetRotation();
  rightMotorC.resetRotation();
  leftMotorD.resetRotation();
  rightMotorD.resetRotation();
  leftMotorE.resetRotation();
  rightMotorE.resetRotation();
}