#include "robot24.h"

// Motor ports Left: 1R, 2F, 3F,  20T Right: 12R, 11F, 13F
// gear ratio is 60/36
Robot24::Robot24(controller* c, bool _isSkills) : BaseRobot(PORT16), leftMotorA(0), leftMotorB(0), leftMotorC(0), leftMotorD(0), leftMotorE(0), 
  rightMotorA(0), rightMotorB(0), rightMotorC(0), rightMotorD(0), rightMotorE(0), backCamera(PORT9), frontCamera(PORT10), rightArm1(0), rightArm2(0), 
  leftArm1(0), leftArm2(0), gyroSensor(PORT2) {

  isSkills = _isSkills;

  leftMotorA = motor(PORT7, ratio18_1, true); 
  leftMotorB = motor(PORT6, ratio18_1, true);
  leftMotorC = motor(PORT3, ratio18_1, true);
  leftMotorD = motor(PORT4, ratio18_1, true);
  leftMotorE = motor(PORT5, ratio18_1, true);
  leftDrive = motor_group(leftMotorA, leftMotorB, leftMotorC, leftMotorD, leftMotorE);

  rightMotorA = motor(PORT16, ratio18_1, false);
  rightMotorB = motor(PORT11, ratio18_1, false);
  rightMotorC = motor(PORT12, ratio18_1, false);
  rightMotorD = motor(PORT15, ratio18_1, false);
  rightMotorE = motor(PORT14, ratio18_1, false);
  rightDrive = motor_group(rightMotorA, rightMotorB, rightMotorC, rightMotorD, rightMotorE);

  rightArm1 = motor(PORT10, ratio36_1, true);
  rightArm2 = motor(PORT20, ratio36_1, true);
  leftArm1 = motor(PORT8, ratio36_1, false);
  leftArm2 = motor(PORT18, ratio36_1, false);  

  rightArm1.setBrake(hold);
  rightArm2.setBrake(hold);
  leftArm1.setBrake(hold);
  leftArm2.setBrake(hold);

  //gyroSensor = inertial(PORT11);

  driveType = TWO_STICK_ARCADE;
  robotController = c; 

  setControllerMapping(DEFAULT_MAPPING);

  globalEncoderLeft = 0;
  globalEncoderRight = 0;


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

void Robot24::goForwardUntilSensor(float maxDistance, float speed, float rampUpInches, int timeout, std::function<bool(void)> func, bool stopAfter) {

  Trapezoid trap(maxDistance, speed, speed, rampUpInches, 0);

  int startTime = vex::timer::system();
  resetEncoderDistance();
  gyroSensor.resetRotation();

  // finalDist is 0 if we want driveTimed instead of drive some distance
  while (!trap.isCompleted() && !isTimeout(startTime, timeout) && clawSensor.value()) {

    // if there is a concurrent function to run, run it
    if (func) {
      bool done = func();
      if (done) {
        // if func is done, make it empty
        func = {};
      }
    }

    float speed = trap.tick(degreesToDistance((leftMotorA.position(degrees) + rightMotorA.position(degrees)) / 2));

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

// angleDegrees is positive if clockwise, negative if counterclockwise
void Robot24::goTurn(float angleDegrees, std::function<bool(void)> func) {

  PID anglePID(2, 0.00, 0.05, 5, 1, 35);

  float timeout = 5;
  float speed;

  int startTime = vex::timer::system();
  resetEncoderDistance();
  gyroSensor.resetRotation();

  while (!anglePID.isCompleted() && !isTimeout(startTime, timeout)) {


    speed = anglePID.tick(angleDegrees - gyroSensor.rotation());

    setLeftVelocity(forward, speed);
    setRightVelocity(reverse, speed);

    wait(20, msec);
  }

  stopLeft();
  stopRight();
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
void Robot24::gyroCurve(float distInches, float maxSpeed, float turnAngle, int timeout, bool stopAfter, std::function<bool(void)> func) {

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

    // if there is a concurrent function to run, run it
    if (func) {
      bool done = func();
      if (done) {
        // if func is done, make it empty
        func = {};
      }
    }

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

vision Robot24::getCamera(directionType dir, Goal goal) {
  vision camera = vision((dir == forward) ? PORT10 : PORT15, goal.bright, goal.sig);
  return camera;
}

void Robot24::updateCamera(Goal goal) {
  backCamera = vision(PORT9, goal.bright, goal.sig);
  frontCamera = vision(PORT10, goal.bright, goal.sig);
}



// Go forward with vision tracking towards goal
// PID for distance and for correction towards goal
// distInches is positive if forward, negative if reverse
void Robot24::goVision(float distInches, float maxSpeed, Goal goal, directionType cameraDir, float rampUpInches, float slowDownInches,
 int timeout, bool stopAfter, float K_P, std::function<bool(void)> func) {

  updateCamera(goal);

  vision *camera = (cameraDir == forward) ? &frontCamera : &backCamera;
  
  Trapezoid trap(distInches, maxSpeed, FORWARD_MIN_SPEED, rampUpInches, slowDownInches);

  int startTime = vex::timer::system();
  resetEncoderDistance();

  PID anglePID(0.5, 0, 0);
  logController("start vision");

  while (!trap.isCompleted() && !isTimeout(startTime, timeout)) {

    if (func) {
      if (func()) {
        // if func is done, make it empty
        func = {};
      }
    }

    camera->takeSnapshot(goal.sig);
    
    float correction = 0; // between -1 and 1
    if(camera->largestObject.exists)  correction = anglePID.tick(camera->largestObject.centerX - VISION_CENTER_X);

    float distInDegrees = fabs((leftMotorA.position(degrees) + rightMotorA.position(degrees))/2);
    float speed = trap.tick(degreesToDistance(distInDegrees));

    float left = speed  * (cameraDir == forward? 1 : -1) + correction;
    float right =  speed * (cameraDir == forward? 1 : -1) - correction;


    setLeftVelocity(forward, left);
    setRightVelocity(forward, right);

    wait(20, msec);
  }
  if (stopAfter) {
    stopLeft();
    stopRight();
  }
  logController("stop vison");

}

// Returns true if aligned to goal, false if timed out or maxTurnAngle reached
bool Robot24::goTurnVision(Goal goal, bool defaultClockwise, directionType cameraDir, float maxTurnAngle) {


  float delta;
  int timeout = 4;
  int startTime = vex::timer::system();
  updateCamera(goal);

  vision *camera = (cameraDir == forward) ? &frontCamera : &backCamera;
  camera->takeSnapshot(goal.sig);
  if (!camera->largestObject.exists) return false;

  gyroSensor.resetRotation();

  PID vPID(70, 0, 0, 0.05, 3, 25, 100);

  while (!vPID.isCompleted()) {

    // failure exit conditions
    if (isTimeout(startTime, timeout) || fabs(gyroSensor.rotation()) > maxTurnAngle) return false;

    camera->takeSnapshot(goal.sig);
    
    // correction is between -1 and 1. Positive if overshooting to right, negative if overshooting to left
    if(camera->largestObject.exists)  delta = (VISION_CENTER_X - camera->largestObject.centerX) / VISION_CENTER_X;
    else delta = defaultClockwise ? -1 : 1;

    float speed = vPID.tick(delta);
    setLeftVelocity(reverse, speed);
    setRightVelocity(forward, speed);

    wait(20, msec);

  }

  stopLeft();
  stopRight();

  // did not exit on failure conditions, so successfully aligned
  return true;
}

void Robot24::alignToGoalVision(Goal goal, bool clockwise, directionType cameraDirection, int timeout, float maxSpeed) {

  // spin speed is proportional to distance from center, but must be bounded between MIN_SPEED and MAX_SPEED

  updateCamera(goal);
  vision *camera = (cameraDirection == forward) ? &frontCamera : &backCamera;

  int startTime = vex::timer::system();
  resetEncoderDistance();

  int spinSign = clockwise ? -1 : 1;

  while (!isTimeout(startTime, timeout)) {

    camera->takeSnapshot(goal.sig);

    float mod;
    if (camera->largestObject.exists) {
      mod = (VISION_CENTER_X-camera->largestObject.centerX) / VISION_CENTER_X;

      // If goal is [left side of screen if clockwise, right side of scree if counterclockwise], that means it's arrived at center and is aligned
      if (fabs(mod) <= 0.05) {
        break;
      }

    } else {
      // If largest object not detected, then spin in the specified direction
      mod = spinSign;
    }

    float speed = (mod > 0 ? 1 : -1) * TURN_MIN_SPEED + mod * (maxSpeed - TURN_MIN_SPEED);

    setLeftVelocity(reverse, speed);
    setRightVelocity(forward, speed);
    wait(20, msec);
  }

  stopLeft();
  stopRight();
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
float Robot24::getEncoderDistance() {
  return degreesToDistance((leftMotorA.rotation(deg) + rightMotorA.rotation(deg)) / 2);
}

void Robot24::resetEncoderDistance() {
  globalEncoderLeft += leftMotorA.rotation(degrees);
  globalEncoderRight += rightMotorA.rotation(degrees);
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