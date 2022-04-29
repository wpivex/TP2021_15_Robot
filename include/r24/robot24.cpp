#include "robot24.h"

// Motor ports Left: 1R, 2F, 3F,  20T Right: 12R, 11F, 13F
// gear ratio is 60/36
Robot24::Robot24() : BaseRobot(15.0, PORT17), leftMotorA(0), leftMotorB(0), leftMotorC(0), leftMotorD(0), leftMotorE(0), 
  rightMotorA(0), rightMotorB(0), rightMotorC(0), rightMotorD(0), rightMotorE(0), rightArm1(0), rightArm2(0), 
  leftArm1(0), leftArm2(0), leftEncoder(Brain.ThreeWirePort.E), rightEncoder(Brain.ThreeWirePort.G) {

  leftMotorA = motor(PORT3, ratio18_1, true); 
  leftMotorB = motor(PORT4, ratio18_1, true);
  leftMotorC = motor(PORT5, ratio18_1, true);
  leftMotorD = motor(PORT6, ratio18_1, true);
  leftMotorE = motor(PORT7, ratio18_1, true);

  rightMotorA = motor(PORT11, ratio18_1, false);
  rightMotorB = motor(PORT12, ratio18_1, false);
  rightMotorC = motor(PORT14, ratio18_1, false);
  rightMotorD = motor(PORT15, ratio18_1, false);
  rightMotorE = motor(PORT16, ratio18_1, false);

  rightArm1 = motor(PORT10, ratio36_1, true);
  rightArm2 = motor(PORT20, ratio36_1, true);
  leftArm1 = motor(PORT8, ratio36_1, false);
  leftArm2 = motor(PORT18, ratio36_1, false);  

  rightArm1.setBrake(hold);
  rightArm2.setBrake(hold);
  leftArm1.setBrake(hold);
  leftArm2.setBrake(hold);


  FRONT_CAMERA_PORT = PORT10;
  BACK_CAMERA_PORT = PORT1;

  driveType = TWO_STICK_ARCADE;

  setControllerMapping(DEFAULT_MAPPING);
}

void Robot24::setControllerMapping(ControllerMapping mapping) {

  cMapping = mapping;

  if (mapping == DEFAULT_MAPPING) {
    driveType = TWO_STICK_ARCADE;

    FRONT_CLAMP_TOGGLE = BTN::L1;
    BACK_CLAMP_TOGGLE = BTN::R1;
    CLAW_TOGGLE = BTN::A;
  } 
}

void Robot24::driveTeleop() {

  if (driveHold) setBrakeType(hold);
  else setBrakeType(coast);

  basicDriveTeleop();
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
  if (buttons.pressing(BTN::R2)) {
    setArmPercent(forward, 100);
  } else if (buttons.pressing(BTN::L2)) {
    setArmPercent(reverse, 100);
  } else {
    stopArm();
  }

  // Toggle limiting arm current
  if (buttons.pressed(BTN::X)) {
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

// does NOT stop motors
void Robot24::goVisionUntilSensor(directionType cameraDir, float maxDistance, float speed, digital_in sensor, float rampUpFrames, bool disableVision) {

  Trapezoid trap(maxDistance, speed, 30, rampUpFrames, 0);
  PID pidTurn(50, 0, 0);

  int startTime = vex::timer::system();

  Goal g = YELLOW;
  int32_t port = cameraDir == forward ? FRONT_CAMERA_PORT : BACK_CAMERA_PORT;
  vision camera(port, g.bright, g.sig);

  resetEncoderDistance();

  // finalDist is 0 if we want driveTimed instead of drive some distance
  while (!trap.isCompleted() && !isTimeout(startTime, 5) && sensor.value()) {

    camera.takeSnapshot(g.sig);

    float speed = trap.tick(getEncoderDistance());
    float correction = camera.largestObject.exists ? pidTurn.tick((VISION_CENTER_X-camera.largestObject.centerX) / VISION_CENTER_X) : 0;

    setLeftVelocity(forward, speed - correction);
    setRightVelocity(forward, speed + correction);
    
    wait(20, msec);
  }
}

void Robot24::goForwardUntilSensor(float maxDistance, float speed, digital_in sensor, float rampUpFrames) {
  goVisionUntilSensor(forward, maxDistance, speed, sensor, rampUpFrames, true);
}


// Go forward a number of inches, maintaining a specific heading
// Calling general function with 24-specifc params

void Robot24::goForwardU(float distInches, float maxSpeed, float universalAngle, float rampUpFrames, float slowDownInches, float endSlowInches,
    bool stopAfter, float minSpeed, float timeout) {
  BaseRobot::goForwardU_Abstract(1.0, distInches, maxSpeed, universalAngle, rampUpFrames, slowDownInches, endSlowInches, stopAfter,
    minSpeed, -1, timeout);
}


// Turn to some universal angle based on starting point. Turn direction is determined by smallest angle to universal angle
// Calling general function with 24-specifc params
// 0.075
// I = 0.96
void Robot24::goTurnU(float universalAngleDegrees, int direction, bool stopAfter, float timeout, float maxSpeed) {
  BaseRobot::goTurnU_Abstract(3, 0, 0.075, 1, 3, 20, universalAngleDegrees, direction, stopAfter, timeout, maxSpeed);
}

//.void Robot24::goTurnUFast(float universalAngleDegrees)

float Robot24::distanceToDegrees(float distInches) {
  return 2* (distInches * 360.0 / 2.0 / M_PI / (4.0 / 2.0) / SPEED_RATIO); // 4 in diameter wheels
}

float Robot24::degreesToDistance(float distDegrees) {
  return SPEED_RATIO * distDegrees / (360.0 / 2.0 / M_PI / (4.0 / 2.0)); // 4 in diameter wheels
}

// Go forward until the maximum distance is hit or the timeout is reached
// for indefinite timeout, set to -1
void Robot24::goVision(float distInches, float speed, Goal goal, directionType cameraDir, float rampUpFrames, 
    float slowDownInches, float endSlowInches, bool stopAfter, float timeout) {
    
  int32_t port = cameraDir == forward ? FRONT_CAMERA_PORT : BACK_CAMERA_PORT;
  BaseRobot::goVision_Abstract(50, FORWARD_MIN_SPEED, port, distInches, speed, goal, rampUpFrames, slowDownInches, endSlowInches, stopAfter, timeout);
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

void Robot24::setArmBrakeType(brakeType b) {
  leftArm1.setBrake(b);
  leftArm2.setBrake(b);
  rightArm1.setBrake(b);
  rightArm2.setBrake(b);
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
float Robot24::getLeftEncoderAbsolute() {
  return -leftEncoder.rotation(deg)*M_PI*2.75/360.0/3;
}

// return in inches
float Robot24::getRightEncoderAbsolute() {
  return rightEncoder.rotation(deg)*M_PI*2.75/360.0/3;
}

// return in inches
float Robot24::getLeftEncoderDistance() {
  return leftEncoder.rotation(degrees) - zeroedRelLeft;
}

// return in inches
float Robot24::getRightEncoderDistance() {
  return rightEncoder.rotation(degrees) - zeroedRelRight;
}

void Robot24::resetEncoderDistance() {
  zeroedRelLeft = 0;
  zeroedRelRight = 0;
}

void Robot24::resetOdom() {
  absoluteX = 0;
  absoluteY = 0;
  recordedL = 0;
  recordedR = 0;
  recordedTheta = 0;
  leftEncoder.resetRotation();
  rightEncoder.resetRotation();
  resetEncoderDistance();
}

float Robot24::getDriveCurrent() {
  float currentSum = leftMotorA.current() + leftMotorB.current() + leftMotorC.current() + leftMotorD.current() + leftMotorE.current() 
    + rightMotorA.current() + rightMotorB.current() + rightMotorC.current() + rightMotorD.current() + rightMotorE.current();
  return currentSum / 10;
}

void Robot24::activeLocation() {
  //Figure out how much each encoder has changed
  //Enocoder code
  //float deltaL = recordedL - degreesToDistance(leftEncoder);
  //Non-encoder code
  float deltaR = getRightEncoderAbsolute() - this->recordedR;
  float deltaL = getLeftEncoderAbsolute() - this->recordedL;
  float deltaTheta = bound180(gyroSensor.heading()- this->recordedTheta)*M_PI/180;
  //float deltaR = recordedR - degreesToDistance(rightEncoder);
  float dist = (deltaL+deltaR)/2;
  float deltaX;
  float deltaY;
  if(deltaTheta!=0){
    float radius = dist/deltaTheta;
    deltaX = radius * (cos((gyroSensor.heading()*M_PI)/180) - cos(recordedTheta*M_PI/180));
    deltaY = radius * (sin((gyroSensor.heading()*M_PI)/180) - sin(recordedTheta*M_PI/180));
  }else{
    deltaX = dist * (cos((gyroSensor.heading()*M_PI)/180));
    deltaY = dist * (sin((recordedTheta*M_PI)/180));
  }
  //Split vector to X and Y 
  
  //Add x and Y distance to current values
  this->absoluteX -= deltaX;
  this->absoluteY += deltaY;
  this->recordedR = getRightEncoderDistance();
  this->recordedL = getLeftEncoderDistance();
  this->recordedTheta = gyroSensor.heading();
}

void Robot24::goFightOdom(float backUpDist, float slowDownInches) {
  float ay = this->absoluteY;

  Trapezoid trap(backUpDist, 100, 30, 0, slowDownInches);
  
  while (!trap.isCompleted()) {
    
    float speed = trap.tick(ay - this->absoluteY);

    setLeftVelocity(reverse, speed);
    setRightVelocity(reverse, speed);

    wait(20, msec);
  }
  stopLeft();
  stopRight();
}

void Robot24::goToPoint(float x, float y, float speed, float onlyTurn) {
  float ax = this->absoluteX;
  float ay = this->absoluteY;
  float universalAngle = gyroSensor.heading() - (atan2(y - ay, x - ax)*180/M_PI - 90);
  // universal turn to point
  // goTurnU(universalAngle);
  // drive to point
  if (!onlyTurn) {
    float dist = sqrt(pow(x - ax, 2) + pow(y - ay, 2));
    logController("angle %f", universalAngle);
    // goForwardU(dist, speed, gyroSensor.heading(), 5, 5);
  }
}

void Robot24::gotToY(float yValue, float speed) {
  float ay = absoluteY;

  Trapezoid trap(absoluteY - ay, speed, 30, 0, 5);
  
  while (!trap.isCompleted()) {
    float speed = trap.tick(absoluteY - ay);

    setLeftVelocity(forward, speed);
    setRightVelocity(forward, speed);

    wait(20, msec);
  }
  stopLeft();
  stopRight();
}
