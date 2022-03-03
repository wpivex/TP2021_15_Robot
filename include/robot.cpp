#include "robot.h"


Robot::Robot(controller* c) : leftMotorA(0), leftMotorB(0), leftMotorC(0), leftMotorD(0), rightMotorA(0), rightMotorB(0), 
  rightMotorC(0), rightMotorD(0), frontArmL(0), frontArmR(0), backLiftL(0), backLiftR(0), intake(0), camera(0), buttons(c), gyroSensor(PORT14) {

  leftMotorA = motor(PORT1, ratio6_1, true); 
  leftMotorB = motor(PORT2, ratio6_1, true);
  leftMotorC = motor(PORT12, ratio6_1, true);
  leftMotorD = motor(PORT10, ratio6_1, true);
  leftDrive = motor_group(leftMotorA, leftMotorB, leftMotorC, leftMotorD);


  rightMotorA = motor(PORT4, ratio6_1, false);
  rightMotorB = motor(PORT18, ratio6_1, false);
  rightMotorC = motor(PORT20, ratio6_1, false);
  rightMotorD = motor(PORT21, ratio6_1, false);
  rightDrive = motor_group(rightMotorA, rightMotorB, rightMotorC, rightMotorD);

  // forward is UP, reverse is DOWN
  frontArmL = motor(PORT19, ratio36_1, true);
  frontArmR = motor(PORT3, ratio36_1, false);

  backLiftL = motor(PORT6, ratio36_1, true);
  backLiftR = motor(PORT15, ratio36_1, true);

  intake = motor(PORT7, ratio18_1, false);
  
  robotController = c; 

  frontArmL.setBrake(hold);
  frontArmR.setBrake(hold);
  backLiftL.setBrake(hold);
  backLiftR.setBrake(hold);

  setControllerMapping(DEFAULT_MAPPING);
}

void Robot::setControllerMapping(ControllerMapping mapping) {

  cMapping = mapping;

  driveType = ONE_STICK_ARCADE;

  //Controls that don't change:
  BACK_LIFT_UP = Buttons::A;
  BACK_LIFT_MID = Buttons::R1;
  BACK_LIFT_DOWN = Buttons::R2;
  BACK_LIFT_SLIGHT = Buttons::INVALID;
  BACK_LIFT_UPPING = Buttons::RIGHT;
  BACK_LIFT_DOWNING = Buttons::LEFT;

  INTAKE_TOGGLE = Buttons::UP;
  INTAKE_TOGGLE_REV = Buttons::DOWN;

  CLAW_UP = Buttons::L2;
  CLAW_DOWN = Buttons::L1;

  FRONT_ARM_UP = Buttons::INVALID; // brian uses left-stick controls
  FRONT_ARM_DOWN = Buttons::INVALID;

  if (mapping == BRIAN_MAPPING) {
    BACK_LIFT_UP = Buttons::X;
  }

}

void Robot::waitGpsCallibrate() {
  while (GPS11.isCalibrating() || gyroSensor.isCalibrating()) wait(20, msec);
  log("calibrated");
}


void Robot::driveTeleop() {


  if(driveType == TANK) {
    setLeftVelocity(forward,buttons.axis(Buttons::LEFT_VERTICAL));
    setRightVelocity(forward,buttons.axis(Buttons::RIGHT_VERTICAL));
  } else {
    float drive = driveType == ONE_STICK_ARCADE ? buttons.axis(Buttons::RIGHT_VERTICAL) : buttons.axis(Buttons::LEFT_VERTICAL);
    float turn = buttons.axis(Buttons::RIGHT_HORIZONTAL) / 2.0;
    float max = std::max(1.0, std::max(fabs(drive+turn), fabs(drive-turn)));
    setLeftVelocity(forward,100 * (drive+turn)/max);
    setRightVelocity(forward,100 * (drive-turn)/max);
  }
}

// Not a truly blocking function, one second timeout if blocking
void Robot::setBackLift(Buttons::Button b, bool blocking) {

  logController("start back lift");

  float SPEED = 100;

  if (b == BACK_LIFT_UP) {
    log("up");
    backLiftL.rotateTo(0, degrees, SPEED, velocityUnits::pct, false);
    backLiftR.rotateTo(0, degrees, SPEED, velocityUnits::pct, false);
  } else if (b == BACK_LIFT_MID) {
    log("mid");
    backLiftL.rotateTo(130, degrees, SPEED, velocityUnits::pct, false);
    backLiftR.rotateTo(130, degrees, SPEED, velocityUnits::pct, false);
  } else if (b == BACK_LIFT_DOWN) {
    log("down");
    backLiftL.rotateTo(330, degrees, SPEED, velocityUnits::pct, false);
    backLiftR.rotateTo(330, degrees, SPEED, velocityUnits::pct, false);
  } else if (b == BACK_LIFT_SLIGHT) {
    backLiftL.rotateTo(290, degrees, SPEED, velocityUnits::pct, false);
    backLiftR.rotateTo(290, degrees, SPEED, velocityUnits::pct, false);
  }

  if (blocking) wait(800, msec);
  logController("end back lift");

}


void Robot::backLiftTeleop() {
  setBackLift(buttons.get(), false);
  if (buttons.pressing(BACK_LIFT_UPPING)){
    log("upping");
    backLiftL.spin(forward,50,pct);
    backLiftR.spin(forward,50,pct);
  } else if (buttons.pressing(BACK_LIFT_DOWNING)) {
    log("downing");
    backLiftL.spin(reverse,50,pct);
    backLiftR.spin(reverse,50,pct);
  } else if (buttons.released(BACK_LIFT_DOWNING) || buttons.released(BACK_LIFT_UPPING)) {
    log("stop");
    backLiftL.stop();
    backLiftR.stop();
  }
}

void Robot::clawUp() {
  frontClaw.set(false);
}

void Robot::clawDown() {
  frontClaw.set(true);
}

void Robot::moveArmTo(double degr, double speed, bool blocking) {
  frontArmL.rotateTo(degr, degrees, speed, velocityUnits::pct, false);
  frontArmR.rotateTo(degr, degrees, speed, velocityUnits::pct, blocking);
}

void Robot::armTeleop() {

  float MOTOR_SPEED = 100;

  //logController("%f", frontArmL.rotation(degrees));
  
  float brianArm = buttons.axis(Buttons::LEFT_VERTICAL); // Brian's weird shit

  if (buttons.pressing(FRONT_ARM_UP)) {
    frontArmL.spin(forward, MOTOR_SPEED, pct);
    frontArmR.spin(forward, MOTOR_SPEED, pct);
  } else if (buttons.pressing(FRONT_ARM_DOWN)) {

    frontArmL.spin(reverse, MOTOR_SPEED, pct);
    frontArmR.spin(reverse, MOTOR_SPEED, pct);
    
  } else if ((cMapping == BRIAN_MAPPING || cMapping == DEFAULT_MAPPING) && brianArm != 0) {
    frontArmL.spin(forward, brianArm * 100, pct);
    frontArmR.spin(forward, brianArm * 100, pct);
  }
  else {

    frontArmL.stop();
    frontArmR.stop();
  }

  if (buttons.pressing(CLAW_UP)) {
    frontClaw.set(true);
  } else if (buttons.pressing(CLAW_DOWN)) {
    frontClaw.set(false); 
  }

}

void Robot::intakeTeleop() {
  int INTAKE_SPEED = 100;
  if (buttons.pressed(INTAKE_TOGGLE)){
    if(intakeState == 1){
      intakeState = 0;
    }else{
      intakeState = 1;
    }
  }
  else if (buttons.pressed(INTAKE_TOGGLE_REV)){
    if(intakeState == -1){
      intakeState = 0;
    }else{
      intakeState = -1;
    }
  }

  intake.spin(forward, intakeState*INTAKE_SPEED, pct);

}

// Run every tick
void Robot::teleop() {
  
  driveTeleop();
  armTeleop();
  intakeTeleop();
  backLiftTeleop();

  buttons.updateButtonState();
}

void Robot::waitForGPS() {
  while (GPS11.quality() < 90) {
    logController("Quality: %d", GPS11.quality());
    wait(20, msec);
  }
  logController("Quality: %d", GPS11.quality());
}

// return in inches
float Robot::getEncoderDistance() {
  return degreesToDistance((leftMotorA.rotation(deg) + rightMotorA.rotation(deg)) / 2);
}

float Robot::getAngle() {
  return gyroSensor.heading();
}

// Number of samples in 20ms intervals to average and find more accurate x/y location with GPS
float getCoordinate(axisType axis, int numSamples) {
  float total = 0;
  for (int i = 0; i < numSamples; i++) {
    total += (axis == xaxis) ? GPS11.xPosition(inches) : GPS11.yPosition(inches);
  }
  return total / numSamples;
}

float Robot::getX(int numSamples) {
  return getCoordinate(xaxis, numSamples);
}

float Robot::getY(int numSamples) {
  return getCoordinate(yaxis, numSamples);
}

// Essentially return theta for vector [dx, dy]
float angleToPointU(float dx, float dy) {
  return 90 - (180 / PI * atan2(dy, dx));
}

void Robot::goCurve(float distInches, float maxSpeed, float turnPercent, float rampUpInches, float slowDownInches, bool stopAfter, float rampMinSpeed) {
  float timeout = 5;

  Trapezoid trap(distInches, maxSpeed, 4, rampUpInches, slowDownInches, rampMinSpeed);

  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();
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

void Robot::goForwardTimed(float duration, float speed) {

  int startTime = vex::timer::system();

  while (!isTimeout(startTime, duration)) {
    setLeftVelocity(forward, speed);
    setRightVelocity(forward, speed);
    wait(20, msec);
  }
  stopLeft();
  stopRight();

}

// Go forward a number of inches, maintaining a specific heading if angleCorrection = true
void Robot::goForwardU(float distInches, float maxSpeed, float universalAngle, float rampUpInches, float slowDownInches, 
bool stopAfter, float rampMinSpeed, float slowDownMinSpeed, float timeout, bool angleCorrection) {

  Trapezoid trap(distInches, maxSpeed, slowDownMinSpeed, rampUpInches, slowDownInches, rampMinSpeed);
  PID turnPID(1, 0.00, 0);

  float correction = 0;
  float currDist;
  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  //log("start forward %f %f %f", distInches, startX, startY);

  while (!trap.isCompleted() && !isTimeout(startTime, timeout)) {

    currDist = getEncoderDistance();
    
    float speed = trap.tick(currDist);
    float ang = getAngleDiff(universalAngle, getAngle());
    if (angleCorrection) correction = turnPID.tick(ang);
 
    setLeftVelocity(forward, speed + correction);
    setRightVelocity(forward, speed - correction);

    wait(20, msec);
  }
  if (stopAfter) {
    stopLeft();
    stopRight();
  }
  log("straight done");
}

// Go forward with standard internal encoder wheels for distance, and no angle correction
void Robot::goForward(float distInches, float maxSpeed, float rampUpInches, float slowDownInches, bool stopAfter, float rampMinSpeed, 
float slowDownMinSpeed, float timeout) {
  goForwardU(distInches, maxSpeed, -1, rampUpInches, slowDownInches, stopAfter, rampMinSpeed, slowDownMinSpeed, timeout, false);
}

// Go at specified direction and approach given x position with PID motion profiling using GPS absolute positioning
void Robot::goToAxis(axisType axis, bool reverseDirection, float finalValue, float maxSpeed, float timeout) {

  PID pid(7, 0, 0.2, 0.3, 5, 11, maxSpeed);
  PID turnPID(1, 0, 0);
  int startTime = vex::timer::system();
  float h = getAngle(); // maintain current heading

  while (!pid.isCompleted() && !isTimeout(startTime, timeout)) {

    float currDist = axis == axisType::xaxis ? getX() : getY();
    float speed = pid.tick(finalValue - currDist) * (reverseDirection ? -1 : 1);
    float correction = turnPID.tick(getAngleDiff(h, getAngle()));

    setLeftVelocity(forward, speed + correction);
    setRightVelocity(forward, speed - correction);

    log("%f", speed);

    wait(20, msec);
  }
  stopLeft();
  stopRight();
}
// Go forward towards point, making corrections to target. Stop angle correction 15 inches before hitting target
// Uses trapezoidal motion profile for distance and PID for angle corrections
void Robot::goForwardGPS(float x, float y, float maxSpeed, float rampUpInches, float slowDownInches, directionType dir) {
  float sx = getX();
  float sy = getY();

  float distInches = distanceFormula(x - sx, y - sy);
  
  Trapezoid trap(distInches, maxSpeed, 4, rampUpInches, slowDownInches, 20);
  PID turnPID(1.2, 0.00, 0);

  float correction = 0;
  int startTime = vex::timer::system();

  while (!trap.isCompleted() && !isTimeout(startTime, 10)) {

    float cx = getX();
    float cy = getY();

    float currDist = distanceFormula(cx - sx, cy - sy);
    
    float speed = trap.tick(currDist) * (dir == forward ? 1 : -1);
    float ang = getAngleDiff(angleToPointU(x - cx, y - cy), getAngle());
    if (distInches - currDist > 10) correction = turnPID.tick(ang); // adjust heading towards target if over 15 inches away

    //log("Error: %f \n CurrDist: %f \n Speed: %f \n Angle: %f \n Correction: %f", distanceFormula(x-cx,y-cy), currDist, speed, ang, correction);
    //log("Quality: %d", GPS11.quality());

    setLeftVelocity(forward, speed + correction);
    setRightVelocity(forward, speed - correction);

    wait(20, msec);
  }

  stopLeft();
  stopRight();
  log("done");
}

// Turn to some universal angle based on starting point. Turn direction is determined by smallest angle to universal angle
void Robot::goTurnU(float universalAngleDegrees, bool stopAfter, bool faster) {

  PID anglePID(2, 0, 0.13, 1, 5, 12, 75);
  if (faster) {
    //anglePID = PID(2, 0.00, 0.02, 3, 3, 12, 75);
  }

  float timeout = 5;
  float speed;

  log("initing");
  int startTime = vex::timer::system();
  log("about to loop");

  while (!anglePID.isCompleted() && !isTimeout(startTime, timeout)) {

    float ang = getAngleDiff(universalAngleDegrees, getAngle());
    speed = anglePID.tick(ang);

    //log("Turn \nTarget: %f \nCurrent: %f \nDiff: %f\nSpeed: %f \nGPS: %f", universalAngleDegrees, getAngle(), ang, speed, GPS11.heading());
    //log("heading: %f", GPS11.heading());
    log("Quality: %d \n Current: %f \n Target: %f", GPS11.quality(), getAngle(), universalAngleDegrees);

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


// go to (x,y) in inches. First, make a fast point turn, which need not be perfectly accurate
// Then, it drives aiming at the target, making realtime corrections to any initial error
// It is more deliberate at close speeds and more aggressive at faster speeds
void Robot::goPointGPS(float x, float y, directionType dir) {

  waitForGPS();

  float sx = getX();
  float sy = getY();

  float proj_x = x - sx;
  float proj_y = y - sy;
  float distFinal = distanceFormula(proj_x, proj_y);

  // Point turn to orient towards target
  float angleU = angleToPointU(proj_x, proj_y);
  if (dir == reverse) angleU = fmod((angleU + 180), 360);

  if (distFinal < 25) { // For closer distances, have a more accurate initial turn and slower approach speed
    goTurnU(angleU, true, false);
    goForwardGPS(x, y, 40, 1, 4, dir);
  }
  else { // For longer distances, have a faster initial turn and approach speed
    goTurnU(angleU, true, true);
    goForwardGPS(x, y, 80, 6, 15, dir);
  }
}

// Detect whether the robot is fighting another robot based on measuring current.
// If current is always above the threshold, never stop backing up or releasing claw (because fighting other robot in mid)
void Robot::driveStraightFighting(float distInches, float speed, directionType dir) {

  float CURRENT_THRESHHOLD = 1.0; // The threshhold in which "fighting" is detecting
  int NUM_CURRENT_NEEDED = 4; // The number of times the current must be below threshold in a row to count as stopped fighting

  int numCurrentReached = 0;

  float finalDist = fabs(distanceToDegrees(distInches));
  float currentDist = 0;

  leftMotorA.resetRotation();
  rightMotorA.resetRotation();

  // Keep running while distance is not reached or the current has not dipped below threshold for a significant period of time
  while ((currentDist < finalDist || numCurrentReached < NUM_CURRENT_NEEDED)) {

    float c = (leftMotorA.current() + rightMotorA.current()) / 2.0;

    if (c <= CURRENT_THRESHHOLD) {
      numCurrentReached++;
    } else numCurrentReached = 0;
    logController("%d %f", numCurrentReached, c);

    currentDist = fabs(getEncoderDistance());
    setLeftVelocity(dir, speed);
    setRightVelocity(dir, speed);

    wait(20, msec);
  }

  // take four inches to slow down to not break motors
  Trapezoid trap(4, speed, 0, 0, 4);
  while (!trap.isCompleted()) {

    float speed = trap.tick(getEncoderDistance());

    setLeftVelocity(dir, speed);
    setRightVelocity(dir, speed);

    wait(20, msec);
  }
  stopLeft();
  stopRight();
}


void Robot::updateCamera(Goal goal) {
  camera = vision(PORT13, goal.bright, goal.sig);
}

// Go forward until the maximum distance is hit or the timeout is reached
// for indefinite timeout, set to -1
void Robot::goVision(float distInches, float speed, Goal goal, float rampUpInches, float slowDownInches, bool stopAfter, float timeout) {

  Trapezoid trapDist(distInches, speed, 12, rampUpInches, slowDownInches);
  PID pidTurn(35, 0, 0);

  updateCamera(goal);

  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  // forward until the maximum distance is hit, the timeout is reached, or limitSwitch is turned on
  while (!trapDist.isCompleted() && !isTimeout(startTime, timeout)) {

    camera.takeSnapshot(goal.sig);
    
    float correction = camera.largestObject.exists ? pidTurn.tick((VISION_CENTER_X-camera.largestObject.centerX) / VISION_CENTER_X) : 0;
    float speed = trapDist.tick(getEncoderDistance());

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
void Robot::goAlignVision(Goal goal, float timeout) {

  updateCamera(goal);

  int startTime = vex::timer::system();
  float speed = 0;

  PID vTurnPID(40, 0, 2, 0.04, 3, 12);

  while (!vTurnPID.isCompleted() && !isTimeout(startTime, timeout)) {

    camera.takeSnapshot(goal.sig);

    if (camera.largestObject.exists) speed = vTurnPID.tick((VISION_CENTER_X-camera.largestObject.centerX) / VISION_CENTER_X);

    setLeftVelocity(reverse, speed);
    setRightVelocity(forward, speed);

    wait(20, msec);
  }

  stopLeft();
  stopRight();
}


void Robot::setLeftVelocity(directionType d, double percent) {
  if (percent < 0) {
    d = (d == forward) ? reverse : forward;
    percent = -percent;
  }
  leftMotorA.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
  leftMotorB.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
  leftMotorC.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
  leftMotorD.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
}

void Robot::setRightVelocity(directionType d, double percent) {
  if (percent < 0) {
    d = (d == forward) ? reverse : forward;
    percent = -percent;
  }
  rightMotorA.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
  rightMotorB.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
  rightMotorC.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
  rightMotorD.spin(d, percent / 100.0 * MAX_VOLTS, voltageUnits::volt);
}

void Robot::startIntake(directionType dir) {
  intake.spin(dir, 100, pct);
}

void Robot::stopIntake() {
  intake.stop();
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

void Robot::setBrakeType(brakeType b) {
  leftMotorA.setBrake(b);
  leftMotorB.setBrake(b);
  leftMotorC.setBrake(b);
  leftMotorD.setBrake(b);
  rightMotorA.setBrake(b);
  rightMotorB.setBrake(b);
  rightMotorC.setBrake(b);
  rightMotorD.setBrake(b);
}