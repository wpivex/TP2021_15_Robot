#include "robot.h"


Robot::Robot(controller* c) : leftMotorA(0), leftMotorB(0), leftMotorC(0), leftMotorD(0), rightMotorA(0), rightMotorB(0), 
  rightMotorC(0), rightMotorD(0), frontArmL(0), frontArmR(0), backLiftL(0), backLiftR(0), intake(0), camera(0), gyroSensor(PORT4), buttons(c) {

  leftMotorA = motor(PORT1, ratio6_1, true); 
  leftMotorB = motor(PORT2, ratio6_1, true);
  leftMotorC = motor(PORT9, ratio6_1, true);
  leftMotorD = motor(PORT12, ratio6_1, true);
  leftDrive = motor_group(leftMotorA, leftMotorB, leftMotorC, leftMotorD);

  // 3L 11L 12L 14L left
  // 16r 18r 19r 20r right

  rightMotorA = motor(PORT4, ratio6_1, false);
  rightMotorB = motor(PORT19, ratio6_1, false);
  rightMotorC = motor(PORT20, ratio6_1, false);
  rightMotorD = motor(PORT21, ratio6_1, false);
  rightDrive = motor_group(rightMotorA, rightMotorB, rightMotorC, rightMotorD);

  // forward is UP, reverse is DOWN
  frontArmL = motor(PORT14, ratio36_1, true);
  frontArmR = motor(PORT3, ratio36_1, false);

  backLiftL = motor(PORT5, ratio36_1, true);
  backLiftR = motor(PORT13, ratio36_1, true);

  intake = motor(PORT8, ratio18_1, false);

  // 8 and 13??
  
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
  BACK_LIFT_MID = Buttons::R1;
  BACK_LIFT_DOWN = Buttons::R2;
  BACK_LIFT_UPPING = Buttons::RIGHT;
  BACK_LIFT_DOWNING = Buttons::LEFT;

  INTAKE_TOGGLE = Buttons::UP;
  INTAKE_TOGGLE_REV = Buttons::DOWN;

  CLAW_UP = Buttons::L1;
  CLAW_DOWN = Buttons::L2;

  FRONT_ARM_UP = Buttons::INVALID; // brian uses left-stick controls
  FRONT_ARM_DOWN = Buttons::INVALID;
  BACK_LIFT_UP = Buttons::A;


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

void Robot::setBackLift(Buttons::Button b, bool blocking) {

  float SPEED = 100;

  if (b == BACK_LIFT_UP) {
    log("up");
    backLiftL.rotateTo(0, degrees, SPEED, velocityUnits::pct, false);
    backLiftR.rotateTo(0, degrees, SPEED, velocityUnits::pct, blocking);
  } else if (b == BACK_LIFT_MID) {
    log("mid");
    backLiftL.rotateTo(130, degrees, SPEED, velocityUnits::pct, false);
    backLiftR.rotateTo(130, degrees, SPEED, velocityUnits::pct, blocking);
  } else if (b == BACK_LIFT_DOWN) {
    log("down");
    backLiftL.rotateTo(350, degrees, SPEED, velocityUnits::pct, false);
    backLiftR.rotateTo(350, degrees, SPEED, velocityUnits::pct, blocking);
  }

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
  
  float brianArm = buttons.axis(Buttons::RIGHT_VERTICAL); // Brian's weird shit

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

  if (buttons.pressed(Buttons::A)) {
    leftMotorA.resetRotation();
    rightMotorA.resetRotation();
    gyroSensor.resetRotation();
  }

  float left = degreesToDistance(leftMotorA.rotation(degrees));
  float right = degreesToDistance(rightMotorA.rotation(degrees));
  log("%f %f %f", left, right, gyroSensor.rotation());
  logController("%f %f %f", left, right, gyroSensor.rotation());
  
  driveTeleop();
  armTeleop();
  intakeTeleop();
  backLiftTeleop();

  buttons.updateButtonState();
}

void Robot::waitForGPS() {
  while (GPS11.quality() < 80) {
    logController("Quality: %f", GPS11.quality());
    wait(20, msec);
  }
}

float Robot::getEncoderDistance() {
  return (leftMotorA.rotation(deg) + rightMotorA.rotation(deg)) / 2;
}

float Robot::getAngle() {
  return (GPS11.quality() > 90) ? GPS11.heading(degrees) : gyroSensor.heading();
}

// Go forward a number of inches.
// If angleCorrection = true, use gps/gyro for angle correction to given universal heading
// If useGpsDistance = true, use gps to find distance from startX and startY instead of encoder values
void Robot::goForwardU(float distInches, float maxSpeed, float universalAngle, float rampUpInches, float slowDownInches, 
bool stopAfter, float timeout, bool angleCorrection, bool useGpsDistance, float startX, float startY) {

  Trapezoid trap(distInches, maxSpeed, 0, rampUpInches, slowDownInches);
  PID turnPID(0.42, 0.00, 0);

  float correction = 0;
  float currDist;
  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  while (!trap.isCompleted() && !isTimeout(startTime, timeout)) {

    if (useGpsDistance) {
      currDist = distanceFormula(GPS11.xPosition(inches) - startX, GPS11.yPosition(inches) - startY);
    } else {
      currDist = getEncoderDistance();
    }

    float speed = trap.tick(currDist);
    
    if (angleCorrection) correction = turnPID.tick(getAngleDiff(universalAngle, getAngle()));

    // reduce speed if turn causes speed to exceed max
    if (speed + correction > 100) speed = 100 - correction;
    if (speed - correction < -100) speed = -100 + correction;
 
    setLeftVelocity(forward, speed - correction);
    setRightVelocity(forward, speed + correction);

    wait(20, msec);
  }
  if (stopAfter) {
    stopLeft();
    stopRight();
  }
}

// call goForwardU wtih all the distance and angle gps enhancements
void Robot::goForwardGPS(float distInches, float maxSpeed, float universalAngle, float rampUpInches, float slowDownInches, 
float startX, float startY, bool stopAfter, float timeout) {
  goForwardU(distInches, maxSpeed, universalAngle, rampUpInches, slowDownInches, stopAfter, timeout, true, true, startX, startY);
}

// Go forward with standard internal encoder wheels for distance, and no angle correction
void Robot::goForward(float distInches, float maxSpeed, float rampUpInches, float slowDownInches, bool stopAfter, float timeout) {
  goForwardU(distInches, maxSpeed, -1, rampUpInches, slowDownInches, stopAfter, timeout, false);
}

// angleDegrees is positive if clockwise, negative if counterclockwise
void Robot::goTurn(float angleDegrees) {

  PID anglePID(0.42, 0.00, 0.05, 3, 3);

  float timeout = 5;
  float speed;

  log("initing");
  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();
  gyroSensor.resetRotation();
  log("about to loop");

  while (!anglePID.isCompleted() && !isTimeout(startTime, timeout)) {

    float ang = angleDegrees - getAngle();
    speed = anglePID.tick(ang);

    log("%f \n %f \n %f \n %f \n %f", angleDegrees, ang, speed, GPS11.heading(), gyroSensor.heading());

    setLeftVelocity(forward, speed);
    setRightVelocity(reverse, speed);

    wait(20, msec);
  }
  logController("wtf done");

  stopLeft();
  stopRight();
}

// Turn to some universal angle based on starting point. Turn direction is determined by smallest angle
void Robot::goTurnU(float universalAngleDegrees) {
  goTurn(getAngleDiff(universalAngleDegrees, getAngle()));
}


// go to (x,y) in inches
void Robot::goPointGPS(float x, float y, float maxSpeed, float rampUpInches, float slowDownInches, bool stopAfter, float timeout) {

  waitForGPS();

  float sx = GPS11.xPosition(inches);
  float sy = GPS11.yPosition(inches);

  float proj_x = x - sx;
  float proj_y = x - sy;
  float distFinal = distanceFormula(proj_x, proj_y);

  // Point turn to orient towards target
  float angleU = 90 - (180 / PI * atan2(proj_y, proj_x));
  goTurnU(angleU);

  log("done");
  wait(2000, msec);

  // Go forwards to target
  goForwardU(distFinal, maxSpeed, angleU, rampUpInches, slowDownInches, stopAfter, timeout);

}

void Robot::waitGyroCallibrate() {
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
  Brain.Screen.setFillColor(green);
  Brain.Screen.drawRectangle(0, 0, 250, 250);
  Brain.Screen.render();
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
  camera = vision(PORT5, goal.bright, goal.sig);
}

// Go forward until the maximum distance is hit or the timeout is reached
// for indefinite timeout, set to -1
void Robot::goVision(float distInches, float speed, Goal goal, float rampUpInches, float slowDownInches, bool stopAfter, float timeout) {

  Trapezoid trapDist(distInches, speed, 2, rampUpInches, slowDownInches);
  PID pidTurn(1, 0, 0);

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

  PID vTurnPID(10, 0, 0, 0.05, 3, 2);

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