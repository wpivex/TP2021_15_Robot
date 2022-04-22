#include "robot15.h"


Robot15::Robot15() : BaseRobot(9.5, PORT16), 
  leftMotorA(0), leftMotorB(0), leftMotorC(0), leftMotorD(0), rightMotorA(0), rightMotorB(0), 
  rightMotorC(0), rightMotorD(0), intake(0), frontArmL(0), frontArmR(0), backLiftL(0), backLiftR(0) {

  leftMotorA = motor(PORT1, ratio6_1, true); 
  leftMotorB = motor(PORT2, ratio6_1, true);
  leftMotorC = motor(PORT12, ratio6_1, true);
  leftMotorD = motor(PORT10, ratio6_1, true);

  rightMotorA = motor(PORT4, ratio6_1, false);
  rightMotorB = motor(PORT18, ratio6_1, false);
  rightMotorC = motor(PORT20, ratio6_1, false);
  rightMotorD = motor(PORT21, ratio6_1, false);

  // forward is UP, reverse is DOWN
  frontArmL = motor(PORT19, ratio36_1, true);
  frontArmR = motor(PORT3, ratio36_1, false);

  backLiftL = motor(PORT6, ratio36_1, true);
  backLiftR = motor(PORT15, ratio36_1, true);

  intake = motor(PORT7, ratio18_1, false);

  CAMERA_PORT = PORT17;
  
  frontArmL.setBrake(hold);
  frontArmR.setBrake(hold);
  backLiftL.setBrake(hold);
  backLiftR.setBrake(hold);

  setControllerMapping(EZEQUIEL_MAPPING);
}

void Robot15::setControllerMapping(ControllerMapping mapping) {

  cMapping = mapping;

  //Controls that don't change:
  BACK_LIFT_UP = cMapping == BRIAN_MAPPING ? Buttons::X : Buttons::A;
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

}

float Robot15::distanceToDegrees(float distInches) {
  return distInches * (5/3.0) * 360 / 2 / M_PI / (3.25 / 2); // 4 in diameter wheels
}

float Robot15::degreesToDistance(float distDegrees) {
  return distDegrees * (3/5.0) / (360 / 2 / M_PI / (3.25 / 2)); // 4 in diameter wheels
}


// Not a truly blocking function, one second timeout if blocking
void Robot15::setBackLift(Buttons::Button b, bool blocking) {

  float SPEED = 100;

  if (b == BACK_LIFT_UP) {
    log("up");
    targetIsIntake = false;
    backLiftL.rotateTo(0, degrees, SPEED, velocityUnits::pct, false);
    backLiftR.rotateTo(0, degrees, SPEED, velocityUnits::pct, false);
  } else if (b == BACK_LIFT_MID) {
    log("mid");
    targetIsIntake = true;
    backLiftL.rotateTo(130, degrees, SPEED, velocityUnits::pct, false);
    backLiftR.rotateTo(130, degrees, SPEED, velocityUnits::pct, false);
  } else if (b == BACK_LIFT_DOWN) {
    log("down");
    targetIsIntake = false;
    backLiftL.rotateTo(360, degrees, 60, velocityUnits::pct, false); // gentler set down
    backLiftR.rotateTo(360, degrees, 60, velocityUnits::pct, false);
    if (blocking) wait(400, msec);
  } else if (b == BACK_LIFT_SLIGHT) {
    targetIsIntake = false;
    backLiftL.rotateTo(260, degrees, SPEED, velocityUnits::pct, false);
    backLiftR.rotateTo(260, degrees, SPEED, velocityUnits::pct, false);
  }

  if (blocking) wait(800, msec);

}


void Robot15::backLiftTeleop() {
  setBackLift(buttons.get(), false);
  if (buttons.pressing(BACK_LIFT_UPPING)){
    targetIsIntake = false;
    log("upping");
    backLiftL.spin(forward,50,pct);
    backLiftR.spin(forward,50,pct);
  } else if (buttons.pressing(BACK_LIFT_DOWNING)) {
    targetIsIntake = false;
    log("downing");
    backLiftL.spin(reverse,50,pct);
    backLiftR.spin(reverse,50,pct);
  } else if (buttons.released(BACK_LIFT_DOWNING) || buttons.released(BACK_LIFT_UPPING)) {
    targetIsIntake = false;
    log("stop");
    backLiftL.stop();
    backLiftR.stop();
  }

  if (targetIsIntake && backLiftR.isDone() && !backIsDown) backDown();
  else if (!targetIsIntake && backIsDown) backUp();

}

void Robot15::clawUp() {
  frontClaw.set(false);
}

void Robot15::clawDown() {
  frontClaw.set(true);
}

void Robot15::backUp() {
  backIsDown = false;
  backClamp.set(true);
  
}

void Robot15::backDown() {
  backIsDown = true;
  backClamp.set(false);
  
}

void Robot15::moveArmTo(double degr, double speed, bool blocking) {
  frontArmL.rotateTo(degr, degrees, speed, velocityUnits::pct, false);
  frontArmR.rotateTo(degr, degrees, speed, velocityUnits::pct, blocking);
}

// Move at speed until crossing threshold. No PID, is heaviside
// Returns true if there's a goal on arm, false if not, using current thresholds
bool Robot15::moveArmToManual(double degr, double speed) {

  VisualGraph g(0, 3, 10, 200, 2);
  g.configureAutomaticDisplay();

  PID diffPID(1, 0, 0);
  float correction;
  float pos = (frontArmL.position(deg) + frontArmR.position(deg));

  // Find average current over arm lift
  float sumCurrent = 0;
  int numSamples = 0;

  bool risingEdge = pos < degr;

  while (risingEdge ? pos < degr : pos > degr) {
    pos = (frontArmL.position(deg) + frontArmR.position(deg));
    float diff = (frontArmR.position(deg) - frontArmL.position(deg));
    correction = diffPID.tick(diff);

    setMotorVelocity(frontArmL, forward, speed + correction);
    setMotorVelocity(frontArmR, forward, speed - correction);

    float curr = (frontArmL.current() + frontArmR.current()) / 2.0;
    sumCurrent += curr;
    numSamples++;
    g.push(curr, 0);
    g.push(speed / 100.0, 1);

    log("%f", frontArmR.position(deg));

    wait(20, msec);
  }
  g.push((frontArmL.current() + frontArmR.current()) / 2.0);

  frontArmL.stop();
  frontArmR.stop();

  float avgCurrent = (numSamples == 0) ? 0 : sumCurrent / numSamples;
  logController("Average current: %f", avgCurrent);

  return avgCurrent > 0.6; // no load current ~0.45A, goal current ~0.86A
  
}

void Robot15::armTeleop() {

  float MOTOR_SPEED = 100;

  //logController("%f", frontArmL.rotation(degrees));
  
  float arm = buttons.axis(cMapping == BRIAN_MAPPING ? Buttons::LEFT_VERTICAL : Buttons::RIGHT_VERTICAL); // Brian's weird shit

  if (buttons.pressing(FRONT_ARM_UP)) {
    frontArmL.spin(forward, MOTOR_SPEED, pct);
    frontArmR.spin(forward, MOTOR_SPEED, pct);
  } else if (buttons.pressing(FRONT_ARM_DOWN)) {

    frontArmL.spin(reverse, MOTOR_SPEED, pct);
    frontArmR.spin(reverse, MOTOR_SPEED, pct);
    
  } else if (arm != 0) {
    frontArmL.spin(forward, arm * 100, pct);
    frontArmR.spin(forward, arm * 100, pct);
  }
  else {

    frontArmL.stop();
    frontArmR.stop();
  }

  if (buttons.pressing(CLAW_UP)) {
    clawUp();
  } else if (buttons.pressing(CLAW_DOWN)) {
    clawDown();
  }

}

void Robot15::intakeTeleop() {
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
void Robot15::teleop() {
  
  basicDriveTeleop();
  armTeleop();
  intakeTeleop();
  backLiftTeleop();

  buttons.updateButtonState();
}

// return in inches
float Robot15::getLeftEncoderDistance() {
  float sum = leftMotorA.rotation(deg) + leftMotorB.rotation(deg) + leftMotorC.rotation(deg) + leftMotorD.rotation(deg);
  return degreesToDistance(sum / 4.0);
}

// return in inches
float Robot15::getRightEncoderDistance() {
  float sum = rightMotorA.rotation(deg) + rightMotorB.rotation(deg) + rightMotorC.rotation(deg) + rightMotorD.rotation(deg);
  return degreesToDistance(sum / 4.0);
}

void Robot15::resetEncoderDistance() {
  leftMotorA.resetRotation();
  leftMotorB.resetRotation();
  leftMotorC.resetRotation();
  leftMotorD.resetRotation();
  rightMotorA.resetRotation();
  rightMotorB.resetRotation();
  rightMotorC.resetRotation();
  rightMotorD.resetRotation();
}

// Go forward a number of inches, maintaining a specific heading
// Calling general function with 15-specifc params

void Robot15::goForwardU(float distInches, float maxSpeed, float universalAngle, bool stopAfter, float timeout) {

  float minSpeed = 15;
  float rampUpFrames = maxSpeed * 0.2; // rampUp slope
  float slowDownInches = (maxSpeed - minSpeed) * 0.1; // slowDown slope
  float endSlowInches = 2;

  BaseRobot::goForwardU_Abstract(1.0, distInches, maxSpeed, universalAngle, rampUpFrames, slowDownInches, endSlowInches, 
    stopAfter, minSpeed, timeout);
}

// Turn to some universal angle based on starting point. Turn direction is determined by smallest angle to universal angle
// Calling general function with 15-specifc params
void Robot15::goTurnU(float universalAngleDegrees, int direction, bool stopAfter, float timeout, float maxSpeed) {
  BaseRobot::goTurnU_Abstract(2, 0, 0.13, 1, 5, 12, universalAngleDegrees, direction, stopAfter, timeout, maxSpeed);
}

// Go forward until the maximum distance is hit or the timeout is reached
// for indefinite timeout, set to -1
void Robot15::goVision(float distInches, float speed, Goal goal, float rampUpFrames, float slowDownInches, bool stopAfter, float timeout) {
  BaseRobot::goVision_Abstract(25, 12, CAMERA_PORT, distInches, speed, goal, rampUpFrames, slowDownInches, stopAfter, timeout);
}

// Align to the goal of specified color with PID
void Robot15::goAlignVision(Goal goal, float timeout, bool stopAfter) {
  BaseRobot::goAlignVision_Abstract(40, 0, 1, 0.05, 3, 12, CAMERA_PORT, goal, timeout, stopAfter);
}

void Robot15::setLeftVelocity(directionType d, double percent) {

  setMotorVelocity(leftMotorA, d, percent);
  setMotorVelocity(leftMotorB, d, percent);
  setMotorVelocity(leftMotorC, d, percent);
  setMotorVelocity(leftMotorD, d, percent);
}

void Robot15::setRightVelocity(directionType d, double percent) {
  
  setMotorVelocity(rightMotorA, d, percent);
  setMotorVelocity(rightMotorB, d, percent);
  setMotorVelocity(rightMotorC, d, percent);
  setMotorVelocity(rightMotorD, d, percent);
}

void Robot15::startIntake(directionType dir) {
  intake.spin(dir, 100, pct);
}

void Robot15::stopIntake() {
  intake.stop();
}

void Robot15::stopLeft() {
  leftMotorA.stop();
  leftMotorB.stop();
  leftMotorC.stop();
  leftMotorD.stop();
}

void Robot15::stopRight() {
  rightMotorA.stop();
  rightMotorB.stop();
  rightMotorC.stop();
  rightMotorD.stop();
}

void Robot15::setBrakeType(brakeType b) {
  leftMotorA.setBrake(b);
  leftMotorB.setBrake(b);
  leftMotorC.setBrake(b);
  leftMotorD.setBrake(b);
  rightMotorA.setBrake(b);
  rightMotorB.setBrake(b);
  rightMotorC.setBrake(b);
  rightMotorD.setBrake(b);
}

float Robot15::getDriveCurrent() {
  float currentSum = leftMotorA.current() + leftMotorB.current() + leftMotorC.current() + leftMotorD.current() + rightMotorA.current()
    + rightMotorB.current() + rightMotorC.current() + rightMotorD.current();
  return currentSum / 8;
}