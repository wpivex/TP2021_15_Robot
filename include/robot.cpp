#include "robot.h"

Robot::Robot(controller* c) : leftMotorA(0), leftMotorB(0), leftMotorC(0), leftMotorD(0), rightMotorA(0), rightMotorB(0), 
  rightMotorC(0), rightMotorD(0), frontArmL(0), frontArmR(0), backLiftL(0), backLiftR(0), frontCamera(0), gyroSensor(PORT4), buttons(c) {

  leftMotorA = motor(PORT3, ratio6_1, true); 
  leftMotorB = motor(PORT11, ratio6_1, true);
  leftMotorC = motor(PORT12, ratio6_1, true);
  leftMotorD = motor(PORT14, ratio6_1, true);
  leftDrive = motor_group(leftMotorA, leftMotorB, leftMotorC, leftMotorD);

  // 3L 11L 12L 14L left
  // 16r 18r 19r 20r right

  rightMotorA = motor(PORT17, ratio6_1, false);
  rightMotorB = motor(PORT18, ratio6_1, false);
  rightMotorC = motor(PORT19, ratio6_1, false);
  rightMotorD = motor(PORT20, ratio6_1, false);
  rightDrive = motor_group(rightMotorA, rightMotorB, rightMotorC, rightMotorD);

  // forward is UP, reverse is DOWN
  frontArmL = motor(PORT1, ratio36_1, true);
  frontArmR = motor(PORT9, ratio36_1, false);

  backLiftL = motor(PORT8, ratio36_1, true);
  backLiftR = motor(PORT13, ratio36_1, true);

  // 8 and 13??
  
  robotController = c; 

  frontArmL.setBrake(hold);
  frontArmR.setBrake(hold);
  backLiftL.setBrake(hold);
  backLiftR.setBrake(hold);

  setControllerMapping(BRIAN_MAPPING);
}

void Robot::setControllerMapping(ControllerMapping mapping) {

  cMapping = mapping;


  //Controls that don't change:
  BACK_LIFT_MID = Buttons::R1;
  BACK_LIFT_DOWN = Buttons::R2;
  BACK_LIFT_UPPING = Buttons::RIGHT;
  BACK_LIFT_DOWNING = Buttons::LEFT;

  CLAW_UP = Buttons::L1;
  CLAW_DOWN = Buttons::L2;

  //Controls that do change:
  if (mapping == DEFAULT_MAPPING) {

    driveType = TWO_STICK_ARCADE;

    FRONT_ARM_UP = Buttons::UP;
    FRONT_ARM_DOWN = Buttons::DOWN;
    BACK_LIFT_UP = Buttons::NONE;


  } else if (mapping == BRIAN_MAPPING) {
    driveType = ONE_STICK_ARCADE;

    FRONT_ARM_UP = Buttons::NONE; // brian uses left-stick controls
    FRONT_ARM_DOWN = Buttons::NONE;
    BACK_LIFT_UP = Buttons::UP;

  }

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
  
  float brianArm = buttons.axis(Buttons::LEFT_VERTICAL); // Brian's weird shit

  if (buttons.pressing(FRONT_ARM_UP)) {
    frontArmL.spin(forward, MOTOR_SPEED, pct);
    frontArmR.spin(forward, MOTOR_SPEED, pct);
  } else if (buttons.pressing(FRONT_ARM_DOWN)) {

    frontArmL.spin(reverse, MOTOR_SPEED, pct);
    frontArmR.spin(reverse, MOTOR_SPEED, pct);
    
  } else if (cMapping == BRIAN_MAPPING && brianArm != 0) {
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
  backLiftTeleop();

  buttons.updateButtonState();
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
  
  initialPitch = gyroSensor.roll(); 
}

void Robot::driveStraightTimed(float speed, directionType dir, float timeout, bool stopAfter, std::function<bool(void)> func) {
  driveStraight(0, speed, dir, timeout, 0, stopAfter, func);
}


void Robot::driveStraight(float distInches, float speed, directionType dir, float timeout, 
float slowDownInches, bool stopAfter, std::function<bool(void)> func, float startUpInches) {

  driveCurved(distInches, speed, dir, timeout, slowDownInches, 0, stopAfter, func, startUpInches);

}

void Robot::driveCurved(float distInches, float speed, directionType dir, float timeout, 
float slowDownInches, float turnPercent, bool stopAfter, std::function<bool(void)> func, float startUpInches) {

  smartDrive(distInches, speed, dir, dir, timeout, slowDownInches, turnPercent, stopAfter, func, startUpInches);

}

void Robot::driveTurn(float degrees, float speed, bool isClockwise, float timeout, float slowDownInches, 
bool stopAfter, std::function<bool(void)> func) {

  smartDrive(degrees, speed, isClockwise ? forward : reverse, isClockwise ? reverse: forward,
  timeout, slowDownInches, 0, stopAfter, func);

}

// distInches is positive distance in inches to destination. -1 means indefinite (until timeout)
// speed is percent 1-100
// direction is for left motor, right depends if turning
// timeout (optional parameter defaults to -1 -> none) in ms, terminates once reached
// slowDownInches representing from what distance to destination the robot starts slowing down with proportional speed
//  control in relation to distInches. Set by default to 10. 0 means attempt instant stop.
//  a higher value is more controlled/consistent, a lower value is faster/more variable
// turnPercent (from 0-1) is percent of speed to curve (so curvature now independent from speed). optional, default to 0
// stopAfter whether to stop motors after function call.
// func is an optional nonblocking function you can use to run as the same time as this method. It returns true when nonblocking function is gone
void Robot::smartDrive(float distInches, float speed, directionType left, directionType right, float timeout, 
float slowDownInches, float turnPercent, bool stopAfter, std::function<bool(void)> func, float startUpInches) {

  if (slowDownInches > distInches) slowDownInches = distInches;

  float finalDist = distInches == 0? -1 : distanceToDegrees(distInches);
  float slowDown = distanceToDegrees(slowDownInches);
  float startUp = distanceToDegrees(startUpInches);

  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  // finalDist is 0 if we want driveTimed instead of drive some distance
  float currentDist = 0;
  while ((finalDist == -1 || currentDist < finalDist) && (timeout == -1 || vex::timer::system() < startTime + timeout*1000)) {

    // if there is a concurrent function to run, run it
    if (func) {
      if (func()) {
        // if func is done, make it empty
        func = {};
      }
    }

    currentDist = (fabs(leftMotorA.position(degrees)) + fabs(rightMotorA.position(degrees))) / 2;

     // from 0 to 1 indicating proportion of velocity. Starts out constant at 1 until it hits the slowDown interval,
     // where then it linearly decreases to 0
    float proportion = slowDown == 0 ? 1 : fmin(1, 1 - (currentDist - (finalDist - slowDown)) / slowDown);
    if (currentDist < startUp && startUp != 0) proportion = currentDist / startUp;
    float baseSpeed = FORWARD_MIN_SPEED + (speed-FORWARD_MIN_SPEED) * proportion;

    

    // reduce baseSpeed so that the faster motor always capped at max speed
    baseSpeed = fmin(baseSpeed, 100 - baseSpeed*turnPercent);

    log("%f %f", degreesToDistance(leftMotorA.position(degrees)), degreesToDistance(rightMotorA.position(degrees)));

    setLeftVelocity(left, baseSpeed*(1 + turnPercent));
    setRightVelocity(right, baseSpeed*(1 - turnPercent));
    
    wait(20, msec);

  }

  if (stopAfter) {
    stopLeft();
    stopRight();
  }

  log("done");

}

// Detect whether robot has reached some velocity. If not, never exit or release claw (fighting other robot in mid)
// velocity is sigma of acceleration * delta t
void Robot::driveStraightFighting(float distInches, float speed, directionType dir) {

  float CURRENT_THRESHHOLD = 1.0;
  int numCurrentReached = 0;
  int NUM_CURRENT_NEEDED = 4;

  float finalDist = fabs(distanceToDegrees(distInches));
  float currentDist = 0;

  leftMotorA.resetRotation();
  rightMotorA.resetRotation();

  while (currentDist < finalDist || numCurrentReached < NUM_CURRENT_NEEDED) {

    float c = (leftMotorA.current() + rightMotorA.current()) / 2.0;
    if (c <= CURRENT_THRESHHOLD) {
      numCurrentReached++;
    } else numCurrentReached = 0;
    logController("%d %f", numCurrentReached, c);

    currentDist = fabs(leftMotorA.rotation(degrees) + rightMotorA.rotation(degrees)) / 2;
    

    setLeftVelocity(dir, speed);
    setRightVelocity(dir, speed);

    wait(20, msec);
  }

}

// Move forward/backward with proportional gyro feedback.
// finalDegrees is the delta yaw angle at the end of the curve
void Robot::driveStraightGyro(float distInches, float speed, directionType dir, float timeout, float slowDownInches, 
bool resetEncoder, std::function<bool(void)> func, float startUpInches) {

  if (slowDownInches > distInches) slowDownInches = distInches;

  float finalDist = distanceToDegrees(distInches);
  float slowDown = distanceToDegrees(slowDownInches);
  float startUp = distanceToDegrees(startUpInches);


  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();
  if (resetEncoder) gyroSensor.resetRotation();

  const float GYRO_CONSTANT = 0.008;
  bool hasSetToDone = false;

  // finalDist is 0 if we want driveTimed instead of drive some distance
  float currentDist = 0;
  while ((finalDist == 0 || currentDist < finalDist) && !isTimeout(startTime, timeout)) {

    // if there is a concurrent function to run, run it
    if (func) {
      bool done = func();
      log("running concurrent %d %d", done ? 1 : 0, hasSetToDone ? 1 : 0);
      if (done) {
        // if func is done, make it empty
        func = {};
        hasSetToDone = true;
      }
    }

    currentDist = (fabs(leftMotorA.position(degrees)) + fabs(rightMotorA.position(degrees))) / 2;

     // from 0 to 1 indicating proportion of velocity. Starts out constant at 1 until it hits the slowDown interval,
     // where then it linearly decreases to 0
    float proportion = slowDown == 0 ? 1 : fmin(1, 1 - (currentDist - (finalDist - slowDown)) / slowDown);
    if (currentDist < startUp && startUp != 0) proportion = currentDist / startUp;
    float baseSpeed = FORWARD_MIN_SPEED + (speed-FORWARD_MIN_SPEED) * proportion;

    float gyroCorrection = gyroSensor.rotation() * GYRO_CONSTANT;


    // reduce baseSpeed so that the faster motor always capped at max speed
    baseSpeed = fmin(baseSpeed, 100 - baseSpeed*gyroCorrection);

    float left = baseSpeed*(1 - gyroCorrection);
    float right = baseSpeed*(1 + gyroCorrection);
    setLeftVelocity(dir, left);
    setRightVelocity(dir, right);
    //log("%f %f %f", gyroCorrection, left, right);
    
    wait(20, msec);
  }

  stopLeft();
  stopRight();

  
  
}
// drive straight in a specific direction (0-360)
void Robot::driveStraightGyroHeading(float distInches, float speed, float head, directionType dir, float timeout, float slowDownInches, 
std::function<bool(void)> func, float startUpInches) {

  float correction = gyroSensor.heading() - head;
  if (correction > 180) correction -= 360;
  else if (correction < -180) correction += 360;
  logController("correction: %f", correction);
  gyroSensor.setRotation(correction, degrees);

  driveStraightGyro(distInches, speed, dir, timeout, slowDownInches, false, func, startUpInches);

}


// void Robot::dumbGyroTurn(bool clockwise, float angleDegrees, float speed, int timeout) {

//   turnToAngleGyro(clockwise, angleDegrees, speed, 180, timeout);

//   /*

//   int startTime = vex::timer::system();
//   leftMotorA.resetPosition();
//   rightMotorA.resetPosition();
//   gyroSensor.resetRotation();

//   while (fabs(gyroSensor.rotation()) < angleDegrees && !isTimeout(startTime, timeout)) {
//     setLeftVelocity(clockwise ? forward : reverse, speed);
//     setRightVelocity(clockwise ? reverse : forward, speed);
//     wait(20, msec);
//   }
//   stopLeft();
//   stopRight();

//   */

// }

// BOTH angleDegrees AND startSlowDownDegrees SHOULD BE POSITIVE
// angleDegrees indicates the angle to turn to.
// startSlowDownDegrees is some absolute angle less than angleDegrees, where gyro proportional control starts
// maxSpeed is the starting speed of the turn. Will slow down once past startSlowDownDegrees theshhold
void Robot::gyroTurn(bool clockwise, float angleDegrees) {

  float K_PROPORTIONAL = 0.42;
  float K_DERIVATIVE = 0.003;
  float tolerance = 4;

  float timeout = 4;
  if (angleDegrees < 25){
    timeout = 2;
  }

  float speed;

  log("initing");
  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();
  gyroSensor.resetRotation();
  log("about to loop");

  float currDegrees = 0; // always positive with abs
  float delta = 0;
  float delta_prev = 0;
  float delta_dir = 0;

  int NUM_VALID_THRESHOLD = 10;
  int numValid = 0;

  while (numValid < NUM_VALID_THRESHOLD && !isTimeout(startTime, timeout)) {

    currDegrees = fabs(gyroSensor.rotation());

    delta = angleDegrees - currDegrees;
    delta_dir = (delta - delta_prev)/0.02;

    speed = delta * K_PROPORTIONAL + delta_dir * K_DERIVATIVE;
    

    //logController("%d %f %f", clockwise? 1:0, speed, delta);
    logController("%f %f", delta*K_PROPORTIONAL, delta_dir*K_DERIVATIVE);
    setLeftVelocity(clockwise ? forward : reverse, speed);
    setRightVelocity(clockwise ? reverse : forward, speed);

    delta_prev = delta;

    wait(20, msec);
    if (fabs(currDegrees - angleDegrees) < tolerance) numValid++;
    else numValid = 0;
    //logController("%d %f", numValid, delta);
  }

  stopLeft();
  stopRight();
}

void Robot::gyroTurnU(float universalAngleDegrees) {
  float universalHeading = gyroSensor.heading(degrees);
  float turnAngle = fabs(universalHeading-universalAngleDegrees);
  bool clockwise = universalHeading < universalAngleDegrees;
  if (turnAngle > 180) {
    clockwise = !clockwise;
    turnAngle = 360 - turnAngle;
  }
  gyroTurn(clockwise, turnAngle);
}

// void Robot::dumbUniversalGyroTurn(float universalAngleDegrees, float maxSpeed, int timeout) {

//   turnToUniversalAngleGyro(universalAngleDegrees, maxSpeed, 180, timeout);

//   /*
//   float universalHeading = gyroSensor.heading(degrees);
//   float turnAngle = fabs(universalHeading-universalAngleDegrees);
//   bool clockwise = universalHeading < universalAngleDegrees;
//   if (turnAngle > 180) {
//     clockwise = !clockwise;
//     turnAngle = 360 - turnAngle;
//   }
//   dumbGyroTurn(clockwise, turnAngle, maxSpeed, timeout);
//   */
// }

void Robot::updateCamera(Goal goal) {
  frontCamera = vision(PORT5, goal.bright, goal.sig);
}

// Go forward until the maximum distance is hit, the timeout is reached, or limitSwitch is turned on (collision with goal)
// for indefinite timeout, set to -1
void Robot::goForwardVision(Goal goal, float speed, directionType dir, float maxDistanceInches, int timeout, 
digital_in* limitSwitch, std::function<bool(void)> func) {

  // The proportion to turn in relation to how offset the goal is. Is consistent through all speeds
  const float PMOD_MULTIPLIER = 0.3;

  int pMod = speed * PMOD_MULTIPLIER;
  float baseSpeed = fmin(speed, 100 - pMod);

  float totalDist = distanceToDegrees(maxDistanceInches);
  float dist = 0;

  updateCamera(goal);

  vision *camera = &frontCamera;

  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  int sign = (dir == forward) ? 1 : -1;

  // forward until the maximum distance is hit, the timeout is reached, or limitSwitch is turned on
  while (dist < totalDist && !isTimeout(startTime, timeout) && (limitSwitch == nullptr || !limitSwitch->value())) {
    // log("Start of loop");
    // if there is a concurrent function to run, run it
    if (func) {
      if (func()) {
        // if func is done, make it empty
        func = {};
      }
    }

    // log("Before snapshot");
    camera->takeSnapshot(goal.sig);
    
    if(camera->largestObject.exists) {
      log("%d", camera->largestObject.centerX);
      float mod = pMod * (VISION_CENTER_X-camera->largestObject.centerX) / VISION_CENTER_X;
      setLeftVelocity(dir, baseSpeed - mod*sign);
      setRightVelocity(dir, baseSpeed + mod*sign);
    }

    wait(20, msec);
    dist = fabs((leftMotorA.position(degrees) + rightMotorA.position(degrees)) / 2.0);
  }

  stopLeft();
  stopRight();
  
}

void Robot::alignToGoalVision(Goal goal, bool clockwise, directionType cameraDirection, int timeout) {

  // spin speed is proportional to distance from center, but must be bounded between MIN_SPEED and MAX_SPEED
  const float MAX_SPEED = 10;

  updateCamera(goal);
  vision *camera = &frontCamera;

  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  // // At the initial snapshot we check where goal is seen. If so, we set our direction to be towards the goal and stop when we pass the center.
  // // Otherwise, the spin will default to the direction specified by the 'clockwise' parameter
  // camera->takeSnapshot(goal.sig);
  // if (camera->largestObject.exists) {
  //   clockwise = (camera->largestObject.centerX / VISION_CENTER_X) > VISION_CENTER_X;
  // }

  int spinSign = clockwise ? -1 : 1;

  while (!isTimeout(startTime, timeout)) {

    camera->takeSnapshot(goal.sig);

    float mod;
    if (camera->largestObject.exists) {
      // log("%d", camera->largestObject.centerX);
      mod = (VISION_CENTER_X-camera->largestObject.centerX) / VISION_CENTER_X;

      // If goal is [left side of screen if clockwise, right side of scree if counterclockwise], that means it's arrived at center and is aligned
      if (fabs(mod) <= 0.05) {
        break;
      }

    } else {
      // If largest object not detected, then spin in the specified direction
      mod = spinSign;
    }

    float speed = (mod > 0 ? 1 : -1) * TURN_MIN_SPEED + mod * (MAX_SPEED - TURN_MIN_SPEED);

    setLeftVelocity(reverse, speed);
    setRightVelocity(forward, speed);
    wait(20, msec);
  }

  stopLeft();
  stopRight();
}


void Robot::setLeftVelocity(directionType d, double percent) {
  leftMotorA.spin(d, percent, pct);
  leftMotorB.spin(d, percent, pct);
  leftMotorC.spin(d, percent, pct);
  leftMotorD.spin(d, percent, pct);
}

void Robot::setRightVelocity(directionType d, double percent) {
  rightMotorA.spin(d, percent, pct);
  rightMotorB.spin(d, percent, pct);
  rightMotorC.spin(d, percent, pct);
  rightMotorD.spin(d, percent, pct);
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