#include "robot.h"

Robot::Robot(controller* c) : leftMotorA(0), leftMotorB(0), leftMotorC(0), leftMotorD(0), rightMotorA(0), rightMotorB(0), 
  rightMotorC(0), rightMotorD(0), sixBarFL(0), sixBarFR(0), sixBarBL(0), sixBarBR(0), frontArmL(0), frontArmR(0), camera(0), gyroSensor(PORT5), buttons(c) {

  leftMotorA = motor(PORT1, ratio6_1, false); 
  leftMotorB = motor(PORT2, ratio6_1, false);
  leftMotorC = motor(PORT3, ratio6_1, false);
  leftMotorD = motor(PORT4, ratio6_1, false);
  leftDrive = motor_group(leftMotorA, leftMotorB, leftMotorC, leftMotorD);

  rightMotorA = motor(PORT7, ratio6_1, true);
  rightMotorB = motor(PORT8, ratio6_1, true);
  rightMotorC = motor(PORT9, ratio6_1, true);
  rightMotorD = motor(PORT10, ratio6_1, true);
  rightDrive = motor_group(rightMotorA, rightMotorB, rightMotorC, rightMotorD);

  sixBarFL = motor(PORT17, ratio36_1, true);
  sixBarFR = motor(PORT19, ratio36_1, false);
  sixBarBL = motor(PORT18, ratio36_1, false);
  sixBarBR = motor(PORT20, ratio36_1, true);

  // forward is UP, reverse is DOWN
  frontArmL = motor(PORT17, ratio36_1, true);
  frontArmR = motor(PORT19, ratio36_1, false);


  driveType = TWO_STICK_ARCADE;
  robotController = c;

  sixBarFL.setBrake(hold);
  sixBarFR.setBrake(hold);
  sixBarBL.setBrake(hold);
  sixBarBR.setBrake(hold);

}

// take in axis value between -100 to 100, discard (-5 to 5) values, divide by 100, and cube
// output is num between -1 and 1
float normalize(float axisValue) {
  if (fabs(axisValue) <= 5) {
    return 0;
  }
  return pow(axisValue / 100.0, 3);

}

void Robot::driveTeleop() {

  float leftVert = normalize(robotController->Axis3.position());
  float leftHoriz = normalize(robotController->Axis4.position());
  float rightVert = normalize(robotController->Axis2.position());
  float rightHoriz = normalize(robotController->Axis1.position());


  if(driveType == TANK) {
    setLeftVelocity(forward,leftVert);
    setRightVelocity(forward,rightVert);
  }else{
    float drive = driveType == ONE_STICK_ARCADE ? rightVert:leftVert;
    float turn = rightHoriz / 2.0;
    float max = std::max(1.0, std::max(fabs(drive+turn), fabs(drive-turn)));
    setLeftVelocity(forward,100 * (drive+turn)/max);
    setRightVelocity(forward,100 * (drive-turn)/max);
  }
}

void Robot::armTeleop() {

  float MOTOR_SPEED = 100;

  if (robotController->ButtonUp.pressing()) {
    sixBarFL.spin(forward, MOTOR_SPEED, pct);
    sixBarFR.spin(forward, MOTOR_SPEED, pct);
  } else if (robotController->ButtonDown.pressing()) {
    sixBarFL.spin(reverse, MOTOR_SPEED, pct);
    sixBarFR.spin(reverse, MOTOR_SPEED, pct);
  } else {
    sixBarFL.stop();
    sixBarFR.stop();
  }
}

// Run every tick
void Robot::teleop() {
  
  driveTeleop();
  armTeleop();
}

void Robot::waitGyroCallibrate() {
  int i = 0;
  while (gyroSensor.isCalibrating()) {
    wait(20, msec);
    i++;
  }
}

void Robot::driveStraightTimed(float speed, directionType dir, int timeout, bool stopAfter, std::function<bool(void)> func) {
  driveStraight(0, speed, dir, timeout, 0, stopAfter, func);
}


void Robot::driveStraight(float distInches, float speed, directionType dir, int timeout, 
float slowDownInches, bool stopAfter, std::function<bool(void)> func) {

  driveCurved(distInches, speed, dir, timeout, slowDownInches, 0, stopAfter, func);

}

void Robot::driveCurved(float distInches, float speed, directionType dir, int timeout, 
float slowDownInches, float turnPercent, bool stopAfter, std::function<bool(void)> func) {

  smartDrive(distInches, speed, dir, dir, timeout, slowDownInches, turnPercent, stopAfter, func);

}

void Robot::driveTurn(float degrees, float speed, bool isClockwise, int timeout, float slowDownInches, 
bool stopAfter, std::function<bool(void)> func) {

  smartDrive(getTurnAngle(degrees), speed, isClockwise ? forward : reverse, isClockwise ? reverse: forward,
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
void Robot::smartDrive(float distInches, float speed, directionType left, directionType right, int timeout, 
float slowDownInches, float turnPercent, bool stopAfter, std::function<bool(void)> func) {


  float finalDist = distInches == 0? -1 : distanceToDegrees(distInches);
  float slowDown = distanceToDegrees(slowDownInches);

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
    float baseSpeed = FORWARD_MIN_SPEED + (speed-FORWARD_MIN_SPEED) * proportion;

    //log("%f", baseSpeed);

    // reduce baseSpeed so that the faster motor always capped at max speed
    baseSpeed = fmin(baseSpeed, 100 - baseSpeed*turnPercent);

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

// Move forward/backward with proportional gyro feedback.
// finalDegrees is the delta yaw angle at the end of the curve
void Robot::driveStraightGyro(float distInches, float speed, directionType dir, int timeout, float slowDownInches, bool stopAfter, std::function<bool(void)> func) {

  float finalDist = distanceToDegrees(distInches);
  float slowDown = distanceToDegrees(slowDownInches);

  log("straight gyro");


  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();
  gyroSensor.resetRotation();

  const float GYRO_CONSTANT = 0.01;
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

  if (stopAfter) {
    stopLeft();
    stopRight();
  }
  
  
}

// BOTH angleDegrees AND startSlowDownDegrees SHOULD BE POSITIVE
// angleDegrees indicates the angle to turn to.
// startSlowDownDegrees is some absolute angle less than angleDegrees, where gyro proportional control starts
// maxSpeed is the starting speed of the turn. Will slow down once past startSlowDownDegrees theshhold
void Robot::turnToAngleGyro(bool clockwise, float angleDegrees, float maxSpeed, int startSlowDownDegrees,
int timeout, std::function<bool(void)> func) {


  float speed;

  log("initing");
  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();
  gyroSensor.resetRotation();
  log("about to loop");

  float currDegrees = 0; // always positive with abs

  while (currDegrees < angleDegrees && !isTimeout(startTime, timeout)) {
    // if there is a concurrent function to run, run it
    if (func) {
      if (func()) {
        // if func is done, make it empty
        func = {};
      }
    }

    currDegrees = fabs(gyroSensor.rotation());
    if (currDegrees < startSlowDownDegrees) {
      // before hitting theshhold, speed is constant at starting speed
      speed = maxSpeed;
    } else {
      float delta = (angleDegrees - currDegrees) / (angleDegrees - startSlowDownDegrees); // starts at 1 @deg=startSlowDeg, becomes 0 @deg = final
      speed = TURN_MIN_SPEED + delta * (speed - TURN_MIN_SPEED);
    }

    log("%f %f", speed, currDegrees);
    setLeftVelocity(clockwise ? forward : reverse, speed);
    setRightVelocity(clockwise ? reverse : forward, speed);
    wait(20, msec);
  }

  stopLeft();
  stopRight();
}

  void Robot::updateCamera(Goal goal) {
    camera = vision(PORT16, goal.bright, goal.sig);
  }

// Go forward until the maximum distance is hit, the timeout is reached, or limitSwitch is turned on (collision with goal)
// for indefinite timeout, set to -1
void Robot::goForwardVision(Goal goal, float speed, directionType dir, float maxDistanceInches, int timeout, 
digital_in* limitSwitch, std::function<bool(void)> func) {

  // The proportion to turn in relation to how offset the goal is. Is consistent through all speeds
  const float PMOD_MULTIPLIER = 0.35;

  int pMod = speed * PMOD_MULTIPLIER;
  float baseSpeed = fmin(speed, 100 - pMod);

  float totalDist = distanceToDegrees(maxDistanceInches);
  float dist = 0;

  updateCamera(goal);

  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  int sign = (dir == forward) ? 1 : -1;

  // forward until the maximum distance is hit, the timeout is reached, or limitSwitch is turned on
  while (dist < totalDist && !isTimeout(startTime, timeout) && (limitSwitch == nullptr || limitSwitch->value())) {
    // log("Start of loop");
    // if there is a concurrent function to run, run it
    if (func) {
      if (func()) {
        // if func is done, make it empty
        func = {};
      }
    }

    // log("Before snapshot");
    camera.takeSnapshot(goal.sig);
    displayVision(&camera);
    
    if(camera.largestObject.exists) {
      //log("%d", camera.largestObject.centerX);
      float mod = pMod * (VISION_CENTER_X-camera.largestObject.centerX) / VISION_CENTER_X;
      setLeftVelocity(dir, baseSpeed - mod*sign);
      setRightVelocity(dir, baseSpeed + mod*sign);
    }

    wait(20, msec);
    dist = fabs((leftMotorA.position(degrees) + rightMotorA.position(degrees)) / 2.0);
  }

  stopLeft();
  stopRight();
  
}

void Robot::alignToGoalVision(Goal goal, bool clockwise, int timeout) {

    // spin speed is proportional to distance from center, but must be bounded between MIN_SPEED and MAX_SPEED
  const float MAX_SPEED = 40;

  updateCamera(goal);

  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  // // At the initial snapshot we check where goal is seen. If so, we set our direction to be towards the goal and stop when we pass the center.
  // // Otherwise, the spin will default to the direction specified by the 'clockwise' parameter
  // camera.takeSnapshot(goal.sig);
  // if (camera.largestObject.exists) {
  //   clockwise = (camera.largestObject.centerX / VISION_CENTER_X) > VISION_CENTER_X;
  // }

  int spinSign = clockwise ? -1 : 1;

  while (!isTimeout(startTime, timeout)) {

    camera.takeSnapshot(goal.sig);

    float mod;
    if (camera.largestObject.exists) {
      mod = (VISION_CENTER_X-camera.largestObject.centerX) / VISION_CENTER_X;

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

void Robot::clampArmsDown() {

  // Weird Kohmei thing to keep arms low
  sixBarFL.spin(reverse, 20, percent);
  sixBarFR.spin(reverse, 20, percent);
  sixBarBL.spin(reverse, 20, percent);
  sixBarBR.spin(reverse, 20, percent);
}

void Robot::setFrontClamp(bool clamp) {
  frontClaw.set(clamp);
}

void Robot::setBackClamp(bool clamp) {
  backClaw.set(clamp);
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

void Robot::setTransmission(bool slow) {
  drivePistonLeft.set(slow);
  drivePistonRight.set(slow);
}

void Robot::raiseFrontArm(double amount, double vel, bool blocking) {
  sixBarFL.rotateFor(forward, amount, degrees, vel, velocityUnits::pct, false); // always false here, so both arms raise concurrently
  sixBarFR.rotateFor(forward, amount, degrees, vel, velocityUnits::pct, blocking);
}

void Robot::raiseBackArm(double amount, double vel, bool blocking) {
  sixBarBL.rotateFor(forward, amount, degrees, vel, velocityUnits::pct, false); // always false here, so both arms raise concurrently
  sixBarBR.rotateFor(forward, amount, degrees, vel, velocityUnits::pct, blocking);
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

