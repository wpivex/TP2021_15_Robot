#include "../include/robot.h"

// Motor ports Left: 1R, 2F, 3F,  20T Right: 12R, 11F, 13F
// gear ratio is 60/36
Robot::Robot(controller* c) : leftMotorA(0), leftMotorB(0), leftMotorC(0), leftMotorD(0), rightMotorA(0), rightMotorB(0), 
  rightMotorC(0), rightMotorD(0), sixBarFL(0), sixBarFR(0), sixBarBL(0), sixBarBR(0), gyroSensor(PORT6), buttons(c) {
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

  // forward is UP, reverse is DOWN
  sixBarFL = motor(PORT17, ratio36_1, true);
  sixBarFR = motor(PORT19, ratio36_1, false);
  sixBarBL = motor(PORT18, ratio36_1, false);
  sixBarBR = motor(PORT20, ratio36_1, true);

  driveType = ARCADE;
  robotController = c;

  sixBarFL.setBrake(hold);
  sixBarFR.setBrake(hold);
  sixBarBL.setBrake(hold);
  sixBarBR.setBrake(hold);
}

// Run every tick
void Robot::teleop() {

  // call at the start of each tick to get which buttons are pressed
  buttons.updateButtonState();

  driveTeleop();
  sixBarTeleop();
  clawTeleop();
  pneumaticsTeleop();

  wait(20, msec);
}

void Robot::driveTeleop() {
  float leftVert = (float) robotController->Axis3.position();
  float rightHoriz = (pow((float) robotController->Axis1.position()/100.0, 3)*100.0);

  if(driveType == ARCADE) {
    float left;
    float right;
    if(rightHoriz < 0) {
      left = leftVert + rightHoriz;
      right = leftVert - rightHoriz;
    } else {
      left = leftVert + rightHoriz;
      right = leftVert - rightHoriz;
    }

    setLeftVelocity(forward, left/fabs(left)*fmin(fabs(left), 100));
    setRightVelocity(forward, right/fabs(right)*fmin(fabs(right), 100));
  }
}

void Robot::driveStraightTimed(float speed, directionType dir, int timeMs, bool stopAfter, std::function<bool(void)> func) {
  driveStraight(0, speed, dir, timeMs, 1, stopAfter, func);
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

// distInches is positive distance in inches to destination. 0 means indefinite (until timeout)
// speed is percent 1-100
// direction is for left motor, right depends if turning
// timeout (optional parameter defaults to 0 -> none) in ms, terminates once reached
// slowDownInches representing from what distance to destination the robot starts slowing down with proportional speed
//  control in relation to distInches. Set by default to 10. 0 means attempt instant stop.
//  a higher value is more controlled/consistent, a lower value is faster/more variable
// turnPercent (from 0-1) is percent of speed to curve (so curvature now independent from speed). optional, default to 0
// stopAfter whether to stop motors after function call.
// func is an optional nonblocking function you can use to run as the same time as this method. It returns true when nonblocking function is gone
void Robot::smartDrive(float distInches, float speed, directionType left, directionType right, int timeout, 
float slowDownInches, float turnPercent, bool stopAfter, std::function<bool(void)> func) {

  float finalDist = distanceToDegrees(distInches);
  float slowDown = distanceToDegrees(slowDownInches);

  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  // finalDist is 0 if we want driveTimed instead of drive some distance
  float currentDist = 0;
  while ((finalDist == 0 || currentDist < finalDist) && (timeout == 0 || vex::timer::system() < startTime + timeout)) {

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
    float baseSpeed = speed*proportion;

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

}



void Robot::waitGyroCallibrate() {
  while (gyroSensor.isCalibrating()) {
    wait(20, msec);
  }
}



void Robot::driveTimed(float percent, float driveTime) {
  int milliseconds = vex::timer::system();
  while (vex::timer::system() < milliseconds + driveTime) {
    setLeftVelocity(forward, percent);
    setRightVelocity(forward, percent);
  }
  stopLeft();
  stopRight();
}

//12.375 in wheelbase
void Robot::turnToAngle(float percent, float turnAngle, bool PID, directionType direction) {

  int targetDist = getTurnAngle(turnAngle);
  
  while (true) {
    if(turnToAngleNonblocking(percent, targetDist, PID, direction)) break;
    wait(20, msec);
  }
  stopLeft();
  stopRight();
}



// Call this method every tick. Must reset encoders of left and right motor A
// Return true if execution completed
bool Robot::turnToAngleNonblocking(float percent, float targetDist, bool PID, directionType direction) {
  float currPos = (fabs(leftMotorA.position(degrees)) + fabs(rightMotorA.position(degrees))) / 2;

  if (currPos < targetDist) {
    setLeftVelocity(direction, 5 + (percent - 5) * (PID? ((targetDist - currPos) / targetDist) : 1));
    setRightVelocity(direction == forward ? reverse : forward, 5 + (percent - 5) * (PID? ((targetDist - currPos) / targetDist) : 1));
    return false;
  } else {
    return true;
  }
}

void Robot::driveCurvedTimed(directionType d, int delta, int speed, float driveTime) {

  int baseSpeed = speed-abs(delta)/2.0;
  int velLeft = baseSpeed + delta/2.0;
  int velRight = baseSpeed - delta/2.0;

  int milliseconds = vex::timer::system();
  while (vex::timer::system() < milliseconds + driveTime) {

    setLeftVelocity(d, velLeft);
    setRightVelocity(d, velRight);

    wait(20, msec);
  }
  stopLeft();
  stopRight();
}

void Robot::printYaw() {
  while (true) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print(gyroSensor.rotation());
    wait(20,msec);
  }
}




// proportional turn with gyro. Degrees MUST be positive
// forward = clockwise, reverse = counterclockwise
void Robot::gyroTurn(directionType dir, float degrees) {


  double PROPORTIONAL_SCALE = 2;

  gyroSensor.resetRotation();

  while (true) {

    double delta = degrees - fabs(gyroSensor.rotation());

    if (delta <= 0) break;

    double speed = fmax(10, fmin(100, delta*PROPORTIONAL_SCALE));
    setLeftVelocity(dir, speed);
    setRightVelocity(dir, -speed);

    wait(20, msec);

  }

}

void Robot::raiseFrontArm(double amount, double vel, bool blocking) {
  sixBarFL.rotateFor(forward, amount, degrees, vel, velocityUnits::pct, false); // always false here, so both arms raise concurrently
  sixBarFR.rotateFor(forward, amount, degrees, vel, velocityUnits::pct, blocking);
}

void Robot::raiseBackArm(double amount, double vel, bool blocking) {
  sixBarBL.rotateFor(forward, amount, degrees, vel, velocityUnits::pct, false); // always false here, so both arms raise concurrently
  sixBarBR.rotateFor(forward, amount, degrees, vel, velocityUnits::pct, blocking);
}

void Robot::driveCurved(directionType d, float dist, int delta) {
  driveCurved(d, dist, delta, 100);
}

// delta ranges from -100 (hard left) and 100 (hard right). 0 is straight
void Robot::driveCurved(directionType d, float dist, int delta, int speed) {
  int baseSpeed = speed-abs(delta)/2.0;
  int velLeft = baseSpeed + delta/2.0;
  int velRight = baseSpeed - delta/2.0;

  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  float currPos = 0.0;
  float targetPos = distanceToDegrees(dist);

  while (currPos < targetPos) {

    setLeftVelocity(d, velLeft);
    setRightVelocity(d, velRight);

    float currLeft = leftMotorA.position(degrees);
    float currRight = rightMotorA.position(degrees);
    currPos = fabs((currLeft + currRight) / 2.0);
    Robot::robotController->Screen.clearScreen();
    Robot::robotController->Screen.print(currPos);

    wait(20, msec);
  }
  stopLeft();
  stopRight();
}

void Robot::setFrontClamp(bool clamp) {
  frontClaw.set(clamp);
}

void Robot::setBackClamp(bool clamp) {
  backClaw.set(clamp);
}

void Robot::setTransmission(bool slow) {
  drivePistonLeft.set(slow);
  drivePistonRight.set(slow);
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