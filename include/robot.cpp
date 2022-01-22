#include "robot.h"
#include <math.h>

// Motor ports Left: 1R, 2F, 3F,  20T Right: 12R, 11F, 13F
// gear ratio is 60/36
Robot::Robot(controller* c) : leftMotorA(0), leftMotorB(0), leftMotorC(0), leftMotorD(0), rightMotorA(0), rightMotorB(0), 
  rightMotorC(0), rightMotorD(0), sixBarFL(0), sixBarFR(0), sixBarBL(0), sixBarBR(0), claw(0), frontCamera(0), 
  backCamera(0) {
  leftMotorA = motor(PORT1, ratio18_1, true); 
  leftMotorB = motor(PORT2, ratio18_1, true);
  leftMotorC = motor(PORT3, ratio18_1, true);
  leftMotorD = motor(PORT4, ratio18_1, true);
  leftDrive = motor_group(leftMotorA, leftMotorB, leftMotorC, leftMotorD);

  rightMotorA = motor(PORT11, ratio18_1, false);
  rightMotorB = motor(PORT12, ratio18_1, false);
  rightMotorC = motor(PORT13, ratio18_1, false);
  rightMotorD = motor(PORT14, ratio18_1, false);
  rightDrive = motor_group(rightMotorA, rightMotorB, rightMotorC, rightMotorD);

  sixBarFL = motor(PORT10, ratio18_1, true);
  sixBarFR = motor(PORT8, ratio18_1, false);
  sixBarBL = motor(PORT19, ratio18_1, true);
  sixBarBR = motor(PORT7, ratio18_1, false);
  claw = motor(16, ratio18_1, true);



  driveType = ARCADE;
  robotController = c; 
  frontCamera = vision(PORT20, 50, *SIG_1);
  backCamera = vision(PORT6, 50, *SIG_1);


  sixBarFL.setBrake(hold);
  sixBarFR.setBrake(hold);
  sixBarBL.setBrake(hold);
  sixBarBR.setBrake(hold);
  claw.setBrake(hold);
}

void Robot::driveTeleop() {
  float leftVert = (float) robotController->Axis3.position();
  float rightVert = (float) robotController->Axis2.position();
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


void Robot::goalClamp() {
  /*if (Robot::robotController->ButtonL1.pressing()) {
    time_t now = std::time(nullptr);
    if(now - lastLeftPress > 0.5) {
      frontGoal.set(!frontGoal.value());
      lastLeftPress = now;
    }
  }
  if (Robot::robotController->ButtonR1.pressing()) {
    time_t now = std::time(nullptr);
    if(now - lastRightPress > 0.5) {
      backGoal.set(!backGoal.value());
      lastRightPress = now;
    }
  }*/
}

void Robot::setFrontClamp(bool intaking) {
  //frontGoal.set(intaking);
}

void Robot::setBackClamp(bool intaking) {
  //backGoal.set(intaking);
}

// Run every tick
void Robot::teleop() {
  driveTeleop();
  goalClamp();
  wait(50, msec);
}




// dist in inches
float Robot::distanceToDegrees(float dist) {
  return dist * 360 / 2 / M_PI / (4 / 2) * 15 / 14; // 4 in diameter wheels
}

void Robot::driveStraight(float percent, float dist) {
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();
  // currPos is the current average encoder position, travelDist is the total encoder distance to be traversed, 
  // targetDist is the target encoder position, and currLeft/Right are the current left and right encoder positions
  float currLeft = leftMotorA.position(degrees);
  float currRight = rightMotorA.position(degrees);
  float currPos = (currLeft + currRight) / 2;
  float travelDist = distanceToDegrees(dist);
  float targetDist = currPos + travelDist;
  
  while (currPos < targetDist) {
    setLeftVelocity(dist > 0 ? forward : reverse, 5 + (percent - 5) * ((targetDist - currPos) / travelDist));
    setRightVelocity(dist > 0 ? forward : reverse, 5 + (percent - 5) * ((targetDist - currPos) / travelDist) - ((currRight - currLeft) / travelDist * 10));
    currLeft = leftMotorA.position(degrees);
    currRight = rightMotorA.position(degrees);
    currPos = (currLeft + currRight) / 2;
  }
  leftDrive.stop();
  rightDrive.stop();
}

void Robot::driveTimed(float percent, float driveTime) {
  int milliseconds = vex::timer::system();
  while (vex::timer::system() < milliseconds + driveTime) {
    setLeftVelocity(forward, percent);
    setRightVelocity(forward, percent);
  }
  leftDrive.stop();
  rightDrive.stop();
}

//12.375 in wheelbase
void Robot::turnToAngle(float percent, float turnAngle, bool PID, directionType direction) {

  int targetDist = getTurnAngle(turnAngle);
  
  while (true) {
    if(turnToAngleNonblocking(percent, targetDist, PID, direction)) break;
    wait(100, msec);
  }
  stopLeft();
  stopRight();
}


int Robot::getTurnAngle(float turnAngle) {
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  // currPos is the current average encoder position, travelDist is the total encoder distance to be traversed, 
  // and targetDist is the target encoder position
  float targetDist = fabs(distanceToDegrees(turnAngle / 360 * 2 * M_PI * (15.125 / 2)));

  return targetDist;
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

// delta ranges from -100 (hard left) and 100 (hard right). 0 is straight
void Robot::driveCurved(directionType d, float dist, int delta) {
  int baseSpeed = 100-abs(delta)/2.0;
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

    wait(100, msec);
  }
  //stopLeft();
  //stopRight();
}

float CENTER_X = 157.0;


//void Robot::openClaw() {}
//void Robot::closeClaw() {}

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