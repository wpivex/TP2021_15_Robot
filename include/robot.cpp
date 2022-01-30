#include "../include/robot.h"

// Motor ports Left: 1R, 2F, 3F,  20T Right: 12R, 11F, 13F
// gear ratio is 60/36
Robot::Robot(controller* c) : leftMotorA(0), leftMotorB(0), leftMotorC(0), leftMotorD(0), rightMotorA(0), rightMotorB(0), 
  rightMotorC(0), rightMotorD(0), sixBarFL(0), sixBarFR(0), sixBarBL(0), sixBarBR(0), gyroSensor(PORT6) {
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

void Robot::handleSixBarMechanism(motor* l, motor* r, controller::button* up, controller::button* down) {

  float MOTOR_SPEED = 100;

  if (up->pressing()) {
    // arm go up
    l->spin(forward, MOTOR_SPEED, pct);
    r->spin(forward, MOTOR_SPEED, pct);
  } else if (down->pressing()) {
    // arm go down
    l->spin(reverse, MOTOR_SPEED, pct);
    r->spin(reverse, MOTOR_SPEED, pct);
  } else {
    l->stop();
    r->stop();
  }
}

void Robot::clampArmsDown() {

  // Weird Kohmei thing to keep arms low
  sixBarFL.spin(reverse, 20, percent);
  sixBarFR.spin(reverse, 20, percent);
  sixBarBL.spin(reverse, 20, percent);
  sixBarBR.spin(reverse, 20, percent);
}

void Robot::waitGyroCallibrate() {
  int i = 0;
  while (gyroSensor.isCalibrating()) {
    wait(20, msec);
    i++;
  }
}

void Robot::armMovementVCAT() {


}

template<typename Functor>
void Robot::platformAction(Functor condition, double speed) {

  double YAW_CONSTANT = 1;
  double left, right;

  while (true) {

    double pitch = gyroSensor.pitch();
    double yaw = gyroSensor.yaw();

    // Exit condition
    if (condition(pitch)) break;

    left = speed - yaw * YAW_CONSTANT;
    right = speed + yaw * YAW_CONSTANT;

    setLeftVelocity(forward, left);
    setRightVelocity(forward, right);
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("%f,%f", left, right);

    wait(20, msec);
  }
}

// Use inertial sensor for proportional control in both yaw and pitch
void Robot::balancePlatform() {

  double START_SPEED = 25;
  double REGULAR_SPEED = 20;
  double margin = 5;

  platformAction([margin] (double pitch) {return pitch < margin;}, -START_SPEED);
  platformAction([] (double pitch) {return pitch < 0;}, REGULAR_SPEED);
  platformAction([] (double pitch) {return pitch > 0;}, REGULAR_SPEED);

  stopLeft();
  stopRight();

}

void Robot::sixBarTeleop() {
  // front
  handleSixBarMechanism(&sixBarFL, &sixBarFR, &robotController->ButtonL1, &robotController->ButtonL2);
  // back
  handleSixBarMechanism(&sixBarBL, &sixBarBR, &robotController->ButtonR1, &robotController->ButtonR2);
}

// transmission
void Robot::pneumaticsTeleop() {

    if (robotController->ButtonLeft.pressing() || robotController->ButtonDown.pressing()) {
        // torque mode
        drivePistonRight.set(true);
        drivePistonLeft.set(true);

    } else if (robotController->ButtonRight.pressing() || robotController->ButtonUp.pressing()) {
        // fast mode
        drivePistonRight.set(false);
        drivePistonLeft.set(false);
    }
        
}
  
void Robot::clawTeleop() {

  //back
  if (robotController->ButtonY.pressing()) {
    backClaw.set(true);
  } else if (robotController->ButtonX.pressing()) {
    backClaw.set(false);
  }

  //front
  if (robotController->ButtonA.pressing()) {
    frontClaw.set(true);
  } else if (robotController->ButtonB.pressing()) {
    frontClaw.set(false);
  }

}

// Run every tick
void Robot::teleop() {
  driveTeleop();
  sixBarTeleop();
  clawTeleop();
  pneumaticsTeleop();
  wait(20, msec);
}

// dist in inches
float Robot::distanceToDegrees(float dist) {
  return dist * 360 / 2 / M_PI / (3.25 / 2); // 4 in diameter wheels
}

void Robot::driveStraight(float percent, float dist) {
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();
  // currPos is the current average encoder position, travelDist is the total encoder distance to be traversed, 
  // targetDist is the target encoder position, and currLeft/Right are the current left and right encoder positions
  float currLeft = 0;
  float currRight = 0;
  float currPos = (currLeft + currRight) / 2;
  float travelDist = fabs(distanceToDegrees(dist));
  
  while (currPos < travelDist) {
    setLeftVelocity(dist > 0 ? forward : reverse, 5 + (percent - 5) * 1.5 * ((travelDist - currPos) / travelDist));
    setRightVelocity(dist > 0 ? forward : reverse, 5 + (percent - 5) * 1.5 * ((travelDist - currPos) / travelDist));
    currLeft = leftMotorA.position(degrees);
    currRight = rightMotorA.position(degrees);
    currPos = fabs((currLeft + currRight) / 2);
    wait(20, msec);
  }
  stopLeft();
  stopRight();
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