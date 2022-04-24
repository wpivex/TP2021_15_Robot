
#include "robot.h"


Robot::Robot(controller* c) : leftMotorA(0), leftMotorB(0), leftMotorC(0), leftMotorD(0), rightMotorA(0), rightMotorB(0), 
  rightMotorC(0), rightMotorD(0), frontArmL(0), frontArmR(0), backLiftL(0), backLiftR(0), intake(0), camera(0), buttons(c), gyroSensor(PORT16) {

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

  setControllerMapping(EZEQUIEL_MAPPING);
}

void Robot::setControllerMapping(ControllerMapping mapping) {

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


void Robot::driveTeleop() {


  if(false) { // tank
    setLeftVelocity(forward,buttons.axis(Buttons::LEFT_VERTICAL));
    setRightVelocity(forward,buttons.axis(Buttons::RIGHT_VERTICAL));
  } else {
    float drive = cMapping == BRIAN_MAPPING ? buttons.axis(Buttons::RIGHT_VERTICAL) : buttons.axis(Buttons::LEFT_VERTICAL);
    float turn = buttons.axis(Buttons::RIGHT_HORIZONTAL) / 1.0;
    float max = std::max(1.0, std::max(fabs(drive+turn), fabs(drive-turn)));
    setLeftVelocity(forward,100 * (drive+turn)/max);
    setRightVelocity(forward,100 * (drive-turn)/max);
  }
}

// Not a truly blocking function, one second timeout if blocking
void Robot::setBackLift(Buttons::Button b, bool blocking) {

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


void Robot::backLiftTeleop() {
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

void Robot::clawUp() {
  frontClaw.set(false);
}

void Robot::clawDown() {
  frontClaw.set(true);
}

void Robot::backUp() {
  backIsDown = false;
  backClamp.set(true);
  
}

void Robot::backDown() {
  backIsDown = true;
  backClamp.set(false);
  
}

void Robot::moveArmTo(double degr, double speed, bool blocking) {
  frontArmL.rotateTo(degr, degrees, speed, velocityUnits::pct, false);
  frontArmR.rotateTo(degr, degrees, speed, velocityUnits::pct, blocking);
}

void Robot::armTeleop() {

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
  log("done calibration");
}

// return in inches
float Robot::getEncoderDistance() {
  return degreesToDistance((leftMotorA.rotation(deg) + rightMotorA.rotation(deg)) / 2);
}

void Robot::resetEncoderDistance() {
  leftMotorA.resetRotation();
  rightMotorA.resetRotation();
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

// If the robot is known to have a given heading (i.e. from wall align) and the gyro heading is close enough to heading, recalibrate gyro heading
void Robot::possiblyResetGyro(float targetAngle) {

  if (fabs(getAngleDiff(targetAngle, getAngle())) < 10) {
    logController("YES set heading\nfrom:%f\nto:%f", getAngle(), targetAngle);
    gyroSensor.setHeading(targetAngle, degrees);
  } else {
    logController("NO set heading\nfrom:%f\nto:%f", getAngle(), targetAngle);
  }
}

void Robot::getGPSData(float *x, float *y, float *headingAngle, int numSamples) {

  // Get averaged gps value
  float startH = GPS11.heading();
  float sumH = 0;
  float sumX = 0;
  float sumY = 0;
  for (int i = 1; i < numSamples; i++) {
      wait(20, msec);
      sumH += getAngleDiff(GPS11.heading(), startH);
      sumX += GPS11.xPosition(inches);
      sumY += GPS11.yPosition(inches);
  }
  float avgH = GPS11.heading() + sumH / numSamples;
  *headingAngle = fmod(avgH + 270, 360);
  *x = sumX / numSamples;
  *y = sumY /numSamples;

  logController("head gps/curr: %.2f %.2f\nx: %f\ny: %f", *headingAngle, getAngle(), *x, *y);
}

void Robot::goCurve(float distInches, float maxSpeed, float turnPercent, float rampUpInches, float slowDownInches, bool stopAfter, float rampMinSpeed, float slowMinSpeed) {
  float timeout = 5;

  Trapezoid trap(fabs(distInches), maxSpeed, slowMinSpeed, rampUpInches, slowDownInches, rampMinSpeed);

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
bool stopAfter, float rampMinSpeed, float slowDownMinSpeed, float timeout) {

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
    correction = (universalAngle == -1) ? 0 : turnPID.tick(ang);
 
    setLeftVelocity(forward, speed + correction);
    setRightVelocity(forward, speed - correction);

    //log("Target: %f\nActual:%f\nLeft:%f\nRight:%f\n", universalAngle, getAngle(), speed+correction, speed-correction);
    //log("%f", gyroSensor.heading());

    wait(20, msec);
  }
  if (stopAfter) {
    stopLeft();
    stopRight();
  }
  //log("straight done");
}

// Go forward with standard internal encoder wheels for distance, maintain current heading
void Robot::goForward(float distInches, float maxSpeed, float rampUpInches, float slowDownInches, bool stopAfter, float rampMinSpeed, 
float slowDownMinSpeed, float timeout) {
  goForwardU(distInches, maxSpeed, -1, rampUpInches, slowDownInches, stopAfter, rampMinSpeed, slowDownMinSpeed, timeout);
}

// Go at specified direction and approach given x position with PID motion profiling using GPS absolute positioning
void Robot::goToAxis(axisType axis, bool reverseDirection, float finalValue, float maxSpeed, float timeout) {

  float startDist = axis == axisType::xaxis ? getX() : getY();

  Trapezoid trap(finalValue - startDist, maxSpeed, 12, 3, 8);
  PID turnPID(1, 0, 0);
  int startTime = vex::timer::system();
  float h = getAngle(); // maintain current heading

  while (!trap.isCompleted() && !isTimeout(startTime, timeout)) {

    float currDist = axis == axisType::xaxis ? getX() : getY();
    float speed = trap.tick(currDist - startDist) * (reverseDirection ? -1 : 1);
    float correction = turnPID.tick(getAngleDiff(h, getAngle()));

    setLeftVelocity(forward, speed + correction);
    setRightVelocity(forward, speed - correction);

    log("Current: %f \n Final: %f", currDist, finalValue);

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
void Robot::goTurnU(float universalAngleDegrees, bool stopAfter, float timeout, float maxSpeed) {

  PID anglePID(2.5, 0, 0.1, 1.3, 3, 18, maxSpeed);

  float speed;

  log("initing");
  int startTime = vex::timer::system();
  log("about to loop");

  while (!anglePID.isCompleted() && !isTimeout(startTime, timeout)) {

    float ang = getAngleDiff(universalAngleDegrees, getAngle());
    speed = anglePID.tick(ang);

    //log("Turn \nTarget: %f \nCurrent: %f \nDiff: %f\nSpeed: %f \nGPS: %f", universalAngleDegrees, getAngle(), ang, speed, GPS11.heading());
    //log("heading: %f", GPS11.heading());
    log("%f", gyroSensor.heading());

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

void Robot::goFightBackwards() {

  VisualGraph g(-0.1, 2.9, 8, 50);

  setLeftVelocity(reverse, 100);
  setRightVelocity(reverse, 100);
  wait(500, msec);
  //int startTime = vex::timer::system();
  float curr = 3;
  bool display = true;
  while (curr > 0.9) {
    
    curr = (leftMotorA.current() + rightMotorA.current()) / 2;
    g.push(curr);
    if (display) g.display();
    display = !display;
    
    wait(20, msec);
  }
  g.push(curr);
  g.display();
  goForward(-5, 100, 0, 5);

}


void Robot::updateCamera(Goal goal) {
  camera = vision(PORT17, goal.bright, goal.sig);
}

// Go forward until the maximum distance is hit or the timeout is reached
// for indefinite timeout, set to -1
void Robot::goVision(float distInches, float speed, Goal goal, float rampUpInches, float slowDownInches, bool stopAfter, float timeout) {

  Trapezoid trapDist(distInches, speed, 12, rampUpInches, slowDownInches);
  PID pidTurn(25, 0, 0);

  updateCamera(goal);

  int startTime = vex::timer::system();
  leftMotorA.resetPosition();
  rightMotorA.resetPosition();

  // forward until the maximum distance is hit, the timeout is reached, or limitSwitch is turned on
  while (!trapDist.isCompleted() && !isTimeout(startTime, timeout)) {

    camera.takeSnapshot(goal.sig);
    
    float correction = camera.largestObject.exists ? pidTurn.tick((VISION_CENTER_X-camera.largestObject.centerX) / VISION_CENTER_X) : 0;
    float distDegrees = fmin(leftMotorA.rotation(deg), rightMotorA.rotation(deg)); // take smaller of two distances because arcs
    float speed = trapDist.tick(degreesToDistance(distDegrees));

    setLeftVelocity(forward, speed - correction);
    setRightVelocity(forward, speed + correction);

    wait(20, msec);
  }
  if (stopAfter) {
    stopLeft();
    stopRight();
  }
  
}

// Trapezoidal motion profiling
// Will use gyro sensor
// distAlongCirc is positive if forward, negative if reverse
// curveDirection is true for right, false for left
void Robot::goRadiusCurve(float radius, float numRotations, bool curveDirection, float maxSpeed, float rampUp, float slowDown, bool stopAfter, float timeout) {

  float distAlongCircum = numRotations * 2 * M_PI;

  Trapezoid trap(distAlongCircum, maxSpeed, 12,rampUp,slowDown);
  //      kp, kd, ki
  PID anglepid(0.025, 0, 0); //definitely no kd imo


  int startTime = vex::timer::system();
  resetEncoderDistance();

  // Repeat until either arrived at target or timed out
  while (!trap.isCompleted() && !isTimeout(startTime, timeout)) {

    float distSoFar =  getEncoderDistance();

    float v_avg = trap.tick(distSoFar); 
    float v_ratio = fabs((radius+DISTANCE_BETWEEN_WHEELS)/(radius-DISTANCE_BETWEEN_WHEELS));

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

// Align to the goal of specified color with PID
void Robot::goAlignVision(Goal goal, float timeout) {

  updateCamera(goal);

  int startTime = vex::timer::system();
  float speed = 0;

  PID vTurnPID(40, 0, 1, 0.05, 3, 12);

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

GoalPosition* getGoalFromID(std::vector<GoalPosition> &goals, int targetID) {
  for (int i = 0; i < goals.size(); i++) {
    if (goals[i].id == targetID) return &goals[i];
  }
  return nullptr;
}

// Track each yellow goal across time, and label each with an id
// targetID only for visual purposes to highlight target goal, if targetID != -1
void Robot::trackObjectsForCurrentFrame(std::vector<GoalPosition> &goals, int targetID) {


  static int nextAvailableID = 0;

  // Reset goal linking status to false
  for (int i = 0; i < goals.size(); i++) {
    goals[i].isLinkedThisFrame = false;
  }

  
  
  // Go through all of the current frame's detected objects and link with persistent goals vector
  for (int i = 0; i < camera.objectCount; i++) {
    vision::object o = camera.objects[i];

    if (oArea(o) < 160) continue; 

    // Find the matching goal from the previous frame
    int closestDist = 60; // maximum distance from previous frame location that can link
    int closestIndex = -1;
    for (int j = 0; j < goals.size(); j++) {
      int dist = (int) distanceFormula(o.centerX - goals[j].cx, o.centerY - goals[j].cy);
      if (!goals[j].isLinkedThisFrame && dist < closestDist) {
        closestDist = dist;
        closestIndex = j;
      }
    }

    if (closestIndex == -1) {
      // No matching goal, create new GoalPosition
      goals.push_back(GoalPosition(nextAvailableID++, o.originX, o.centerX, o.originY, o.centerY, o.width, o.height));
    } else {
      // If found link to persistent goal, update the goal's location
      goals[closestIndex].update(o.originX, o.centerX, o.originY, o.centerY, o.width, o.height);
    }
  }


  // Now that we've linked all the available goals, delete any that didn't show up in this frame
  // For those who showed up, increment lifetime.
  // Also, draw a helpful UI
  for (int i = 0; i < goals.size(); i++) {
    if (!goals[i].isLinkedThisFrame) {
      if (goals[i].unlinkedTime > 20) {
        std::swap(goals[i], goals.back());
        goals.pop_back();
        i--;
      } else {
        goals[i].unlinkedTime++;
      }
      
    } else {
      goals[i].lifetime++;        
    }
  }

  for (int i = 0; i < goals.size(); i++) {
    if (goals[i].isPersistent()) {
      color c = (targetID == -1) ? goals[i].col : (goals[i].id == targetID ? green : red); // show red/green if in strafe phase, otherwise display mapped color
        Brain.Screen.setFillColor(c);
        Brain.Screen.drawRectangle(goals[i].cx, goals[i].cy, goals[i].w, goals[i].h);
        Brain.Screen.printAt(50, 70+i*20, "a %d %d", goals[i].id, goals[i].averageArea());
    }
  }

  // goal UI
  Brain.Screen.setFillColor(transparent);
  Brain.Screen.printAt(50, 50, "size %d %d %d", goals.size(), nextAvailableID, camera.objectCount);

}

// search through goals list to find persistent goal with size > 1200, and on the right side of screen.
// return -1 if does not exist
int Robot::findGoalID(std::vector<GoalPosition> &goals) {

  // Find the left-most goal

  int leftmostValidGoalIndex = -1;
  for (int i = 0; i < goals.size(); i++) {

    if (!goals[i].isPersistent()) continue;
    //if (goals[i].cx < VISION_CENTER_X) continue; // disregard goals to the left of robot
    if (goals[i].averageArea() < 1200) continue;

    // Since met all pre-conditions, goal is valid. Check if the left-most one.
    if (leftmostValidGoalIndex == -1 || goals[i].cx < goals[leftmostValidGoalIndex].cx) {
        leftmostValidGoalIndex = i;
    }
  }
  return (leftmostValidGoalIndex == -1) ? leftmostValidGoalIndex : goals[leftmostValidGoalIndex].id;
}


// return area of object when arrived
int Robot::detectionAndStrafePhase(float *horizonalDistance, int matchStartTime) {

  static const float MAX_TRAVEL_DISTANCE = 80;

  std::vector<GoalPosition> goals;

  resetEncoderDistance();
  float rampUpInches = 3;

  float ANGLE = 270;
  float MIN_SPEED = 15;
  float COAST_SPEED = 65;

  Goal g = YELLOW;
  int targetID = -1;

  PID strafePID(0.8, 0, 0.01, 5, 5, 14, 50);
  PID anglePID(1, 0, 0);
  float speed, offset, ang, correction;
  int area = -1;

  int pt = 0;

  while (targetID == -1 || !strafePID.isCompleted()) {

    pt = (pt + 1) % 10;
    if (pt == 0) logController("Dist: %.1f", *horizonalDistance);

    if (isTimeout(matchStartTime, 43) || (*horizonalDistance - getEncoderDistance()) > MAX_TRAVEL_DISTANCE) {
      area = -1;
      break;
    };

    // Initial ramp up for idle
    speed = COAST_SPEED; // if no target goal detected, this is default speed
    float dist = fabs(getEncoderDistance());
    if (dist < rampUpInches) speed = MIN_SPEED + (COAST_SPEED - MIN_SPEED) * (dist / rampUpInches);

    offset = -1;

    camera.takeSnapshot(g.sig);
    Brain.Screen.clearScreen();
    
    trackObjectsForCurrentFrame(goals, targetID);

    if (targetID == -1) {
      targetID = findGoalID(goals);
      if (targetID != -1) clawUp(); // New goal detected, so on first frame of starting to get new goal, drop current yellow goal
    }

    // if there's a target goal, make sure it still exists
    if (targetID != -1) {
      GoalPosition *goal = getGoalFromID(goals, targetID);
      if (goal != nullptr) {
        // Perform strafe towards goal
        offset = goal->cx - VISION_CENTER_X;
        area = goal->averageArea();
        speed = strafePID.tick(offset);

      } else {
        // Tracked goal was lost, abort target and find new
        targetID = -1;
      }
    }

    ang = getAngleDiff(ANGLE, getAngle());
    correction = anglePID.tick(ang);

    setLeftVelocity(forward, -speed + correction);
    setRightVelocity(forward, -speed - correction);

    Brain.Screen.printAt(20, 20, "Speed: %.1f Offset: %f", -speed, offset);
    
    Brain.Screen.render();
    wait(20, msec);

  }

  Brain.Screen.printAt(20, 20, "Speed: 0");

  stopLeft();
  stopRight();

  *horizonalDistance = -getEncoderDistance() + *horizonalDistance; // increment horizontal distance throughout AI

  return area;
}

// Get distance to goal from area of goal using empirical formula: https://www.desmos.com/calculator/pvbkwhu5lc
float Robot::getDistanceFromArea(int area) {
  if (area > 6000) return 24;
  else if (area > 2200) return 36 - 12.0 * (area - 2200.0) / (6000.0 - 2200);
  else if (area > 1200) return 47 - 11.0 * (area - 1200.0) / (2200.0 - 1200);
  else return 47;
}

/* Initial box rush, grab left yellow goal with front clamp, go back and wall align. Then wall align with left wall forwards.
Back up, grab alliance goal with 1dof, do match loads. Wall align with side, drop yellow goal, then curve to strafe position.

At the start of AI, 1dof is holding alliance goal with rings, front claw is free.
This AI consists of a cyclic four-state machine:

1. Detection phase. Keep moving forward until any goal is seen. Ignore any goals to the left of the centerline
A goal is seen when an object with a size > 1000 is detected. Set target to its id.
2. Strafing phase. Forward PID until perpendicular to target. 
3. Attack phase. Turn 90 and go towards goal until limit switch is activated or hard limit (to avoid crossing line) is reached.
Grab yellow goal with front claw. Intake the whole time. Then, back up and align with blue goal with side camera
4. Undocking phase. Drop the yellow goal behind, and then turn so that 1dof faces other alliance goal again to reset. Go back to step 1.

Keep repeating until timer threshold. */
void Robot::runAI(int matchStartTime) {

  Goal g = YELLOW;
  updateCamera(g);
  clawUp();
  Brain.Screen.setFont(mono20);

  
  //moveArmTo(-20, 50, false);

  float hDist = 0;
  //float initialForward = 11;
  
  while (true) {

    // Detection phase
    moveArmTo(200, 100, false);
    int area = detectionAndStrafePhase(&hDist, matchStartTime);
    if (area == -1) break; // exit if timeout or exceed x value
    int dist = getDistanceFromArea(area);
    logController("Area: %d\nDist: %d", area, dist);
    startIntake();
    goTurnU(0); // point to goal

    // Attack phase
    goForwardU(dist * 2.0 / 3.0, 40, 0, 2, 3);
    moveArmTo(-20, 100, true);
    stopIntake();
    goForwardU(dist / 3.0, 85, 0, 7, 12);
    clawDown();
    moveArmTo(100, 100, false);
    wait(100, msec);
    
    goForwardU(-dist, 85, 0, 7, 12);
    goTurnU(270, true, 2);
    moveArmTo(-20, 50, false);
  }
  logController("timer done");

  // Go to final horizontal distance
  goForwardU(hDist - 35, 70, 270, 5, 10);
  goTurnU(0);

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