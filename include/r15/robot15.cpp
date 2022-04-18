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
    backLiftL.rotateTo(0, degrees, SPEED, velocityUnits::pct, false);
    backLiftR.rotateTo(0, degrees, SPEED, velocityUnits::pct, false);
  } else if (b == BACK_LIFT_MID) {
    log("mid");
    backLiftL.rotateTo(130, degrees, SPEED, velocityUnits::pct, false);
    backLiftR.rotateTo(130, degrees, SPEED, velocityUnits::pct, false);
  } else if (b == BACK_LIFT_DOWN) {
    log("down");
    backLiftL.rotateTo(360, degrees, 60, velocityUnits::pct, false); // gentler set down
    backLiftR.rotateTo(360, degrees, 60, velocityUnits::pct, false);
    if (blocking) wait(400, msec);
  } else if (b == BACK_LIFT_SLIGHT) {
    backLiftL.rotateTo(260, degrees, SPEED, velocityUnits::pct, false);
    backLiftR.rotateTo(260, degrees, SPEED, velocityUnits::pct, false);
  }

  if (blocking) wait(800, msec);

}


void Robot15::backLiftTeleop() {
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

void Robot15::clawUp() {
  frontClaw.set(false);
}

void Robot15::clawDown() {
  frontClaw.set(true);
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

void Robot15::goForwardU(float distInches, float maxSpeed, float universalAngle, float rampUpFrames, float slowDownInches, 
bool stopAfter, float rampMinSpeed, float slowDownMinSpeed, float timeout) {
  BaseRobot::goForwardU_Abstract(1.0, distInches, maxSpeed, universalAngle, rampUpFrames, slowDownInches, 0, 
    stopAfter, rampMinSpeed, slowDownMinSpeed, timeout);
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

GoalPosition* getGoalFromID(std::vector<GoalPosition> &goals, int targetID) {
  for (int i = 0; i < goals.size(); i++) {
    if (goals[i].id == targetID) return &goals[i];
  }
  return nullptr;
}

// Track each yellow goal across time, and label each with an id
// targetID only for visual purposes to highlight target goal, if targetID != -1
void Robot15::trackObjectsForCurrentFrame(vision *camera, std::vector<GoalPosition> &goals, int targetID) {


  static int nextAvailableID = 0;

  // Reset goal linking status to false
  for (int i = 0; i < goals.size(); i++) {
    goals[i].isLinkedThisFrame = false;
  }

  
  
  // Go through all of the current frame's detected objects and link with persistent goals vector
  for (int i = 0; i < camera->objectCount; i++) {
    vision::object o = camera->objects[i];

    if (oArea(o) < 160) continue; // ignore very small yellow blips

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

  // Draw graphics for this frame
  for (int i = 0; i < goals.size(); i++) {
    if (goals[i].isPersistent()) {
      color c = (targetID == -1) ? goals[i].col : (goals[i].id == targetID ? green : red); // show red/green if in strafe phase, otherwise display mapped color
        Brain.Screen.setFillColor(c);
        Brain.Screen.drawRectangle(goals[i].cx, goals[i].cy, goals[i].w, goals[i].h);
        Brain.Screen.printAt(50, 70+i*20, "a %d %d", goals[i].id, goals[i].averageArea());
    }
  }

  // Print general info for this frame
  Brain.Screen.setFillColor(transparent);
  Brain.Screen.printAt(50, 50, "size %d %d %d", goals.size(), nextAvailableID, camera->objectCount);

}

// search through goals list to find persistent goal with size > 1200, and on the right side of screen.
// return -1 if does not exist
int Robot15::findGoalID(std::vector<GoalPosition> &goals) {

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
int Robot15::detectionAndStrafePhase(vision *camera, float *horizonalDistance, int matchStartTime) {

  static const float MAX_TRAVEL_DISTANCE = 100;

  std::vector<GoalPosition> goals;

  resetEncoderDistance();
  float rampUpInches = 3;

  float ANGLE = 270;
  float MIN_SPEED = 15;
  float COAST_SPEED = 65;

  Goal g = YELLOW;
  int targetID = -1;

  PID strafePID(0.75, 0, 0.01, 5, 5, 8, 50);
  PID anglePID(1, 0, 0);
  float speed, offset, ang, correction;
  int area = -1;

  int pt = 0;

  while (targetID == -1 || !strafePID.isCompleted()) {

    pt = (pt + 1) % 10;
    if (pt == 0) logController("Dist: %.1f\nTime: %.1f", *horizonalDistance - getEncoderDistance(), (timer::system() - matchStartTime)/1000.0);

    if (isTimeout(matchStartTime, 43.5) || (*horizonalDistance - getEncoderDistance()) > MAX_TRAVEL_DISTANCE) {
      if (isTimeout(matchStartTime, 43.5)) logController("timeout");
      else logController("distance");
      area = -1;
      break;
    };

    // Initial ramp up for idle
    speed = COAST_SPEED; // if no target goal detected, this is default speed
    float dist = fabs(getEncoderDistance());
    if (dist < rampUpInches) speed = MIN_SPEED + (COAST_SPEED - MIN_SPEED) * (dist / rampUpInches);

    offset = -1;

    camera->takeSnapshot(g.sig);
    Brain.Screen.clearScreen();
    
    trackObjectsForCurrentFrame(camera, goals, targetID);

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
float Robot15::getDistanceFromArea(int area) {
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
void Robot15::runAI(int matchStartTime) {

  Goal g = YELLOW;
  vision camera(CAMERA_PORT, g.bright, g.sig);
  clawUp();
  Brain.Screen.setFont(mono20);

  
  //moveArmTo(-20, 50, false);

  float hDist = 0;
  //float initialForward = 11;
  
  while (true) {

    // Detection phase
    moveArmTo(200, 100, false);
    int area = detectionAndStrafePhase(&camera, &hDist, matchStartTime);
    if (area == -1) break; // exit if timeout or exceed x value
    int dist = getDistanceFromArea(area);
    //logController("Area: %d\nDist: %d", area, dist);
    startIntake();
    goTurnU(0); // point to goal

    // Attack phase
    goForwardU(dist * 2.0 / 3.0, 40, 0, 10, 3);
    moveArmTo(-20, 100, true);
    stopIntake();
    goForwardU(dist / 3.0, 85, 0, 10, 12);
    clawDown();
    moveArmTo(100, 100, false);
    wait(100, msec);
    
    goForwardU(-dist, 85, 0, 10, 12);
    goTurnU(270, true, 2);
    moveArmTo(-20, 50, false);
  }
  //logController("timer done");

  // Go to final horizontal distance
  goForwardU(hDist - 35, 70, 270, 10, 10);
  goTurnU(0);

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