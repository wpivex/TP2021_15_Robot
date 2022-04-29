#include "robot15.cpp"
#include "Utility/GoalPosition.cpp"

static GoalPosition* getGoalFromID(std::vector<GoalPosition> &goals, int targetID) {
  for (int i = 0; i < goals.size(); i++) {
    if (goals[i].id == targetID) return &goals[i];
  }
  return nullptr;
}

// Track each yellow goal across time, and label each with an id
// targetID only for visual purposes to highlight target goal, if targetID != -1
static void trackObjectsForCurrentFrame(vision *camera, std::vector<GoalPosition> &goals, int targetID) {


  static int nextAvailableID = 0;

  // Reset goal linking status to false
  for (int i = 0; i < goals.size(); i++) {
    goals[i].isLinkedThisFrame = false;
  }

  // Go through all of the current frame's detected objects and link with persistent goals vector
  for (int i = 0; i < camera->objectCount; i++) {
    vision::object o = camera->objects[i];

    if (o.width * o.height < 160) continue; 

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
  Brain.Screen.printAt(50, 50, "size %d %d %d", goals.size(), nextAvailableID, camera->objectCount);

}

// search through goals list to find persistent goal with size > 1200, and on the right side of screen.
// return -1 if does not exist
static int findGoalID(std::vector<GoalPosition> &goals) {

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
int AI_Direction;
static int detectionAndStrafePhase(Robot15 *robot, vision *camera, float *horizonalDistance, int matchStartTime) {

  static const float MAX_TRAVEL_DISTANCE = 80;

  std::vector<GoalPosition> goals;

  robot->resetEncoderDistance();
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

    if (isTimeout(matchStartTime, 43) || (*horizonalDistance + AI_Direction * robot->getEncoderDistance()) > MAX_TRAVEL_DISTANCE) {
      area = -1;
      break;
    };

    // Initial ramp up for idle
    speed = AI_Direction * COAST_SPEED; // if no target goal detected, this is default speed
    float dist = fabs(robot->getEncoderDistance());
    // if (dist < rampUpInches) speed = MIN_SPEED + (COAST_SPEED - MIN_SPEED) * (dist / rampUpInches);

    offset = -1;

    camera->takeSnapshot(g.sig);
    Brain.Screen.clearScreen();
    
    trackObjectsForCurrentFrame(camera, goals, targetID);

    if (targetID == -1) {
      targetID = findGoalID(goals);
      if (targetID != -1) robot->clawUp(); // New goal detected, so on first frame of starting to get new goal, drop current yellow goal
    }

    // if there's a target goal, make sure it still exists
    if (targetID != -1) {
      GoalPosition *goal = getGoalFromID(goals, targetID);
      if (goal != nullptr) {
        // Perform strafe towards goal
        offset = goal->cx - VISION_CENTER_X;
        area = goal->averageArea();
        speed = -strafePID.tick(offset);

      } else {
        // Tracked goal was lost, abort target and find new
        targetID = -1;
      }
    }

    ang = getAngleDiff(ANGLE, robot->getAngle());
    correction = anglePID.tick(ang);

    robot->setLeftVelocity(forward, speed + correction);
    robot->setRightVelocity(forward, speed - correction);

    Brain.Screen.printAt(20, 20, "Speed: %.1f Offset: %f", -speed, offset);
    
    Brain.Screen.render();
    wait(20, msec);

  }

  Brain.Screen.printAt(20, 20, "Speed: 0");

  robot->stopLeft();
  robot->stopRight();

  *horizonalDistance = AI_Direction * robot->getEncoderDistance() + *horizonalDistance; // increment horizontal distance throughout AI

  return area;
}

// Get distance to goal from area of goal using empirical formula: https://www.desmos.com/calculator/pvbkwhu5lc
static float getDistanceFromArea(int area) {
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
static void runAI(Robot15 *robot, int32_t port, int matchStartTime) {

  Goal g = YELLOW;
  vision camera(port, g.bright, g.sig);

  robot->clawUp();
  Brain.Screen.setFont(mono20);

  
  //moveArmTo(-20, 50, false);

  float hDist = 0;
  //float initialForward = 11;
  
  while (true) {

    // Detection phase
    robot->moveArmTo(200, 100, false);
    int area = detectionAndStrafePhase(robot, &camera, &hDist, matchStartTime);
    if (area == -1) break; // exit if timeout or exceed x value
    int dist = getDistanceFromArea(area);
    logController("Area: %d\nDist: %d", area, dist);
    robot->startIntake();
    robot->goTurnU(0); // point to goal

    // Attack phase
    robot->goForwardU(dist / 2.0, 40, 0, 2, 3);
    robot->moveArmTo(-20, 100, true);
    robot->stopIntake();
    robot->goForwardU(dist / 2.0, 85, 0, 7, 12);
    robot->clawDown();
    robot->moveArmTo(100, 100, false);
    wait(100, msec);
    
    robot->goForwardU(-dist, 85, 0, 7, 12);
    robot->goTurnU(270, true, 2);
    robot->moveArmTo(-20, 50, false);
  }
  logController("timer done");

  // Go to final horizontal distance
  robot->goForwardU(hDist - 35, 70, 270, 5, 10);
  robot->goTurnU(0);

}