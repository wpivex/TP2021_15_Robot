#include "robot24.cpp"
#include "Utility/GoalPosition.cpp"

static float getDistanceFromWidth(int width);

// Search for goal given ID. Return nullptr if not found
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

    if (o.width * o.height < 160) continue; // ignore very small yellow blips

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

// Strafe and lock onto the location of a goal
// return area of object when arrived
static float detectionAndStrafePhase(Robot24 *robot, vision *camera, int matchStartTime) {

  static const float MAX_TRAVEL_DISTANCE = 95;

  std::vector<GoalPosition> goals;

  robot->resetEncoderDistance();
  float rampUpInches = 3;

  float ANGLE = 270;
  float MIN_SPEED = 15;
  float COAST_SPEED = 65;

  Goal g = YELLOW;
  int targetID = -1;

  PID strafePID(0.75, 0, 0.01, 5, 5, 8, 50);
  PID anglePID(1, 0, 0);
  float speed, offset, ang, correction;
  int width = -1;
  int area = -1;

  int pt = 0;
  float AiEndTime = 35; // Give 10 sec for rest of auto
  
  while (targetID == -1 || !strafePID.isCompleted()) {

    pt = (pt + 1) % 10;
    if (pt == 0) logController("Dist: %.1f\nTime: %.1f", 10-robot->absoluteX, (timer::system() - matchStartTime)/1000.0);

    if (isTimeout(matchStartTime, AiEndTime) || 10-robot->absoluteX > MAX_TRAVEL_DISTANCE) {
      if (isTimeout(matchStartTime, AiEndTime)) logController("timeout");
      else logController("distance");
      width = -1;
      break;
    };

    // Initial ramp up for idle
    speed = COAST_SPEED; // if no target goal detected, this is default speed
    float dist = fabs(10-robot->absoluteX);
    if (dist < rampUpInches) speed = MIN_SPEED + (COAST_SPEED - MIN_SPEED) * (dist / rampUpInches);

    offset = -1;

    camera->takeSnapshot(g.sig);
    Brain.Screen.clearScreen();
    
    trackObjectsForCurrentFrame(camera, goals, targetID);

    if (targetID == -1) {
      targetID = findGoalID(goals);
      if (targetID != -1) ;//clawUp(); // New goal detected, so on first frame of starting to get new goal, drop current yellow goal
    }

    // if there's a target goal, make sure it still exists
    if (targetID != -1) {
      GoalPosition *goal = getGoalFromID(goals, targetID);
      if (goal != nullptr) {
        // Perform strafe towards goal
        offset = goal->cx - VISION_CENTER_X;
        width = goal->w;
        area = goal->area();
        speed = strafePID.tick(offset);

      } else {
        // Tracked goal was lost, abort target and find new
        targetID = -1;
      }
    }

    ang = getAngleDiff(ANGLE, robot->getAngle());
    correction = anglePID.tick(ang);

    robot->setLeftVelocity(forward, -speed + correction);
    robot->setRightVelocity(forward, -speed - correction);

    Brain.Screen.printAt(20, 20, "Speed: %.1f Offset: %f", -speed, offset);
    
    Brain.Screen.render();
    wait(20, msec);

  }

  Brain.Screen.printAt(20, 20, "Speed: 0");

  robot->stopLeft();
  robot->stopRight();

  return width;
}

// Get distance to goal from area of goal using empirical formula: https://www.desmos.com/calculator/pvbkwhu5lc
static float getDistanceFromArea(int area) {
  if (area > 6000) return 24;
  else if (area > 2200) return 36 - 12.0 * (area - 2200.0) / (6000.0 - 2200);
  else if (area > 1200) return 47 - 11.0 * (area - 1200.0) / (2200.0 - 1200);
  else return 47;
}

// Theoretically better and the mathematically correct way to gauge distance from area
static float getDistanceFromWidth(int width) {

  static const float DEGREES_PER_PIXEL = (63.0 * M_PI / 180.0) / VISION_MAX_X;
  static const float GOAL_WIDTH = 13.5;
  static const float CAMERA_HEIGHT = 14.375;

  float theta = DEGREES_PER_PIXEL*width;
  float hypotenuseDistanceToGoal = GOAL_WIDTH / tan(theta);
  float horizontalDistance = sqrt(pow(hypotenuseDistanceToGoal, 2) - pow(CAMERA_HEIGHT, 2));
  return fmax(0,horizontalDistance - 3.5); // Account for camera not being at the very front of robot
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

Keep repeating until timer threshold, OR reaches the end. */
void runAI(Robot24 *robot, int32_t port, int matchStartTime) {
  robot->setMaxArmTorque(CURRENT::HIGH);
  Goal g = YELLOW;
  vision camera(port, g.bright, g.sig); // Fix port
  //clawUp();
  Brain.Screen.setFont(mono20);

  const float GOAL_DISTANCE_OFFSET = 0;
  const int widthThreshold = 0; 
  
  //moveArmTo(-20, 50, false);

  //float initialForward = 11;
  robot->setArmDegrees(300);
  while (true) {

    // Detection phase
    int width = detectionAndStrafePhase(robot, &camera, matchStartTime);
    if (width == -1) break; // exit if timeout or exceed x value
    float dist = getDistanceFromWidth(width) + GOAL_DISTANCE_OFFSET;
    logController("Width: %d\nDist: %d", width, dist);
    if(width>widthThreshold){
      robot->setArmDegrees(500);
      if(robot->frontSlideSensor){
        // High if empty
        // Grab with back slide (TODO: Fix var name)
        robot->goTurnU(180);
        robot->goVisionUntilSensor(reverse, 36, 100, robot->frontSlideSensor, 0);
      }else if(robot->clawSensor){
        // Grabs with front claw
        
        robot->goTurnU(0);
        robot->goForwardUntilSensor(36, 100, robot->clawSensor, 0);
        
      }else break;

      // Return to line :)
      robot->gotToY(30,100);
      robot->goTurnU(270);
      robot->setArmDegrees(300);
    }
  }
  //logController("timer done");

  // Go to final horizontal distance
  robot->goForwardU(robot->absoluteX + 80, 70, 270, 10, 10);
  robot->goTurnU(0);

}
