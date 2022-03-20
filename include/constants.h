#if !defined(MYLIB_CONSTANTS_H)
#define MYLIB_CONSTANTS_H 1

#include "vex.h"

vex::brain Brain;
vex::controller Controller1(vex::controllerType::primary);
vex::competition Competition;
vex::gps GPS11 = vex::gps(vex::PORT17, 5.9, 3, vex::inches, 90);

typedef struct Point {
  float x;
  float y;
  Point(int X, int Y) {x = X; y = Y;}
} Point;

struct Goal {
  int id;
  int bright;
  vex::vision::signature sig;
};

const float PI = 3.1415;

const float DISTANCE_BETWEEN_WHEELS = 15; // in inches


const struct Goal YELLOW = {0, 13, vex::vision::signature (1, 1849, 2799, 2324, -3795, -3261, -3528, 2.500, 0)};
const struct Goal RED = {1, 56, vex::vision::signature (1, 5767, 9395, 7581, -685, 1, -342, 3.000, 0)};
const struct Goal BLUE = {2, 67, vex::vision::signature (1, -2675, -1975, -2324, 8191, 14043, 11116, 3.000, 0)};

namespace BackLift {
  enum State {DOWN, MID, UP};
}

static const float VISION_CENTER_X = 157.0;
const float MAX_VOLTS = 12.0; // maximum volts for vex motors


static inline float distanceToDegrees(float distInches) {
  return distInches * (5/3.0) * 360 / 2 / M_PI / (3.25 / 2); // 4 in diameter wheels
}

static inline float degreesToDistance(float distDegrees) {
  return distDegrees * (3/5.0) / (360 / 2 / M_PI / (3.25 / 2)); // 4 in diameter wheels
}

static inline float distanceFormula(float dx, float dy) {
  return sqrt(dx*dx + dy*dy);
}

static inline float bound180(float angle) {
  if (angle < -180) angle += 360;
  else if (angle > 180) angle -= 360;
  return angle;
}

// Find the closest angle between two universal angles
static inline float getAngleDiff(float targetAngle, float currentAngle) {
  return bound180(targetAngle - currentAngle);
}

// return distance in inches if wanting to turn turnAngle degrees
static inline float getTurnAngle(float turnAngle) {

  return fabs(distanceToDegrees(turnAngle / 360 * 2 * M_PI * (15.125 / 2)));
}

// timeout in seconds
static inline bool isTimeout(int startTime, float timeout) {
  return timeout != -1 && vex::timer::system() >= startTime + timeout*1000;
}

// log output to controller
template <class ... Args>
static inline void logController(const char *f, Args ... args) {

  Controller1.Screen.clearScreen();
  int row = 1;

  char buffer[200];
  sprintf(buffer, f, args...);

  char* pch = strtok (buffer,"\n");
  while (pch != NULL)
  {
    Controller1.Screen.setCursor(row, 1);
    Controller1.Screen.print(pch);
    pch = strtok (NULL, "\n");
    row++;
  }
}

// log output to brain display the way you would with printf
template <class ... Args>
static inline void log(const char *f, Args ... args) {

  Brain.Screen.clearScreen();
  Brain.Screen.setFont(vex::mono60);
  int row = 1;

  char buffer[200];
  sprintf(buffer, f, args...);

  char* pch = strtok (buffer,"\n");
  while (pch != NULL)
  {
    Brain.Screen.setCursor(row, 1);
    Brain.Screen.print(pch);
    pch = strtok (NULL, "\n");
    row++;
  }
}

#endif