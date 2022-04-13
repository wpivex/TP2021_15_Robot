#if !defined(MYLIB_CONSTANTS_H)
#define MYLIB_CONSTANTS_H 1

#include "vex.h"
#include <string>
#include <queue>

vex::brain Brain;
vex::controller Controller1(vex::controllerType::primary);
vex::competition Competition;


struct Goal {
  int id;
  int bright;
  vex::vision::signature sig;
};


const struct Goal YELLOW = {0, 13, vex::vision::signature (1, 979, 1701, 1340, -4143, -3541, -3842, 6.000, 0)};
const struct Goal RED = {1, 56, vex::vision::signature (1, 5767, 9395, 7581, -685, 1, -342, 3.000, 0)};
const struct Goal BLUE = {2, 67, vex::vision::signature (1, -2675, -1975, -2324, 8191, 14043, 11116, 3.000, 0)};


static const float VISION_CENTER_X = 157.0;
const float MAX_VOLTS = 12.0; // maximum volts for vex motors

float oArea(vex::vision::object o) {
  return o.width * o.height;
}

static inline float distanceFormula(float dx, float dy) {
  return sqrt(dx*dx + dy*dy);
}

// Bound angle to between -180 and 180
static inline float bound180(float angle) {
  if (angle < -180) angle += 360;
  else if (angle > 180) angle -= 360;
  return angle;
}

// Find the closest angle between two universal angles
static inline float getAngleDiff(float targetAngle, float currentAngle) {
  return bound180(targetAngle - currentAngle);
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
  Brain.Screen.setFont(vex::mono20);
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