#if !defined(MYLIB_CONSTANTS_H)
#define MYLIB_CONSTANTS_H 1

#include "vex.h"

vex::brain Brain;
vex::controller Controller1(vex::controllerType::primary);

struct Goal {
  int id;
  int bright;
  vex::vision::signature sig;
};


const struct Goal YELLOW = {0, 13, vex::vision::signature (1, 1849, 2799, 2324, -3795, -3261, -3528, 2.500, 0)};
const struct Goal RED = {1, 56, vex::vision::signature (1, 5767, 9395, 7581, -685, 1, -342, 3.000, 0)};
const struct Goal BLUE = {2, 67, vex::vision::signature (1, -2675, -1975, -2324, 8191, 14043, 11116, 3.000, 0)};


static const float VISION_CENTER_X = 157.0;

static const float FORWARD_MIN_SPEED = 20; // the robot approaches this speed at the end of going forward
static const float TURN_MIN_SPEED = 8; // the robot approaches this speed at the end of turning

static const int ARM_TIMEOUT = 3000;


static inline float distanceToDegrees(float distInches) {
  return distInches * 360 / 2 / M_PI / (3.25 / 2); // 4 in diameter wheels
}



// return distance in inches if wanting to turn turnAngle degrees
static inline float getTurnAngle(float turnAngle) {

  return fabs(distanceToDegrees(turnAngle / 360 * 2 * M_PI * (15.125 / 2)));
}

// timeout in seconds
static inline bool isTimeout(int startTime, int timeout) {
  return timeout != -1 && vex::timer::system() >= startTime + timeout*1000;
}

// log output to brain display the way you would with printf
template <class ... Args>
static inline void log(const char *format, Args ... args) {

  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print(format, args...);

}

// log output to controller display the way you would with printf
template <class ... Args>
static inline void logController(const char *format, Args ... args) {

  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print(format, args...);

}

#endif