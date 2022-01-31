#include "vex.h"

static int distanceToDegrees(float dist) {
  return dist * 360 / 2 / M_PI / (3.25 / 2); // 4 in diameter wheels
}

// return distance in inches if wanting to turn turnAngle degrees
static int getTurnAngle(float turnAngle) {

  return fabs(distanceToDegrees(turnAngle / 360 * 2 * M_PI * (15.125 / 2)));

}