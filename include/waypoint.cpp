#ifndef WAYPOINT_CPP
#define WAYPOINT_CPP

#include "constants.h"

class Waypoint{
  public:
    float x; //all distance units inches
    float y;
    float theta; //degrees with CW is positvie (LEFT HAND RULE)
    float accel;

    Waypoint(float p_x, float p_y, float p_thtea, float p_accel){
      this->x = p_x;
      this->y = p_y;
      this->theta = p_thtea;
      this->accel = p_accel;
    }

    void printPoint(){
      Brain.Screen.print("Point (%.02f,%.02f,%.02f) A: %.02f",this->x,this->y,this->theta,this->accel);
      Brain.Screen.newLine();
    }
};

static inline float distanceBetween(Waypoint pointOne, Waypoint pointTwo) {
    float dx = pointOne.x - pointTwo.x;
    float dy = pointOne.y - pointTwo.y;
    return sqrt(dx * dx + dy * dy);
}

#endif // PLOTMARKER_H
