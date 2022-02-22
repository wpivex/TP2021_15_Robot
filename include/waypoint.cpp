#ifndef WAYPOINT_CPP
#define WAYPOINT_CPP

#include "vex.h"
#include <string>

class Waypoint{
  public:
    float x; //all distance units in meters
    float y;
    float theta;
    float accel;

    Waypoint(float p_x, float p_y, float p_thtea, float p_accel){
      this->x = p_x;
      this->y = p_y;
      this->theta = p_thtea;
      this->accel = p_accel;
    }

    char* printPoint(){
      char buff[100];
      snprintf(buff, sizeof(buff),"Point (%.02f,%.02f,%.02f) A: %.02f",this->x,this->y,this->theta,this->accel);
      return buff;
    }
};


#endif // PLOTMARKER_H
