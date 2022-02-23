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

    Waypoint robotToWorld(Waypoint robotLocation){
      Eigen::MatrixXd m(2,2);
      m(0,0) = cos(deg2rad(robotLocation.theta));
      m(1,0) = sin(deg2rad(robotLocation.theta));
      m(0,1) = -sin(deg2rad(robotLocation.theta));
      m(1,1) = cos(deg2rad(robotLocation.theta));

      Eigen::VectorXd robotPos(2,1);
      robotPos(0,0) = robotLocation.x;
      robotPos(1,0) = robotLocation.y;

      Eigen::VectorXd v(2,1);
      v(0,0) = this->x;
      v(1,0) = this->y;

      Eigen::MatrixXd ans = m*v + robotPos;

      return Waypoint(ans(0,0),ans(1,0),0,0);
    }

    Waypoint worldToRobot(Waypoint robotLocation){
      Eigen::MatrixXd m(2,2);
      m(0,0) = cos(deg2rad(robotLocation.theta));
      m(1,0) = -sin(deg2rad(robotLocation.theta));
      m(0,1) = sin(deg2rad(robotLocation.theta));
      m(1,1) = cos(deg2rad(robotLocation.theta));

      Eigen::VectorXd robotPos(2,1);
      robotPos(0,0) = robotLocation.x;
      robotPos(1,0) = robotLocation.y;

      Eigen::VectorXd v(2,1);
      v(0,0) = this->x;
      v(1,0) = this->y;

      Eigen::MatrixXd ans = m*(v - robotPos);

      return Waypoint(ans(0,0),ans(1,0),0,0);
    }
};

static inline float distanceBetween(Waypoint pointOne, Waypoint pointTwo) {
    float dx = pointOne.x - pointTwo.x;
    float dy = pointOne.y - pointTwo.y;
    return sqrt(dx * dx + dy * dy);
}

#endif // PLOTMARKER_H
