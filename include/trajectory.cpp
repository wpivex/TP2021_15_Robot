#include "constants.h"
#include <vector>

class Trajectory {       // The class
  public :             // Access specifier
  std::vector<Waypoint> keyPoints;
  std::vector<Waypoint> points;
  Trajectory(){
    keyPoints[0] = Waypoint(0,0,0,0);
  }
  void printKeyPoints(){
    for(Waypoint point: keyPoints){
	    Brain.Screen.print(point.printPoint());
      Brain.Screen.newLine();
    }
  }
  void addPoint(Waypoint newPoint){
    Waypoint test = Waypoint(0,0,0,0);
    keyPoints.push_back(test);
  }

  void interpolatePoints(){
    float miniumSpace = 0.01; //meters
  }
};