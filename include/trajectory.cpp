#include "constants.h"
#include "waypoint.cpp"
#include <vector>

class Trajectory {       // The class
  public :             // Access specifier
  std::vector<Waypoint> keyPoints;
  std::vector<Waypoint> points;
  Trajectory(){
  }
  void printKeyPoints(){
    for(Waypoint point: keyPoints){
	    point.printPoint();
    }
  }

  void printPoints(){
    for(Waypoint point: points){
	    point.printPoint();
    }
  }

  void addKeyPoint(Waypoint newPoint){
    keyPoints.push_back(newPoint);
  }

  void addKeyPoint(float x, float y, float t, float a){
    Waypoint test = Waypoint(x, y, t, a);
    keyPoints.push_back(test);
  }

  void addPoint(float x, float y, float t, float a){
    Waypoint temp = Waypoint(x, y, t, a);
    points.push_back(temp);
  }

  void addPoint(Waypoint newPoint){
    points.push_back(newPoint);
  }

  void interpolatePoints(){
    float minSpace = 0.5; //inches
    if (keyPoints.size() < 2){
      Brain.Screen.clearScreen();
      Brain.Screen.print("Interpolation Failed, KeyPoints vector too short");
      wait(10000,vex::msec);
    }

    points.push_back(keyPoints[0]);

    // Brain.Screen.print("Keypoints vector length: %d",keyPoints.size());
    // Brain.Screen.newLine();   

    for(int i = 1; i < keyPoints.size(); i++){
      float lengthOfSegment = distanceBetween(keyPoints[i-1], keyPoints[i]);
      // Brain.Screen.print("Length of segment: %f",lengthOfSegment);
      // Brain.Screen.newLine();   
      float xDiff = keyPoints[i].x - keyPoints[i-1].x;
      float yDiff = keyPoints[i].y - keyPoints[i-1].y;
      float tDiff = keyPoints[i].theta - keyPoints[i-1].theta;
      float aDiff = keyPoints[i].accel - keyPoints[i-1].accel;
      if (lengthOfSegment > minSpace){
        int numberOfMiddlePoints = floor(((lengthOfSegment / minSpace)-1));
        // Brain.Screen.print("Adding %d midpoints",numberOfMiddlePoints);
        // Brain.Screen.newLine();
        // Brain.Screen.print("xDiff %f / yDiff %f",xDiff, yDiff);
        // Brain.Screen.newLine();
        for(int j = 0; j < numberOfMiddlePoints; j++){
          float newXVal = keyPoints[i-1].x + (j+1)*(xDiff / (numberOfMiddlePoints+1));
          float newYVal = keyPoints[i-1].y + (j+1)*(yDiff / (numberOfMiddlePoints+1));
          float newTVal = keyPoints[i-1].theta + (j+1)*(tDiff / (numberOfMiddlePoints+1));
          float newAVal = keyPoints[i-1].accel + (j+1)*(aDiff / (numberOfMiddlePoints+1));
          addPoint(newXVal, newYVal, newTVal, newAVal);
          // Brain.Screen.print("Adding new point at %f, %f", newXVal, newYVal);
          // Brain.Screen.newLine();
        }
      }
      //alaways add final point
      addPoint(keyPoints[i]);
    }

    // Brain.Screen.print("Interpolated points  vector length: %d",points.size());
    // Brain.Screen.newLine();   

  }

  // Waypoint findLookAheadPoint(int numberOfPointsToLookAhead, float curr_x, float curr_y){
  //   float minimumDistSoFar = 999; //Inches
  //   for(Waypoint point: points){
	//     if (distanceBetween)
  //   }
  // }
};