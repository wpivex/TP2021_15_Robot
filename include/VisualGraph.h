#pragma once
#include "FixedRingQueue.cpp"
#include "constants.h"

using namespace vex;

class VisualGraph {

  public:
  VisualGraph(float minY, float maxY, int numMarkersY, int numX);
  void push(float dataPoint);
  void display();

  private:
  float minY, maxY;
  int numMarkersX, numMarkersY, numX;
  RingQueue data;
  int valueToY(float value);

};