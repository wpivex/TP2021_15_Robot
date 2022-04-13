#pragma once
#include "FixedRingQueue.cpp"
#include "constants.h"
#include <vector>

using namespace vex;

class VisualGraph {

  public:
  VisualGraph(float minY, float maxY, int numMarkersY, int sizeRingQueue, int numFunctions = 1);
  void push(float dataPoint, int func = 1);
  void display();
  int *a;

  private:
  float minY, maxY;
  int numMarkersX, numMarkersY, numX;
  std::vector<RingQueue*> data;
  int valueToY(float value);
  color lineColors[5];
  // void displayFunction(std::function<int()> func, vex::color col = black);

};