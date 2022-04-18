#pragma once
#include "vex.h"
#include "constants.h"

class Trapezoid {

  public:
  Trapezoid(float targetValue, float maxSpeedP, float minSpeedP, int numRampUpFrames, float slowDownValue, float endSlowValue = 0, float rampMinSpeedP = -1);
  float tick(float currentValue);
  bool isCompleted();
  
  private:
  bool risingEdge;
  bool firstFrame = true;
  int xi = 0;
  int xn;
  float curr, target, maxSpeed, minSpeed, rampMinSpeed, rampUp, slowDown, endSlow;

};