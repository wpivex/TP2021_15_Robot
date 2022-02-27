#pragma once
#include "vex.h"
#include "constants.h"

class Trapezoid {

  public:
  Trapezoid(float distInches, float maxSpeedP, float minSpeedP, float rampUpInches, float slowDownInches, float rampMinSpeedP = -1);
  float tick(float currDistance);
  bool isCompleted();
  
  private:

  float direction;
  float finalDist, maxSpeed, minSpeed, rampMinSpeed, rampUp, slowDown;
  float dist = 0;

};