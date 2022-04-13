#include "TrapezoidController.h"

/*
A trapezoidal object allows you to approach the targetValue from any starting value.
The direction is dtermined by starting value (from first tick() call)
rampUp is determined by numRampUpFrames, which linearly interpolates from 0 to maxSpeed over the number of frames (so over time, not error)
*/
Trapezoid::Trapezoid(float targetValue, float maxSpeedP, float minSpeedP, int numRampUpFrames, float slowDownValue, float rampMinSpeedP) {

  curr = 0;
  target = targetValue;
  maxSpeed = maxSpeedP;
  minSpeed = minSpeedP;
  xn = numRampUpFrames;
  slowDown = slowDownValue;

  maxSpeed = fmax(minSpeed, maxSpeed);
  
  if (rampMinSpeedP == -1) rampMinSpeed = minSpeedP;
  else rampMinSpeed = rampMinSpeedP;
  
}

float Trapezoid::tick(float currentValue) {

  curr = currentValue;

  if (firstFrame) {
    risingEdge = curr < target; // set trapezoidal direction. When crossed threshold from that direction, isCompleted
    firstFrame = false;
  }

  float delta, speed;
  if (fabs(target - curr) < slowDown && slowDown > 0) delta = fabs(target - curr) / slowDown;
  else delta = 1;

  delta = fmin((xi+1.0) / (xn+1.0), delta); // Apply ramp up. If there is both slowdown and rampUp, pick the smaller one
  speed = minSpeed + (maxSpeed - minSpeed) * delta;

  if (xi < xn) xi++;
  return (risingEdge ? 1 : -1) * speed;
}

bool Trapezoid::isCompleted() {
  if (firstFrame) return false;

  if (risingEdge) return curr >= target;
  else return curr <= target;
}