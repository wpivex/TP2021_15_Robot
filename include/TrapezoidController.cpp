#include "TrapezoidController.h"

Trapezoid::Trapezoid(float distInches, float maxSpeedP, float minSpeedP, float rampUpFrames, float slowDownInches, float rampMinSpeedP) {

  dist = 0;
  direction = (distInches > 0 ? 1 : -1);
  finalDist = fabs(distInches);
  maxSpeed = maxSpeedP;
  minSpeed = minSpeedP;
  xn = rampUpFrames;
  slowDown = slowDownInches;

  maxSpeed = fmax(minSpeed, maxSpeed);
  
  if (rampMinSpeedP == -1) rampMinSpeed = minSpeedP;
  else rampMinSpeed = rampMinSpeedP;
  
}

float Trapezoid::tick(float currDistance) {

  dist = fabs(currDistance);
  

  float delta;
  if (finalDist - dist < slowDown && slowDown > 0) delta = fabs(finalDist - dist) / slowDown;
  else delta = 1;
  
  float slowDownSpeed = minSpeed + (maxSpeed - minSpeed) * delta;
  float rampUpSpeed = minSpeed + (maxSpeed - minSpeed) * (xi+1.0) / (xn+1.0);
  if (xi < xn) xi++;

  return direction * fmin(slowDownSpeed, rampUpSpeed);
}

bool Trapezoid::isCompleted() {
  return dist >= finalDist;
}