#include "TrapezoidController.h"

Trapezoid::Trapezoid(float distInches, float maxSpeedP, float minSpeedP, float rampUpInches, float slowDownInches, float rampMinSpeedP) {

  dist = 0;
  direction = (distInches > 0 ? 1 : -1);
  finalDist = fabs(distInches);
  maxSpeed = maxSpeedP;
  minSpeed = minSpeedP;
  rampUp = rampUpInches;
  slowDown = slowDownInches;

  slowDown = fmin(slowDown, finalDist);
  rampUp = fmin(rampUp, finalDist);
  maxSpeed = fmax(minSpeed, maxSpeed);
  
  if (rampMinSpeedP == -1) rampMinSpeed = minSpeedP;
  else rampMinSpeed = rampMinSpeedP;
  
}

float Trapezoid::tick(float currDistance) {

  dist = fabs(currDistance);
  float delta, speed;

  if (dist < rampUp) speed = rampMinSpeed + (maxSpeed - rampMinSpeed) * dist / rampUp;
  else {
    if (finalDist - dist < slowDown) delta = (finalDist - dist) / slowDown;
    else delta = 1;

    speed = minSpeed + (maxSpeed - minSpeed) * delta;
  }
  return direction * speed;
}

bool Trapezoid::isCompleted() {
  return dist >= finalDist;
}