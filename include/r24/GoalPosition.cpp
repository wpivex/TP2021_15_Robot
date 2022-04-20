#pragma once
#include "GoalPosition.h"

void GoalPosition::update(int OX, int CX, int OY, int CY, int W, int H) {

  ox = OX;
  cx = CX;
  oy = OY;
  cy = CY;
  w = W;
  h = H;

  isLinkedThisFrame = true;
  sizeQueue.push(w * h); 
}

GoalPosition::GoalPosition(int ID, int OX, int CX, int OY, int CY, int W, int H): sizeQueue(10) {
  id = ID;

  update(OX, CX, OY, CY, W, H);
  col = vex::color(id*100%255, id*200%255, id*300%255); // cheap way to map each id to some color
}

int GoalPosition::area() {
  return w * h;
}

int GoalPosition::averageArea() {
  return sizeQueue.getAverage();
}

bool GoalPosition::isPersistent() {
  return lifetime >= 5;
} // only counts as stable goal if it has appeared at least 5 frames in a row