#pragma once
#include "Utility/FixedRingQueue.cpp"
#include "vex.h"

class GoalPosition {

  public:
  GoalPosition(int ID, int OX, int CX, int OY, int CY, int W, int H);
  void update(int OX, int CX, int OY, int CY, int W, int H);

  bool isPersistent();
  int area();
  int averageArea();
    
  int ox, cx, oy, cy, w, h;
  int id;
  int unlinkedTime = 0;
  vex::color col;
  bool isLinkedThisFrame = true;
  int lifetime = 0; // number of linked frames since spawn
  RingQueue sizeQueue;
};