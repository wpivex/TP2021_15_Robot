#pragma once

#include <vector>

class RingQueue {

  public:
  RingQueue(int sizeP);
  void push(int value); // Push to queue. If at capacity, pop.
  int getAverage();

  private:
  std::vector<int> arr;
  int capacity;
  int size = 0;
  int firstElement = 0;
};