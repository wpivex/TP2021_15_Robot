#pragma once

#include <vector>


class RingQueue {

  public:
  RingQueue(int sizeP);
  void push(float value); // Push to queue. If at capacity, pop.
  float getAverage();
  float get(int index);
  int getSize();

  private:
  std::vector<float> arr;
  int capacity;
  int size = 0;
  int firstElement = 0;
};