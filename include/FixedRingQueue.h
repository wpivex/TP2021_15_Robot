#pragma once

class RingQueue {

  public:
  RingQueue(int sizeP);
  ~RingQueue();
  void push(int value); // Push to queue. If at capacity, pop.
  int getAverage();

  private:
  int* arr;
  int capacity;
  int size = 0;
  int firstElement = 0;
};