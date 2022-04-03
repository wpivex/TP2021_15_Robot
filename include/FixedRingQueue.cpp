#pragma once
#include "FixedRingQueue.h"

RingQueue::RingQueue(int sizeP) {
  capacity = sizeP;
  arr = new int[size];
}

// If at capacity, the first element is popped
void RingQueue::push(int value) {

  if (size < capacity) {
    arr[size++] = value;
  } else {
    arr[firstElement] = value;
    firstElement = (firstElement + 1) % capacity;
  }
}

int RingQueue::getAverage() {

  if (size == 0) return 0; 

  int sum = 0;
  for (int i = 0; i < size; i++) sum += arr[i];
  
  return sum / size;
}

RingQueue::~RingQueue() {
  delete[] arr;
}