#pragma once
#include "FixedRingQueue.h"

//template <typename T>

RingQueue::RingQueue(int sizeP) {
  capacity = sizeP;
  arr = std::vector<float>(sizeP);
}

// If at capacity, the first element is popped
bool RingQueue::push(float value) {
  if (size < capacity) {
    arr[size] = value;
    size++;
    return false;
  } else {
    arr[firstElement] = value;
    firstElement = (firstElement + 1) % capacity;
    return true;
  }
}

float RingQueue::getAverage() {

  if (size == 0) return 0; 

  float sum = 0;
  for (int i = 0; i < size; i++) sum += arr[i];
  
  return sum / size;
}

float RingQueue::get(int index) {
  return arr[(firstElement + index) % capacity];
}

int RingQueue::getSize() {
  return size;
}
