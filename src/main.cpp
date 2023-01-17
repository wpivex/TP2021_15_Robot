// INCLUDE RIGHT VERSION OF ROBOT (15 OR 24)
// #include "main15.cpp"
#include "main24.cpp"

#include "Utility/VisualGraph.h"
#include <time.h>
#include <stdlib.h>

using namespace vex;

int main() {

  Competition.bStopAllTasksBetweenModes = true;
  //return mainFunc();
  srand(time(NULL));

  VisualGraph g(0, 5, 6, 100);

  while (true) {
    int num = rand() % 5;
    g.push(num);
    g.display();
    wait(20, vex::msec);
  }

}
