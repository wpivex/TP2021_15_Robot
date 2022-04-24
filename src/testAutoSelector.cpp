#include "AutonSelector/AutonSelector.cpp"

AutonSelector selector;
bool runningFunction = false;

// Blocking auton functions that can be selected to run
int test1() {
  return 0;
}

int test2() {
  return 0;
}

int test3() {
  return 0;
}

int teleop() {
  return 0;
}



void preAuton() {
  // do blocking preauton things here
}

int runAuton() {
  return selector.runSelectedAuton();
}

int autonSelectorTask() {
  while (!runningFunction) {
    selector.tick();
    wait(20, msec);
  }
  return 0;
}

void autonomous() { runningFunction = true; task auto1(runAuton); }
void userControl(void) { runningFunction = true; task controlLoop1(teleop); }

void wouldBeCalledMainFunctionButIsJustForTestingAutonSelector() {

  selector.addAuton(test1, "test 1", BTN::LEFT);
  selector.addAuton(test2, "test 2", BTN::LEFT);
  selector.addAuton(test3, "test 3", BTN::LEFT);

  task runAutonSelector(autonSelectorTask);

  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  while (true) {
    wait(20, msec);
  }

}