#include "AutonSelector/AutonSelector.cpp"

AutonSelector selector;
bool runningFunction = false;

// Blocking auton functions that can be selected to run
int test1() {
  log("one");
  return 0;
}

int test2() {
  log("two");
  return 0;
}

int test3() {
  log("three");
  return 0;
}

int teleop() {
  log("drive %d", Competition.isDriverControl() ? 1 : 0);
  
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

void autonomous() { Brain.Screen.renderDisable(); runningFunction = true; task auto1(runAuton); }
void userControl(void) { Brain.Screen.renderDisable(); runningFunction = true; task controlLoop1(teleop); }

void wouldBeCalledMainFunctionButIsJustForTestingAutonSelector() {

  selector.addAuton(test1, "test 1", BTN::LEFT);
  selector.addAuton(test2, "test 2", BTN::RIGHT);
  selector.addAuton(test3, "test 3", BTN::UP);

  task runAutonSelector(autonSelectorTask);

  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  while (true) {
    wait(20, msec);
    logController("%d", Controller1->Axis1.position());
  }

}