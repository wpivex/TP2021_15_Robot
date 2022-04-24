#include "AutonSelector/AutonSelector.cpp"

AutonSelector selector;

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

int runAuton() {
  return selector.runSelectedAuton();
}

void autonomous() { task auto1(runAuton); }

void userControl(void) { task controlLoop1(teleop); }

void testSelector() {

  selector.setAuton(test1, "test 1", BTN::LEFT);
  selector.setAuton(test2, "test 2", BTN::LEFT);
  selector.setAuton(test3, "test 3", BTN::LEFT);

  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  while (true) {
    selector.tick();
    wait(20, msec);
  }

}