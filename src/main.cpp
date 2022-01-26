// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision6              vision        6               
// DigitalInE           digital_in    E               
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "../include/robot.cpp"

// CODE FOR 15" ROBOT

competition Competition;
controller Controller1(controllerType::primary);

Robot fifteen = Robot(&Controller1);

int mainTeleop() {
  while (true) {
    fifteen.teleop();
    wait(100, msec);
  }
  return 0;
}

void userControl(void) { task controlLoop1(mainTeleop); }

void mainAuto(void) { // 1:1.08

  fifteen.setTransmission(false);
  wait(500, msec);

  // Weird Kohmei thing to keep arm low
  fifteen.sixBarFL.spin(reverse, 20, percent);
  fifteen.sixBarFR.spin(reverse, 20, percent);

  // Initialize pneumatics
  fifteen.setFrontClamp(false);
  fifteen.setTransmission(false);

  // Forward to goal
  fifteen.driveStraight(100, 60);
  fifteen.setFrontClamp(true);

  // Lift arm a little before going back to not drag on ground
  //fifteen.sixBarFL.rotateTo(forward, 100, degrees, 100, velocityUnits::pct, false);
  //fifteen.sixBarFR.spinFor(forward, 100, degrees, 100, velocityUnits::pct, false);
  fifteen.sixBarFL.rotateFor(forward, 100, degrees, 100, velocityUnits::pct, false);
  fifteen.sixBarFR.rotateFor(forward, 100, degrees, 100, velocityUnits::pct, false);

  // Switch to torque mode and go back
  fifteen.setTransmission(true);

  fifteen.driveCurved(reverse, 140, 13);

  // Final segment of backtracking to wall alignment
  fifteen.driveCurvedTimed(reverse, 13, 50, 1000);

  // Forward for clearance and turn 90 degrees to platform
  fifteen.driveStraight(30, 20);
  fifteen.turnToAngle(30, 220, true, reverse);

  // Go to platform and pick up home goal
  fifteen.driveStraight(30, -40);
  fifteen.setBackClamp(true);

  // Use inertial sensor to climb to center of platform 
}

int tetherAuto(void) { return 0; }

void autonomous() { thread auto1(mainAuto); }


int main() {


  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  while (true) {
    wait(100, msec);
  }
}
