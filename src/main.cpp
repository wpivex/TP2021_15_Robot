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
  int startAmount = 180;
  fifteen.sixBarFL.rotateFor(forward, startAmount, degrees, 100, velocityUnits::pct, false);
  fifteen.sixBarFR.rotateFor(forward, startAmount, degrees, 100, velocityUnits::pct, false);

  // Switch to torque mode and go back
  fifteen.setTransmission(true);

  fifteen.driveCurved(reverse, 120, 15);

  // Final segment of backtracking to wall alignment
  fifteen.driveTimed(-40, 3000);

  // Forward for clearance and turn 90 degrees to platform
  fifteen.driveStraight(40, 12);
  wait(300, msec);
  fifteen.turnToAngle(40, 200, true, reverse);
  wait(300, msec);

  // Go to platform and pick up home goal
  fifteen.driveStraight(40, -15);
  fifteen.setBackClamp(true);
  fifteen.driveStraight(20, 15);

  // Lock bridge with wheel and lift both armbars parallel to ground
  fifteen.driveStraight(40, -25);
  int amount = 200;
  int frontDelta = 0;
  fifteen.sixBarFL.rotateFor(forward, amount - startAmount + frontDelta, degrees, 100, velocityUnits::pct, false);
  fifteen.sixBarFR.rotateFor(forward, amount - startAmount + frontDelta, degrees, 100, velocityUnits::pct, false);
  fifteen.sixBarBL.rotateFor(reverse, amount, degrees, 100, velocityUnits::pct, false);
  fifteen.sixBarBR.rotateFor(reverse, amount, degrees, 100, velocityUnits::pct, true);

  // Use inertial sensor to climb to center of platform 
  wait(1000, msec);
  fifteen.driveStraight(40, -30);
}

void testBalancePlatform(void) {

  fifteen.setTransmission(true);

  /*
  // Weird Kohmei thing to keep arm low
  fifteen.setFrontClamp(false);
  fifteen.setBackClamp(false);
  fifteen.sixBarFL.spin(reverse, 20, percent);
  fifteen.sixBarFR.spin(reverse, 20, percent);
  fifteen.sixBarBL.spin(forward, 20, percent);
  fifteen.sixBarBR.spin(forward, 20, percent);
  */

  while (fifteen.gyroSensor.isCalibrating()) {
    wait(100, msec);
  }

  /*wait(5000, msec);

    
  fifteen.setFrontClamp(true);
  fifteen.setBackClamp(true);
  wait(500, msec);

  fifteen.sixBarFL.stop();
  fifteen.sixBarFR.stop();
  fifteen.sixBarBL.stop();
  fifteen.sixBarBR.stop();

  int startAmount = 200;
  fifteen.sixBarFL.rotateFor(forward, startAmount, degrees, 100, velocityUnits::pct, false);
  fifteen.sixBarFR.rotateFor(forward, startAmount, degrees, 100, velocityUnits::pct, true);

  wait(1000, msec);
  */

  fifteen.driveStraight(100, 20);

  fifteen.balancePlatform();


}

int tetherAuto(void) { return 0; }

void autonomous() { thread auto1(testBalancePlatform); }


int main() {

  fifteen.gyroSensor.calibrate();
  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  while (true) {
    wait(100, msec);
  }
}
