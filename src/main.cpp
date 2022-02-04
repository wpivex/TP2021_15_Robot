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
  //fifteen.setTransmission(true);
  while (true) {
    fifteen.teleop();
  }
  return 0;
}

void userControl(void) { task controlLoop1(mainTeleop); }

/*

void mainAuto(void) { // 1:1.08

  fifteen.setTransmission(false);
  
  // Weird Kohmei thing to keep arm low
  fifteen.clampArmsDown();

  // Initialize pneumatics
  fifteen.setFrontClamp(false);

  // Forward to goal
  fifteen.driveStraight(100, 60);
  fifteen.setFrontClamp(true);

  // Switch to torque mode and go back
  fifteen.setTransmission(true);

  // Lift arm a little before going back to not drag on ground
  fifteen.raiseFrontArm(180, 100, false);

  fifteen.driveCurved(reverse, 120, 20);

  // Final segment of backtracking to wall alignment
  fifteen.driveTimed(-40, 1500);

  // Forward for clearance and turn 90 degrees to platform
  fifteen.driveStraight(40, 12);
  wait(300, msec);
  fifteen.turnToAngle(40, 170, true, reverse);
  wait(300, msec);

  // Go to platform and pick up home goal
  fifteen.driveStraight(30, -20);
  fifteen.setBackClamp(true);
  fifteen.driveStraight(30, 40);
}


void simpleSkillsAuto() {
  fifteen.setTransmission(true);
  wait(200, msec);
  fifteen.raiseFrontArm(400, 70, true);
  fifteen.driveStraight(70, 42);
  fifteen.raiseFrontArm(-300, 70, true);
  fifteen.driveStraight(70, -42);
}

*/

/*

void testBalancePlatform(void) {

  fifteen.setTransmission(true);

  // Weird Kohmei thing to keep arm low
  fifteen.setFrontClamp(false);
  fifteen.setBackClamp(false);
  fifteen.clampArmsDown();
  
  fifteen.waitGyroCallibrate();

  wait(5000, msec);

    
  fifteen.setFrontClamp(true);
  fifteen.setBackClamp(true);
  wait(500, msec);

  fifteen.sixBarFL.stop();
  fifteen.sixBarFR.stop();
  fifteen.sixBarBL.stop();
  fifteen.sixBarBR.stop();

  fifteen.sixBarFL.rotateFor(forward, 250, degrees, 100, velocityUnits::pct, false);
  fifteen.sixBarFR.rotateFor(forward, 250, degrees, 100, velocityUnits::pct, false);
  fifteen.sixBarBL.rotateFor(forward, 180, degrees, 100, velocityUnits::pct, false);
  fifteen.sixBarBR.rotateFor(forward, 180, degrees, 100, velocityUnits::pct, true);

  wait(1000, msec);
  

  fifteen.driveStraight(60, 80);

  fifteen.balancePlatform();
}
*/
//void autonomous() { thread auto1(mainAuto); }


int main() {

  fifteen.gyroSensor.calibrate();
  
  // DRIVER SKILLS TRUE, OTHERWISE FALSE
  //fifteen.setTransmission(false);

  //Competition.autonomous(autonomous);
  //Competition.drivercontrol(userControl);

  mainTeleop();

  while (true) {
    wait(100, msec);
  }
}
