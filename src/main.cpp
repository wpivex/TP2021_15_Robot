// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision6              vision        6               
// DigitalInE           digital_in    E               
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "robot.cpp"
#include <string>
#include <sstream>

// CODE FOR 15" ROBOT


Robot fifteen = Robot(&Controller1);

int mainTeleop() {
  //fifteen.setTransmission(true);
  while (true) {
    fifteen.teleop();
    wait(20, msec);
  }
  return 0;
}


int armStartup() {

  fifteen.frontArmL.spin(forward, 100, percentUnits::pct);
  fifteen.frontArmR.spin(forward, 100, percentUnits::pct);
  task::sleep(200);
  fifteen.frontArmL.spin(reverse, 100, percentUnits::pct);
  fifteen.frontArmR.spin(reverse, 100, percentUnits::pct);
  task::sleep(350);
  fifteen.frontArmL.stop();
  fifteen.frontArmR.stop();
  fifteen.frontArmL.resetRotation();
  fifteen.frontArmL.resetRotation();

  return 0;
}

int worldSkills() {

  int autonStart = vex::timer::system();

  float lowArmAngle = -20;
  float highArmAngle = 680;

  fifteen.clawUp();
  fifteen.backLiftL.resetRotation();
  fifteen.backLiftR.resetRotation();
  fifteen.frontArmL.resetRotation();
  fifteen.frontArmR.resetRotation();

  // grab home goal
  fifteen.moveArmTo(highArmAngle, 100, false);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, true);
  fifteen.goForwardU(-10.5, 40, 270, 3, 5);
  fifteen.startIntake();
  wait(200, msec);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);
  fifteen.goForwardU(23, 30, 270, 2, 5, true, 20, 10, 3);
  fifteen.goForwardU(-19, 35, 270, 2, 5, true, 20, 10, 3); // go three passes to pick up rings
  fifteen.goForwardU(19, 30, 270, 2, 5);

  // Go to blue
  fifteen.goCurve(-28, 85, 0.33, 3, 4, false);
  fifteen.stopIntake();
  float ang = 88;
  fifteen.moveArmTo(lowArmAngle, 100, false);
  fifteen.goTurnU(ang);
  fifteen.goForwardU(76, 95, ang, 0, 10, false, 20, 40);
  fifteen.goForwardU(10, 40, ang, 0, 3);
  fifteen.clawDown(); // clamp blue

  // Turn to blue zone
  fifteen.goTurnU(45);
  fifteen.moveArmTo(highArmAngle, 100, false);
  fifteen.goCurve(10, 100, -0.4, 3, 0, false);

  // Head to blue platform with curvy path
  fifteen.goForwardU(40, 95, 0, 0, 0, false);
  fifteen.goCurve(20, 100, -0.35, 0, 0, false);
  fifteen.goForwardU(48, 95, 270, 0, 15);
  fifteen.goTurnU(0);

  // Do sweep maneuver
  fifteen.moveArmTo(450, 50);
  fifteen.goTurnU(330, true, 5, true); // "sweep" the goal left
  fifteen.moveArmTo(highArmAngle, 100, false);
  fifteen.goTurnU(0);
  fifteen.moveArmTo(highArmAngle, 100, true); // finish arm movement
  fifteen.goForwardU(5, 40, 0, 1, 2, true, 20, 10, 2);
  fifteen.clawUp();
  fifteen.stopIntake();
  wait(100, msec);

  // Swap goal from back to front
  fifteen.goForwardU(-3, 30, 0, 0.5, 0.5);
  float angle = 330;
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, false);
  fifteen.goTurnU(angle);
  fifteen.moveArmTo(lowArmAngle, 100, false);
  fifteen.goForwardU(-6, 60, angle, 1, 3);
  fifteen.goForwardU(7, 60, angle, 1, 3);
  fifteen.setBackLift(fifteen.BACK_LIFT_UP, false);

  // Grab dropped goal
  fifteen.goTurnU(angle - 180);
  fifteen.goForwardU(10, 50, angle - 180, 1, 0, false);
  fifteen.clawDown(); // slam into goal and claw down so momentum will lessen chance of rings blocking claw
  fifteen.moveArmTo(highArmAngle, 100, false);
  fifteen.goForwardU(3, 50, angle - 180, 0, 3);

  // Elevate other goal
  fifteen.goCurve(10, 100, 0.45, 5, 0, false);
  fifteen.goTurnU(0);
  fifteen.clawUp();
  fifteen.goForwardU(-5, 40, 0, 1, 2);

  // Get to red
  fifteen.goTurnU(90);
  fifteen.stopIntake();
  fifteen.moveArmTo(lowArmAngle, 100, false);
  fifteen.goForwardU(48, 95, 90, 5, 15);

  // Get red with 1dof
  angle = 160;
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, true);
  fifteen.goTurnU(angle);
  fifteen.goForwardU(-15, 50, angle, 2, 5, true, 20, 10, 3);
  fifteen.setBackLift(fifteen.BACK_LIFT_SLIGHT, true);
  fifteen.goForwardU(18, 50, angle, 1, 3);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);
  fifteen.goTurnU(200);

  // Ge yellow
  fifteen.goVision(35, 60, YELLOW, 2, 10);
  fifteen.clawDown();
  fifteen.goTurnU(200);
  fifteen.moveArmTo(highArmAngle, 100, false);
  fifteen.goForwardU(-12, 60, 200, 2, 5);
  fifteen.goTurnU(180);

  // Localize robot position with GPS to determine the distances for the rest of the run
  float *x, *y, *headingAngle;
  fifteen.getGPSData(x, y, headingAngle);
  fifteen.gyroSensor.setHeading(*headingAngle, degrees);
  fifteen.goTurnU(180); // redo angle to 180 after gps localization

  // Use known current position to get to y = 0
  fifteen.goForwardU(75, 60, 180, 3, 12, false, 20, 30); // get field then match rings
  fifteen.goForwardTimed(1.5, 30); // wall align
  fifteen.possiblyResetGyro(180);
  fifteen.goForwardU(-26, 80, 180, 3, 10);
  fifteen.goTurnU(270);
  fifteen.goForwardU(0 - *y, 80, 270, 3, 20);

  return 0;

}

int testForward() {
  fifteen.goForward(100, 80, 5, 15);
  return 0;
}

/*

int worldsAuton() {

  int autonStart = vex::timer::system();

  float lowArmAngle = -20;
  float highArmAngle = 680;

  fifteen.clawUp();
  fifteen.backLiftL.resetRotation();
  fifteen.backLiftR.resetRotation();
  fifteen.frontArmL.resetRotation();
  fifteen.frontArmR.resetRotation();

  // go forward and grab goal
  // go backwards fighting

  // Relocalize with wall aligns
  fifteen.goForwardTimed(2, 30); // wall align back wall
  fifteen.moveArmTo(highArmAngle, 100, false);
  fifteen.goForwardU(4, 40, 0, 1, 1);
  fifteen.goTurnU(270);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, false);
  fifteen.goForwardTimed(3, 40);

  // Get blue goal
  fifteen.goForwardU(32, 60, 270, 2, 5);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);

  // do match load rings
  fifteen.goForwardU(23, 30, 270, 2, 5, true, 20, 10, 3);
  fifteen.goForwardU(-19, 35, 270, 2, 5, true, 20, 10, 3); // go three passes to pick up rings
  fifteen.goForwardU(19, 30, 270, 2, 5);

  // Get to strafe position
  fifteen.goCurve(-28, 50, 0.33, 3, 4, false);
  fifteen.goTurnU(90);

  // run AI until 30 second mark


  return 0;
}
*/
void testDisplay() {

  VisualGraph g(-0.1, 100, 8, 100,4);

  for(int i = 0; i < 10000; i++) {
    
    float f1 = i/2*sin(i)+i/2;
    float f2 = std::rand()%50;
    float f3 = 5*floor(i/5.0);
    float f4 = i+5;
    g.push(f1,0);
    g.push(f2,1);
    g.push(f3,2);
    g.push(f4,3);
    g.display();
    
    wait(20, msec);
  }
  g.display();


}



int autonAI() {

  int matchStartTime = timer::system();

  float lowArmAngle = -20;
  float highArmAngle = 680;

  fifteen.setBrakeType(hold);

  // Initial go rush
  fifteen.clawUp();
  fifteen.goForward(43, 100, 4, 6, false, 20, 50);
  fifteen.goForward(3, 50, 0, 2, false, -1, 40);
  fifteen.clawDown(); // start claw down motion early
  fifteen.goForward(3, 40, 0, 3, true);
  fifteen.goFightBackwards();

  // Get back to wall align but avoiding platform
  fifteen.moveArmTo(200, 100, false);
  fifteen.goTurnU(50);
  fifteen.goForwardU(-13, 70, 50, 3, 7, false);
  fifteen.goTurnU(0);
  fifteen.goForwardU(-11, 70, 0, 3, 1, false, 20, 35);
  fifteen.moveArmTo(highArmAngle, 100, false);
  fifteen.goForwardTimed(1.5, -35); // wall align back
  //fifteen.gyroSensor.setHeading(0, deg);

  // Align with left wall
  fifteen.goForwardU(2, 30, 0, 0.5, 2);
  wait(150, msec);
  fifteen.goTurnU(270);
  fifteen.goForwardU(15, 70, 270, 3, 5, false, 20, 35);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, false);
  fifteen.goForwardTimed(1.0, 35);

  // Get alliance goal
  
  fifteen.goForwardU(-26, 70, 270, 5, 5, false, 20, 40);
  fifteen.goForwardU(-9, 40, 270, 0, 4, true);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);

  // do match load rings
  fifteen.startIntake();
  fifteen.goForwardU(23, 30, 270, 2, 5, true, 20, 10, 3);
  fifteen.goForwardU(-17, 35, 270, 2, 5, true, 20, 10, 2.5); // go three passes to pick up rings
  fifteen.goForwardU(16, 30, 270, 2, 0, false);
  fifteen.goForwardTimed(0.7, 30);

  // Get into AI strafe position
  fifteen.goCurve(-10, 50, 0.365, 3, 0, false);
  fifteen.clawUp();
  fifteen.moveArmTo(200, 100, false);
  fifteen.goCurve(-15.5, 50, 0.365, 0, 0, false);
  fifteen.goForward(-4, 50, 0, 3, true);
  fifteen.goTurnU(270);

  fifteen.runAI(matchStartTime);

  return 0;
}


void autonomous() { fifteen.setBrakeType(hold); task auto1(autonAI); }
//void autonomous() { thread auto1(mainAuto); }

void userControl(void) { fifteen.setBrakeType(coast); task controlLoop1(mainTeleop); }
//void userControl(void) { task controlLoop1(logDistance); }


int main() {

  
  // fifteen.clawDown();
  // wait(500, msec);
  // fifteen.gyroSensor.calibrate();
  // fifteen.waitGyroCallibrate();

  // Competition.bStopAllTasksBetweenModes = true;

  // fifteen.resetEncoderDistance();
  
  // Competition.autonomous(autonomous);
  // Competition.drivercontrol(userControl);

  //platformClimb2();

  testDisplay();

  while (true) {
    wait(20, msec);
  }

}