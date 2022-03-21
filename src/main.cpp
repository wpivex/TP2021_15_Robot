// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Vision6              vision        6               
// DigitalInE           digital_in    E               
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "robot.cpp"

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
  float startAngle = fifteen.gyroSensor.heading();
  fifteen.moveArmTo(highArmAngle, 100, false);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, true);
  fifteen.goForward(-10.5, 40, 1, 2);
  fifteen.startIntake();
  wait(200, msec);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);
  fifteen.goForwardU(21.5, 22, startAngle, 2, 2);
  fifteen.goForwardU(-15, 35, startAngle, 2, 2.5); // go three passes to pick up rings
  fifteen.goForwardU(15, 22, startAngle, 2, 2);
  fifteen.goForwardU(-8, 35, startAngle, 2, 2.5);
  
  // Wall align
  fifteen.goTurnU(180);
  fifteen.goForwardU(5, 50, 180, 1, 2, false, 20, 30);
  fifteen.goForwardTimed(0.7, 30);
  fifteen.possiblyResetGyro(180);

  // Go to blue
  fifteen.goForwardU(-22.5, 70, 180, 2, 10);
  fifteen.goTurnU(90);
  fifteen.moveArmTo(lowArmAngle, 100, false);
  fifteen.goForwardU(44, 90, 90, 3, 4, false, 20, 50);
  fifteen.goVision(45, 50, BLUE, 2, 3);
  fifteen.clawDown(); // clamp blue
  wait(100, msec);

  // Wall align
  fifteen.goForwardU(-7, 60, 90, 1, 1);
  fifteen.moveArmTo(highArmAngle, 100, false); // start the arm movement
  fifteen.goTurnU(90);
  fifteen.moveArmTo(highArmAngle, 100, true); // finish arm movement
  fifteen.goForwardU(13, 60, 90, 1, 2, false, 20, 30);
  fifteen.goForwardTimed(0.7, 30);
  fifteen.possiblyResetGyro(90);

  // Head to blue zone
  fifteen.goForwardU(-6, 50, 90, 1, 3);
  fifteen.goTurnU(0);
  fifteen.goForwardU(57, 70, 0, 3, 10);

  // Move front blue goal to platform
  fifteen.goTurnU(290);
  fifteen.goForwardU(36, 80, 290, 3, 5, false, 20, 40);
  fifteen.goForwardU(15, 40, 290, 0, 3);

  // Do sweep maneuver
  fifteen.moveArmTo(500, 50);
  fifteen.moveArmTo(highArmAngle, 100);
  fifteen.goTurnU(310);
  fifteen.goForwardU(3, 40, 310, 1, 1);
  fifteen.clawUp();
  wait(100, msec);

  // Swap goal from back to front
  fifteen.goForwardU(-4, 40, 310, 1, 1);
  fifteen.goTurnU(270);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, true);
  fifteen.goForwardU(7, 60, 270, 1, 2);
  fifteen.setBackLift(fifteen.BACK_LIFT_UP, false);
  fifteen.moveArmTo(lowArmAngle, 100, false);
  wait(1000, msec);
  fifteen.goTurnU(90);
  fifteen.goForwardU(10, 40, 90, 1, 1);
  fifteen.clawDown();
  wait(100, msec);

  // Elevate other goal
  fifteen.moveArmTo(highArmAngle, 100, false); // start movement upwards, concurrency
  fifteen.goForwardU(-15, 50, 90, 1, 2);
  fifteen.moveArmTo(highArmAngle, 100, true);
  fifteen.goTurnU(0);
  fifteen.goForwardU(5, 40, 0, 1, 1);
  fifteen.clawUp();
  wait(100, msec);

  // Wall align right side
  fifteen.goForwardU(-5, 40, 0, 1, 1);
  fifteen.goTurnU(270);
  fifteen.moveArmTo(lowArmAngle, 100, false);
  fifteen.goForwardU(-50, 100, 270, 2, 5, false, 20, 30);
  fifteen.goForwardTimed(1, -30);
  fifteen.possiblyResetGyro(270);

  // Get red by first aiming with vision then 180, pick up with 1dof
  fifteen.goCurve(10, 50, 0.3, 2, 5);
  fifteen.goAlignVision(RED, 2);
  fifteen.goVision(5, 30, RED, 1, 1);
  float head = fmod((fifteen.getAngle() + 180),360.0);
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, false);
  fifteen.goTurnU(head);
  fifteen.goForwardU(-21, 40, head, 1, 3, true, 20, 10, 3);
  fifteen.setBackLift(fifteen.BACK_LIFT_SLIGHT, true);
  fifteen.goForwardU(15, 50, head, 1, 3);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);
  fifteen.goTurnU(200);

  // Ge yellow
  fifteen.goVision(35, 50, YELLOW, 1, 5);
  fifteen.clawDown();
  wait(100, msec);
  fifteen.moveArmTo(highArmAngle, 60, false);
  fifteen.goTurnU(180);

  // Get  rings
  fifteen.goCurve(18, 60, -0.3, 2, 5);
  fifteen.goTurnU(180);
  fifteen.startIntake();
  fifteen.goForwardU(72, 95, 180, 3, 5, false, 20, 30);
  fifteen.goForwardTimed(1.5, 30);
  fifteen.stopIntake();
  fifteen.possiblyResetGyro(180);
  
  // localize on other wall
  fifteen.goForwardU(30, 70, 180, 2, 5);
  fifteen.goTurnU(90);
  fifteen.goForwardU(15, 80, 90, 2, 4, false, 20, 30);
  fifteen.goForwardTimed(1, 30);
  fifteen.possiblyResetGyro(90);
  
  // Go to 24 pullup position
  fifteen.goForwardU(64, 80, 90, 3, 15);

  



  return 0;

}



void autonomous() { fifteen.setBrakeType(hold); task auto1(worldSkills); }
//void autonomous() { thread auto1(mainAuto); }

void userControl(void) { fifteen.setBrakeType(coast); task controlLoop1(mainTeleop); }
//void userControl(void) { task controlLoop1(logDistance); }


int main() {

  wait(500, msec);
  fifteen.gyroSensor.calibrate();
  fifteen.waitGyroCallibrate();

  Competition.bStopAllTasksBetweenModes = true;
  fifteen.clawDown();

  
  fifteen.leftMotorA.resetRotation();
  fifteen.rightMotorA.resetRotation();
  fifteen.backLiftL.resetRotation();
  fifteen.backLiftR.resetRotation();
  
  
  // DRIVER SKILLS TRUE, OTHERWISE FALSE
  //fifteen.setTransmission(false);

  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);


  //platformClimb2();

  while (true) {
    wait(20, msec);
  }
}