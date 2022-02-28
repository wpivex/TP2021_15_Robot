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

int logDistance() {

  fifteen.leftMotorA.resetRotation();
  fifteen.rightMotorA.resetRotation();

  while (true) {

    if (fifteen.buttons.pressed(Buttons::A)) {
      fifteen.leftMotorA.resetRotation();
      fifteen.rightMotorA.resetRotation();
    }

    float left = degreesToDistance(fifteen.leftMotorA.rotation(degrees));
    float right = degreesToDistance(fifteen.rightMotorA.rotation(degrees));
    log("%f %f %f", left, right, fifteen.getAngle());
    wait(20, msec);
  }

  return 0;
}


int testGPS() {
  fifteen.goPointGPS(-26, -54);

  return 0;
}

int ringSkills() {

  const float INTAKE_RING = 200;
  const float LOW_ARM = 0;

  fifteen.frontArmL.resetRotation();
  fifteen.frontArmR.resetRotation();
  fifteen.backLiftL.resetRotation();
  fifteen.backLiftR.resetPosition();
  fifteen.frontArmL.setBrake(hold);
  fifteen.frontArmR.setBrake(hold);
  fifteen.backLiftL.setBrake(hold);
  fifteen.backLiftR.setBrake(hold);

  fifteen.clawUp();
  fifteen.moveArmTo(INTAKE_RING, 100, false); // move arm in position to take rings

  // Grab home goal
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, true);
  fifteen.goForward(10, 40, 1, 2);
  fifteen.setBackLift(fifteen.BACK_LIFT_MID, true);

  // Grab match rings
  fifteen.intake.spin(forward, 100, pct);
  fifteen.goForward(20, 40, 1, 2);
  fifteen.goForward(-18, 60, 2, 4);
  fifteen.goForward(18, 40, 1, 0, false);

  // Head to left yellow goal
  fifteen.goPointGPS(-23, 36);
  fifteen.moveArmTo(LOW_ARM, 100, false);
  fifteen.intake.stop();
  fifteen.goTurnU(90);
  fifteen.goForward(10, 50, 2, 0, false); // go forward and pick up at same time without stopping

  // Drop yellow goal on blue side
  fifteen.clawDown();
  fifteen.moveArmTo(300, 100, false);
  fifteen.goForwardU(50, 100, 90, 5, 10, true, 50);
  fifteen.clawUp(); // drop yellow goal

  // Drop blue goal from back
  fifteen.setBackLift(fifteen.BACK_LIFT_DOWN, false);
  fifteen.goForward(-10, 40, 1, 1);
  





  return 0;

}

void autonomous() { fifteen.setBrakeType(hold); task auto1(ringSkills); }
//void autonomous() { thread auto1(mainAuto); }

void userControl(void) { fifteen.setBrakeType(coast); task controlLoop1(mainTeleop); }
//void userControl(void) { task controlLoop1(logDistance); }


int main() {

  wait(500, msec);
  fifteen.waitGpsCallibrate();

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
    wait(100, msec);
  }
}
