// CODE FOR 24" ROBOT
#include "r24/robot24.cpp"

Robot24 twentyFour = Robot24();

int mainTeleop24() {
  twentyFour.setBackClamp(false);
  twentyFour.setFrontClamp(false);

  while (true) {
    twentyFour.teleop();
    wait(20, msec);
  }
  return 0;
}

/*
void middleFirst(void) {
  int color = 1; //red is 1, blue is 2
  robot.setBackClamp(true);
  robot.driveCurved(reverse, 20, 55);
  wait(2000, msec);
  robot.goForwardVision(true, 100, 30, 25, 0);
  robot.driveStraight(100, -10);
  robot.setBackClamp(false);
  
  //Go back 2 ft
  robot.driveStraight(100, 24);
  robot.blindAndVisionTurn(80, 0);
  
  robot.setFrontClamp(true);
  robot.goForwardVision(false, 100, 24, 40, 0);
  robot.setFrontClamp(false);
  wait(250, msec);

  //Go back 2 ft
  robot.driveStraight(100, -15);  
  robot.intakeOverGoal(color);
}
*/

int matchAuto() {
  Goal allianceColor = RED;
  twentyFour.setBrakeType(hold);

  // ~~~~~~~~~~~~~ Box Rush Right ~~~~~~~~~~~~~~~
  twentyFour.openClaw();
  // Drive forwards at full speed (while adjusting towards goal if needed)
  twentyFour.setArmDegrees(5, 50, false);
  // robot.goForward(36, 100, 3, 20, 5); 
  twentyFour.goForwardUntilSensor(36, 100, 3, 5);
  twentyFour.closeClaw();
  wait(200, msec);
  // Raise arm a bit (so that other team cannot grab it)
  twentyFour.setArmDegrees(215);
  // RETREAT
  twentyFour.goForward(-8, 100, 0, 5); 

  // // ~~~~~~~~~~~ Middle Goal Check ~~~~~~~~~~~~~~
  //robot.cursedTurn(150,70);

  // robot.goTurn(120);
  // robot.setBackClamp(true);
  // robot.goVision(50, 65, YELLOW, reverse, 0, 0);
  // robot.setBackClamp(false);
  // wait(200, msec);

  // robot.goForward(40, 100, 5, 0, 5, {}, false);
  // robot.goCurve(20, 100, 0.2, 0, 10); // get back to base


  // // ~~~~~~~~~~~~~~ Alliance Goal ~~~~~~~~~~~~~~~~
  // robot.encoderTurnU(130);
  // robot.goForward(24*sqrt(3), 100);
  // robot.encoderTurnU(90);
  // robot.setFrontClamp(true);
  // robot.goForward(24, 50);
  // robot.setFrontClamp(false);

  // // ~~~~~~~~~~~ Get out of the 15's way~~~~~~~~~~~
  // robot.encoderTurnU(0);
  // robot.goForward(-24,30);

  return 0;
}

int testCurrent() {

  twentyFour.setMaxArmTorque(CURRENT::MID);
  twentyFour.setLeftVelocity(reverse, 100);
  twentyFour.setRightVelocity(reverse, 100);
  int startTime = timer::system();
  VisualGraph g(0, 3, 10, 200);
  int a = 0;
  while (!isTimeout(startTime, 2.5)) {
    
    float curr = (twentyFour.leftMotorA.current() + twentyFour.rightMotorA.current()) / 2;
    g.push(curr);

    a = (a + 1) % 10;
    if (a == 0) g.display();

    wait(20, msec);

  }
  twentyFour.stopLeft();
  twentyFour.stopRight();

  return 0;
}

const float DEGREES_PER_PIXEL = (63.0 * M_PI / 180.0) / VISION_MAX_X;
const float GOAL_WIDTH = 13.5;
const float CAMERA_HEIGHT = 15.5;
float getDistanceFromWidth(int width){

  float theta = DEGREES_PER_PIXEL*width;
  float hypotenuseDistanceToGoal = GOAL_WIDTH / tan(theta);
  float horizontalDistance = sqrt(pow(hypotenuseDistanceToGoal, 2) - pow(CAMERA_HEIGHT, 2));
  log("%d %d %d",width, hypotenuseDistanceToGoal,horizontalDistance);
  return horizontalDistance;
}

int testVisionDistance() {

  Goal g = YELLOW;

  vision camera(twentyFour.BACK_CAMERA_PORT, g.bright, g.sig);
  
  while (true) {

    camera.takeSnapshot(g.sig);

    if (camera.largestObject.exists) {
      float width = camera.largestObject.width;
      float dist = getDistanceFromWidth(width);
      //log("%f\n%f", width, dist);
    } else {
      log("no object");
    }
    wait(20, msec);
  }
  return 0;
}

int testRadiusCurve() {
  twentyFour.goRadiusCurve(48, 1, 1, 100, 20, 10);
  return 0;
}

void autonomous24() { twentyFour.setBrakeType(hold); task auto1(testVisionDistance); }
void userControl24(void) { twentyFour.setBrakeType(coast); task controlLoop1(mainTeleop24); }

int main24() {
  testRadiusCurve();
  return 0;
}

/*
int main24() {
  Competition.bStopAllTasksBetweenModes = true;

  twentyFour.setBackClamp(false);
  twentyFour.setFrontClamp(false);

  // Callibrate Gyro
  twentyFour.waitGyroCallibrate();
  
  twentyFour.resetArmRotation();
  Competition.autonomous(autonomous24);
  Competition.drivercontrol(userControl24);

  while (true) {
    wait(20, msec);
  }

}

*/