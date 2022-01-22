#pragma once
#include "vex.h"
#include <string>
#include <fstream>
#include <vector>
#include <utility> // std::pair
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream
#include <string>
#include <iostream>
#include <sstream>
#include <stdlib.h>     /* atof */
#include <math.h>       /* sin */
#include <stdio.h>      /* printf, fgets */
#include <unistd.h>

using namespace vex;
brain Brain;

class Robot {
  public:
    // four drivebase motors will not be accessible for a while
    Robot(controller* c);
    motor leftMotorA;
    motor leftMotorB;
    motor leftMotorC;
    motor leftMotorD;
    motor rightMotorA;
    motor rightMotorB;
    motor rightMotorC;
    motor rightMotorD;

    motor_group leftDrive;
    motor_group rightDrive;

    vision backCamera;
    vision frontCamera;

    //Front/Back Left/Right motors for controlling the sixbar lift
    motor sixBarFL;
    motor sixBarFR;
    motor sixBarBL;
    motor sixBarBR;
    motor claw;


    //Pneumatics
    // slight forward, slight back
    // vertical goal clamp

    //Two per lift (left/right)
    
    digital_out leftLift = digital_out(Brain.ThreeWirePort.A);
    digital_out rightLift = digital_out(Brain.ThreeWirePort.B);

    //two forks for transmission
    digital_out drivePiston1 = digital_out(Brain.ThreeWirePort.C);
    digital_out drivePiston2 = digital_out(Brain.ThreeWirePort.D);

    controller* robotController;

    vision::signature* SIG_1;

    void driveStraight(float percent, float dist);
    void driveStraight(float percent, float dist, float accPercent);
    void driveTimed(float percent, float driveTime);
    int getTurnAngle(float turnAngle);
    void turnToAngle(float percent, float turnAngle, bool PID, directionType direction);
    bool turnToAngleNonblocking(float percent, float targetDist, bool PID, directionType direction);
    void driveCurved(directionType d, float dist, int delta);
    float distanceToDegrees(float dist);
    void openClaw();
    void closeClaw();
    void goalClamp();
    void setFrontClamp(bool intaking);
    void setBackClamp(bool intaking);

    void userControl( void );
    void teleop( void );
    void setLeftVelocity(directionType d, double percent);
    void setRightVelocity(directionType d, double percent);
    void stopLeft();
    void stopRight();

    enum DriveType { ARCADE, TANK };
    DriveType driveType;

  private:

    // fourbar, chainbar
    //double angles[6][2] = {{422, 821}, //intaking (0)
    //                      {1311, 1247}, //intermediate (1)
    //                      {1060, 428}, //ring front (2)
    //                      {1271, 327}, //ring middle (3)
    //                      {1446, -26}, //right back (4)
    //                      {500, 220}}; //place goal (5)

    void driveTeleop();
    void pneumaticsTeleop();
    


    // State variables for goal clamp
    time_t lastLeftPress = std::time(nullptr);
    time_t lastRightPress = std::time(nullptr);
};