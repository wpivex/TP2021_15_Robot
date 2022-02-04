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
#include "Buttons.cpp"
#include <constants.h>

using namespace vex;

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

    //Front/Back Left/Right motors for controlling the sixbar lift
    motor frontArmL;
    motor frontArmR;

    vision backCamera;
    vision frontCamera;


    // digital_out backClaw = digital_out(Brain.ThreeWirePort.D);
    // digital_out frontClaw = digital_out(Brain.ThreeWirePort.C);

    controller* robotController;

    inertial gyroSensor;


    Buttons buttons;

    enum ControllerMapping {DEFAULT_MAPPING};
    Buttons::Button FRONT_CLAMP_TOGGLE, BACK_CLAMP_TOGGLE, CLAW_TOGGLE; 

    void setControllerMapping(ControllerMapping mapping);

    void smartDrive(float distInches, float speed, directionType left, directionType right, int timeout, float slowDownInches, 
                    float turnPercent, bool stopAfter, std::function<bool(void)> func);
    void driveTurn(float degrees, float speed, bool isClockwise, int timeout, float slowDownInches = 10, 
                    bool stopAfter = true, std::function<bool(void)> func = {});
    void driveCurved(float distInches, float speed, directionType dir, int timeout, 
                      float slowDownInches, float turnPercent, bool stopAfter = true, std::function<bool(void)> func = {});
    void driveStraight(float distInches, float speed, directionType dir, int timeout, 
                      float slowDownInches, bool stopAfter = true, std::function<bool(void)> func = {});
    void driveStraightTimed(float speed, directionType dir, int timeMs, bool stopAfter = true, std::function<bool(void)> func = {});

    void goForwardVision(Goal goal, float speed, directionType dir, float maximumDistance, int timeout, 
                        digital_in* limitSwitch, std::function<bool(void)> func = {});
    void alignToGoalVision(Goal goal, bool clockwise, directionType cameraDirection, int timeout);
    void updateCamera(Goal goal);

    void driveStraightGyro(float distInches, float speed, directionType dir, int timeout, float slowDownInches,
                            std::function<bool(void)> func = {});
    void turnToAngleGyro(bool clockwise, float angleDegrees, float maxSpeed, int startSlowDownDegrees,
                        int timeout, std::function<bool(void)> func = {});


    void userControl( void );
    void teleop( void );
    void armTeleop();
    void clawTeleop();
    void setLeftVelocity(directionType d, double percent);
    void setRightVelocity(directionType d, double percent);
    void stopLeft();
    void stopRight();
    void waitGyroCallibrate();


    enum DriveType { ARCADE1, ARCADE2, TANK };
    DriveType driveType;

  private:
    void driveTeleop();

};