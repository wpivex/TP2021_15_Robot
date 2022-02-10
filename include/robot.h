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

    motor backLiftL;
    motor backLiftR;


    vision frontCamera;


    digital_out frontClaw = digital_out(Brain.ThreeWirePort.A);

    controller* robotController;

    inertial gyroSensor;


    Buttons buttons;

    enum DriveType { ONE_STICK_ARCADE, TWO_STICK_ARCADE, TANK };
    DriveType driveType;

    enum ControllerMapping {DEFAULT_MAPPING, BRIAN_MAPPING};
    ControllerMapping cMapping;
    Buttons::Button FRONT_ARM_UP, FRONT_ARM_DOWN, FRONT_CLAW_ON, FRONT_CLAW_OFF, CLAW_UP, CLAW_DOWN, BACK_LIFT_UP, BACK_LIFT_DOWN;

    void setControllerMapping(ControllerMapping mapping);

    void clawUp();
    void clawDown();

    void moveArmTo(double degr, double speed);

    void smartDrive(float distInches, float speed, directionType left, directionType right, float timeout, float slowDownInches, 
                    float turnPercent, bool stopAfter, std::function<bool(void)> func);
    void driveTurn(float degrees, float speed, bool isClockwise, float timeout, float slowDownInches = 10, 
                    bool stopAfter = true, std::function<bool(void)> func = {});
    void driveCurved(float distInches, float speed, directionType dir, float timeout, 
                      float slowDownInches, float turnPercent, bool stopAfter = true, std::function<bool(void)> func = {});
    void driveStraight(float distInches, float speed, directionType dir, float timeout, 
                      float slowDownInches, bool stopAfter = true, std::function<bool(void)> func = {});
    void driveStraightTimed(float speed, directionType dir, float timeMs, bool stopAfter = true, std::function<bool(void)> func = {});

    void goForwardVision(Goal goal, float speed, directionType dir, float maximumDistance, int timeout, 
                        digital_in* limitSwitch = nullptr, std::function<bool(void)> func = {});
    void alignToGoalVision(Goal goal, bool clockwise, directionType cameraDirection, int timeout);
    void updateCamera(Goal goal);

    void driveStraightGyro(float distInches, float speed, directionType dir, float timeout, float slowDownInches, bool resetEncoder = true,
                            std::function<bool(void)> func = {});
    void driveStraightGyroHeading(float distInches, float speed, float head, directionType dir, float timeout, float slowDownInches, 
std::function<bool(void)> func);
    void turnToAngleGyro(bool clockwise, float angleDegrees, float maxSpeed, int startSlowDownDegrees,
                        int timeout, std::function<bool(void)> func = {});
    void turnToUniversalAngleGyro(float universalAngleDegrees, float maxSpeed, int startSlowDownDegrees,
int timeout, std::function<bool(void)> func = {});

    float normalize(float axis);

    void userControl( void );
    void teleop( void );
    void armTeleop();
    void clawTeleop();
    void setLeftVelocity(directionType d, double percent);
    void setRightVelocity(directionType d, double percent);
    void stopLeft();
    void stopRight();
    void waitGyroCallibrate();

  private:
    void driveTeleop();
    void checkLowerLimit(std::function<void(void)> doInstead);

};