#pragma once
#include "vex.h"
#include "ButtonHandler.h"
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
#include <functional>

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

    //Front/Back Left/Right motors for controlling the sixbar lift
    motor sixBarFL;
    motor sixBarFR;
    motor sixBarBL;
    motor sixBarBR;

    digital_out backClaw = digital_out(Brain.ThreeWirePort.D);
    digital_out frontClaw = digital_out(Brain.ThreeWirePort.C);

    digital_out drivePistonRight = digital_out(Brain.ThreeWirePort.B);
    digital_out drivePistonLeft = digital_out(Brain.ThreeWirePort.A);

    controller* robotController;

    vision::signature* SIG_1;
    inertial gyroSensor;

    void driveStraight(float percent, float dist);
    void driveStraight(float percent, float dist, float accPercent);
    void driveTimed(float percent, float driveTime);
    int getTurnAngle(float turnAngle);
    void turnToAngle(float percent, float turnAngle, bool PID, directionType direction);
    void gyroTurn(directionType direction, float degrees);
    bool turnToAngleNonblocking(float percent, float targetDist, bool PID, directionType direction);
    void driveCurved(directionType d, float dist, int delta);
    void driveCurved(directionType d, float dist, int delta, int speed);
    void driveCurvedTimed(directionType d, int delta, int speed, float driveTime);
    float distanceToDegrees(float dist);

    void userControl( void );
    void teleop( void );
    void sixBarTeleop();
    void clawTeleop();
    void setLeftVelocity(directionType d, double percent);
    void setRightVelocity(directionType d, double percent);
    void stopLeft();
    void stopRight();
    void clampArmsDown();
    void waitGyroCallibrate();




    void smartDrive(float distInches, float speed, directionType left, directionType right, int timeout, float slowDownInches, 
      float turnPercent, bool stopAfter, std::function<bool(void)> func);
    void driveTurn(float degrees, float speed, bool isClockwise, int timeout, float slowDownInches = 10, 
      bool stopAfter = true, std::function<bool(void)> func = {});
    void driveCurved(float distInches, float speed, directionType dir, int timeout, 
      float slowDownInches, float turnPercent, bool stopAfter = true, std::function<bool(void)> func = {});
    void driveStraight(float distInches, float speed, directionType dir, int timeout, 
      float slowDownInches, bool stopAfter = true, std::function<bool(void)> func = {});
    void Robot::driveStraightTimed(float speed, directionType dir, int timeMs, bool stopAfter, std::function<bool(void)> func) {
      driveStraight(0, speed, dir, timeMs, 1, stopAfter, func);

    void raiseFrontArm(double amount, double vel, bool blocking);
    void raiseBackArm(double amount, double vel, bool blocking);

    void setFrontClamp(bool clamp);
    void setBackClamp(bool clamp);
    void setTransmission(bool fast);

    void balancePlatform();

    void printYaw();

    void armMovementVCAT();

    enum DriveType { ARCADE, TANK };
    DriveType driveType;

  private:

    ButtonHandler buttons;

    void driveTeleop();

    void _gyroTurn(directionType direction, float degrees);

    void handleSixBarMechanism(motor* l, motor* r, controller::button* up, controller::button* down);


    void pneumaticsTeleop();
    
};