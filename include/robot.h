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

    //Front/Back Left/Right motors for controlling the sixbar lift
    motor frontArmL;
    motor frontArmR;


    digital_out backClaw = digital_out(Brain.ThreeWirePort.D);
    digital_out frontClaw = digital_out(Brain.ThreeWirePort.C);

    digital_out drivePistonRight = digital_out(Brain.ThreeWirePort.B);
    digital_out drivePistonLeft = digital_out(Brain.ThreeWirePort.A);

    controller* robotController;

    vision::signature* SIG_1;
    inertial gyroSensor;

    void driveStraight(float percent, float dist, float fasterAccel = 1.0);
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
    void armTeleop();
    void clawTeleop();
    void setLeftVelocity(directionType d, double percent);
    void setRightVelocity(directionType d, double percent);
    void stopLeft();
    void stopRight();
    void clampArmsDown();
    void waitGyroCallibrate();

    void raiseFrontArm(double amount, double vel, bool blocking);
    void raiseBackArm(double amount, double vel, bool blocking);

    void setFrontClamp(bool clamp);
    void setBackClamp(bool clamp);
    void setTransmission(bool fast);

    void balancePlatform();

    void printYaw();

    enum DriveType { ARCADE, TANK };
    DriveType driveType;

  private:
    void driveTeleop();

    void _gyroTurn(directionType direction, float degrees);

    void handleSixBarMechanism(motor* l, motor* r, controller::button* up, controller::button* down);

    template<typename Functor>
    void platformAction(Functor condition, double speed);

    void pneumaticsTeleop();

    bool invertControls = false;
    
    // State variables for claw
    time_t lastBackClaw = std::time(nullptr);
    time_t lastFrontClaw = std::time(nullptr);
};