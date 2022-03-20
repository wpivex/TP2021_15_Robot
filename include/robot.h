
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
#include "PIDController.cpp"
#include "TrapezoidController.cpp"
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

    motor intake;

    //Front/Back Left/Right motors for controlling the sixbar lift
    motor frontArmL;
    motor frontArmR;

    motor backLiftL;
    motor backLiftR;

    inertial gyroSensor;
    vision camera;


    digital_out frontClaw = digital_out(Brain.ThreeWirePort.G);

    controller* robotController;

    Buttons buttons;

    //enum DriveType { ONE_STICK_ARCADE, TWO_STICK_ARCADE, TANK };
    //DriveType driveType;

    enum ControllerMapping {EZEQUIEL_MAPPING, BRIAN_MAPPING};
    ControllerMapping cMapping;
    Buttons::Button FRONT_ARM_UP, FRONT_ARM_DOWN, FRONT_CLAW_ON, FRONT_CLAW_OFF, CLAW_UP, CLAW_DOWN;
    Buttons::Button BACK_LIFT_UP, BACK_LIFT_MID, BACK_LIFT_SLIGHT, BACK_LIFT_DOWN, BACK_LIFT_UPPING, BACK_LIFT_DOWNING, INTAKE_TOGGLE, INTAKE_TOGGLE_REV;

    void setControllerMapping(ControllerMapping mapping);

    float getEncoderDistance();
    void resetEncoderDistance();

    float getAngle();
    float getX(int numSamples = 1);
    float getY(int numSamples = 1);
    void possiblyResetGyro(float targetAngle);

    void setBackLift(Buttons::Button b, bool blocking);
    void backLiftTeleop();

    void clawUp();
    void clawDown();

    void goForwardTimed(float duration, float speed);

    void goCurve(float distInches, float maxSpeed, float turnPercent, float rampUpInches, float slowDownInches, bool stopAfter = true, float rampMinSpeed = 20);

    void goTurnU(float universalAngleDegrees, bool stopAfter = true, float timeout = 5, bool fast = false);

    void goForwardU(float distInches, float maxSpeed, float universalAngle, float rampUpInches, float slowDownInches, 
bool stopAfter = true, float rampMinSpeed = 20, float slowDownMinSpeed = 10, float timeout = 10, bool angleCorrection = true);
    void goForward(float distInches, float maxSpeed, float rampUpInches, float slowDownInches, bool stopAfter = true, 
    float rampMinSpeed = 20, float slowDownMinSpeed = 12, float timeout = 5);

    void goToAxis(axisType axis, bool reverseDirection, float finalValue, float maxSpeed, float timeout = 10);
    void goForwardGPS(float x, float y, float maxSpeed, float rampUpInches, float slowDownInches, directionType dir = forward); 

    void waitForGPS();
    void goPointGPS(float x, float y, directionType dir = forward);

    void goVision(float distInches, float speed, Goal goal, float rampUpInches, float slowDownInches, bool stopAfter = true, float timeout = 5);
    void goAlignVision(Goal goal, float timeout);

    void goRadiusCurve(float radius, float numRotations, bool curveDirection, float maxSpeed, float rampUp, float slowDown, float timeout = 5);

    void moveArmTo(double degr, double speed, bool blocking = true);

    void driveStraightFighting(float distInches, float speed, directionType dir);


    float normalize(float axis);
    void updateCamera(Goal goal);

    void userControl( void );
    void teleop( void );
    void armTeleop();
    void clawTeleop();
    void intakeTeleop();
    void setLeftVelocity(directionType d, double percent);
    void setRightVelocity(directionType d, double percent);
    void startIntake(directionType dir = forward);
    void stopIntake();
    void stopLeft();
    void stopRight();
    void setBrakeType(brakeType b);

    void waitGyroCallibrate();

  private:
    void driveTeleop();

    int intakeState;
};