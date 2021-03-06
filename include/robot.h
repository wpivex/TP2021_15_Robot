
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
#include <algorithm>
#include "Buttons.cpp"
#include "PIDController.cpp"
#include "TrapezoidController.cpp"
#include "GoalPosition.cpp"
#include "VisualGraph.cpp"
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
    digital_out backClamp = digital_out(Brain.ThreeWirePort.D);

    controller* robotController;

    Buttons buttons;

    //enum DriveType { ONE_STICK_ARCADE, TWO_STICK_ARCADE, TANK };
    //DriveType driveType;

    enum ControllerMapping {EZEQUIEL_MAPPING, BRIAN_MAPPING};
    ControllerMapping cMapping;
    Buttons::Button FRONT_ARM_UP, FRONT_ARM_DOWN, FRONT_CLAW_ON, FRONT_CLAW_OFF, CLAW_UP, CLAW_DOWN;
    Buttons::Button BACK_LIFT_UP, BACK_LIFT_MID, BACK_LIFT_SLIGHT, BACK_LIFT_DOWN, BACK_LIFT_UPPING, BACK_LIFT_DOWNING, INTAKE_TOGGLE, INTAKE_TOGGLE_REV;

    void setControllerMapping(ControllerMapping mapping);

    void setSecondaryCurrent(bool setOn);

    float getEncoderDistance();
    void resetEncoderDistance();

    float getAngle();
    float getX(int numSamples = 1);
    float getY(int numSamples = 1);
    void possiblyResetGyro(float targetAngle);
    void getGPSData(float *x, float *y, float *headingAngle, int numSamples = 5);

    void setBackLift(Buttons::Button b, bool blocking);
    void backLiftTeleop();

    void clawUp();
    void clawDown();

    void goForwardTimed(float duration, float speed);

    void goCurve(float distInches, float maxSpeed, float turnPercent, float rampUpInches, float slowDownInches, bool stopAfter = true, float rampMinSpeed = 20, float slowMinSpeed = 12);

    void goTurnU(float universalAngleDegrees, int direction = 0, bool stopAfter = true, float timeout = 5, float maxSpeed = 60);
    void climbPlatform(float startPitch, float speed);
    void goForwardU(float distInches, float maxSpeed, float universalAngle, float rampUpFrames, float slowDownInches, 
bool stopAfter = true, float rampMinSpeed = 20, float slowDownMinSpeed = 16, float timeout = 10);
    void goForward(float distInches, float maxSpeed, float rampUpInches, float slowDownInches, bool stopAfter = true, 
    float rampMinSpeed = 20, float slowDownMinSpeed = 12, float timeout = 5);

    void goToAxis(axisType axis, bool reverseDirection, float finalValue, float maxSpeed, float timeout = 10);
    void goForwardGPS(float x, float y, float maxSpeed, float rampUpInches, float slowDownInches, directionType dir = forward); 

    void waitForGPS();
    void goPointGPS(float x, float y, directionType dir = forward);

    void goVision(float distInches, float speed, Goal goal, float rampUpInches, float slowDownInches, bool stopAfter = true, float timeout = 5);
    void goAlignVision(Goal goal, float timeout);

    void drawVision();

    void goRadiusCurve(float radius, float numRotations, bool curveDirection, float maxSpeed, float rampUp, float slowDown, bool stopAfter = true, float timeout = 5);

    void moveArmTo(double degr, double speed, bool blocking = true);
    bool moveArmToManual(double degr, double speed);

    void goFightBackwards();


    float normalize(float axis);
    void updateCamera(Goal goal);

    void userControl( void );
    void teleop( void );
    void armTeleop();
    void clawTeleop();
    void intakeTeleop();
    void setLeftVelocity(directionType d, double percent);
    void setRightVelocity(directionType d, double percent);
    void setMotorVelocity(motor m, directionType d, double percent);
    void startIntake(directionType dir = forward);
    void stopIntake();
    void stopLeft();
    void stopRight();
    void setBrakeType(brakeType b);

    void waitGyroCallibrate();

    void trackObjectsForCurrentFrame(std::vector<GoalPosition> &goals, int targetID = -1);
    int findGoalID(std::vector<GoalPosition> &goals);
    int detectionAndStrafePhase(float *horizontalDistance, int matchStartTime);
    float getDistanceFromArea(int area);
    void runAI(int matchStartTime);

    void backDown();
    void backUp();

    bool isThereGoal();


  private:
    void driveTeleop();
    int goMidFrames = 1;
    int intakeState;
    bool targetIsIntake = false;
    bool backIsDown = false;
};