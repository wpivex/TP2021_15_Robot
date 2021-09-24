#pragma once
#include "vex.h"

using namespace vex;

class Robot {
  public:
    // four drivebase motors will not be accessible for a while
    Robot(controller* c);
    motor leftMotorA;
    motor leftMotorB;
    motor leftMotorC;
    // motor leftMotorD;
    // motor leftMotorE;
    motor rightMotorA;
    motor rightMotorB;
    motor rightMotorC;
    // motor rightMotorD;
    // motor rightMotorE;

    motor_group leftDrive;
    motor_group rightDrive;

    motor fourBarLeft;
    motor fourBarRight;
    motor chainBarLeft;
    motor chainBarRight;
    motor claw;

    controller* robotController;

    void driveStraight(float percent, float dist);
    void driveStraight(float percent, float dist, float accPercent);
    void driveTimed(float percent, float driveTime);
    void turnToAngle(float percent, float turnAngle);
    void openClaw();
    void closeClaw();
    void liftFourBar(float percentHeight);
    void lowerFourBar(float percentHeight);

    void userControl( void );
    void teleop( void );
    void init();
    void setLeftVelocity(directionType d, double percent);
    void setRightVelocity(directionType d, double percent);
    void stopLeft();
    void stopRight();

    enum DriveType { ARCADE, TANK };
    DriveType driveType;
};