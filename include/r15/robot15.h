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

#include "Utility/VisualGraph.cpp"
#include "BaseRobot.cpp"
#include "constants.h"

using namespace vex;

class Robot15 : public BaseRobot {
  public:

    // four drivebase motors will not be accessible for a while
    Robot15();
    motor leftMotorA;
    motor leftMotorB;
    motor leftMotorC;
    motor leftMotorD;
    motor rightMotorA;
    motor rightMotorB;
    motor rightMotorC;
    motor rightMotorD;

    motor intake;
    motor frontArmL;
    motor frontArmR;
    motor backLiftL;
    motor backLiftR;

    int32_t CAMERA_PORT;

    digital_out frontClaw = digital_out(Brain.ThreeWirePort.G);

    enum ControllerMapping {EZEQUIEL_MAPPING, BRIAN_MAPPING};
    ControllerMapping cMapping;
    Buttons::Button FRONT_ARM_UP, FRONT_ARM_DOWN, FRONT_CLAW_ON, FRONT_CLAW_OFF, CLAW_UP, CLAW_DOWN;
    Buttons::Button BACK_LIFT_UP, BACK_LIFT_MID, BACK_LIFT_SLIGHT, BACK_LIFT_DOWN, BACK_LIFT_UPPING, BACK_LIFT_DOWNING, INTAKE_TOGGLE, INTAKE_TOGGLE_REV;

    void setControllerMapping(ControllerMapping mapping);

    // Joint control methods
    void moveArmTo(double degr, double speed, bool blocking = true);
    bool moveArmToManual(double degr, double maxSpeed);
    void setBackLift(Buttons::Button b, bool blocking);
    void backLiftTeleop();
    void startIntake(directionType dir = forward);
    void stopIntake();
    void clawUp();
    void clawDown();

    // Teleop methods
    void userControl( void );
    void teleop( void ) override;
    void armTeleop();
    void clawTeleop();
    void intakeTeleop();

    // Calling parent drive functions with params
    void goForwardU(float distInches, float maxSpeed, float universalAngle, float rampUpFrames, float slowDownInches, 
        bool stopAfter = true, float minSpeed = 10, float timeout = 10);
    void goTurnU(float universalAngleDegrees, int direction = 0, bool stopAfter = true, float timeout = 5, float maxSpeed = 75);
    void goVision(float distInches, float speed, Goal goal, float rampUpFrames, float slowDownInches, bool stopAfter, float timeout = 5);
    void goAlignVision(Goal goal, float timeout = 5, bool stopAfter = true);

    // Implementing abstract functions
    float distanceToDegrees(float distInches) override;
    float degreesToDistance(float distInches) override;
    float getLeftEncoderDistance() override;
    float getRightEncoderDistance() override;
    void resetEncoderDistance() override;
    void setLeftVelocity(directionType d, double percent) override;
    void setRightVelocity(directionType d, double percent) override;
    void stopLeft() override;
    void stopRight() override;
    void setBrakeType(brakeType b) override;
    float getDriveCurrent() override;

  private:
    int intakeState;
};