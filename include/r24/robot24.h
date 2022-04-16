#pragma once
#include "vex.h"
#include "BaseRobot.cpp"

class Robot24 : public BaseRobot {

  public:
    // four drivebase motors will not be accessible for a while
    Robot24();

    motor leftMotorA;
    motor leftMotorB;
    motor leftMotorC;
    motor leftMotorD;
    motor leftMotorE;
    motor rightMotorA;
    motor rightMotorB;
    motor rightMotorC;
    motor rightMotorD;
    motor rightMotorE;

    motor rightArm1;
    motor rightArm2;
    motor leftArm1;
    motor leftArm2;  

    int TURN_MIN_SPEED = 10;
    int FORWARD_MIN_SPEED = 10;
    int DIST_BETWEEN_WHEELS = 15;

    digital_out frontGoal = digital_out(Brain.ThreeWirePort.B);
    digital_out backGoal = digital_out(Brain.ThreeWirePort.A);
    digital_out clawPiston = digital_out(Brain.ThreeWirePort.C);
    digital_in clawSensor = digital_in(Brain.ThreeWirePort.D);

    //inertial gyroSensor;
    int32_t FRONT_CAMERA_PORT, BACK_CAMERA_PORT;

    enum DriveType { ONE_STICK_ARCADE, TWO_STICK_ARCADE, TANK };
    DriveType driveType;

    enum ControllerMapping {DEFAULT_MAPPING};
    ControllerMapping cMapping;
    Buttons::Button FRONT_CLAMP_TOGGLE, BACK_CLAMP_TOGGLE, CLAW_TOGGLE; 

    void setControllerMapping(ControllerMapping mapping);

    void openClaw();
    void closeClaw();
    void goalClamp();
    void setFrontClamp(bool intaking);
    void setBackClamp(bool intaking);
    void stopArm();
    void setArmPercent(directionType d, double percent);

    void teleop() override;
    void setLeftVelocity(directionType d, double percent) override;
    void setRightVelocity(directionType d, double percent) override;
    void stopLeft() override;
    void stopRight() override;
    void setBrakeType(brakeType b) override;
    void setMaxArmTorque(float c);
    void setMaxDriveTorque(float c);


    // Drive Functions
    void goForwardUntilSensor(float maxDistance, float speed, float rampUpInches = 0, int timeout = 5, bool stopAfter = true);
    void goForwardU(float distInches, float maxSpeed, float universalAngle, float rampUpFrames, float slowDownInches, 
      bool stopAfter = true, float rampMinSpeed = 20, float slowDownMinSpeed = 10, float timeout = 10);

    // Turning Functions
    void goTurnU(float universalAngleDegrees, bool stopAfter = true, float timeout = 5, float maxSpeed = 75);

    // Curves
    void goRadiusCurve(float radius, float distAlongCircum, bool curveDirection, float maxSpeed, float rampUp, float slowDown, 
      bool stopAfter = true, float timeout = 5);
    void gyroCurve(float distInches, float maxSpeed, float turnAngle, int timeout, bool stopAfter = true);
    
    // Vision Functions
    void goVision(float distInches, float speed, Goal goal, directionType cameraDir, float rampUpFrames, 
    float slowDownInches, bool stopAfter, float timeout = 5);
    void goAlignVision(Goal goal, directionType cameraDir, float timeout = 5, bool stopAfter = true);
    void goAlignVisionTrap(Goal goal, directionType cameraDir, float timeout = 5, bool stopAfter = true);
    
    // Misc.
    void driveArmDown(float timeout);
    void resetArmRotation();
    void setArmDegrees(float degrees, float speed = 100, bool blocking = true);

    float getLeftEncoderDistance() override;
    float getRightEncoderDistance() override;
    void resetEncoderDistance() override;

  private:

    void driveTeleop();
    void armTeleop();
    void pneumaticsTeleop();
    void clawMovement();
    

    bool driveHold = false;
    bool teleopArmLimited = false;

};