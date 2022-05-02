#pragma once
#include "vex.h"
#include "BaseRobot.cpp"

class Robot24 : public BaseRobot {

  public:
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

    encoder leftEncoder;
    encoder rightEncoder;

    triport expander = triport(PORT9);

    float absoluteY = 0;
    float absoluteX = 0;
    float recordedL = 0;
    float recordedR = 0;
    float recordedTheta;

    float TURN_MIN_SPEED = 10;
    float FORWARD_MIN_SPEED = 10;
    float SPEED_RATIO = 1.5;

    digital_out frontGoal = digital_out(Brain.ThreeWirePort.B);
    digital_out backGoal = digital_out(Brain.ThreeWirePort.A);
    digital_out clawPiston = digital_out(Brain.ThreeWirePort.C);
    digital_in clawSensor = digital_in(Brain.ThreeWirePort.D);
    digital_in frontSlideSensor = digital_in(expander.H);

    int32_t FRONT_CAMERA_PORT, BACK_CAMERA_PORT, SIDE_CAMERA_PORT;

    enum DriveType { ONE_STICK_ARCADE, TWO_STICK_ARCADE, TANK };
    DriveType driveType;

    enum ControllerMapping {DEFAULT_MAPPING};
    ControllerMapping cMapping;
    BTN::Button FRONT_CLAMP_TOGGLE, BACK_CLAMP_TOGGLE, CLAW_TOGGLE; 

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
    float getDriveCurrent() override;
    float getLeftCurrent();
    float getRightCurrent();

    // Drive Functions
    void goForwardUntilSensor(float maxDistance, float speed, digital_in sensor, float rampUpFrames = 0);
    void goVisionUntilSensor(float maxDistance, float speed, digital_in sensor, float rampUpFrames, bool disableVision = false); 
    void goForwardU(float distInches, float maxSpeed, float universalAngle, float rampUpFrames, float slowDownInches, float endSlowInches = 0,
      bool stopAfter = true, float minSpeed = 10, float timeout = 10);

    // Turning Functions
    void goTurnU(float universalAngleDegrees, int direction = 0, bool stopAfter = true, float timeout = 5, float maxSpeed = 100);
    
    // Vision Functions
    void goVision(float distInches, float speed, Goal goal, directionType cameraDir, float rampUpFrames, 
    float slowDownInches, float endSlowInches = 0, bool stopAfter = true, float timeout = 5);
    void goAlignVision(Goal goal, directionType cameraDir, float timeout = 5, bool stopAfter = true);
    void goAlignVisionTrap(Goal goal, directionType cameraDir, float timeout = 5, bool stopAfter = true);
    
    // Misc.
    void driveArmDown(float timeout);
    void resetArmRotation();
    void setArmDegrees(float degrees, float speed = 100, bool blocking = true);
    float getLeftEncoderDistance() override;
    float getRightEncoderDistance() override;
    float getLeftEncoderAbsolute();
    float getRightEncoderAbsolute();
    void resetEncoderDistance() override;
    void resetOdom();
    float distanceToDegrees(float distInches) override;
    float degreesToDistance(float distDegrees) override;
    void activeLocation();
    void goToPoint(float x, float y, float speed, float onlyTurn = false);
    void goFightOdom(float backUpDist, float slowDownInches);
    void setArmBrakeType(brakeType b);
    void gotToY(float yValue, float speed);
    void gotToX(float xValue, float speed);

  private:

    float zeroedRelLeft, zeroedRelRight;

    void driveTeleop();
    void armTeleop();
    void pneumaticsTeleop();
    void clawMovement();
    bool driveHold = false;
    bool teleopArmLimited = false;

};