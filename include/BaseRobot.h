#include "vex.h"
#include "TrapezoidController.cpp"
#include "PIDController.cpp"
#include "Buttons.cpp"

using namespace vex;

class BaseRobot {

public:

  inertial gyroSensor;
  Buttons buttons;

  BaseRobot(int32_t gyroPort);

  virtual void teleop() = 0;

  virtual void resetEncoderDistance() = 0;
  virtual float getLeftEncoderDistance() = 0;
  virtual float getRightEncoderDistance() = 0;
  float getEncoderDistance() {return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2;}

  float getAngle();
  void waitGyroCallibrate();
  void possiblyResetGyro(float targetAngle, float angleTolerance = 10);

  void setMotorVelocity(motor m, directionType d, double percent);
  virtual void setLeftVelocity(directionType d, double percent) = 0;
  virtual void setRightVelocity(directionType d, double percent) = 0;
  virtual void stopLeft() = 0;
  virtual void stopRight() = 0;
  virtual void setBrakeType(brakeType b) = 0;

  virtual float distanceToDegrees(float distInches) = 0;
  virtual float degreesToDistance(float distDegrees) = 0;

  void goCurve(float distInches, float maxSpeed, float turnPercent, float rampUpInches, float slowDownInches, 
      bool stopAfter = true, float rampMinSpeed = 20, float slowMinSpeed = 12);
  virtual void goForwardU(float distInches, float maxSpeed, float universalAngle, float rampUpInches, float slowDownInches, 
      bool stopAfter = true, float rampMinSpeed = 20, float slowDownMinSpeed = 10, float timeout = 10) = 0;
  void goForward(float distInches, float maxSpeed, float rampUpInches, float slowDownInches, bool stopAfter = true, 
      float rampMinSpeed = 20, float slowDownMinSpeed = 12, float timeout = 5);
  void goForwardTimed(float duration, float speed);

protected:

  void goForwardU_Abstract(float K_P, float distInches, float maxSpeed, float universalAngle, float rampUpInches, float slowDownInches, 
      bool stopAfter = true, float rampMinSpeed = 20, float slowDownMinSpeed = 10, float timeout = 10);
  void goTurnU_Abstract(float KP, float KI, float KD, float TOLERANCE, float REPEATED, float MINUMUM,
      float universalAngleDegrees, bool stopAfter = true, float timeout = 5, float maxSpeed = 75);

  void goVision_Abstract(float K_P, float MIN_SPEED, int32_t CAMERA_PORT, float distInches, float speed, Goal goal,
  float rampUpInches, float slowDownInches, bool stopAfter = true, float timeout = 5);

  void goAlignVision_Abstract(float K_P, float K_I, float K_D, float TOLERANCE, float REPEATED, float MINIMUM, int32_t CAMERA_PORT, 
    Goal goal, float timeout = 5, bool stopAfter = true);
  

  

};