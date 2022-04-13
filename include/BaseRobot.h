#include "vex.h"
#include "TrapezoidController.cpp"
#include "PIDController.cpp"

using namespace vex;

class BaseRobot {

  public:

  inertial gyroSensor;

  virtual void resetEncoderDistance() = 0;
  virtual float getEncoderDistance() = 0;
  virtual float getAngle() = 0;

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
    bool stopAfter = true, float rampMinSpeed = 20, float slowDownMinSpeed = 10, float timeout = 10);

  void goForward(float distInches, float maxSpeed, float rampUpInches, float slowDownInches, bool stopAfter = true, 
    float rampMinSpeed = 20, float slowDownMinSpeed = 12, float timeout = 5) {
      goForwardU(distInches, maxSpeed, getAngle(), rampUpInches, slowDownInches, stopAfter, rampMinSpeed, slowDownMinSpeed, timeout);
  }
    
  void goForwardTimed(float duration, float speed);

  protected:

  void goForwardU_Abstract(float K_P, float distInches, float maxSpeed, float universalAngle, float rampUpInches, float slowDownInches, 
    bool stopAfter = true, float rampMinSpeed = 20, float slowDownMinSpeed = 10, float timeout = 10);
  

  

};