#include "PIDController.h"
#include "constants.h"

PID::PID(float kp, float ki, float kd, float TOLERANCE, int REPEATED, float minimum) {

  TOLERANCE_THRESHOLD = TOLERANCE; // the error threshold to be considered "done"
  REPEATED_THRESHOLD = REPEATED; // the number of times the threshold must be reached in a row to be considered "done"
  repeated = 0;

  K_p = kp;
  K_i = ki;
  K_d = kd;
  min = minimum;
}



// bound is // max/min value of output. Useful for stuff like capping speed
float PID::tick(float error, float bound) {
  float integral = prevIntegral + error * 0.02;
  float derivative = (error - prevError) / 0.02;

  float output = K_p * error + K_i * integral + K_d * derivative;
  prevError = error;
  prevIntegral = integral;

  if (fabs(error) < TOLERANCE_THRESHOLD) repeated++;
  else repeated = 0;

  // If bound is not 0 (meaning unbounded), output should be clamped between -bound and +bound
  if (bound != UNBOUNDED) output = fmax(-bound, fmin(bound, output));

  // Set mininum output value
  if (output > 0) {
    output = fmax(min, output);
  } else {
    output = fmin(-min, output);
  }

  //logController("%f | %d | %d %d %d | %d", (float) error, (int) output, (int) (K_p * error), (int) (K_i * integral), (int) (K_d * derivative), repeated);
  return output;
}

// Call to check whether PID has converged to a value. Use with stuff like arm movement and aligns/turns but not with stuff like driving straight
bool PID::isCompleted() {
  //return false;
  //logController("asdf %d %d", repeated, REPEATED_THRESHOLD);
  return repeated >= REPEATED_THRESHOLD;
}