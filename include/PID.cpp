#import "waypoint.cpp"

class PID {       // The class
  private:
    double kp;
    double ki;  // Not currently used, but might be in the future.
    double kd;
    double kv;
    double ka;
    double last_error;
  public :             // Access specifier


  PID(double p,double i,double d,double v,double a){
    kp = p;
    ki = i;
    kd = d;
    kv = v;
    ka = a;
  }

  double calculate(Waypoint targetWaypoint, Waypoint currentPoint) {
      double error = distanceBetween(targetWaypoint, currentPoint);
      // double output = kp * error + kd * ((error - last_error) / segment.dt - segment.vel) + (kv * segment.vel + ka * segment.acc);
      double output = kp * error;
      return output;
  };

};