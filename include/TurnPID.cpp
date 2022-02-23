#import "waypoint.cpp"

class TurnPID {       // The class
  private:
    double kp;
  public :             // Access specifier

  TurnPID(double p){
    kp = p;
  }

  double calculate(Waypoint targetWaypoint, Waypoint currentPoint) {
      double error = targetWaypoint.theta - targetWaypoint.theta;
      // double output = kp * error + kd * ((error - last_error) / segment.dt - segment.vel) + (kv * segment.vel + ka * segment.acc);
      double output = kp * error;
      return output;
  };

};