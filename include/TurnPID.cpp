#import "waypoint.cpp"

class TurnPID {       // The class
  private:
    double kp;
  public :             // Access specifier

  TurnPID(double p){
    kp = p;
  }

  double calculate(Waypoint targetWaypoint, Waypoint currentPoint) {
      // double error = targetWaypoint.theta - currentPoint.theta;
      // double xDiff = targetWaypoint.x - currentPoint.x;
      // double yDiff = targetWaypoint.y - currentPoint.y;
      // double error = atan2(-xDiff, -yDiff);
      // double output = kp * error + kd * ((error - last_error) / segment.dt - segment.vel) + (kv * segment.vel + ka * segment.acc);
      // double output = kp * error;

      return 0;
  };

};