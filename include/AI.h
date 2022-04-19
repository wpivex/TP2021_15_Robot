#include "BaseRobot.cpp"
#include "GoalPosition.cpp"
class AI{
  public:
    // AI methods
    void trackObjectsForCurrentFrame(BaseRobot *robot, vision *camera, std::vector<GoalPosition> &goals, int targetID = -1);
    int findGoalID(std::vector<GoalPosition> &goals);
    int detectionAndStrafePhase(BaseRobot *robot, vision *camera, float *horizontalDistance, int matchStartTime);
    float getDistanceFromArea(int area);
    float getDistanceFromWidth(int width);
    void runAI(BaseRobot *robot, int matchStartTime);
    GoalPosition* getGoalFromID(std::vector<GoalPosition> &goals, int targetID);
};