#include "r24/robot24.cpp"
#include "GoalPosition.cpp"
class AI24{
  public:
    // AI methods
    void trackObjectsForCurrentFrame(Robot24 *robot, vision *camera, std::vector<GoalPosition> &goals, int targetID = -1);
    int findGoalID(std::vector<GoalPosition> &goals);
    int detectionAndStrafePhase(Robot24 *robot, vision *camera, float *horizontalDistance, int matchStartTime);
    float getDistanceFromArea(int area);
    float getDistanceFromWidth(int width);
    void runAI(Robot24 *robot, int matchStartTime);
    GoalPosition* getGoalFromID(std::vector<GoalPosition> &goals, int targetID);
};