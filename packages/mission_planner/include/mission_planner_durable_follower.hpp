#include "mission_planner.hpp"

class MissionPlannerDurableFollower : public MissionPlanner {
    public:
        MissionPlannerDurableFollower(parameters params);
        ~MissionPlannerDurableFollower();
    private:
        float formation_angle_; // radians
        std::vector<state> initialTrajectory(const state &_state);
        void optimalTrajectory(const std::vector<state> &initial_trajectory);

};