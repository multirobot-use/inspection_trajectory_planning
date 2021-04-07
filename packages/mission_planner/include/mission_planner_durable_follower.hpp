#include "mission_planner.hpp"

class MissionPlannerDurableFollower : public MissionPlanner {
    public:
        MissionPlannerDurableFollower(parameters params);
        ~MissionPlannerDurableFollower();
    
    private:
        std::vector<state> initialTrajectory(const state &_state);
        Eigen::Vector3d pointOnCircle(const Eigen::Vector3d &_point);
        void optimalTrajectory(const std::vector<state> &initial_trajectory);

};