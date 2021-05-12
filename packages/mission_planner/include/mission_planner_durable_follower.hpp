#include "mission_planner.hpp"

class MissionPlannerDurableFollower : public MissionPlanner {
    public:
        MissionPlannerDurableFollower(parameters params);
        ~MissionPlannerDurableFollower();
    private:
        float formation_angle_ = 0.78; // radians
        std::vector<state> initialTrajectory(const state &_state) override;
        bool checks();
        std::vector<state> initialTrajectoryToInspect(const state &initial_pose) override;
};