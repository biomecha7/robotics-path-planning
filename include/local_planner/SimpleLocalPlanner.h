#pragma once 

#include "local_planner/LocalPlannerInterface.h"
#include "perception/PerceptionLayer.h"

namespace local_planner {

class SimpleLocalPlanner : public LocalPlannerInterface {
public:
    SimpleLocalPlanner(perception::PerceptionLayer& perceptionLayer, int maxStepLookahead = 10);

    void setGlobalPath(const std::vector<std::pair<int, int>>& path) override;
    LocalPlannerResult plan(int currentX, int currentY) override;

private:
    perception::PerceptionLayer& perceptionLayer_;
    std::vector<std::pair<int, int>> globalPath_;
    size_t currentPathIndex_;
    int maxStepLookahead_;
};

}   // namespace local_planner
