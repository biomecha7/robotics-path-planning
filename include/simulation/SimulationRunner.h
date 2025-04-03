#pragma once

#include <string>
#include "behavior/Behavior.h"
#include "perception/PerceptionLayer.h"
#include "local_planner/SimpleLocalPlanner.h"
#include "simulation/FrameRenderer.h"
#include "GridVisualizer.h"

namespace simulation {

class SimulationRunner {
public:
    SimulationRunner(Behavior& mission,
                     perception::PerceptionLayer& perception,
                     local_planner::SimpleLocalPlanner& localPlanner,
                     const std::vector<std::pair<int, int>>& globalPath,
                     const std::pair<int, int>& start,
                     const std::pair<int, int>& goal,
                     int gridWidth,
                     int gridHeight);

    void run(int maxSteps = 200, const std::string& outputDir = "img/");

private:
    Behavior& mission_;
    perception::PerceptionLayer& perception_;
    local_planner::SimpleLocalPlanner& localPlanner_;
    std::vector<std::pair<int, int>> globalPath_;
    std::pair<int, int> goal_;
    std::pair<int, int> currentPose_;
    FrameRenderer renderer_;
    int gridWidth_, gridHeight_;

    void injectRandomObstacle(int probabilityPercent);
};

} // namespace simulation

