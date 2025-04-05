#include <iostream>
#include <thread>
#include "core/CSpaceBuilder.h"
#include "planner/AStarPlanner.h"
#include "planner/PRMPlanner.h"
#include "planner/RRTPlanner.h"
#include "simulation/GridVisualizer.h"
#include "trajectory/TrajectoryGenerator.h"
#include "behavior/docking/DockingMission.h"
#include "planner/PlannerInterface.h"
#include "perception/PerceptionLayer.h"
#include "perception/GridConversion.h"
#include "local_planner/SimpleLocalPlanner.h"
#include "simulation/SimulationRunner.h"

using namespace perception;
using namespace local_planner;

constexpr int GRID_SIZE = 100;
constexpr double ROBOT_RADIUS = 2.0;

std::unique_ptr<PlannerInterface> createPlanner(const std::string& flag, const Grid& grid) {
    if (flag == "astar") {
        return std::make_unique<AStarPlanner>(grid);
    } else if (flag == "prm") {
        return std::make_unique<PRMPlanner>(grid, 500, 10);
    } else if (flag == "rrt") {
        return std::make_unique<RRTPlanner>(grid, 2000, 7.0);
    } else {
        return nullptr;
    }
}

int main(int argc, char* argv[]) {
    if (argc < 6 || argc > 7) {
        std::cerr << "Usage: " << argv[0] << " <start_x> <start_y> <goal_x> <goal_y> <planner_flag> [sim]\n";
        return 1;
    }

    std::pair<int, int> start = {std::stoi(argv[1]), std::stoi(argv[2])};
    std::pair<int, int> goal  = {std::stoi(argv[3]), std::stoi(argv[4])};
    std::string plannerFlag = argv[5];
    bool runSimulation = (argc == 7 && std::string(argv[6]) == "sim");

    // Build Configuration Space
    CSpaceBuilder cspace(GRID_SIZE, ROBOT_RADIUS);
    cspace.addObstacle({20, 20, 30, 30});
    cspace.addObstacle({50, 50, 55, 60});
    cspace.addObstacle({70, 10, 90, 25});
    cspace.addObstacle({20, 5, 30, 45});
    cspace.addObstacle({30, 85, 80, 85});
    cspace.buildConfigurationSpace();
    Grid grid = cspace.getGrid();

    // Create global planner
    auto planner = createPlanner(plannerFlag, grid);
    if (!planner) {
        std::cerr << "Unknown planner flag: " << plannerFlag << "\n";
        return 1;
    }

    // Plan path
    auto path = planner->plan(start, goal);

    // Set up Perception + Local Planner
    PerceptionLayer perceptionLayer(GRID_SIZE, GRID_SIZE, 0.1);
    SimpleLocalPlanner localPlanner(perceptionLayer, 10);
    localPlanner.setGlobalPath(path);

    // Create mission
    std::unique_ptr<Behavior> mission = std::make_unique<DockingMission>(grid, *planner, start, goal);
    mission->initialize();

    if (runSimulation) {
        std::cout << "[Main] Running simulation mode...\n";
        simulation::SimulationRunner simRunner(
            *mission, perceptionLayer, localPlanner, path, start, goal, GRID_SIZE, GRID_SIZE);
        simRunner.run(200, "img/");
    } else {
        std::cout << "[Main] Running live mission update mode...\n";
        while (!mission->isComplete()) {
            mission->update();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::cout << "[Main] Mission '" << mission->name() << "' completed.\n";
    }

    return 0;
}
