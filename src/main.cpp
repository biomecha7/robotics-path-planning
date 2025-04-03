#include <iostream>
#include "CSpaceBuilder.h"
#include "planner/AStarPlanner.h"
#include "planner/PRMPlanner.h"
#include "planner/RRTPlanner.h"
#include "GridVisualizer.h"
#include "TrajectoryGenerator.h"
#include "behavior/docking/DockingMission.h"
#include <thread>
#include "planner/PlannerInterface.h"

constexpr int GRID_SIZE = 100;
constexpr double ROBOT_RADIUS = 2.0;

// Factory function to create the right planner
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
    if (argc != 6) {
        std::cerr << "Usage: " << argv[0] << " <start_x> <start_y> <goal_x> <goal_y> <planner_flag>\n";
        return 1;
    }

    std::pair<int, int> start = {std::stoi(argv[1]), std::stoi(argv[2])};
    std::pair<int, int> goal  = {std::stoi(argv[3]), std::stoi(argv[4])};
    std::string plannerFlag = argv[5];

    // Build Configuration Space
    CSpaceBuilder cspace(GRID_SIZE, ROBOT_RADIUS);
    cspace.addObstacle({20, 20, 30, 30});
    cspace.addObstacle({50, 50, 55, 60});
    cspace.addObstacle({70, 10, 90, 25});
    cspace.buildConfigurationSpace();
    Grid grid = cspace.getGrid();


    // Mission: Docking
    auto planner = createPlanner(plannerFlag, grid);
    if (!planner) {
        std::cerr << "Unknown planner flag: " << plannerFlag << "\n";
        return 1;
    }

    std::unique_ptr<Behavior> mission = std::make_unique<DockingMission>(grid, *planner, start, goal);

    mission->initialize();

    while (!mission->isComplete()) {
        mission->update();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "[Main] Mission '" << mission->name() << "' completed.\n";
 
    return 0;
}
