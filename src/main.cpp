#include <iostream>
#include "core/CSpaceBuilder.h"
#include "planner/AStarPlanner.h"
#include "planner/PRMPlanner.h"
#include "planner/RRTPlanner.h"
#include "simulation/GridVisualizer.h"
#include "trajectory/TrajectoryGenerator.h"
#include "behavior/docking/DockingMission.h"
#include <thread>
#include "planner/PlannerInterface.h"
#include "perception/PerceptionLayer.h"
#include "perception/GridConversion.h"
#include "local_planner/SimpleLocalPlanner.h"

using namespace perception;
using namespace local_planner;

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

void testPerceptionAndVisualization() {
    using namespace perception;

    PerceptionLayer perception(100, 100, 0.1);

    // Add some test obstacles
    for (int i = 40; i <= 60; ++i) {
        perception.addObstacle(i, 50); // Horizontal wall
    }
    perception.update();

    // Convert to int grid
    Grid grid = convertToIntGrid(perception.getGrid());

    // Visualize
    GridVisualizer viz(grid);
    viz.saveToImage("img/perception_output.png");
}

void testLocalPlanner() {
    // Create a clean, empty grid
    Grid grid(100, std::vector<int>(100, 0));

    // Global path planner
    PRMPlanner prm(grid, 200, 10);
    std::pair<int, int> start = {10, 10};
    std::pair<int, int> goal  = {90, 90};
    auto path = prm.plan(start, goal);

    // Set up perception
    PerceptionLayer perception(100, 100, 0.1);
    for (int x = 30; x <= 70; ++x) {
        perception.addObstacle(x, 50); // horizontal wall
    }
    perception.update();

    // Local planner
    SimpleLocalPlanner localPlanner(perception, 10);
    localPlanner.setGlobalPath(path);
    auto result = localPlanner.plan(start.first, start.second);

    // Visualize
    Grid gridForViz = convertToIntGrid(perception.getGrid());
    GridVisualizer viz(gridForViz);
    viz.setStartGoal(start, goal);
    viz.overlayPath(path);               // full global path
    viz.overlayPath(result.localPath);   // local path segment
    viz.saveToImage("img/local_planner_output.png");
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
    cspace.addObstacle({20, 5, 30, 45});
    cspace.addObstacle({30, 85, 80, 85});
    cspace.buildConfigurationSpace();
    Grid grid = cspace.getGrid();

    testPerceptionAndVisualization();
    testLocalPlanner();

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
