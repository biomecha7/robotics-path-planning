#include <iostream>
#include "CSpaceBuilder.h"
#include "AStarPlanner.h"
#include "PRMPlanner.h"
#include "GridVisualizer.h"

constexpr int GRID_SIZE = 100;
constexpr double ROBOT_RADIUS = 2.0;

int main(int argc, char* argv[]) {
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " <start_x> <start_y> <goal_x> <goal_y>\n";
        return 1;
    }

    std::pair<int, int> start = {std::stoi(argv[1]), std::stoi(argv[2])};
    std::pair<int, int> goal  = {std::stoi(argv[3]), std::stoi(argv[4])};

    // ----------------------------
    // 1. Configuration Space
    // ----------------------------
    CSpaceBuilder cspace(GRID_SIZE, ROBOT_RADIUS);

    cspace.addObstacle({20, 20, 30, 30});
    cspace.addObstacle({50, 50, 55, 60});
    cspace.addObstacle({70, 10, 90, 25});

    cspace.buildConfigurationSpace();
    Grid grid = cspace.getGrid();

    // ----------------------------
    // 2. A* Planning + Animation
    // ----------------------------
    AStarPlanner planner(grid);
    auto path = planner.search(start, goal);

    if (path.empty()) {
        std::cout << "A* path not found.\n";
    } else {
        std::cout << "A* path found with " << path.size() << " waypoints.\n";

        for (size_t i = 1; i <= path.size(); ++i) {
            GridVisualizer frameVisualizer(grid);
            frameVisualizer.setStartGoal(start, goal);
            frameVisualizer.overlayPath({path.begin(), path.begin() + i});
            frameVisualizer.saveToImage("img/frame_" + std::to_string(i) + ".png");
        }

        GridVisualizer finalVisualizer(grid);
        finalVisualizer.setStartGoal(start, goal);
        finalVisualizer.overlayPath(path);
        finalVisualizer.saveToImage("img/config_space_final.png");
    }

    // ----------------------------
    // 3. PRM Sampling Visualization
    // ----------------------------
    PRMPlanner prm(grid, 500, 10);  // Reduced K=10 to prevent over-connecting
    prm.sampleFreePoints();

    GridVisualizer sampleVisualizer(grid);
    sampleVisualizer.setStartGoal(start, goal);
    sampleVisualizer.overlayNodes(prm.getSampledNodes());
    sampleVisualizer.saveToImage("img/prm_samples.png");

    // ----------------------------
    // 4. PRM Roadmap Visualization
    // ----------------------------
    prm.buildRoadmap();

    GridVisualizer roadmapVisualizer(grid);
    roadmapVisualizer.setStartGoal(start, goal);
    roadmapVisualizer.overlayNodes(prm.getSampledNodes());
    roadmapVisualizer.overlayEdges(prm.getEdges());
    roadmapVisualizer.saveToImage("img/prm_roadmap.png");

    // ---------------------------
    // 5. PRM Final Path Visualization
    // ---------------------------
    auto prmPath = prm.findPath(start, goal);
    if (prmPath.empty()) {
        std::cout << "PRM path NOT found.\n";
    } else {
        std::cout << "PRM path FOUND with " << prmPath.size() << " steps.\n";
    }

    GridVisualizer prmResult(grid);
    prmResult.setStartGoal(start, goal);
    prmResult.overlayPath(prmPath);
    prmResult.overlayNodes(prm.getSampledNodes());
    prmResult.overlayEdges(prm.getEdges());
    prmResult.saveToImage("img/prm_path.png");

    // ----------------------------
    // 6. PRM Smoothed Path Visualization
    // ----------------------------
    auto prmSmoothedPath = prm.smoothPath(prmPath);

    GridVisualizer smoothVisualizer(grid);
    smoothVisualizer.setStartGoal(start, goal);
    smoothVisualizer.overlayPath(prmSmoothedPath);
    smoothVisualizer.overlayNodes(prm.getSampledNodes());
    smoothVisualizer.overlayEdges(prm.getEdges());
    smoothVisualizer.saveToImage("img/prm_path_smoothed.png");

    // ----------------------------
    // 7. Auto-open images
    // ----------------------------
    #ifdef __APPLE__
        system("open img/config_space_final.png");
        system("open img/prm_samples.png");
        system("open img/prm_roadmap.png");
        system("open img/prm_path.png");
        system("open img/prm_path_smoothed.png");
    #elif __linux__
        system("xdg-open img/config_space_final.png");
        system("xdg-open img/prm_samples.png");
        system("xdg-open img/prm_roadmap.png");
    #elif _WIN32
        system("start img\\config_space_final.png");
        system("start img\\prm_samples.png");
        system("start img\\prm_roadmap.png");
    #endif

    return 0;
}
