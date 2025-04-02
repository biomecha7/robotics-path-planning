#include <iostream>
#include "CSpaceBuilder.h"
#include "planner/AStarPlanner.h"
#include "planner/PRMPlanner.h"
#include "planner/RRTPlanner.h"
#include "GridVisualizer.h"
#include "TrajectoryGenerator.h"

constexpr int GRID_SIZE = 100;
constexpr double ROBOT_RADIUS = 2.0;

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

    bool ranAstar = false, ranPrm = false, ranRrt = false;

    if (plannerFlag == "astar" || plannerFlag == "all") {
        ranAstar = true;
        AStarPlanner planner(grid);
        auto path = planner.search(start, goal);

        if (!path.empty()) {
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
    }

    if (plannerFlag == "prm" || plannerFlag == "all") {
        ranPrm = true;
        PRMPlanner prm(grid, 500, 10);
        prm.sampleFreePoints();
        prm.buildRoadmap();

        auto prmPath = prm.findPath(start, goal);

        GridVisualizer sampleVisualizer(grid);
        sampleVisualizer.setStartGoal(start, goal);
        sampleVisualizer.overlayNodes(prm.getSampledNodes());
        sampleVisualizer.saveToImage("img/prm_samples.png");

        GridVisualizer roadmapVisualizer(grid);
        roadmapVisualizer.setStartGoal(start, goal);
        roadmapVisualizer.overlayNodes(prm.getSampledNodes());
        roadmapVisualizer.overlayEdges(prm.getEdges());
        roadmapVisualizer.saveToImage("img/prm_roadmap.png");

        GridVisualizer prmResult(grid);
        prmResult.setStartGoal(start, goal);
        prmResult.overlayPath(prmPath);
        prmResult.overlayNodes(prm.getSampledNodes());
        prmResult.overlayEdges(prm.getEdges());
        prmResult.saveToImage("img/prm_path.png");

        auto prmSmoothedPath = prm.smoothPath(prmPath);
        GridVisualizer smoothVisualizer(grid);
        smoothVisualizer.setStartGoal(start, goal);
        smoothVisualizer.overlayPath(prmSmoothedPath);
        smoothVisualizer.overlayNodes(prm.getSampledNodes());
        smoothVisualizer.overlayEdges(prm.getEdges());
        smoothVisualizer.saveToImage("img/prm_path_smoothed.png");

        if (!prmPath.empty()) {
            std::vector<TrajectoryGenerator::Point> prmPointPath;
            for (const auto& [x, y] : prmPath) {
                prmPointPath.emplace_back(static_cast<double>(x), static_cast<double>(y), 0.0);
            }
            auto timedPath = TrajectoryGenerator::applyTimeProfile(prmPointPath, 1.0);
            auto splinePath = TrajectoryGenerator::generateCatmullRomSpline(timedPath, 10);

            GridVisualizer visualizer(grid);
            visualizer.setStartGoal(start, goal);
            std::vector<std::pair<int, int>> roundedSplinePath;
            for (const auto& [x, y, t] : splinePath) {
                roundedSplinePath.emplace_back(static_cast<int>(std::round(x)), static_cast<int>(std::round(y)));
            }
            visualizer.overlayPath(roundedSplinePath);
            visualizer.overlayNodes(prm.getSampledNodes());
            visualizer.overlayEdges(prm.getEdges());
            visualizer.saveToImage("img/prm_smoothed_trajectory.png");
        }
    }

    if (plannerFlag == "rrt" || plannerFlag == "all") {
        ranRrt = true;
        RRTPlanner rrt(grid, 2000, 7.0);
        auto rrtPath = rrt.plan(start, goal);
        GridVisualizer rrtVisualizer(grid);
        rrtVisualizer.setStartGoal(start, goal);
        rrtVisualizer.overlayPath(rrtPath);

        std::vector<std::pair<Point, Point>> rrtEdges;
        const auto& tree = rrt.getTree();
        for (size_t i = 1; i < tree.size(); ++i) {
            rrtEdges.emplace_back(tree[i].pos, tree[tree[i].parentIdx].pos);
        }
        rrtVisualizer.overlayEdges(rrtEdges);
        rrtVisualizer.saveToImage("img/rrt_path.png");
    }

    // Auto open images
    #ifdef __APPLE__
        if (ranAstar) system("open img/config_space_final.png");
        if (ranPrm) {
            system("open img/prm_samples.png");
            system("open img/prm_roadmap.png");
            system("open img/prm_path.png");
            system("open img/prm_path_smoothed.png");
            system("open img/prm_smoothed_trajectory.png");
        }
        if (ranRrt) system("open img/rrt_path.png");
    #elif __linux__
        if (ranAstar) system("xdg-open img/config_space_final.png");
        if (ranPrm) {
            system("xdg-open img/prm_samples.png");
            system("xdg-open img/prm_roadmap.png");
        }
        if (ranRrt) system("xdg-open img/rrt_path.png");
    #elif _WIN32
        if (ranAstar) system("start img\\config_space_final.png");
        if (ranPrm) {
            system("start img\\prm_samples.png");
            system("start img\\prm_roadmap.png");
            system("start img\\prm_path.png");
            system("start img\\prm_path_smoothed.png");
            system("start img\\prm_smoothed_trajectory.png");
        }
        if (ranRrt) system("start img\\rrt_path.png");
    #endif

    return 0;
}
