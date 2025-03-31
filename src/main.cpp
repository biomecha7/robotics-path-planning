#include <iostream>
#include "CSpaceBuilder.h"
#include "AStarPlanner.h"
#include "PRMPlanner.h"
#include "GridVisualizer.h"

constexpr int GRID_SIZE = 100;
constexpr double ROBOT_RADIUS = 2.0;

int main(int argc, char* argv[]) {
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << "<start_x> <start_y> <goal_x> <goal_y>\n";
        return 1;
    }

    std::pair<int, int> start = {std::stoi(argv[1]), std::stoi(argv[2])};
    std::pair<int, int> goal = {std::stoi(argv[3]), std::stoi(argv[4])};

    // 1. Build Configuration Space
    CSpaceBuilder cspace(GRID_SIZE, ROBOT_RADIUS);

    cspace.addObstacle({20, 20, 30, 30});
    cspace.addObstacle({50, 50, 55, 60});
    cspace.addObstacle({70, 10, 90, 25});

    cspace.buildConfigurationSpace();
    Grid grid = cspace.getGrid();

    // 2. A* Path Planning 
    AStarPlanner planner(grid);

    auto path = planner.search(start, goal);

    if (path.empty()) {
        std::cout << "No path found.\n";
    } else {
        std::cout << "Path found with " << path.size() << " waypoints.\n";
        for (const auto& [x, y] : path) {
            std::cout << "(" << x << ", " << y << ") ";
        }
        std::cout << '\n';
    }

    // 3. Animate Path Drawing Frame-by-Frame
    for (size_t i = 1; i <= path.size(); ++i) {
        GridVisualizer frameVisualizer(grid); // fresh copy of grid
        frameVisualizer.setStartGoal(start, goal);
        frameVisualizer.overlayPath(std::vector<std::pair<int, int>>(path.begin(), path.begin() + i));
        frameVisualizer.saveToImage("img/frame_" + std::to_string(i) + ".png");
    }

    std::cout << "Saved animation frames in img/frame_#.png\n";

    // Optional: save the final result too
    GridVisualizer visualizer(grid);
    visualizer.setStartGoal(start, goal);
    visualizer.overlayPath(path);
    visualizer.saveToImage("img/config_space_final.png");

    // Probabilistic Roadmap Planner (PRM)
    PRMPlanner prm(grid, 500, 100);
    prm.sampleFreePoints();

    visualizer.overlayNodes(prm.getSampledNodes());
    visualizer.saveToImage("img/prm_samples.png");

    std::cout << "Saved final image: img/config_space_final.png\n";
    std::cout << "Saved final image: img/prm_samples.png\n";

    #ifdef __APPLE__
        system("open img/config_space_final.png");
        system("open img/prm_samples.png");
    #elif __linux__
        system("xdg-open img/config_space_final.png");
        system("xdg-open img/prm_samples.png");
    #elif _WIN32
        system("start img\\config_space_final.png");
        system("start img/prm_samples.png");
    #endif

    return 0;
}
