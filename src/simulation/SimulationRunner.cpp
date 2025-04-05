#include "simulation/SimulationRunner.h"
#include "perception/GridConversion.h"
#include <random>
#include <filesystem>
#include <iostream>

namespace simulation {

namespace fs = std::filesystem;

SimulationRunner::SimulationRunner(Behavior& mission,
                                   perception::PerceptionLayer& perception,
                                   local_planner::SimpleLocalPlanner& localPlanner,
                                   const std::vector<std::pair<int, int>>& globalPath,
                                   const std::pair<int, int>& start,
                                   const std::pair<int, int>& goal,
                                   int gridWidth,
                                   int gridHeight)
    : mission_(mission),
      perception_(perception),
      localPlanner_(localPlanner),
      globalPath_(globalPath),
      goal_(goal),
      currentPose_(start),
      renderer_(gridWidth, gridHeight),
      gridWidth_(gridWidth),
      gridHeight_(gridHeight) {}

void SimulationRunner::run(int maxSteps, const std::string& outputDir) {
    fs::create_directories(outputDir);
    int step = 0;

    while (!mission_.isComplete() && step < maxSteps) {
        // Inject obstacle randomly (e.g. 2% chance per step)
        injectRandomObstacle(2);

        perception_.update();
        auto result = localPlanner_.plan(currentPose_.first, currentPose_.second);

        std::string debugText;

        if (result.status == local_planner::LocalPlannerStatus::Blocked) {
            debugText = "Status: BLOCKED";
        } else if (result.status == local_planner::LocalPlannerStatus::GoalReached) {
            debugText = "Status: GOAL REACHED";
        } else {
            debugText = "Status: FOLLOWING PATH";
        }

        if (!result.localPath.empty()) {
            currentPose_ = result.localPath.front();
        }

        Grid renderGrid = perception::convertToIntGrid(perception_.getGrid());
        char filename[128];
        snprintf(filename, sizeof(filename), "%s/frame_%03d.png", outputDir.c_str(), step);

        renderer_.renderFrame(renderGrid,
                              currentPose_,
                              globalPath_,
                              result.localPath,
                              goal_,
                              step,
                              debugText,
                              filename);

        mission_.update();
        step++;
    }

    // Optional: generate video automatically
    std::string videoCmd = "ffmpeg -y -framerate 10 -i " + outputDir + "/frame_%03d.png -c:v libx264 -pix_fmt yuv420p " + outputDir + "/docking_simulation.mp4";
    std::cout << "[Simulation] Creating video: " << videoCmd << std::endl;
    int ffmpegResult = std::system(videoCmd.c_str());

    if (ffmpegResult == 0) {
#ifdef __APPLE__
        std::system(("open " + outputDir + "/docking_simulation.mp4").c_str());
#elif __linux__
        std::system(("xdg-open " + outputDir + "/docking_simulation.mp4").c_str());
#elif _WIN32
        std::system(("start " + outputDir + "/docking_simulation.mp4").c_str());
#else
        std::cout << "Video saved. Please open manually.\n";
#endif
    } else {
        std::cerr << "[Simulation] Failed to generate video using ffmpeg.\n";
    }

    std::cout << "[Simulation] Completed in " << step << " steps.\n";
}

void SimulationRunner::injectRandomObstacle(int probabilityPercent) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_int_distribution<> prob(0, 99);
    std::uniform_int_distribution<> xDist(0, gridWidth_ - 1);
    std::uniform_int_distribution<> yDist(0, gridHeight_ - 1);

    if (prob(gen) < probabilityPercent) {
        int x = xDist(gen);
        int y = yDist(gen);
        perception_.addObstacle(x, y);
        std::cout << "[Simulation] Injected obstacle at (" << x << ", " << y << ")\n";
    }
}

} // namespace simulation

