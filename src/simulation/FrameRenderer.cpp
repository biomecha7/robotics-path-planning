#include "simulation/FrameRenderer.h"

namespace simulation {

using namespace cv;

FrameRenderer::FrameRenderer(int width, int height, int cellSize)
    : width_(width), height_(height), cellSize_(cellSize) {}

void FrameRenderer::renderFrame(const Grid& grid,
                                const std::pair<int, int>& robotPose,
                                const std::vector<std::pair<int, int>>& globalPath,
                                const std::vector<std::pair<int, int>>& localPath,
                                const std::pair<int, int>& goal,
                                int step,
                                const std::string& debugText,
                                const std::string& filename) {
    int rows = grid.size();
    int cols = grid[0].size();

    Mat image(rows * cellSize_, cols * cellSize_, CV_8UC3, Scalar(255, 255, 255));

    // Draw grid cells
    for (int y = 0; y < rows; ++y) {
        for (int x = 0; x < cols; ++x) {
            Rect cell(x * cellSize_, y * cellSize_, cellSize_, cellSize_);
            if (grid[y][x] == 1) {
                rectangle(image, cell, Scalar(0, 0, 0), FILLED); // Obstacle = black
            }
        }
    }

    // Draw global path (light red)
    for (size_t i = 1; i < globalPath.size(); ++i) {
        Point p1(globalPath[i-1].first * cellSize_ + cellSize_ / 2,
                 globalPath[i-1].second * cellSize_ + cellSize_ / 2);
        Point p2(globalPath[i].first * cellSize_ + cellSize_ / 2,
                 globalPath[i].second * cellSize_ + cellSize_ / 2);
        line(image, p1, p2, Scalar(100, 100, 255), 1);
    }

    // Draw local path (blue, thicker)
    for (size_t i = 1; i < localPath.size(); ++i) {
        Point p1(localPath[i-1].first * cellSize_ + cellSize_ / 2,
                 localPath[i-1].second * cellSize_ + cellSize_ / 2);
        Point p2(localPath[i].first * cellSize_ + cellSize_ / 2,
                 localPath[i].second * cellSize_ + cellSize_ / 2);
        line(image, p1, p2, Scalar(255, 0, 0), 2);
    }

    // Draw robot position (green)
    circle(image,
           Point(robotPose.first * cellSize_ + cellSize_ / 2,
                 robotPose.second * cellSize_ + cellSize_ / 2),
           cellSize_, Scalar(0, 255, 0), FILLED);

    // Draw goal (red circle)
    circle(image,
           Point(goal.first * cellSize_ + cellSize_ / 2,
                 goal.second * cellSize_ + cellSize_ / 2),
           cellSize_, Scalar(0, 0, 255), FILLED);

    // Add debug text and step number
    std::string stepText = "Step: " + std::to_string(step);
    putText(image, stepText, Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
    putText(image, debugText, Point(10, 40), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);

    // Save image
    imwrite(filename, image);
}

} // namespace simulation

