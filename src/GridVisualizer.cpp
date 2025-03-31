#include "GridVisualizer.h"
#include <opencv2/opencv.hpp>

GridVisualizer::GridVisualizer(Grid grid) : grid_(std::move(grid)) {}

void GridVisualizer::setStartGoal(std::pair<int, int> start, std::pair<int, int> goal) {
    start_ = start;
    goal_ = goal;
    hasStartGoal_ = true;
}

void GridVisualizer::overlayPath(const std::vector<std::pair<int, int>>& path) {
    for (const auto& [x, y] : path) {
        if (y >= 0 && y < grid_.size() && x >= 0 && x < grid_[0].size()) {
            grid_[y][x] = 2;  // Mark path cells with a special value
        }
    }
}

void GridVisualizer::saveToImage(const std::string& filename) {
    int cellSize = 5;
    int height = grid_.size();
    int width = grid_[0].size();

    image_ = cv::Mat(height * cellSize, width * cellSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            cv::Rect cell(x * cellSize, y * cellSize, cellSize, cellSize);
            if (grid_[y][x] == 1) {
                cv::rectangle(image_, cell, cv::Scalar(0, 0, 0), cv::FILLED);  // Obstacle = black
            } else if (grid_[y][x] == 2) {
                cv::rectangle(image_, cell, cv::Scalar(0, 0, 255), cv::FILLED);  // Path = red
            } else if (grid_[y][x] == 3) {
                cv::rectangle(image_, cell, cv::Scalar(200, 200, 200), cv::FILLED); // light gray or white dots
            }
        }
    }

        if (hasStartGoal_) {
        // Draw start = green
        cv::circle(image_,
            cv::Point(start_.first * cellSize + cellSize / 2, start_.second * cellSize + cellSize / 2),
            cellSize, cv::Scalar(0, 255, 0), cv::FILLED);
        // Draw goal = blue
        cv::circle(image_,
            cv::Point(goal_.first * cellSize + cellSize / 2, goal_.second * cellSize + cellSize / 2),
            cellSize, cv::Scalar(255, 0, 0), cv::FILLED);
    }

    cv::imwrite(filename, image_);
}

void GridVisualizer::overlayNodes(const std::vector<std::pair<int, int>>& nodes) {
    for (const auto& [x, y] : nodes) {
        if (y >= 0 && y < grid_.size() && x >= 0 && x < grid_[0].size()) {
            grid_[y][x] = 3; // new marker for sampled PRM nodes
        }
    }
}

void GridVisualizer::overlayEdges(const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& edges) {
    int cellSize = 5;
    for (const auto& [a, b] : edges) {
        cv::line(
            image_,
            cv::Point(a.first * cellSize + cellSize / 2, a.second * cellSize + cellSize / 2),
            cv::Point(b.first * cellSize + cellSize / 2, b.second * cellSize + cellSize / 2),
            cv::Scalar(200, 200, 200), 
            1 // light gray line
        );
    }
}
