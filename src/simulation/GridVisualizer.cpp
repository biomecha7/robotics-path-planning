#include "simulation/GridVisualizer.h"
#include <opencv2/opencv.hpp>

GridVisualizer::GridVisualizer(Grid grid) : grid_(std::move(grid)) {}

void GridVisualizer::setStartGoal(std::pair<int, int> start, std::pair<int, int> goal) {
    start_ = start;
    goal_ = goal;
    hasStartGoal_ = true;
}

void GridVisualizer::overlayPath(const std::vector<std::pair<int, int>>& path) {
    path_ = path;
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
            }
        }
    }

    if (hasStartGoal_) {
        // Start = green
        cv::circle(image_,
            cv::Point(start_.first * cellSize + cellSize / 2, start_.second * cellSize + cellSize / 2),
            cellSize, cv::Scalar(0, 255, 0), cv::FILLED);
        // Goal = blue
        cv::circle(image_,
            cv::Point(goal_.first * cellSize + cellSize / 2, goal_.second * cellSize + cellSize / 2),
            cellSize, cv::Scalar(255, 0, 0), cv::FILLED);
    }

    overlayEdges(edges_);

    for (const auto& [x, y] : path_) {
        cv::circle(image_,
            cv::Point(x * cellSize + cellSize / 2, y * cellSize + cellSize / 2),
            1, cv::Scalar(0, 0, 255), cv::FILLED);
    }

    for (size_t i = 1; i < path_.size(); ++i) {
        const auto& [x1, y1] = path_[i - 1];
        const auto& [x2, y2] = path_[i];
        cv::line(image_,
            cv::Point(x1 * cellSize + cellSize / 2, y1 * cellSize + cellSize / 2),
            cv::Point(x2 * cellSize + cellSize / 2, y2 * cellSize + cellSize / 2),
            cv::Scalar(0, 0, 255), 2);
    }

    // Save image
    cv::imwrite(filename, image_);

    // Open image automatically
#ifdef __APPLE__
    std::string command = "open " + filename;
#elif __linux__
    std::string command = "xdg-open " + filename;
#elif _WIN32
    std::string command = "start " + filename;
#endif

#ifdef _WIN32
    system(command.c_str());
#else
    int ret = std::system(command.c_str());
    if (ret != 0) {
        std::cerr << "Failed to open image: " << filename << std::endl;
    }
#endif
}

void GridVisualizer::overlayNodes(const std::vector<std::pair<int, int>>& nodes) {
    for (const auto& [x, y] : nodes) {
        if (y >= 0 && y < grid_.size() && x >= 0 && x < grid_[0].size()) {
            grid_[y][x] = 3; // new marker for sampled PRM nodes
        }
    }
}

void GridVisualizer::overlayEdges(const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& edges) {
    edges_ = edges;
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
