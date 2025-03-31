#pragma once
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

using Grid = std::vector<std::vector<int>>;

class GridVisualizer {
public:
    GridVisualizer(Grid grid);

    void overlayPath(const std::vector<std::pair<int, int>>& path);
    void setStartGoal(std::pair<int, int> start, std::pair<int, int> goal);
    void saveToImage(const std::string& filename);

    void overlayNodes(const std::vector<std::pair<int, int>>& nodes);
    void overlayEdges(const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& edges);

private:
    Grid grid_;
    std::pair<int, int> start_;
    std::pair<int, int> goal_;
    bool hasStartGoal_ = false;
    cv::Mat image_;
};
