#pragma once 

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "GridVisualizer.h"

namespace simulation {
class FrameRenderer {
public:
    FrameRenderer(int width, int height, int cellSize = 5);

    void renderFrame(
        const Grid& grid,
        const std::pair<int, int>& robotPose,
        const std::vector<std::pair<int, int>>& globalPath,
        const std::vector<std::pair<int, int>>& localPath,
        const std::pair<int, int>& goal,
        int step,
        const std::string& debugText,
        const std::string& filename
    );

private:
    int width_;
    int height_;
    int cellSize_;
    
};

} // namespace simulation 
