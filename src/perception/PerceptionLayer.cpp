#include "perception/PerceptionLayer.h"

namespace perception {

    PerceptionLayer::PerceptionLayer(int width, int height, double resolution)
        : grid_(width, height, resolution) {}

    void PerceptionLayer::update() {
        grid_.reset();
        obstacleManager_.updateGrid(grid_);
    }

    const OccupancyGrid2D& PerceptionLayer::getGrid() const {
        return grid_;
    }

    void PerceptionLayer::addObstacle(int x, int y) {
        obstacleManager_.addObstacle(x, y);
    }

    void PerceptionLayer::removeObstacle(int x, int y) {
        obstacleManager_.removeObstacle(x, y);
    }

    void PerceptionLayer::clearObstacles() {
        obstacleManager_.clear();
    }

}   // namespace perception
