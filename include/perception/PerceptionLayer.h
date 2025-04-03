#pragma once

#include "perception/OccupancyGrid2D.h"
#include "perception/ObstacleManager.h"

namespace perception {

class PerceptionLayer {
public:
    PerceptionLayer(int width, int height, double resolution);

    // Call once per planning/control cycle to update the grid
    void update();

    // Query the latest grid (read-only)
    const OccupancyGrid2D& getGrid() const;

    // Add/remove obstacles via public API
    void addObstacle(int x, int y);
    void removeObstacle(int x, int y);
    void clearObstacles();

private:
    OccupancyGrid2D grid_;
    ObstacleManager obstacleManager_;
};

}   // namespace perception
