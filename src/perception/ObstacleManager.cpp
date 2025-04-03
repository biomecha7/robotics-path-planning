#include "perception/ObstacleManager.h"
#include "perception/OccupancyGrid2D.h"

namespace perception {
    void ObstacleManager::addObstacle(int x, int y) {
        obstacles_.push_back({x, y});
    }

    void ObstacleManager::removeObstacle(int x, int y) {
        obstacles_.erase(
            std::remove_if(obstacles_.begin(), obstacles_.end(),
                [x, y](const Obstacle& obs) { return obs.x == x && obs.y == y; }),
            obstacles_.end()
        );
    }

    void ObstacleManager::clear() {
        obstacles_.clear();
    }

    template<typename GridT>
    void ObstacleManager::updateGrid(GridT& grid) const {
        for (const auto& obs : obstacles_) {
            grid.setCell(obs.x, obs.y, CellState::Occupied);
        }
    }

    // Explicit instantiation for OccupancyGrid2D
    template void ObstacleManager::updateGrid<OccupancyGrid2D>(OccupancyGrid2D& grid) const;
}   // namespace perception
