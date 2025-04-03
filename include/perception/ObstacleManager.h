#pragma once
#include <vector>

namespace perception {
    struct Obstacle {
        int x;
        int y;
    };

    class ObstacleManager {
    public:
        void addObstacle(int x, int y);
        void removeObstacle(int x, int y);
        void clear();

        // Apply all current obstacles to the occupancy grid
        template<typename GridT>
        void updateGrid(GridT& grid) const;

    private:
        std::vector<Obstacle> obstacles_;

    };

}   // namespace perception
