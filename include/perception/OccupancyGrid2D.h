#pragma once
#include <vector>

namespace perception {
    // Possible state of each cell in the occupancy grid
    enum class CellState {
        Unknown,    // No information yet
        Free,       // Known to be free
        Occupied    // Known to contain an obstacle
    };

    class OccupancyGrid2D {
    public:
        OccupancyGrid2D(int width, int height, double resolution);

        void setCell(int x, int y, CellState state);
        CellState getCell(int x, int y) const;
        bool isOccupied(int x, int y) const;

        void reset();   // Resets all cells to Unknown

        int getWidth() const { return width_; }
        int getHeight() const { return height_; }
        double getResolution() const { return resolution_; }

    private:
        int width_;     // Number of columns
        int height_;    // Number of rows
        double resolution_; // Meters per grid cell

        std::vector<std::vector<CellState>> grid_; // 2D matrix of cell states
    };

}   // namespace perception
