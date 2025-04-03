#include "perception/OccupancyGrid2D.h"
#include <stdexcept>    // for std::out_of_range

namespace perception {

    OccupancyGrid2D::OccupancyGrid2D(int width, int height, double resolution)
        : width_(width), height_(height), resolution_(resolution)
    {
        // Initialize the grid with all cells set to Unknown
        grid_ = std::vector<std::vector<CellState>>(height_, std::vector<CellState>(width_, CellState::Unknown));
    }

    void OccupancyGrid2D::setCell(int x, int y, CellState state) {
        if (x >= 0 && x < width_ && y >= 0 && y < height_) {
            grid_[y][x] = state;
        } else {
            throw std::out_of_range("setCell: Coordinates out of bounds");
        }
    }

    CellState OccupancyGrid2D::getCell(int x, int y) const {
        if (x >= 0 && x < width_ && y >= 0 && y < height_) {
            return grid_[x][y];
        } else {
            throw std::out_of_range("getCell: Coordinates out of bounds");
        }
    }

    bool OccupancyGrid2D::isOccupied(int x, int y) const {
        return getCell(x, y) == CellState::Occupied;
    }

    void OccupancyGrid2D::reset() {
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                grid_[y][x] = CellState::Unknown;
            }
        }
    }

}   // namespace perception
