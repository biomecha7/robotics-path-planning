#include "perception/GridConversion.h"

namespace perception {

Grid convertToIntGrid(const OccupancyGrid2D& og) {
    int height = og.getHeight();
    int width = og.getWidth();

    Grid result(height, std::vector<int>(width, 0));

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            auto cell = og.getCell(x, y);
            if (cell == CellState::Occupied) {
                result[y][x] = 1;
            }
            // Free and Uknown are both left as 0 for now
        }
    }

    return result;
}

}
