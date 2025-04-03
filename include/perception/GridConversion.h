#pragma once 
#include "perception/OccupancyGrid2D.h"
#include "simulation/GridVisualizer.h" // for Grid alias

namespace perception {

    // Converts an OccupancyGrid2D to a GridVisualizer-compatible Grid
    Grid convertToIntGrid(const OccupancyGrid2D& grid);
}
