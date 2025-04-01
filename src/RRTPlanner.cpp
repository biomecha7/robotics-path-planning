#include "RRTPlanner.h"
#include <cmath>
#include <random>
#include <limits>
#include <cstdlib>

RRTPlanner::RRTPlanner(const Grid& grid, int maxIterations, double stepSize) 
    : grid_(grid), rows_(grid.size()), cols_(grid[0].size()),
      maxIterations_(maxIterations), stepSize_(stepSize)
{
    std::srand(std::time(nullptr));     // seed RNG
}

/**
 * Concept: Rampling in RRT
 * The sample() function chooses a random (x, y) location within the grid bounds. For now, this is uniform sampling without goal bias.
 *
 */
Point RRTPlanner::sample() {
    int x = std::rand() % cols_;
    int y = std::rand() % rows_;
    return {x, y};
}
