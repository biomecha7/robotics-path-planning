#include "PRMPlanner.h"
#include <cstdlib>
#include <ctime>
#include <random>

PRMPlanner::PRMPlanner(const Grid& grid, int numSamples, int numNeighbors)
    : grid_(grid), rows_(grid.size()), cols_(grid[0].size()), 
        numSamples_(numSamples), numNeighbors_(numNeighbors)
{
    std::srand(std::time(nullptr)); // Seed RNG
}

void PRMPlanner::sampleFreePoints() {
    std::uniform_int_distribution<int> xDist(0, cols_ - 1);
    std::uniform_int_distribution<int> yDist(0, rows_ - 1);
    std::default_random_engine rng(std::random_device{}());

    nodes_.clear();
    std::unordered_set<int> seen;   // prevent duplicate samples

    while (nodes_.size() < static_cast<size_t>(numSamples_)) {
        int x = xDist(rng);
        int y = yDist(rng);

        if (grid_[y][x] == 0) {
            int key = y * cols_ + x;
            if (seen.insert(key).second) {
                nodes_.emplace_back(x, y);
            }
        }
    }
}
