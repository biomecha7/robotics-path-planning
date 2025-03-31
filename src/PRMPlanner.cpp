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
