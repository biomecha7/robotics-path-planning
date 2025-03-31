#pragma once 

#include <vector>
#include <utility>
#include <unordered_map>
#include <unordered_set>

using Grid = std::vector<std::vector<int>>;

class PRMPlanner {
public:
    PRMPlanner(const Grid& grid, int numSamples, int numNeighbors);

    std::vector<std::pair<int, int>> findPath(
        std::pair<int, int> start,
        std::pair<int, int> goal
    );

private:
    const Grid& grid_;
    int rows_, cols_;
    int numSamples_;
    int numNeighbors_;

    std::vector<std::pair<int, int>> nodes_;    // sampled free points
};
