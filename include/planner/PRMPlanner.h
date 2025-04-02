#pragma once 

#include <vector>
#include <utility>
#include <unordered_map>
#include <unordered_set>

#include "planner/PlannerInterface.h"
#include "core/GridTypes.h"

// Hash function for pair<int, int> keys
struct pair_hash {
    size_t operator()(const std::pair<int, int>& p) const {
        return std::hash<int>()(p.first) ^ std::hash<int>()(p.second << 1);
    }
};

class PRMPlanner : public PlannerInterface {
public:
    PRMPlanner(const Grid& grid, int numSamples, int numNeighbors);

    std::vector<std::pair<int, int>> findPath(
        std::pair<int, int> start,
        std::pair<int, int> goal
    );

    const std::vector<std::pair<int, int>>& getSampledNodes() const;

    void sampleFreePoints();

    void buildRoadmap();

    const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& getEdges() const;

    std::vector<std::pair<int, int>> smoothPath(const std::vector<std::pair<int, int>>& path);

    void initialize() override {
        sampleFreePoints();
        buildRoadmap();
    }

    std::vector<std::pair<int, int>> plan(
        const std::pair<int, int>& start,
        const std::pair<int, int>& goal) override {
        return findPath(start, goal);
    }

private:
    const Grid& grid_;
    int rows_, cols_;
    int numSamples_;
    int numNeighbors_;

    std::vector<std::pair<int, int>> nodes_;    // sampled free points

    std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> edges_;
    std::unordered_map<std::pair<int, int>, std::vector<std::pair<std::pair<int, int>, double>>, pair_hash> adj_;

    double euclidean(const std::pair<int, int>& a, const std::pair<int, int>& b);
    bool isCollisionFree(const std::pair<int, int>& p1, const std::pair<int, int>& p2);
};
