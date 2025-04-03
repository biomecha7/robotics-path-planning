// File: AStarPlanner.h
#pragma once

#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <utility>
#include <algorithm>
#include "planner/PlannerInterface.h"
#include "core/GridTypes.h"

class AStarPlanner : public PlannerInterface {
public:
    AStarPlanner(const Grid& grid);

    void initialize() override;

    std::vector<std::pair<int, int>> plan(
        const std::pair<int, int>& start,
        const std::pair<int, int>& goal
    ) override;

private:
    struct Node {
        int x, y;
        double cost;        // g(n)
        double priority;    // f(n) = g(n) + h(n)

        bool operator>(const Node& other) const {
            return priority > other.priority;
        }
    };

    double heuristic(int x1, int y1, int x2, int y2) const;

    std::vector<std::pair<int, int>> reconstructPath(
        std::unordered_map<int, std::pair<int, int>>& cameFrom,
        std::pair<int, int> current
    );

    const Grid& grid_;
    int rows_, cols_;
};
