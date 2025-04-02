#pragma once

#include <vector>
#include <utility>
#include "planner/PlannerInterface.h"
#include "core/GridTypes.h"

class RRTPlanner : public PlannerInterface {
public:
    struct Node {
        Point pos;
        int parentIdx;
    };

    RRTPlanner(const Grid& grid, int maxIterations = 1000, double stepSize = 5.0, double goalBias = 0.05);

    void initialize() override;
    std::vector<Point> plan(const Point& start, const Point& goal) override;

    const std::vector<Node>& getTree() const;

private:
    Grid grid_;
    int rows_, cols_;
    int maxIterations_;
    double stepSize_;
    double goalBias_;
    std::vector<Node> tree_;
    Point goal_;

    Point sample();
    int getNearestNodeIndex(const Point& target);
    Point steer(const Point& from, const Point& to);
    bool isCollisionFree(const Point& from, const Point& to);
    double distance(const Point& a, const Point& b);
};
