#pragma once 

#include <vector>
#include <utility>
#include <unordered_map>

using Grid = std::vector<std::vector<int>>;
using Point = std::pair<int, int>;

class RRTPlanner {
public:
    struct Node {
        Point pos;
        int parentIdx;
    };

    RRTPlanner(const Grid& grid, int maxIterations = 1000, double stepSize = 5.0, double goalBias = 0.05);

    std::vector<Point> plan(Point start, Point goal);

    const std::vector<RRTPlanner::Node>& getTree() const { return tree_; }
private:
    Grid grid_;
    int rows_, cols_;
    int maxIterations_;
    double stepSize_;
    std::vector<Node> tree_;
    double goalBias_ = 0.5; // default 5% chance
    Point goal_;

    Point sample();
    int getNearestNodeIndex(const Point& target);
    Point steer(const Point& from, const Point& to);
    bool isCollisionFree(const Point& from, const Point& to);
    double distance(const Point& a, const Point& b);
};
