#pragma once 

#include <vector>
#include <utility>
#include <unordered_map>

using Grid = std::vector<std::vector<int>>;
using Point = std::pair<int, int>;

class RRTPlanner {
public:
    RRTPlanner(const Grid& grid, int maxIterations = 1000, double stepSize = 5.0);

    std::vector<Point> plan(Point start, Point goal);

private:
    struct Node {
        Point pos;
        int parentIdx;
    };

    Grid grid_;
    int rows_, cols_;
    int maxIterations_;
    double stepSize_;
    std::vector<Node> tree_;

    Point sample();
    int getNearestNodeIndex(const Point& target);
    Point steer(const Point& from, const Point& to);
    bool isCollisionFree(const Point& from, const Point& to);
    double distance(const Point& a, const Point& b);
};
