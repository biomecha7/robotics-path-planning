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

/**
 * Computes Euclidean distance between two grid points.
 */
double RRTPlanner::distance(const Point& a, const Point& b) {
    int dx = a.first - b.first;
    int dy = a.second - b.second;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * Finds the closest node already in the tree.
 */
int RRTPlanner::getNearestNodeIndex(const Point& target) {
    double minDist = std::numeric_limits<double>::max();
    int nearestIdx = -1;

    for (int i = 0; i < tree_.size(); ++i) {
        double d = distance(tree_[i].pos, target);
        if (d < minDist) {
            minDist = d;
            nearestIdx = i;
        }
    }

    return nearestIdx;
}

/**
 * Concept: in each RRT iteration:
 *  1. Sample random point
 *  2. Find the nearest node in the tree
 *  3. Steer that node toward the sample
 *  4. If it's collision-free, add it to the free
 *
 */
std::vector<Point> RRTPlanner::plan(Point start, Point goal) {
    tree_.clear();
    tree_.push_back({start, -1});   // root node has no parent

    // TODO: Loop and grow tree
    return {};
}

/**
 * Moves from a node in the tree toward the sampled point, but only by a fixed stepSize_, not all the way.
 *
 * Concept:
 *  - The sample point may be far away.
 *  - We want to limit expansion per step to encourage gradual growth and maintain control.
 *  - This mimics a robot moving forward with bounded velocity.
 *
 */
Point RRTPlanner::steer(const Point& from, const Point& to) {
    double dx = to.first - from.first;
    double dy = to.second - from.second;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < stepSize_) {
        return to;
    }

    double scale = stepSize_ / dist;
    int newX = static_cast<int>(std::round(from.first + dx * scale));
    int newY = static_cast<int>(std::round(from.second + dy * scale));

    return {newX, newY};
}
