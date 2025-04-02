#include "planner/RRTPlanner.h"
#include <cmath>
#include <random>
#include <limits>
#include <cstdlib>
#include <ctime>
#include <algorithm>

RRTPlanner::RRTPlanner(const Grid& grid, int maxIterations, double stepSize, double goalBias)
    : grid_(grid), rows_(grid.size()), cols_(grid[0].size()),
      maxIterations_(maxIterations), stepSize_(stepSize), goalBias_(goalBias)
{}

void RRTPlanner::initialize() {
    std::srand(std::time(nullptr));
}

std::vector<Point> RRTPlanner::plan(const Point& start, const Point& goal) {
    tree_.clear();
    goal_ = goal;
    tree_.push_back({start, -1});

    for (int iter = 0; iter < maxIterations_; ++iter) {
        Point randPoint = sample();
        int nearestIdx = getNearestNodeIndex(randPoint);
        if (nearestIdx == -1) continue;

        Point nearest = tree_[nearestIdx].pos;
        Point newPoint = steer(nearest, randPoint);

        if (!isCollisionFree(nearest, newPoint)) continue;

        tree_.push_back({newPoint, nearestIdx});

        if (distance(newPoint, goal) < stepSize_) {
            if (isCollisionFree(newPoint, goal)) {
                tree_.push_back({goal, static_cast<int>(tree_.size() - 1)});

                std::vector<Point> path;
                int idx = tree_.size() - 1;
                while (idx != -1) {
                    path.push_back(tree_[idx].pos);
                    idx = tree_[idx].parentIdx;
                }
                std::reverse(path.begin(), path.end());
                return path;
            }
        }
    }

    return {}; // No path
}

Point RRTPlanner::sample() {
    if ((std::rand() / static_cast<double>(RAND_MAX)) < goalBias_) {
        return goal_;
    }
    int x = std::rand() % cols_;
    int y = std::rand() % rows_;
    return {x, y};
}

double RRTPlanner::distance(const Point& a, const Point& b) {
    int dx = a.first - b.first;
    int dy = a.second - b.second;
    return std::sqrt(dx * dx + dy * dy);
}

int RRTPlanner::getNearestNodeIndex(const Point& target) {
    double minDist = std::numeric_limits<double>::max();
    int nearestIdx = -1;

    for (int i = 0; i < static_cast<int>(tree_.size()); ++i) {
        double d = distance(tree_[i].pos, target);
        if (d < minDist) {
            minDist = d;
            nearestIdx = i;
        }
    }

    return nearestIdx;
}

Point RRTPlanner::steer(const Point& from, const Point& to) {
    double dx = to.first - from.first;
    double dy = to.second - from.second;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < stepSize_) return to;

    double scale = stepSize_ / dist;
    int newX = static_cast<int>(std::round(from.first + dx * scale));
    int newY = static_cast<int>(std::round(from.second + dy * scale));

    return {newX, newY};
}

bool RRTPlanner::isCollisionFree(const Point& from, const Point& to) {
    int x0 = from.first, y0 = from.second;
    int x1 = to.first, y1 = to.second;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (x0 < 0 || x0 >= cols_ || y0 < 0 || y0 >= rows_) return false;
        if (grid_[y0][x0] == 1) return false;

        if (x0 == x1 && y0 == y1) break;

        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx)  { err += dx; y0 += sy; }
    }

    return true;
}

const std::vector<RRTPlanner::Node>& RRTPlanner::getTree() const {
    return tree_;
}
