#include "planner/PRMPlanner.h"
#include <cstdlib>
#include <ctime>
#include <random>
#include <queue>

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

const std::vector<std::pair<int, int>>& PRMPlanner::getSampledNodes() const {
    return nodes_;
}

double PRMPlanner::euclidean(const std::pair<int, int>& a, const std::pair<int, int>& b) {
    int dx = a.first - b.first;
    int dy = a.second - b.second;
    return std::sqrt(dx * dx + dy * dy);
}

// Reused from A*
bool PRMPlanner::isCollisionFree(const std::pair<int, int>& p1, const std::pair<int, int>& p2) {
    int x0 = p1.first, y0 = p1.second;
    int x1 = p2.first, y1 = p2.second;

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
        if (e2 < dx) { err += dx; y0 += sy; }
    }

    return true;
}

void PRMPlanner::buildRoadmap() {
    for (const auto& node : nodes_) {
        // Find neighbors
        std::vector<std::pair<double, std::pair<int, int>>> dists;
        for (const auto& other : nodes_) {
            if (node == other) continue;
            double dist = euclidean(node, other);
            dists.emplace_back(dist, other);
        }

        std::sort(dists.begin(), dists.end());

        int connections = 0;
        for (const auto& [dist, neighbor] : dists) {
            if (connections >= numNeighbors_) break;
            if (isCollisionFree(node, neighbor)) {
                edges_.emplace_back(node, neighbor);
                adj_[node].emplace_back(neighbor, dist);
                adj_[neighbor].emplace_back(node, dist);
                ++connections;
            }
        }
    }
}

const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& PRMPlanner::getEdges() const {
    return edges_;
}

std::vector<std::pair<int, int>> PRMPlanner::findPath(
    std::pair<int, int> start,
    std::pair<int, int> goal
) {
    // 1. Add start and goal to graph
    nodes_.push_back(start);
    nodes_.push_back(goal);

    // 2. Connect them like any other node
    auto connectNode = [&](const std::pair<int, int>& node) {
        std::vector<std::pair<double, std::pair<int, int>>> dists;
        for (const auto& other : nodes_) {
            if (node == other) continue;
            double dist = euclidean(node, other);
            dists.emplace_back(dist, other);
        }
        std::sort(dists.begin(), dists.end());

        int connections = 0;
        for (const auto& [dist, neighbor] : dists) {
            if (connections >= numNeighbors_) break;
            if (isCollisionFree(node, neighbor)) {
                edges_.emplace_back(node, neighbor);
                adj_[node].emplace_back(neighbor, dist);
                adj_[neighbor].emplace_back(node, dist);
                ++connections;
            }
        }
    };

    connectNode(start);
    connectNode(goal);

    // 3. Dijkstra's search
    std::unordered_map<std::pair<int, int>, double, pair_hash> costSoFar;
    std::unordered_map<std::pair<int, int>, std::pair<int, int>, pair_hash> cameFrom;
    using PQElem = std::pair<double, std::pair<int, int>>;
    std::priority_queue<PQElem, std::vector<PQElem>, std::greater<>> frontier;

    frontier.push({0.0, start});
    costSoFar[start] = 0.0;

    while (!frontier.empty()) {
        auto [cost, current] = frontier.top();
        frontier.pop();

        if (current == goal) break;

        for (const auto& [neighbor, edgeCost] : adj_[current]) {
            double newCost = costSoFar[current] + edgeCost;
            if (costSoFar.find(neighbor) == costSoFar.end() || newCost < costSoFar[neighbor]) {
                costSoFar[neighbor] = newCost;
                frontier.push({newCost, neighbor});
                cameFrom[neighbor] = current;
            }
        }
    }

    // 4. Reconstruct path
    std::vector<std::pair<int, int>> path;
    std::pair<int, int> current = goal;
    if (cameFrom.find(goal) == cameFrom.end()) return {}; // no path

    while (current != start) {
        path.push_back(current);
        current = cameFrom[current];
    }

    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<std::pair<int, int>> PRMPlanner::smoothPath(const std::vector<std::pair<int, int>>& path) {
    if (path.empty()) return {};

    std::vector<std::pair<int, int>> smoothed;
    size_t i = 0;

    while (i < path.size() - 1) {
        size_t j = path.size() - 1;

        // Find the farthest j > i such that path[i] to path[j] is collision-free
        while (j > i + 1 && !isCollisionFree(path[i], path[j])) {
            --j;
        }

        // If no shortcut found, move just one step forward
        if (j == i + 1) {
            smoothed.push_back(path[i]);
            ++i;
        } else {
            smoothed.push_back(path[i]);
            i = j;
        }
    }

    smoothed.push_back(path.back());
    return smoothed;
}
