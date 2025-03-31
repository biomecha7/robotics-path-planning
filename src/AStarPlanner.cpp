// File: AStarPlanner.cpp

#include "AStarPlanner.h"

double AStarPlanner::heuristic(int x1, int y1, int x2, int y2) const {
    return std::sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
}

std::vector<std::pair<int, int>> AStarPlanner::reconstructPath(
    std::unordered_map<int, std::pair<int, int>>& cameFrom,
    std::pair<int, int> current
) {
    std::vector<std::pair<int, int>> path;
    int key = current.first * cols_ + current.second;
    while (cameFrom.find(key) != cameFrom.end()) {
        path.push_back(current);
        current = cameFrom[key];
        key = current.first * cols_ + current.second;
    }
    path.push_back(current);    // start node
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<std::pair<int, int>> AStarPlanner::search(
    std::pair<int, int> start,
    std::pair<int, int> goal
) {
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
    openSet.push({start.first, start.second, 0.0, heuristic(start.first, start.second, goal.first, goal.second)});

    std::unordered_map<int, double> costSoFar;
    std::unordered_map<int, std::pair<int, int>> cameFrom;
    costSoFar[start.first * cols_ + start.second] = 0.0;

    const std::vector<std::pair<int, int>> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},   // 4-connected
        {1, 1}, {-1, -1}, {1, -1}, {-1, 1}  // diagonals
    };

    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        if (current.x == goal.first && current.y == goal.second) {
            return reconstructPath(cameFrom, {current.x, current.y});
        }

        for (auto [dx, dy] : directions) {
            int nx = current.x + dx;
            int ny = current.y + dy;

            if (nx < 0 || ny < 0 || nx >= cols_ || ny >= rows_) continue;
            if (grid_[ny][nx] == 1) continue;   // obstacle

            double stepCost = std::hypot(dx, dy);
            double newCost = costSoFar[current.x * cols_ + current.y] + stepCost;
            int key = nx * cols_ + ny;

            if (costSoFar.find(key) == costSoFar.end() || newCost < costSoFar[key]) {
                costSoFar[key] = newCost; 
                double priority = newCost + heuristic(nx, ny, goal.first, goal.second);
                openSet.push({nx, ny, newCost, priority});
                cameFrom[key] = {current.x, current.y};
            }
        }
    }

    return {}; // No path found 
}
