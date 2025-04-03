#include "local_planner/SimpleLocalPlanner.h"

namespace local_planner {

SimpleLocalPlanner::SimpleLocalPlanner(perception::PerceptionLayer& perceptionLayer, int maxStepLookahead)
    : perceptionLayer_(perceptionLayer),
    currentPathIndex_(0),
    maxStepLookahead_(maxStepLookahead) {}

void SimpleLocalPlanner::setGlobalPath(const std::vector<std::pair<int, int>>& path) {
    globalPath_ = path;
    currentPathIndex_ = 0;
}

LocalPlannerResult SimpleLocalPlanner::plan(int currentX, int currentY) {
    const auto& grid = perceptionLayer_.getGrid();
    LocalPlannerResult result;

    // Check if path is empty or completed
    if (currentPathIndex_ >= globalPath_.size()) {
        result.status = LocalPlannerStatus::GoalReached;
        return result;
    }

    // Try to find the next reachable goal point
    size_t end = std::min(currentPathIndex_ + maxStepLookahead_, globalPath_.size());
    bool found = false;
    for (size_t i = currentPathIndex_; i < end; ++i) {
        const auto& [x, y] = globalPath_[i];
        if (!grid.isOccupied(x, y)) {
            result.localPath.push_back({x, y});
            currentPathIndex_ = i;
            found = true;
            break;
        }
    }

    if (!found) {
        result.status = LocalPlannerStatus::Blocked;
        return result;
    }

    // Add more path points up to lookahead window
    for (size_t i = currentPathIndex_ + 1; i < end; ++i) {
        const auto& [x, y] = globalPath_[i];
        if (!grid.isOccupied(x, y)) {
            result.localPath.push_back({x, y});
        } else {
            break;  // Stop early if obstacle is encountered ahead
        }
    }

    // Update status
    if (currentPathIndex_ >= globalPath_.size() - 1) {
        result.status = LocalPlannerStatus::GoalReached;
    } else {
        result.status = LocalPlannerStatus::FollowingPath;
    }

    return result;
}

} // namespace local_planner
