#pragma once 

#include <vector>
#include <utility> // for std::pair

namespace local_planner {

enum class LocalPlannerStatus {
    GoalReached,
    Blocked,
    FollowingPath
};

struct LocalPlannerResult {
    std::vector<std::pair<int, int>> localPath;
    LocalPlannerStatus status;
};

class LocalPlannerInterface {
public:
    virtual ~LocalPlannerInterface() = default;

    virtual void setGlobalPath(const std::vector<std::pair<int, int>>& path) = 0;
    virtual LocalPlannerResult plan(int currentX, int currentY) = 0;
};

} // namespace local_planner
