#pragma once 

#include <vector>
#include <utility>

class PlannerInterface {
public:
    virtual ~PlannerInterface() = default;

    virtual void initialize() = 0;

    // Returns path from start to goal
    virtual std::vector<std::pair<int, int>> plan(
        const std::pair<int, int>& start, 
        const std::pair<int, int>& goal
    ) = 0;
};
