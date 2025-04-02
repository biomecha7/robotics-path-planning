#pragma once 

#include "behavior/Behavior.h"
#include "planner/PlannerInterface.h"
#include "core/GridTypes.h"

class DockingMission : public Behavior {
public:
    DockingMission(
        Grid& grid, PlannerInterface& planner,
        const std::pair<int, int>& start,
        const std::pair<int, int>& goal
    );

    void initialize() override;
    void update() override;
    bool isComplete() const override;
    std::string name() const override;

private:
    enum class State {
        Idle, 
        SearchTarget,
        Approach,
        Align,
        Dock,
        Complete
    };

    State currentState;
    Grid& grid;
    PlannerInterface& planner;
    std::pair<int, int> start;
    std::pair<int, int> goal;

    std::vector<std::pair<int, int>> path;
    bool pathPlanned = false;

    void transitionTo(State next);
    void runPlanner();
};
