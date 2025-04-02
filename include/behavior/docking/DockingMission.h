#pragma once 

#include "behavior/Behavior.h"

class DockingMission : public Behavior {
public:
    DockingMission();

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
    bool pathPlanned = false;

    void transitionTo(State next);
    void simulatePlannerCall();
};
