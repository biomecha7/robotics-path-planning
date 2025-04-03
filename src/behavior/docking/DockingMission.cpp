#include "behavior/docking/DockingMission.h"
#include "simulation/GridVisualizer.h"
#include <iostream>
#include <thread>
#include <chrono>

DockingMission::DockingMission(
    Grid& grid,
    PlannerInterface& planner,
    const std::pair<int, int>& start,
    const std::pair<int, int>& goal
) : currentState(State::Idle), grid(grid), planner(planner), start(start), goal(goal) {}

void DockingMission::initialize() {
    std::cout << "[DockingMission] Initializing..." << std::endl;
    transitionTo(State::SearchTarget);
}

void DockingMission::update() {
    switch(currentState) {
        case State::SearchTarget:
            std::cout << "[State] Searching for container..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            transitionTo(State::Approach);
            break;

        case State::Approach:
            std::cout << "[State] Planning approach path..." << std::endl;
            runPlanner();
            transitionTo(State::Align);
            break;

        case State::Align:
            std::cout <<"[State] Aligning with dock..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            transitionTo(State::Dock);
            break;

        case State::Dock:
            std::cout << "[State] Docking..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            transitionTo(State::Complete);
            break;

        case State::Complete:
            std::cout << "[State] Docking complete." << std::endl;
            break;

        default:
            break;
    }
}

bool DockingMission::isComplete() const {
    return currentState == State::Complete;
}

std::string DockingMission::name() const {
    return "DockingMission";
}

void DockingMission::transitionTo(State next) {
    currentState = next;
}

void DockingMission::runPlanner() {
    if (!pathPlanned) {
        std::cout << "[Planner] Initializing...\n";
        planner.initialize();

        path = planner.plan(start, goal);
        if (path.empty()) {
            std::cerr << "[Planner] Failed to find a path!\n";
        } else {
            std::cout << "[Planner] Path found with " <<path.size() << "waypoints.\n";
            GridVisualizer viz(grid);
            viz.setStartGoal(start, goal);
            viz.overlayPath(path);
            viz.saveToImage("img/docking_path.png");
        }
        pathPlanned = true;
    }
}
