#include "behavior/docking/DockingMission.h"
#include <iostream>
#include <thread>
#include <chrono>

DockingMission::DockingMission() : currentState(State::Idle) {}

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
            simulatePlannerCall();
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

void DockingMission::simulatePlannerCall() {
    if (!pathPlanned) {
        std::cout << "[Planner] Calling Planner..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        pathPlanned = true;
        std::cout << "[Planner] Path found." << std::endl;
    }
}
