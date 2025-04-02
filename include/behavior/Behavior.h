#pragma once 

#include <string>

class Behavior {
public:
    virtual ~Behavior() = default;

    // Called once at the beginning
    virtual void initialize() = 0;

    // Called repeatedly in control loop
    virtual void update() = 0;

    // Check if mission is complete
    virtual bool isComplete() const = 0;

    // Human-readable name for logging or debugging
    virtual std::string name()  const = 0;
};
