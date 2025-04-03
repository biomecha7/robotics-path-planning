# Local Planner – Problem Definition

## Objective

The Local Planner is responsible for generating short-horizon motion commands that allow the autonomous vehicle to safely and efficiently follow a global path while avoiding obstacles in its local vicinity.

This component runs in real-time, consulting the Perception Layer to react to newly detected obstacles or changes in the environment. It is responsible for motion feasibility (kinematics, basic dynamics) and responsiveness.

## Inputs

- Current robot pose `(x, y)`
- Global path (from A*, PRM, or RRT)
- Local obstacle map (from the Perception Layer)
- Vehicle constraints (e.g., turning radius, velocity limits)

## Outputs

- Local path segment: a list of grid points or interpolated waypoints
- Status flags: `GoalReached`, `Blocked`, `FollowingPath`, etc.

## Initial Strategy

We will begin with a basic **Goal Point Follower**:
- Always try to move toward the **next valid point on the global path** that is reachable (not obstructed).
- If blocked, attempt a simple avoidance strategy (e.g., sidestep left/right).

## Design Goals

- Compatible with current 2D C-Space
- Clean interface for plugging into Behavior layer
- Future extensibility to:
  - DWA / TEB / MPC-based planners
  - SE(2) motion models
  - Velocity command outputs (`v`, `omega`)

