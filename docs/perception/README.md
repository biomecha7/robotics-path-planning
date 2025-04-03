# Perception Layer – Problem Definition

## Objective

The perception layer is responsible for maintaining a real-time spatial understanding of the environment around the autonomous vehicle. It serves as a foundational component of the autonomy stack, providing updated information about static and dynamic obstacles within a known or partially known 2D environment.

This module outputs a 2D occupancy grid that informs the behavior layer, local planner, and collision checking logic. It is designed to be extendable to SE(2) for orientation-aware planning and control.

## Inputs

- Dimensions of the 2D world (in meters)
- Grid resolution (meters per cell)
- Static or dynamic obstacle definitions (manual, simulated, or sensor-derived)

## Outputs

- A 2D occupancy grid:
  - Each cell is marked as Free, Occupied, or Unknown.
- Interfaces to:
  - Add/remove obstacles
  - Query occupancy
  - Export grid data for visualization

## Design Goals

- Modular: Cleanly separated from planning, behavior, and control
- Lightweight: Efficient real-time queries and updates
- Extendable: Easy upgrade to SE(2) occupancy or voxel grid
- ROS-compatible (optional future support)

## Planned Extension

- Integration with sensor fusion (Lidar, radar, vision)
- Conversion to SE(2) configuration space (orientation-aware)
- Dynamic obstacle tracking and prediction
