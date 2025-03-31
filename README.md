### 🧠 Problem 1: 2D Configuration Space Obstacle Map

**Scenario**:  
You are tasked with designing a 2D configuration space for a ground robot (modeled as a circular truck with a fixed radius) operating in a known environment such as a shipping yard. The yard contains rectangular static obstacles.

**Objective**:  
Write a C++ program that:

1. Takes in a 2D grid map of the environment.
2. Takes a list of rectangular obstacles (defined by bottom-left and top-right corners).
3. Inflates the obstacles to account for the robot’s radius to create the configuration space obstacle map.
4. Outputs the grid map with inflated obstacles as `1`s and free space as `0`s.

---

### 🔧 Specifications

- Grid size: 100x100
- Resolution: 1 grid cell = 1 meter
- Robot radius: `r = 2` meters
- Each obstacle is an axis-aligned rectangle.

---

### 🧩 Inputs

- A list of rectangles:
```cpp
struct Rectangle {
    int x_min, y_min, x_max, y_max;
};
```

---

### 🔢 Output

- A 2D vector `std::vector<std::vector<int>> grid` where:
  - `grid[y][x] = 1` if the point is inside an inflated obstacle.
  - `grid[y][x] = 0` otherwise.

---

### ✅ Constraints

- All rectangle coordinates will be within the grid bounds (0 to 99).
- Inflated obstacles must remain within bounds (clip if needed).
- Use Euclidean distance for inflation — any cell whose center lies within `r` meters of the original rectangle edge should be marked as an obstacle.

---

Awesome — you’ve just built a configuration space (C-space) occupancy grid. The next logical step is to **check whether a motion from point A to point B is feasible**, i.e., **collision-free**.

This is foundational for **sampling-based planners**, **A\***, and even **trajectory rollout methods** used in real robots.

---

## 🧠 Problem 2: Collision-Free Straight Line Check in Configuration Space

### **Scenario**  
You're controlling an autonomous ground vehicle in a shipping yard. Given a 2D C-space grid (from Problem 1), determine whether a straight-line path from a **start** to a **goal** position is collision-free.

---

### **Objective**  
Implement a C++ function that:
- Takes in two points: `(x_start, y_start)` and `(x_goal, y_goal)`
- Checks if the straight line between them passes only through **free space** in the grid.
- Returns `true` if the path is clear; `false` if it intersects any inflated obstacle.

---

### **Input**
- A 2D grid `std::vector<std::vector<int>> grid` where:
  - `1` = obstacle (occupied cell)
  - `0` = free space
- Two points (integers in grid coordinates, e.g., `(20, 20)` and `(80, 80)`)

---

### **Output**
- A `bool` indicating whether the path is collision-free.

---

### ✅ Constraints
- Use **Bresenham's Line Algorithm** to iterate through all the grid cells that the line touches.
- Grid coordinates must be valid (0 ≤ x, y < grid size).
- If any cell the line touches is `1`, it's a collision.

---

## 🧠 Problem 3: A\* Path Planning on a 2D Occupancy Grid

### 🚚 Scenario
Your robot is navigating a known 100x100-meter shipping yard map. The environment contains static obstacles (already inflated into a **C-space grid**). Your goal is to move the robot from a **start** to a **goal** position using a **shortest, collision-free path**.

---

### 🧩 Objective
Implement the A\* algorithm to find the shortest valid path from a start cell `(x_start, y_start)` to a goal cell `(x_goal, y_goal)` on the 2D grid.

- **Free cell** = `0`
- **Obstacle cell** = `1`
- Return a list of waypoints from start to goal.

---

### 🔧 Function Signature

```cpp
std::vector<std::pair<int, int>> aStarSearch(
    const Grid& grid, 
    std::pair<int, int> start, 
    std::pair<int, int> goal
);
```

---

### ✅ Requirements

- Use **Manhattan distance** or **Euclidean distance** as the heuristic.
- Allow **4-way or 8-way connectivity** (your choice for now).
- If no path exists, return an empty vector.
- You may use `std::priority_queue` with a custom comparator for the open set.

---
