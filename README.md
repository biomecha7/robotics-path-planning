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

Awesome — you're about to step into **sampling-based motion planning** with **PRM (Probabilistic Roadmap Method)**, which is foundational in real robotics, including arms, drones, and autonomous vehicles in off-road environments.

---

## 🚧 **Problem 4: Probabilistic Roadmap (PRM) Path Planning**

---

## 🚚 **Scenario**

You’re programming a ground-based autonomous vehicle to navigate a 100x100 meter area with sparse, irregular obstacles. A traditional grid-based planner like A* is inefficient here. Instead, you'll use PRM:

> "Sample valid configurations, connect them into a graph, and search that graph."

---

## 🎯 **Goals**
1. Sample `N` valid points from the configuration space.
2. Connect each point to its `K` nearest neighbors **only if** the straight-line path is collision-free.
3. Add start and goal to the roadmap and connect them.
4. Run Dijkstra or A* over this graph to find the path.
5. Visualize:
   - Sampled nodes
   - Roadmap edges
   - Final path

---

## 🛠️ Inputs

- `Grid` from your `CSpaceBuilder`
- Start and goal position `(x, y)` integers
- Constants:
  - `N = 500` nodes
  - `K = 10` nearest neighbors

---

## 📤 Output

- A vector of `(x, y)` points representing the path
- Image:
  - Obstacle map
  - Roadmap edges (light gray)
  - Path (red)
  - Start (green), Goal (blue)

---

## 🔧 Class Structure (new file: `PRMPlanner.h/cpp`)

```cpp
class PRMPlanner {
public:
    PRMPlanner(const Grid& grid, int numSamples, int numNeighbors);

    std::vector<std::pair<int, int>> findPath(
        std::pair<int, int> start,
        std::pair<int, int> goal
    );

    const std::vector<std::pair<int, int>>& getSampledNodes() const;
    const std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>>& getEdges() const;

private:
    void sampleFreePoints();
    void buildRoadmap();
    void connectNode(const std::pair<int, int>& node);

    bool isCollisionFree(const std::pair<int, int>& p1, const std::pair<int, int>& p2);
    double euclidean(const std::pair<int, int>& a, const std::pair<int, int>& b);

    const Grid& grid_;
    int rows_, cols_;
    int numSamples_;
    int numNeighbors_;

    std::vector<std::pair<int, int>> nodes_;  // sampled nodes
    std::vector<std::pair<std::pair<int, int>, std::pair<int, int>>> edges_; // graph edges
    std::unordered_map<std::pair<int, int>, std::vector<std::pair<int, double>>, pair_hash> adj_; // adjacency list
};
```

> You'll reuse your **collision checker** from Problem 2.

---

## 🧠 Algorithm Summary

1. **Sampling**:
   - Pick `N` random (x, y) in the grid.
   - If the grid cell is free → keep it.
   - Store them in `nodes_`.

2. **Graph construction**:
   - For each node, find `K` nearest others (Euclidean).
   - Connect with an edge **if straight line path is collision-free**.

3. **Add start and goal**:
   - Manually add these to the graph.
   - Connect them to their `K` nearest neighbors as well.

4. **Search**:
   - Dijkstra’s or A* over the graph using `std::priority_queue`.

5. **Visualize**:
   - Gray lines for edges
   - Red for path
   - Green = start, Blue = goal

---

## 🗂️ File Plan

| File                | Description                     |
|---------------------|---------------------------------|
| `PRMPlanner.h/cpp`  | PRM logic, graph building, search |
| `main.cpp`          | Load C-space, run PRM, visualize |
| `GridVisualizer`    | Extend to draw nodes + edges    |

---

Absolutely — here's your formal problem statement, modeled in the same structured and progressive format as before:

---

## 🧭 **Problem 5: Implement a Grid-Based RRT Planner**

### 📍Context

You are developing a motion planning module for a **ground-based autonomous truck** navigating through a 2D grid with static obstacles. The robot is **non-holonomic**, but for the sake of initial implementation, we will relax constraints and assume it can turn instantly and move in any direction (like a point robot).

Later, this planner will evolve to account for turn radius, heading, and full vehicle kinematics.

---

### 🧩 Goal

Implement a **Rapidly-Exploring Random Tree (RRT)** planner that:

1. **Grows a tree** of configurations in free space from the start location
2. Uses **random sampling** and **nearest-neighbor expansion**
3. Attempts to connect the tree toward a specified **goal**
4. **Avoids obstacles** using a **collision checker** over a 2D grid
5. Produces a valid **path from start to goal**
6. Visualizes:
   - The RRT edges (gray lines)
   - The final path (red line)
   - Start and goal markers (green/blue)

---

### 📦 Provided Tools

- A `Grid` object from your `CSpaceBuilder` with:
  - `0 = free`
  - `1 = obstacle`
- A `GridVisualizer` that supports:
  - `.overlayPath(path)`
  - `.overlayEdges(edges)`
  - `.saveToImage(filename)`

---

### 🧪 Requirements

- RRT should:
  - Use random sampling within the grid bounds
  - Use Euclidean distance to find the nearest node
  - Expand toward the sampled point using a **fixed step size**
  - Use **grid-based Bresenham collision checking**
  - Stop when the goal is reached (or a max number of iterations)

- Code should be modular:
  - `RRTPlanner.h / RRTPlanner.cpp`
  - Callable from `main.cpp`

---

### 📈 Extensions (for later)

After the core works:
- Add **goal biasing** (e.g., 5% of the time sample = goal)
- Add **angle + turning radius**
- Add **Dubins or Reeds-Shepp expansion**
- Optimize path with shortcut smoothing

---

### 🔑 Inputs

- `start = (x, y)`
- `goal  = (x, y)`
- `grid` from `CSpaceBuilder`

---

### 🎯 Output

- `std::vector<Point>` path from `start` to `goal`
- Visualization image (`img/rrt_path.png`) showing:
  - Tree growth
  - Final path

---
