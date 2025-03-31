#ifndef CSPACE_BUILDER_H
#define CSPACE_BUILDER_H

#include <vector>

inline constexpr double RESOLUTION = 1.0;

struct Rectangle {
    int x_min, y_min, x_max, y_max;
};

using Grid = std::vector<std::vector<int>>;

class CSpaceBuilder {
public:
    CSpaceBuilder(int gridSize, double robotRadius);

    void addObstacle(const Rectangle& rect);
    void buildConfigurationSpace();
    const Grid& getGrid() const;
    void printGrid() const;
    bool isCollisionFree(const Grid& grid, int x0, int y0, int x1, int y1);

private:
    void inflateAndMark(const Rectangle& rect);

    int gridSize_;
    double robotRadius_;
    Grid grid_;
    std::vector<Rectangle> obstacles_;
};

#endif // CSPACE_BUILDER_H
