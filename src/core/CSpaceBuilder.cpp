#include "core/CSpaceBuilder.h"
#include <iostream>
#include <cmath>
#include <algorithm>

CSpaceBuilder::CSpaceBuilder(int gridSize, double robotRadius)
    : gridSize_(gridSize), robotRadius_(robotRadius) {
    grid_ = Grid(gridSize_, std::vector<int>(gridSize_, 0));
}

void CSpaceBuilder::addObstacle(const Rectangle& rect) {
    obstacles_.push_back(rect);
}

void CSpaceBuilder::buildConfigurationSpace() {
    for (const auto& rect : obstacles_) {
        inflateAndMark(rect);
    }
}

const Grid& CSpaceBuilder::getGrid() const {
    return grid_;
}

void CSpaceBuilder::printGrid() const {
    for (int y = 0; y < gridSize_; ++y) {
        for (int x = 0; x < gridSize_; ++x) {
            std::cout << grid_[y][x];
        }
        std::cout << '\n';
    }
}

bool CSpaceBuilder::isCollisionFree(const Grid& grid, int x0, int y0, int x1, int y1) {
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int x = x0;
    int y = y0;
    int n = 1 + dx + dy;
    int x_inc = (x1 > x0) ? 1 : -1;
    int y_inc = (y1 > y0) ? 1 : -1;
    int error = dx - dy;
    dx *= 2;
    dy *= 2;

    for (int i = 0; i < n; ++i) {
        if (x < 0 || x >= grid[0].size() || y < 0 || y >= grid.size()) return false;
        if (grid[y][x] == 1) return false;

        if (error > 0) {
            x += x_inc;
            error -= dy;
        } else {
            y += y_inc;
            error += dx;
        }
    }
    return true;
}

void CSpaceBuilder::inflateAndMark(const Rectangle& rect) {
    int r = static_cast<int>(std::ceil(robotRadius_));
    int x_min = std::max(0, rect.x_min - r);
    int x_max = std::min(gridSize_ - 1, rect.x_max + r);
    int y_min = std::max(0, rect.y_min - r);
    int y_max = std::min(gridSize_ - 1, rect.y_max + r);

    for (int y = y_min; y <= y_max; ++y) {
        for (int x = x_min; x <= x_max; ++x) {
            double cx = (x + 0.5) * RESOLUTION;
            double cy = (y + 0.5) * RESOLUTION;

            double rx_min = rect.x_min * RESOLUTION;
            double ry_min = rect.y_min * RESOLUTION;
            double rx_max = rect.x_max * RESOLUTION;
            double ry_max = rect.y_max * RESOLUTION;

            double closest_x = std::clamp(cx, rx_min, rx_max);
            double closest_y = std::clamp(cy, ry_min, ry_max);

            double dx = cx - closest_x;
            double dy = cy - closest_y;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist <= robotRadius_) {
                grid_[y][x] = 1;
            }
        }
    }
}
