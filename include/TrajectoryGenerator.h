#pragma once

#include <vector>
#include <utility>

class TrajectoryGenerator {
public:
    using Point = std::pair<double, double>;

    // Generate a Catmull-Rom spline interpolated path
    static std::vector<Point> generateCatmullRomSpline(
        const std::vector<Point>& waypoints,
        int pointsPerSegment = 20
    );

};
