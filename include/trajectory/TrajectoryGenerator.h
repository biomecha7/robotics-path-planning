#pragma once

#include <vector>
#include <utility>

class TrajectoryGenerator {
public:
    using Point = std::tuple<double, double, double>; // (x, y, t)

    // Generate a Catmull-Rom spline interpolated path
    static std::vector<std::tuple<double, double, double>> generateCatmullRomSpline(
        const std::vector<std::tuple<double, double, double>>& timedPath,
        int pointsPerSegment = 20
    );

    // Time profiling api
    static std::vector<std::tuple<double, double, double>> applyTimeProfile(
        const std::vector<Point>& path,
        double velocity = 1.0   // m/s (or grid units/sec)
    );

};
