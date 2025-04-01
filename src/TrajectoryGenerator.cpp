#include "TrajectoryGenerator.h"
#include <cmath>

std::vector<TrajectoryGenerator::Point> TrajectoryGenerator::generateCatmullRomSpline(
    const std::vector<Point>& waypoints,
    int pointsPerSegment
) {
    std::vector<Point> result;
    if (waypoints.size() < 4) return waypoints;

    for (size_t i = 1; i < waypoints.size() - 2; ++i) {
        Point p0 = waypoints[i - 1];
        Point p1 = waypoints[i];
        Point p2 = waypoints[i + 1];
        Point p3 = waypoints[i + 2];
        
        for (int j = 0; j < pointsPerSegment; ++j) {
            double t = static_cast<double>(j) / pointsPerSegment;

            double t2 = t * t;
            double t3 = t2 * t;

            double x = 0.5 * (
                (2.0 * p1.first) +
                (-p0.first + p2.first) * t +
                (2.0 * p0.first - 5.0 * p1.first + 4.0 * p2.first - p3.first) * t2 +
                (-p0.first + 3.0 * p1.first - 3.0 * p2.first + p3.first) * t3
            );

            double y = 0.5 * (
                (2.0 * p1.second) + 
                (-p0.second + p2.second) * t +
                (2.0 * p0.second - 5.0 * p1.second + 4.0 * p2.second - p3.second) * t2 +
                (-p0.second + 3.0 * p1.second - 3.0 * p2.second + p3.second) * t3
            );

            result.emplace_back(x, y);
        }
    }

    // Optionally push last point
    result.push_back(waypoints[waypoints.size() - 2]);

    return result;
}
