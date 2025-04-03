#include "trajectory/TrajectoryGenerator.h"
#include <tuple>
#include <cmath>

std::vector<std::tuple<double, double, double>> TrajectoryGenerator::generateCatmullRomSpline(
    const std::vector<std::tuple<double, double, double>>& timedPath,
    int pointsPerSegment
) {
    using std::get;
    std::vector<std::tuple<double, double, double>> result;
    if (timedPath.size() < 2) return timedPath;

    // Pad with duplicates at beginning and end
    std::vector<std::tuple<double, double, double>> padded;
    padded.push_back(timedPath.front());
    padded.insert(padded.end(), timedPath.begin(), timedPath.end());
    padded.push_back(timedPath.back());

    if (padded.size() < 4) return padded;

    for (size_t i = 1; i < padded.size() - 2; ++i) {
        const auto& p0 = padded[i - 1];
        const auto& p1 = padded[i];
        const auto& p2 = padded[i + 1];
        const auto& p3 = padded[i + 2];

        for (int j = 0; j < pointsPerSegment; ++j) {
            double t = static_cast<double>(j) / pointsPerSegment;
            double t2 = t * t;
            double t3 = t2 * t;

            double x = 0.5 * (
                (2.0 * get<0>(p1)) +
                (-get<0>(p0) + get<0>(p2)) * t +
                (2.0 * get<0>(p0) - 5.0 * get<0>(p1) + 4.0 * get<0>(p2) - get<0>(p3)) * t2 +
                (-get<0>(p0) + 3.0 * get<0>(p1) - 3.0 * get<0>(p2) + get<0>(p3)) * t3
            );

            double y = 0.5 * (
                (2.0 * get<1>(p1)) +
                (-get<1>(p0) + get<1>(p2)) * t +
                (2.0 * get<1>(p0) - 5.0 * get<1>(p1) + 4.0 * get<1>(p2) - get<1>(p3)) * t2 +
                (-get<1>(p0) + 3.0 * get<1>(p1) - 3.0 * get<1>(p2) + get<1>(p3)) * t3
            );

            // Linearly interpolate time between p1 and p2
            double time = (1.0 - t) * get<2>(p1) + t * get<2>(p2);

            result.emplace_back(x, y, time);
        }
    }

    result.push_back(padded[padded.size() - 2]);
    return result;
}

std::vector<std::tuple<double, double, double>> TrajectoryGenerator::applyTimeProfile(
    const std::vector<Point>& path,
    double velocity
) {
    std::vector<std::tuple<double, double, double>> trajectory;
    if (path.empty()) return trajectory;

    double time = 0.0;
    trajectory.emplace_back(std::get<0>(path[0]), std::get<1>(path[0]), time);

    for (size_t i = 1; i < path.size(); ++i) {
        double x0 = std::get<0>(path[i - 1]);
        double y0 = std::get<1>(path[i - 1]);
        double x1 = std::get<0>(path[i]);
        double y1 = std::get<1>(path[i]);

        double dist = std::hypot(x1 - x0, y1 - y0);
        time += dist / velocity;

        trajectory.emplace_back(x1, y1, time);
    }

    return trajectory;
}
