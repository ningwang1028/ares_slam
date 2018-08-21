#include "probability_grid_map.h"

namespace ares_slam
{

void ProbabilityGridMap::createFromScan(const std::vector<std::shared_ptr<LaserScan>>& scans)
{
    for(const std::shared_ptr<LaserScan>& scan : scans) {
        Eigen::Vector2f start = getMapCoords(scan->getPose());
        const PointCloud& point_cloud = scan->getTransformedPointCloud();
        for(const Eigen::Vector2f& point : point_cloud) {
            Eigen::Vector2f end = getMapCoords(point);
            std::vector<Eigen::Vector2i> points;
            bresenham(start[0],  start[1], end[0], end[1], points);

            int n = points.size();
            if(n == 0) {
                continue;
            }

            for(int j = 0; j < n - 1; ++j) {
                int index = getIndex(points[j][0], points[j][1]);
                if(value_[index] + log_odds_free_ >= log_odds_min_) {
                    value_[index] += log_odds_free_;
                }
            }

            int index = getIndex(points[n - 1][0], points[n - 1][1]);
            if(value_[index] + log_odds_occupied_ <= log_odds_max_) {
                value_[index] += log_odds_occupied_;
            }
        }
    }
}

void ProbabilityGridMap::bresenham(int x0, int y0, int x1, int y1, std::vector<Eigen::Vector2i>& cells)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = (dx > dy ? dx : -dy) / 2;

    Eigen::Vector2i point(x0, y0);
    cells.push_back(point);

    while (x0 != x1 || y0 != y1) {
        int e2 = err;
        if (e2 > -dx) { err -= dy; x0 += sx; }
        if (e2 <  dy) { err += dx; y0 += sy; }
        point[0] = x0;
        point[1] = y0;
        cells.push_back(point);
    }
}

} // namespace ares_slam
