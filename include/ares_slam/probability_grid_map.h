#ifndef PROBABILITY_GRID_MAP_H
#define PROBABILITY_GRID_MAP_H

#include <memory>
#include "grid_map.h"
#include "laser_scan.h"

namespace ares_slam
{

const uint8_t LogOdds_Unknown = 50;

class ProbabilityGridMap : public GridMap<uint8_t>
{
public:
    ProbabilityGridMap(int width, int height, float resolution) :
        GridMap(width, height, resolution), log_odds_free_(-1), log_odds_occupied_(2)
    {
        log_odds_min_ = 0;
        log_odds_max_ = 100;
        for(int i = 0; i < size_; ++i) {
             value_[i] = LogOdds_Unknown;
        }
    }
    ~ProbabilityGridMap() {}

    void createFromScan(const std::vector<std::shared_ptr<LaserScan>>& scans);

private:
    void bresenham(int x0, int y0, int x1, int y1, std::vector<Eigen::Vector2i>& cells);

private:
    int8_t log_odds_occupied_;
    int8_t log_odds_free_;
    int8_t log_odds_min_;
    int8_t log_odds_max_;
};

} // namespace ares_slam

#endif // PROBABILITY_GRID_MAP_H
