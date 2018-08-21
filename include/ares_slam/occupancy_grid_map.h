#ifndef OCCUPANCY_GRID_MAP_H
#define OCCUPANCY_GRID_MAP_H

#include <memory>
#include "grid_map.h"
#include "laser_scan.h"

namespace ares_slam
{

class OccupancyGridMap : public GridMap<uint8_t>
{
public:
    OccupancyGridMap(int width, int height, float resolution) :
        GridMap(width, height, resolution), occupancy_threshold_(0.1f), min_pass_threshold_(2)
    {
        cell_pass_cnt_ = new int[size_];
        memset(cell_pass_cnt_, 0, size_ * sizeof(int));
        cell_hit_cnt_ = new int[size_];
        memset(cell_hit_cnt_, 0, size_ * sizeof(int));
        clear();
    }
    ~OccupancyGridMap()
    {
        if(cell_pass_cnt_) {
            delete[] cell_pass_cnt_;
        }
        if(cell_hit_cnt_) {
            delete[] cell_hit_cnt_;
        }
    }
    void createFromScan(const std::vector<std::shared_ptr<LaserScan>>& scans);
    bool isFree(int index) { return value_[index] == GridStates_Free; }
    bool isOccupied(int index) { return value_[index] == GridStates_Occupied; }

private:
    void bresenham(int x0, int y0, int x1, int y1, std::vector<Eigen::Vector2i>& cells);

private:
    float occupancy_threshold_;
    int min_pass_threshold_;
    int* cell_pass_cnt_;
    int* cell_hit_cnt_;
};

} // namespace ares_slam

#endif // OCCUPANCY_GRID_MAP_H
