#ifndef CORRELATIVE_GRID_H
#define CORRELATIVE_GRID_H

#include "grid_map.h"
#include "laser_scan.h"
#include "memory"

namespace ares_slam
{

class CorrelativeGrid : public GridMap<uint8_t>
{
public:
    CorrelativeGrid(float resolution, float standard_deviation);
    virtual ~CorrelativeGrid();
    void setupGrid(int width, int height, const Eigen::Vector2f& origin);
    void setCorrelativePoint(const Eigen::Vector2i& point);
    void addScans(const std::vector<std::shared_ptr<LaserScan>>& base_scan);
    int getKernelSize() { return kernel_size_; }

private:
    void calculateKernel();

private:
    uint8_t* kernel_;
    int kernel_size_;
    float standard_deviation_;
};

} // namespace ares_slam

#endif // CORRELATIVE_GRID_H
