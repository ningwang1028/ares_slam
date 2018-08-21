#ifndef GAUSS_NEWTEN_SCAN_MATCHER_H
#define GAUSS_NEWTEN_SCAN_MATCHER_H

#include <iostream>
#include <memory>
#include "laser_scan.h"
#include "correlative_grid.h"
#include "math_func.h"

namespace ares_slam
{

class GaussNewtenScanMatcher
{
public:
    GaussNewtenScanMatcher();
    ~GaussNewtenScanMatcher() {}

    void setCorrelativeGrid(float resolution, float standard_deviation);
    Eigen::Vector3f matchScan(const std::shared_ptr<LaserScan>& scan,
                              const std::vector<std::shared_ptr<LaserScan>>& base_scan);
    Eigen::Vector3f matchScanToMap(const Eigen::Vector3f& prior_pose, std::shared_ptr<CorrelativeGrid> map,
                                   const PointCloud& scan_data);
    std::shared_ptr<CorrelativeGrid> getCorrelativeGrid() {  return correlative_grid_; }

private:
    void bilinearInterpolation(const Eigen::Vector2f& coords, std::shared_ptr<CorrelativeGrid> map,
                               float& value, Eigen::Vector2f& derivative);

private:
    int max_iterations_;
    std::shared_ptr<CorrelativeGrid> correlative_grid_;
};

} // namespace ares_slam

#endif // GAUSS_NEWTEN_SCAN_MATCHING_H
