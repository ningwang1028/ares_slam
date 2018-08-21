#ifndef CORRELATIVE_SCAN_MATCHER_H
#define CORRELATIVE_SCAN_MATCHER_H

#include "math_func.h"
#include "laser_scan.h"
#include "correlative_grid.h"
#include <memory>

namespace ares_slam
{

class CorrelativeScanMatcher
{
public:
  CorrelativeScanMatcher() {}
    float multiResMatchScan(const std::shared_ptr<LaserScan>& scan, const std::vector<std::shared_ptr<LaserScan>>& base_scan,
                            Eigen::Vector3f& pose, Eigen::Matrix3f& covariance);
    float lowResMatchScan(const std::shared_ptr<LaserScan>& scan, const std::vector<std::shared_ptr<LaserScan>>& base_scan,
                          Eigen::Vector3f& pose, Eigen::Matrix3f& covariance);
    std::shared_ptr<CorrelativeGrid> getCorrelativeGrid() {  return correlative_grid_; }

    void setCorrelativeGrid(float resolution, float standard_deviation);
    void setCoarseXYSearchRange(float range) { coarse_xy_search_range_ = range; }
    void setCoarseXYSearchResolution(float resolution) { coarse_xy_search_resolution_ = resolution; }
    void setFineXYSearchRange(float range) { fine_xy_search_range_ = range; }
    void setFineXYSearchResolution(float resolution) { fine_xy_search_resolution_ = resolution; }

    void setCoarseAngleSearchRange(float range) { coarse_angle_search_range_ = range; }
    void setCoarseAngleSearchResolution(float resolution) { coarse_angle_search_resolution_ = resolution; }
    void setFineAngleSearchRange(float range) { fine_angle_search_range_ = range; }
    void setFineAngleSearchResolution(float resolution) { fine_angle_search_resolution_ = resolution; }

private:
    float matchScan(const std::shared_ptr<LaserScan>& scan, const Eigen::Vector3f& pose,
                       float xy_search_range, float xy_search_resolution,
                       float angle_search_range, float angle_search_resolution,
                       Eigen::Vector3f& best_pose, Eigen::Matrix3f& covariance,
                       bool compute_cov = true);
    void computeCovariance(std::vector<std::pair<Eigen::Vector3f, float>>& pose_score_pair,
                           const Eigen::Vector3f& best_pose, float best_score,
                           Eigen::Matrix3f& covariance);

private:
    std::shared_ptr<CorrelativeGrid> correlative_grid_;
    float coarse_xy_search_range_;
    float coarse_xy_search_resolution_;
    float fine_xy_search_range_;
    float fine_xy_search_resolution_;
    float coarse_angle_search_range_;
    float coarse_angle_search_resolution_;
    float fine_angle_search_range_;
    float fine_angle_search_resolution_;
};

} // namespace ares_slam

#endif // CORRELATIVE_SCAN_MATCHER_H
