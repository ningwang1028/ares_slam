#ifndef MAP_BUILDER_H
#define MAP_BUILDER_H

#include <chrono>
#include "math_func.h"
#include "laser_scan.h"
#include "occupancy_grid_map.h"
#include "probability_grid_map.h"
#include "correlative_scan_matcher.h"
#include "gauss_newten_scan_matcher.h"
#include "g2o/core/sparse_optimizer.h"

namespace ares_slam
{

class MapBuilder
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapBuilder();
    ~MapBuilder() {}

    void initialize();
    void addOdom(const Eigen::Vector3f& pose);
    void addLaserScan(std::shared_ptr<LaserScan> laser_scan);
    std::shared_ptr<OccupancyGridMap> getOccupancyGridMap();
    std::shared_ptr<ProbabilityGridMap> getProbabilityGridMap();
    std::shared_ptr<CorrelativeGrid> getCorrelativeGrid();
    void getGraph(std::vector<Eigen::Vector2f>& nodes,
                  std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> > &edges);
    std::vector<Eigen::Vector3f> getPath();
    void doPoseAdjustment();

    void setOccupancyGridMapResolution(float res) { occupancy_grid_map_resolution_ = res; }
    void setMinUpdateDistance(float distance) { min_update_distance_ = distance; }
    void setMinUpdateOrientation(float angle) { min_update_orientation_ = angle; }
    void setScanBufferSize(int num) { scan_buffer_size_ = num; }
    void setLoopScanSearchDistance(float distance) { loop_scan_search_distance_ = distance; }
    void setLoopMatchMinChainSize(int size) { loop_match_min_chain_size_ = size; }
    void setLoopClosureMinResponse(float response) { loop_closure_min_response_ = response; }
    void setLoopClosureXYVarianceThreshold(float variance) { loop_closure_xy_variance_threshold_ = variance; }
    void setLoopClosureAngleVarianceThreshold(float variance) { loop_closure_angle_variance_threshold_ = variance; }
    void setOptimizeEveryNConstraints(int n) { optimize_every_n_constraint_ = n; }

    void setLoopClosureXYSearchRange(float range) { loop_closure_xy_search_range_ = range; }
    void setLoopClosureAngleSearchRange(float range) { loop_closure_angle_search_range_ = range; }
    void setLoopClosureGridResolution(float res) { loop_closure_grid_resolution_ = res; }
    void setLoopClosureCoarseXYSearchResolution(float res) { loop_closure_coarse_xy_search_resolution_ = res; }
    void setLoopClosureFineXYSearchRange(float range) { loop_closure_fine_xy_search_range_ = range; }
    void setLoopClosureCoarseAngleSearchResolution(float res) { loop_closure_coarse_angle_search_resolution_ = res; }
    void setLoopClosureFineAngleSearchRange(float range) { loop_closure_fine_angle_search_range_ = range; }
    void setLoopClosureFineAngleSearchResolution(float res) { loop_closure_fine_angle_search_resolution_ = res; }

    void useCorrelativeScanMatcher(bool flag) { use_correlative_scan_matcher_ = flag; }
    void setCSMXYSearchRange(float range) { csm_xy_search_range_ = range; }
    void setCSMAngleSearchRange(float range) { csm_angle_search_range_ = range; }
    void setCSMGridResolution(float res) { csm_grid_resolution_ = res; }
    void setCSMXYSearchResolution(float res) { csm_xy_search_resolution_ = res; }
    void setCSMAngleSearchResolution(float res) { csm_angle_search_resolution_ = res; }

private:
    bool checkPose(const std::shared_ptr<LaserScan>& laser_scan);
    void addRunningScan(std::shared_ptr<LaserScan> laser_scan);
    void addVertex(std::shared_ptr<LaserScan> scan);
    void addEdge(std::shared_ptr<LaserScan> source_scan, const Eigen::Vector3f& source_pose,
                 std::shared_ptr<LaserScan> target_scan, const Eigen::Vector3f& target_pose,
                 const Eigen::Matrix3d& information);
    std::shared_ptr<LaserScan> getClosestScan(const std::shared_ptr<LaserScan>& base_scan,
                                             const std::vector<std::shared_ptr<LaserScan>>& chain);
    void detectLoopClosure(std::shared_ptr<LaserScan> scan);

private:
    // parameter
    float occupancy_grid_map_resolution_;
    float min_update_distance_;
    float min_update_orientation_;
    int scan_buffer_size_;

    float loop_scan_search_distance_;
    float loop_closure_min_response_;
    float loop_closure_xy_variance_threshold_;
    float loop_closure_angle_variance_threshold_;
    int loop_match_min_chain_size_;
    int optimize_every_n_constraint_;

    float gn_scan_matcher_grid_resolution_;

    float loop_closure_xy_search_range_;
    float loop_closure_angle_search_range_;
    float loop_closure_grid_resolution_;
    float loop_closure_coarse_xy_search_resolution_;
    float loop_closure_fine_xy_search_range_;
    float loop_closure_coarse_angle_search_resolution_;
    float loop_closure_fine_angle_search_range_;
    float loop_closure_fine_angle_search_resolution_;

    bool use_correlative_scan_matcher_;
    float csm_xy_search_range_;
    float csm_angle_search_range_;
    float csm_grid_resolution_;
    float csm_xy_search_resolution_;
    float csm_angle_search_resolution_;

    float search_space_standard_deviation_;

    bool got_first_scan_;
    bool got_first_odom_;
    int loop_constraint_count_;
    Eigen::Vector3f last_odom_pose_;
    Eigen::Vector3f odom_pose_;

    std::vector<std::shared_ptr<LaserScan>> scans_;
    std::vector<std::shared_ptr<LaserScan>> running_scan_;

    CorrelativeScanMatcher correlative_scan_matcher_;
    CorrelativeScanMatcher loop_closure_scan_matcher_;
    GaussNewtenScanMatcher gauss_newten_scan_matcher_;

    std::chrono::steady_clock::time_point optimize_time_;

    g2o::SparseOptimizer optimizer_;
};

} // namespace ares_slam

#endif // MAP_BUILDER_H
