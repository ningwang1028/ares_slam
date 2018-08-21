#include "map_builder.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/types/slam2d/types_slam2d.h"

namespace ares_slam
{
typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> SlamBlockSolver;
typedef g2o::LinearSolverCholmod<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

MapBuilder::MapBuilder() :
    occupancy_grid_map_resolution_(0.05f), min_update_distance_(0.2f),
    min_update_orientation_(degToRad(10)), scan_buffer_size_(30),
    loop_scan_search_distance_(10.0f), loop_match_min_chain_size_(5),
    loop_closure_min_response_(0.65f), loop_closure_xy_variance_threshold_(0.01f),
    loop_closure_angle_variance_threshold_(0.1f), loop_closure_xy_search_range_(5.0f),
    loop_closure_angle_search_range_(degToRad(20)), loop_closure_grid_resolution_(0.05f),
    loop_closure_coarse_xy_search_resolution_(0.1f), loop_closure_fine_xy_search_range_(0.2f),
    loop_closure_coarse_angle_search_resolution_(degToRad(2)),
    loop_closure_fine_angle_search_range_(degToRad(2)),
    loop_closure_fine_angle_search_resolution_(degToRad(0.2)),
    optimize_every_n_constraint_(20), search_space_standard_deviation_(0.03f),
    gn_scan_matcher_grid_resolution_(0.05f), use_correlative_scan_matcher_(false),
    csm_xy_search_range_(0.1f), csm_angle_search_range_(degToRad(10)),
    csm_grid_resolution_(0.02f), csm_xy_search_resolution_(0.02f),
    csm_angle_search_resolution_(degToRad(0.2)),
    loop_constraint_count_(0), got_first_scan_(false), got_first_odom_(false)
{
    SlamBlockSolver::LinearSolverType* linear_solver = new SlamLinearSolver;
    SlamBlockSolver* solver_ptr = new SlamBlockSolver(linear_solver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); // L-M
    optimizer_.setAlgorithm(solver);
    optimizer_.setVerbose(false);
}

void MapBuilder::initialize()
{
    std::cout << "occupancy_grid_map_resolution : " << occupancy_grid_map_resolution_ << std::endl;
    std::cout << "min_update_distance : " << min_update_distance_ << std::endl;
    std::cout << "min_update_orientation : " << min_update_orientation_ << std::endl;
    std::cout << "scan_buffer_size : " << scan_buffer_size_ << std::endl;
    std::cout << "loop_scan_search_distance : " << loop_scan_search_distance_ << std::endl;
    std::cout << "loop_match_min_chain_size : " << loop_match_min_chain_size_ << std::endl;
    std::cout << "loop_closure_min_response : " << loop_closure_min_response_ << std::endl;
    std::cout << "loop_closure_xy_variance_threshold : " << loop_closure_xy_variance_threshold_ << std::endl;
    std::cout << "loop_closure_angle_variance_threshold : " << loop_closure_angle_variance_threshold_ << std::endl;
    std::cout << "optimize_every_n_constraint : " << optimize_every_n_constraint_ << std::endl;

    std::cout << "loop_closure_xy_search_range : " << loop_closure_xy_search_range_ << std::endl;
    std::cout << "loop_closure_angle_search_range : " << loop_closure_angle_search_range_ << std::endl;
    std::cout << "loop_closure_grid_resolution : " << loop_closure_grid_resolution_ << std::endl;
    std::cout << "loop_closure_coarse_xy_search_resolution : " << loop_closure_coarse_xy_search_resolution_ << std::endl;
    std::cout << "loop_closure_fine_xy_search_range : " << loop_closure_fine_xy_search_range_ << std::endl;
    std::cout << "loop_closure_coarse_angle_search_resolution : " << loop_closure_coarse_angle_search_resolution_ << std::endl;
    std::cout << "loop_closure_fine_angle_search_range : " << loop_closure_fine_angle_search_range_ << std::endl;
    std::cout << "loop_closure_fine_angle_search_resolution : " << loop_closure_fine_angle_search_resolution_ << std::endl;

    gauss_newten_scan_matcher_.setCorrelativeGrid(gn_scan_matcher_grid_resolution_, search_space_standard_deviation_);

    loop_closure_scan_matcher_.setCoarseXYSearchRange(loop_closure_xy_search_range_);
    loop_closure_scan_matcher_.setCoarseXYSearchResolution(loop_closure_coarse_xy_search_resolution_);
    loop_closure_scan_matcher_.setFineXYSearchRange(loop_closure_fine_xy_search_range_);
    loop_closure_scan_matcher_.setFineXYSearchResolution(loop_closure_grid_resolution_);
    loop_closure_scan_matcher_.setCoarseAngleSearchRange(loop_closure_angle_search_range_);
    loop_closure_scan_matcher_.setCoarseAngleSearchResolution(loop_closure_coarse_angle_search_resolution_);
    loop_closure_scan_matcher_.setFineAngleSearchRange(loop_closure_fine_angle_search_range_);
    loop_closure_scan_matcher_.setFineAngleSearchResolution(loop_closure_fine_angle_search_resolution_);
    loop_closure_scan_matcher_.setCorrelativeGrid(loop_closure_grid_resolution_, search_space_standard_deviation_);

    if(use_correlative_scan_matcher_) {
        std::cout << "csm_xy_search_range : " << csm_xy_search_range_ << std::endl;
        std::cout << "csm_angle_search_range : " << csm_angle_search_range_ << std::endl;
        std::cout << "csm_grid_resolution : " << csm_grid_resolution_ << std::endl;
        std::cout << "csm_xy_search_resolution : " << csm_xy_search_resolution_ << std::endl;
        std::cout << "csm_angle_search_resolution : " << csm_angle_search_resolution_ << std::endl;

        correlative_scan_matcher_.setFineXYSearchRange(csm_xy_search_range_);
        correlative_scan_matcher_.setFineXYSearchResolution(csm_grid_resolution_);
        correlative_scan_matcher_.setFineAngleSearchRange(csm_angle_search_range_);
        correlative_scan_matcher_.setFineAngleSearchResolution(csm_angle_search_resolution_);
        correlative_scan_matcher_.setCorrelativeGrid(csm_grid_resolution_, search_space_standard_deviation_);
    }
}

bool MapBuilder::checkPose(const std::shared_ptr<LaserScan>& laser_scan)
{
    std::shared_ptr<LaserScan>& last_laser_scan = scans_.back();

    Eigen::Vector3f delta_pose = last_laser_scan->getPose() - laser_scan->getPose();

    float update_distance = delta_pose[0] * delta_pose[0] + delta_pose[1] * delta_pose[1];
    float update_angle = fabs(normalizeAngle(delta_pose[2]));

    if(update_distance >= min_update_distance_ * min_update_distance_ || update_angle >= min_update_orientation_) {
        return true;
    }
    else {
        return false;
    }
}

void MapBuilder::addRunningScan(std::shared_ptr<LaserScan> laser_scan)
{
    running_scan_.push_back(laser_scan);

    while (running_scan_.size() > scan_buffer_size_) {
        running_scan_.erase(running_scan_.begin());
    }
}

void MapBuilder::addVertex(std::shared_ptr<LaserScan> scan)
{
    Eigen::Vector3f pose = scan->getPose();
    g2o::VertexSE2* vertex = new g2o::VertexSE2;
    vertex->setEstimate(g2o::SE2(pose[0], pose[1], pose[2]));
    vertex->setId(scan->getId());
    optimizer_.addVertex(vertex);
}

void MapBuilder::addEdge(std::shared_ptr<LaserScan> source_scan, const Eigen::Vector3f& source_pose,
                         std::shared_ptr<LaserScan> target_scan, const Eigen::Vector3f& target_pose,
                         const Eigen::Matrix3d& information)
{
    static int edge_count = 0;
    Eigen::AngleAxisf rotation(-source_pose[2], Eigen::Vector3f(0, 0, 1));
    Eigen::Vector3f delta_pose = rotation * (target_pose - source_pose);

    g2o::EdgeSE2* edge = new g2o::EdgeSE2;

    int source_id = source_scan->getId();
    int target_id = target_scan->getId();

    edge->vertices()[0] = optimizer_.vertex(source_id);
    edge->vertices()[1] = optimizer_.vertex(target_id);

    g2o::SE2 measurement(delta_pose[0], delta_pose[1], delta_pose[2]);
    edge->setId(edge_count);
    edge->setMeasurement(measurement);
    edge->setInformation(information);
    edge_count++;

    optimizer_.addEdge(edge);
}

std::shared_ptr<LaserScan> MapBuilder::getClosestScan(const std::shared_ptr<LaserScan>& base_scan,
                                                     const std::vector<std::shared_ptr<LaserScan>>& chain)
{
    float min_distance = std::numeric_limits<float>::max();
    std::shared_ptr<LaserScan> closest_scan;

    for(const std::shared_ptr<LaserScan>& scan : chain) {
        float distance = (scan->getPose().head<2>() - base_scan->getPose().head<2>()).norm();
        if(min_distance > distance) {
            min_distance = distance;
            closest_scan = scan;
        }
    }

    return closest_scan;
}

void MapBuilder::detectLoopClosure(std::shared_ptr<LaserScan> scan)
{
    std::vector<std::shared_ptr<LaserScan>> scan_chain;
    std::vector<std::vector<std::shared_ptr<LaserScan>>> scan_chains;

    int n = scans_.size();
    for(int i = 0; i < n; ++i) {
        float distance = (scan->getPose().head<2>() - scans_[i]->getPose().head<2>()).norm();

        if(distance < loop_scan_search_distance_) {
            if(std::find(running_scan_.begin(), running_scan_.end(), scans_[i]) == running_scan_.end()) {
                scan_chain.push_back(scans_[i]);
            }
            else {
                scan_chain.clear();
            }
        }
        else {
            if(scan_chain.size() > loop_match_min_chain_size_) {
                scan_chains.push_back(scan_chain);
                scan_chain.clear();
            }
            else {
                scan_chain.clear();
            }
        }
    }

    if(scan_chains.empty()) {
        return;
    }

    for(const std::vector<std::shared_ptr<LaserScan>>& chain : scan_chains) {
        Eigen::Vector3f csm_pose;
        Eigen::Matrix3f covariance;
        auto t1 = std::chrono::steady_clock::now();
        float response = loop_closure_scan_matcher_.multiResMatchScan(scan, chain, csm_pose, covariance);
        auto t2 = std::chrono::steady_clock::now();
        auto delta_t = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

        if(response < loop_closure_min_response_) {
            continue;
        }

        if(covariance(0, 0) > loop_closure_xy_variance_threshold_ ||
           covariance(1, 1) > loop_closure_xy_variance_threshold_) {
//            std::cout << "Reject loop closure with large covariance : " << std::endl;
//            std::cout << covariance << std::endl;
            continue;
        }

        Eigen::Vector3f gnsm_pose = gauss_newten_scan_matcher_.matchScanToMap(csm_pose, loop_closure_scan_matcher_.getCorrelativeGrid(),
                                                                  scan->getRawPointCloud());

        float delta_trans = (gnsm_pose.head<2>() - csm_pose.head<2>()).norm();
        float delta_angle = normalizeAngle(gnsm_pose[2] - csm_pose[2]);
        if(fabs(delta_trans) > 0.15 && fabs(delta_angle) > 0.1) {
            std::cout <<  "Loop closure gauss newten scan matcher result got large jump with delta_trans = " << delta_trans
                      << " delta_angle = " << delta_angle << std::endl;
        }

        loop_constraint_count_++;
//        std::cout << "Find loop closure with response " << response << ". Cost time " << delta_t.count() * 1000.0 << "ms." << std::endl;
        std::shared_ptr<LaserScan> closest_scan = getClosestScan(scan, chain);
        addEdge(scan, csm_pose, closest_scan, closest_scan->getPose(), Eigen::Matrix3d::Identity() * response);
    }
}

void MapBuilder::doPoseAdjustment()
{
    std::vector<Eigen::Vector3f> poses;

    g2o::OptimizableGraph::Vertex* v = optimizer_.vertex(0);
    v->setFixed(true);

    optimizer_.initializeOptimization();

    auto t1 = std::chrono::steady_clock::now();
    int iter = optimizer_.optimize(100);
    auto t2 = std::chrono::steady_clock::now();
    auto delta_t = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

    if (iter > 0) {
        std::cout << "Optimization finished after " << iter << " iterations. Cost time " <<
                     delta_t.count() * 1000.0 << "ms." << std::endl;
    }
    else {
        std::cout << "Optimization failed, result might be invalid!" << std::endl;
        return;
    }

    g2o::SparseOptimizer::VertexContainer nodes = optimizer_.activeVertices();

    for(g2o::SparseOptimizer::VertexContainer::const_iterator n = nodes.begin(); n != nodes.end(); ++n) {
        double estimate[3];
        if((*n)->getEstimateData(estimate)) {
            Eigen::Vector3f pose(estimate[0], estimate[1], estimate[2]);
            poses.push_back(pose);
        }
        else {
            std::cout <<"Could not get estimated pose from Optimizer!" << std::endl;
        }
    }

    for(int i = 0; i < scans_.size(); ++i) {
        scans_[i]->setPose(poses[i]);
        scans_[i]->transformPointCloud();
    }
}

void MapBuilder::getGraph(std::vector<Eigen::Vector2f>& nodes,
                          std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f> >& edges)
{
    double data[3];

    for(g2o::SparseOptimizer::VertexIDMap::iterator it = optimizer_.vertices().begin(); it != optimizer_.vertices().end(); ++it) {
        g2o::VertexSE2* v = dynamic_cast<g2o::VertexSE2*>(it->second);
        v->getEstimateData(data);
        Eigen::Vector2f pose(data[0], data[1]);
        nodes.push_back(pose);
    }

   for(g2o::SparseOptimizer::EdgeSet::iterator it = optimizer_.edges().begin(); it != optimizer_.edges().end(); ++it) {
       g2o::EdgeSE2* e = dynamic_cast<g2o::EdgeSE2*>(*it);
       g2o::VertexSE2* v1 = dynamic_cast<g2o::VertexSE2*>(e->vertices()[0]);
       g2o::VertexSE2* v2 = dynamic_cast<g2o::VertexSE2*>(e->vertices()[1]);
       v1->getEstimateData(data);
       Eigen::Vector2f pose1(data[0], data[1]);
       v2->getEstimateData(data);
       Eigen::Vector2f pose2(data[0], data[1]);
       edges.push_back(std::make_pair(pose1, pose2));
   }
}

void MapBuilder::addOdom(const Eigen::Vector3f& pose)
{
    if(!got_first_odom_) {
        last_odom_pose_ = pose;
        got_first_odom_ = true;
    }

    odom_pose_ = pose;
}

void MapBuilder::addLaserScan(std::shared_ptr<LaserScan> laser_scan)
{
    if(!got_first_scan_) {
        laser_scan->setId(scans_.size());
        laser_scan->setPose(Eigen::Vector3f(0, 0, 0));
        laser_scan->transformPointCloud();
        scans_.push_back(laser_scan);
        running_scan_.push_back(laser_scan);
        got_first_scan_ = true;
        addVertex(laser_scan);
        return;
    }

    Eigen::Vector3f last_scan_pose = scans_.back()->getPose();
    Eigen::AngleAxisf rotation(last_scan_pose[2] - last_odom_pose_[2], Eigen::Vector3f(0, 0, 1));
    Eigen::Vector3f update_pose = last_scan_pose + rotation * (odom_pose_ - last_odom_pose_);
    laser_scan->setPose(update_pose);

    if(!checkPose(laser_scan)) {
        return;
    }

    last_odom_pose_ = odom_pose_;

    auto t1 = std::chrono::steady_clock::now();
    if(use_correlative_scan_matcher_) {
        Eigen::Vector3f csm_pose;
        Eigen::Matrix3f covariance;

        float response = correlative_scan_matcher_.lowResMatchScan(laser_scan, running_scan_, csm_pose, covariance);
        laser_scan->setPose(csm_pose);

        Eigen::Vector3f gnsm_pose  = gauss_newten_scan_matcher_.matchScanToMap(csm_pose, correlative_scan_matcher_.getCorrelativeGrid(),
                                                                               laser_scan->getRawPointCloud());

        float delta_trans = (gnsm_pose.head<2>() - csm_pose.head<2>()).norm();
        float delta_angle = normalizeAngle(gnsm_pose[2] - csm_pose[2]);
        if(fabs(delta_trans) < 0.04 && fabs(delta_angle) < 0.03) {
            laser_scan->setPose(gnsm_pose);
        }
        else {
            laser_scan->setPose(csm_pose);
            std::cout << "Gauss newten scan matcher result got large jump with delta_trans = " << delta_trans
                      << " delta_angle = " << delta_angle << std::endl;
        }
    }
    else {
        Eigen::Vector3f gnsm_pose = gauss_newten_scan_matcher_.matchScan(laser_scan, running_scan_);

        float delta_trans = (gnsm_pose.head<2>() - update_pose.head<2>()).norm();
        float delta_angle = normalizeAngle(gnsm_pose[2] - update_pose[2]);
        if(fabs(delta_trans) < 0.15 && fabs(delta_angle) < 0.15) {
            laser_scan->setPose(gnsm_pose);
        }
        else {
            std::cout << "Gauss newten scan matcher result got large jump with delta_trans = " << delta_trans
                      << " delta_angle = " << delta_angle << std::endl;
        }
    }

    auto t2 = std::chrono::steady_clock::now();
    auto delta_t = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
//    std::cout << "delta_t = " << delta_t.count() * 1000.0 << "ms." << std::endl;

    laser_scan->setId(scans_.size());
    laser_scan->transformPointCloud();

    scans_.push_back(laser_scan);
    addRunningScan(laser_scan);

    addVertex(laser_scan);
    addEdge(scans_[laser_scan->getId() - 1], scans_[laser_scan->getId() - 1]->getPose(),
            laser_scan, laser_scan->getPose(),
            Eigen::Matrix3d::Identity());
    detectLoopClosure(laser_scan);

    if(loop_constraint_count_ >= optimize_every_n_constraint_) {
        doPoseAdjustment();
        optimize_time_ = std::chrono::steady_clock::now();
        loop_constraint_count_ = 0;
    }

    if(loop_constraint_count_ > 0) {
        auto delta_t = std::chrono::duration_cast<std::chrono::duration<double>>(
                       std::chrono::steady_clock::now() - optimize_time_);
        if(delta_t.count() > 10.0) {
            doPoseAdjustment();
            optimize_time_ = std::chrono::steady_clock::now();
            loop_constraint_count_ = 0;
        }
    }
}

std::vector<Eigen::Vector3f> MapBuilder::getPath()
{
    std::vector<Eigen::Vector3f> path;
    for(const std::shared_ptr<LaserScan>& scan : scans_) {
        path.push_back(scan->getPose());
    }

    return path;
}

std::shared_ptr<CorrelativeGrid> MapBuilder::getCorrelativeGrid()
{
    if(use_correlative_scan_matcher_) {
        return correlative_scan_matcher_.getCorrelativeGrid();
    }
    else {
        return gauss_newten_scan_matcher_.getCorrelativeGrid();
    }
}

std::shared_ptr<OccupancyGridMap> MapBuilder::getOccupancyGridMap()
{
    Range map_range;
    for(const std::shared_ptr<LaserScan>& scan : scans_) {
        map_range.addRange(scan->getRange());
    }

    const Eigen::Vector2f& max = map_range.getMax();
    const Eigen::Vector2f& min = map_range.getMin();
    int width = ceil((max[0] - min[0]) / occupancy_grid_map_resolution_);
    int height = ceil((max[1] - min[1]) / occupancy_grid_map_resolution_);

    std::shared_ptr<OccupancyGridMap> occupancy_grid_map(new OccupancyGridMap(width, height, occupancy_grid_map_resolution_));
    occupancy_grid_map->setOrigin(min);

    std::vector<std::shared_ptr<LaserScan>> all_scan = scans_;
    occupancy_grid_map->createFromScan(all_scan);

    return occupancy_grid_map;
}

std::shared_ptr<ProbabilityGridMap> MapBuilder::getProbabilityGridMap()
{
    Range map_range;
    for(const std::shared_ptr<LaserScan>& scan : scans_) {
        map_range.addRange(scan->getRange());
    }

    const Eigen::Vector2f& max = map_range.getMax();
    const Eigen::Vector2f& min = map_range.getMin();
    int width = ceil((max[0] - min[0]) / occupancy_grid_map_resolution_);
    int height = ceil((max[1] - min[1]) / occupancy_grid_map_resolution_);

    std::shared_ptr<ProbabilityGridMap> probability_grid_map(new ProbabilityGridMap(width, height, occupancy_grid_map_resolution_));
    probability_grid_map->setOrigin(min);

    std::vector<std::shared_ptr<LaserScan>> all_scan = scans_;
    probability_grid_map->createFromScan(all_scan);

    return probability_grid_map;
}

} // namespace ares_slam
