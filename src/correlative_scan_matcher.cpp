#include "correlative_scan_matcher.h"

namespace ares_slam
{

void CorrelativeScanMatcher::setCorrelativeGrid(float resolution, float standard_deviation)
{
    correlative_grid_ = std::shared_ptr<CorrelativeGrid>(new CorrelativeGrid(resolution, standard_deviation));
}

float CorrelativeScanMatcher::multiResMatchScan(const std::shared_ptr<LaserScan>& scan,
                                                const std::vector<std::shared_ptr<LaserScan>>& base_scan,
                                                Eigen::Vector3f& pose, Eigen::Matrix3f& covariance)
{
    correlative_grid_->addScans(base_scan);

    Eigen::Matrix3f coarse_covariance;
    float best_response = matchScan(scan, scan->getPose(),
                                    coarse_xy_search_range_, coarse_xy_search_resolution_,
                                    coarse_angle_search_range_, coarse_angle_search_resolution_,
                                    pose, coarse_covariance);

    Eigen::Matrix3f fine_covariance;
    best_response = matchScan(scan, pose,
                              fine_xy_search_range_, fine_xy_search_resolution_,
                              fine_angle_search_range_, fine_angle_search_resolution_,
                              pose, fine_covariance, false);

    covariance = coarse_covariance;
    return best_response;
}

float CorrelativeScanMatcher::lowResMatchScan(const std::shared_ptr<LaserScan>& scan,
                                              const std::vector<std::shared_ptr<LaserScan>>& base_scan,
                                              Eigen::Vector3f& pose, Eigen::Matrix3f& covariance)
{
    correlative_grid_->addScans(base_scan);

    float response = matchScan(scan, scan->getPose(),
                                    fine_xy_search_range_, fine_xy_search_resolution_,
                                    fine_angle_search_range_, fine_angle_search_resolution_,
                                    pose, covariance);

    return response;
}

float CorrelativeScanMatcher::matchScan(const std::shared_ptr<LaserScan>& scan, const Eigen::Vector3f& pose,
                                           float xy_search_range, float xy_search_resolution,
                                           float angle_search_range, float angle_search_resolution,
                                           Eigen::Vector3f& best_pose, Eigen::Matrix3f& covariance,
                                           bool compute_cov)
{
    int x_n = xy_search_range / xy_search_resolution + 1;
    int y_n = x_n;
    int angle_n = angle_search_range / angle_search_resolution + 1;

    const PointCloud& point_cloud = scan->getRawPointCloud();

    Eigen::Vector3f pose_offset(pose[0] - xy_search_range / 2.0f, pose[1] - xy_search_range / 2.0f,
                    pose[2] - angle_search_range / 2.0f);
    Eigen::Vector3f new_pose;

    std::vector<std::pair<Eigen::Vector3f, float>> pose_score_pair;
    pose_score_pair.reserve(x_n * y_n * angle_n);
    std::vector<int> angle_indexes(point_cloud.size());

    Eigen::Vector2f origin_coords = correlative_grid_->getMapCoords(Eigen::Vector2f(0, 0));
    int origin_index = correlative_grid_->getIndex(floor(origin_coords[0]), floor(origin_coords[1]));

    for(int angle_index = 0; angle_index < angle_n; ++angle_index) {
        new_pose[2] = normalizeAngle(angle_index * angle_search_resolution + pose_offset[2]);
        Eigen::Rotation2Df transform(new_pose[2]);

        for(int i = 0; i < point_cloud.size(); ++i) {
            Eigen::Vector2f map_coords = correlative_grid_->getMapCoords(transform * point_cloud[i]);
            angle_indexes[i] = correlative_grid_->getIndex(floor(map_coords[0]), floor(map_coords[1]));
        }

        for(int x_index = 0; x_index < x_n; ++x_index) {
            new_pose[0] = x_index * xy_search_resolution + pose_offset[0];

            for(int y_index = 0; y_index < y_n; ++y_index) {
                new_pose[1] = y_index * xy_search_resolution + pose_offset[1];
                Eigen::Vector2f map_coords = correlative_grid_->getMapCoords(new_pose);
                int xy_index = correlative_grid_->getIndex(floor(map_coords[0]), floor(map_coords[1])) - origin_index;

                float sum_score = 0, score = 0;
                for(const int& index : angle_indexes) {
                    int new_index = index + xy_index;
                    if(!correlative_grid_->isOutOfMap(new_index)) {
                        score = correlative_grid_->getGridValue(new_index);
                        sum_score += score;
                    }
                }
                score = sum_score / (point_cloud.size() * GridStates_Occupied);
                pose_score_pair.emplace_back(new_pose, score);
            }
        }
    }

    float best_score = -1.0f;
    for(const std::pair<Eigen::Vector3f, float>& pose_score: pose_score_pair) {
        if(pose_score.second > best_score) {
            best_pose = pose_score.first;
            best_score = pose_score.second;
        }
    }

    if(compute_cov) {
        computeCovariance(pose_score_pair, best_pose, best_score, covariance);
    }

    return best_score;
}

void CorrelativeScanMatcher::computeCovariance(std::vector<std::pair<Eigen::Vector3f, float>>& pose_score_pair,
                                               const Eigen::Vector3f& best_pose, float best_score,
                                               Eigen::Matrix3f& covariance)
{
    covariance.setIdentity();

    if(best_score < 1e-6) {
        covariance(0, 0) = std::numeric_limits<float>::max();
        covariance(1, 1) = std::numeric_limits<float>::max();
        covariance(2, 2) = std::numeric_limits<float>::max();
    }

    float s = 0;
    Eigen::Vector3f u = Eigen::Vector3f::Zero();
    Eigen::Matrix3f k = Eigen::Matrix3f::Zero();

    for(const std::pair<Eigen::Vector3f, float>& pose_score: pose_score_pair) {
        if(pose_score.second >= best_score - 0.1f) {
            s += pose_score.second;
            u += pose_score.first * pose_score.second;
            k += pose_score.first * pose_score.first.transpose() * pose_score.second;
        }
    }
    covariance =  k / s - u * u.transpose() / (s * s);
}

} // namespace ares_slam
