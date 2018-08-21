#include "gauss_newten_scan_matcher.h"

namespace ares_slam
{

GaussNewtenScanMatcher::GaussNewtenScanMatcher() : max_iterations_(25)
{

}

void GaussNewtenScanMatcher::setCorrelativeGrid(float resolution, float standard_deviation)
{
    correlative_grid_ = std::shared_ptr<CorrelativeGrid>(new CorrelativeGrid(resolution, standard_deviation));
}

void GaussNewtenScanMatcher::bilinearInterpolation(const Eigen::Vector2f& coords, std::shared_ptr<CorrelativeGrid> map,
                                                   float& value, Eigen::Vector2f& derivative)
{
    if(map->isOutOfMap(coords)) {
        value = 0.0;
        derivative = Eigen::Vector2f::Zero();
        return;
    }

    Eigen::Vector2i coords_floor(coords.cast<int>());
    Eigen::Vector2f factors(coords - coords_floor.cast<float>());
    Eigen::Vector2f factors_inv(1.0f - factors[0], 1.0f - factors[1]);

    int size_x = map->getSizeX();
    int index = coords_floor[1] * size_x + coords_floor[0];
    float intensities[4];

    intensities[0] = static_cast<float>(map->getGridValue(index)) / 255.0f;
    intensities[1] = static_cast<float>(map->getGridValue(index + 1)) / 255.0f;
    intensities[2] = static_cast<float>(map->getGridValue(index + size_x)) / 255.0f;
    intensities[3] = static_cast<float>(map->getGridValue(index + size_x + 1)) / 255.0f;

    float dx1 = intensities[0] - intensities[1];
    float dx2 = intensities[2] - intensities[3];

    float dy1 = intensities[0] - intensities[2];
    float dy2 = intensities[1] - intensities[3];

    value = ((intensities[0] * factors_inv[0] + intensities[1] * factors[0]) * factors_inv[1]) +
            ((intensities[2] * factors_inv[0] + intensities[3] * factors[0]) * (factors[1]));

    derivative = Eigen::Vector2f(-((dx1 * factors_inv[0]) + (dx2 * factors[0])),
                                 -((dy1 * factors_inv[1]) + (dy2 * factors[1])));
}

Eigen::Vector3f GaussNewtenScanMatcher::matchScanToMap(const Eigen::Vector3f& prior_pose,
                                                        std::shared_ptr<CorrelativeGrid> map,
                                                        const PointCloud& scan_data)
{
    Eigen::Vector3f pose(map->getMapPose(prior_pose));
    Eigen::Vector3f last_pose = pose;

    double cost = 0, last_cost = 0;

    for(int i = 0; i < max_iterations_; ++i) {
        Eigen::Affine2f transform(Eigen::Translation2f(pose[0], pose[1]) * Eigen::Rotation2Df(pose[2]));

        float sin_rot = sin(pose[2]);
        float cos_rot = cos(pose[2]);

        Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
        Eigen::Vector3f JTr = Eigen::Vector3f::Zero();
        cost = 0;

        float value;
        float resolution = map->getResolution();
        Eigen::Vector2f derivative;
        for(const Eigen::Vector2f& scan_point : scan_data) {
            Eigen::Vector2f point = scan_point / resolution;
            bilinearInterpolation(transform * point, map, value, derivative);

            float r = 1.0f - value;
            cost += r * r;
            JTr[0] += derivative[0] * r;
            JTr[1] += derivative[1] * r;

            float rot_deriv = ((-sin_rot * point.x() - cos_rot * point.y()) * derivative[0] +
                  (cos_rot * point.x() - sin_rot * point.y()) * derivative[1]);

            JTr[2] += rot_deriv * r;

            H(0, 0) += derivative[0] * derivative[0];
            H(1, 1) += derivative[1] * derivative[1];
            H(2, 2) += rot_deriv * rot_deriv;

            H(0, 1) += derivative[0] * derivative[1];
            H(0, 2) += derivative[0] * rot_deriv;
            H(1, 2) += derivative[1] * rot_deriv;
        }

        if(i > 0 && cost > last_cost) {
            pose = last_pose;
            break;
        }
        last_cost = cost;

        H(1, 0) = H(0, 1);
        H(2, 0) = H(0, 2);
        H(2, 1) = H(1, 2);

        if(H.determinant() != 0.0f) {
            last_pose = pose;
            Eigen::Vector3f delta = H.inverse() * JTr;
            pose += delta;
            pose[2] = normalizeAngle(pose[2]);
        }
    }

    return map->getWorldPose(pose);
}

Eigen::Vector3f GaussNewtenScanMatcher::matchScan(const std::shared_ptr<LaserScan>& scan,
                                                  const std::vector<std::shared_ptr<LaserScan>>& base_scan)
{
    correlative_grid_->addScans(base_scan);

    return matchScanToMap(scan->getPose(), correlative_grid_, scan->getRawPointCloud());
}

} // namespace ares_slam
