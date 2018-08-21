#include "correlative_grid.h"

namespace ares_slam
{

CorrelativeGrid::CorrelativeGrid(float resolution, float standard_deviation)
{
    standard_deviation_ = standard_deviation;
    resolution_ = resolution;
    kernel_size_ = static_cast<int>(ceil(2.0f * standard_deviation / resolution)) * 2 + 1;

    calculateKernel();
}

CorrelativeGrid::~CorrelativeGrid()
{
    if(kernel_) {
        delete kernel_;
    }
}

void CorrelativeGrid::setupGrid(int width, int height, const Eigen::Vector2f& origin)
{
    size_x_ = width;
    size_y_ = height;

    size_ = size_x_ * size_y_;

    if(value_) {
        delete value_;
    }
    value_ = new uint8_t[size_];

    setOrigin(origin);
    clear();
}


void CorrelativeGrid::calculateKernel()
{
    kernel_ = new uint8_t[kernel_size_ * kernel_size_];

    int half_kernel_size = kernel_size_ / 2;

    for(int i = -half_kernel_size; i <= half_kernel_size; ++i) {
        for(int j = -half_kernel_size; j <= half_kernel_size; ++j) {
            float distance = hypot(i * resolution_, j * resolution_);
            float z = exp(-0.5 * pow(distance / standard_deviation_, 2));
            int kernel_value = static_cast<int>(z * GridStates_Occupied);
            int index = (i + half_kernel_size) + kernel_size_ * (j + half_kernel_size);
            kernel_[index] = kernel_value;
        }
    }
}

void CorrelativeGrid::setCorrelativePoint(const Eigen::Vector2i& point)
{
    int half_kernel_size = kernel_size_ / 2;
    int index = getIndex(point);

    for (int i = -half_kernel_size; i <= half_kernel_size; ++i) {
        int kernel_constant = half_kernel_size + kernel_size_ * (i + half_kernel_size);
        int index_constant = index + size_x_ * i;

        for (int j = -half_kernel_size; j <= half_kernel_size; ++j) {
            int kernel_index = j + kernel_constant;
            if(isOutOfMap(index_constant + j)) {
                continue;
            }
            uint8_t kernel_value = kernel_[kernel_index];

            if (kernel_value > value_[index_constant + j]) {
                value_[index_constant + j] = kernel_value;
            }
        }
    }
}

void CorrelativeGrid::addScans(const std::vector<std::shared_ptr<LaserScan>>& scans)
{
    Range map_range;
    for(const std::shared_ptr<LaserScan>& scan : scans) {
        map_range.addRange(scan->getRange());
    }

    Eigen::Vector2f margin(kernel_size_ * resolution_,
                           kernel_size_ * resolution_);

    Eigen::Vector2f max = map_range.getMax() + margin;
    Eigen::Vector2f min = map_range.getMin() - margin;
    size_x_ = ceil((max[0] - min[0]) / resolution_);
    size_y_ = ceil((max[1] - min[1]) / resolution_);

    size_ = size_x_ * size_y_;

    if(value_) {
        delete value_;
    }
    value_ = new uint8_t[size_];

    setOrigin(min);
    clear();

    for(const std::shared_ptr<LaserScan> scan : scans) {
        const PointCloud& point_cloud = scan->getTransformedPointCloud();
        for(const Eigen::Vector2f& point : point_cloud) {
            Eigen::Vector2f map_coords = getMapCoords(point);
            Eigen::Vector2i map_point(round(map_coords[0]), round(map_coords[1]));
            if(isOutOfMap(map_point)) {
                continue;
            }

            int index = getIndex(map_point);

            if(getGridValue(index) == GridStates_Occupied) {
                continue;
            }

            setGridValue(index, GridStates_Occupied);
            setCorrelativePoint(map_point);
        }
    }
}

} // namespace ares_slam
