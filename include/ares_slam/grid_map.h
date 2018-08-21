#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ares_slam
{

const uint8_t GridStates_Free = 100;
const uint8_t GridStates_Occupied = 255;

template<typename T>
class GridMap
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GridMap () : size_x_(0), size_y_(0), size_(0), value_(nullptr) {}
    GridMap(int width, int height, float resolution)
    {
        initialize(width, height, resolution);
    }
    GridMap(const GridMap& rhs) = delete;
    GridMap& operator =(const GridMap& rhs) = delete;

    virtual ~GridMap()
    {
        if(value_) {
            delete[] value_;
        }
    }

    void initialize(int width, int height, float resolution)
    {
        resolution_ = resolution;
        size_x_ = width;
        size_y_ = height;
        size_ = size_x_ * size_y_;

        value_ = new T[size_];
    }

    void clear()
    {
        memset(value_, 0, size_ * sizeof(T));
    }

    int getSizeX()  const { return size_x_; }
    int getSizeY()  const { return size_y_; }
    int getSize() const { return size_; }
    int getIndex(int x, int y) const {  return y * size_x_ + x; }
    int getIndex(const Eigen::Vector2i& map_coords) const {  return map_coords[1] * size_x_ + map_coords[0]; }
    float getResolution() const { return resolution_; }
    Eigen::Vector2f getOrigin()  const { return origin_; }
    void setOrigin(const Eigen::Vector2f& origin)
    {
        origin_ = origin;
        world_to_map_ = Eigen::AlignedScaling2f(1.0 / resolution_, 1.0 / resolution_) *
                        Eigen::Translation2f(-origin_[0], -origin_[1]);
        map_to_world_ = world_to_map_.inverse();
    }
    T getGridValue(int index)  const { return value_[index]; }
    T getGridValue(int x, int y) const { return value_[getIndex(x, y)]; }
    void setGridValue(int index, T value) { value_[index] = value; }
    bool isOutOfMap(int index)  const { return ((index < 0) || (index > size_ - 1)); }
    bool isOutOfMap(int x, int y)  const { return ((x < 0) || (x > size_x_ - 1) || (y < 0) || (y > size_y_ - 1)); }
    bool isOutOfMap(const Eigen::Vector2i& coords) const
    {
        return ((coords[0] < 0.0f) || (coords[0] > size_x_ - 1) ||
                (coords[1] < 0.0f) || (coords[1] > size_y_ - 1));
    }
    bool isOutOfMap(const Eigen::Vector2f& coords) const
    {
        return ((coords[0] < 0.0f) || (coords[0] > size_x_ - 1) ||
                (coords[1] < 0.0f) || (coords[1] > size_y_ - 1));
    }

    Eigen::Vector2f getWorldCoords(const Eigen::Vector2i& map_coords) const
    {
        return map_to_world_ * map_coords.cast<float>();
    }
    Eigen::Vector2f getWorldCoords(const Eigen::Vector2f& map_coords) const
    {
        return map_to_world_ * map_coords;
    }
    Eigen::Vector2f getMapCoords(const Eigen::Vector2f& world_coords) const
    {
        return world_to_map_ * world_coords;
    }
    Eigen::Vector2f getMapCoords(const Eigen::Vector3f& world_pose) const
    {
        return world_to_map_ * world_pose.head<2>();
    }
    Eigen::Vector3f getWorldPose(const Eigen::Vector3f& map_pose) const
    {
        Eigen::Vector2f world_coords (map_to_world_ * map_pose.head<2>());
        return Eigen::Vector3f(world_coords[0], world_coords[1], map_pose[2]);
    }
    Eigen::Vector3f getMapPose(const Eigen::Vector3f& world_pose) const
    {
        Eigen::Vector2f map_coords (world_to_map_ * world_pose.head<2>());
        return Eigen::Vector3f(map_coords[0], map_coords[1], world_pose[2]);
    }

protected:
    T* value_;
    float resolution_;
    int size_x_;
    int size_y_;
    int size_;
    Eigen::Vector2f origin_;
    Eigen::Affine2f map_to_world_;
    Eigen::Affine2f world_to_map_;
};

} // namespace ares_slam

#endif // GRID_MAP_H
