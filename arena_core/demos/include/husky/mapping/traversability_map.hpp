#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// grid_map
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_core/GridMap.hpp>

#include <mutex>
#include <memory>


namespace arena_demos
{

struct RobotBoundingBox
{
    double x_width = 0;
    double y_width = 0;

    bool isInside(const Eigen::Vector2d &local_map_pose, const grid_map::Position &point_pose)
    {
        double half_x = x_width / 2.0;
        double half_y = y_width / 2.0;

        if (point_pose.x() < (local_map_pose.x() - half_x) || point_pose.x() > (local_map_pose.x() + half_x))
            return false;
        if (point_pose.y() < (local_map_pose.y() - half_y) || point_pose.y() > (local_map_pose.y() + half_y))
            return false;

        return true;
    };
}; // struct RobotBoundingBox


class TraversabilityMap
{
public:

    TraversabilityMap();

    void moveLocalMap(const Eigen::Vector2d &a_pose);
    void updateMap(const pcl::PointCloud<pcl::PointXYZ> &a_cloud);

    std::shared_ptr<grid_map::GridMap> getLocalMap() { return local_map_; };
    std::shared_ptr<grid_map::GridMap> getGlobalMap() { return global_map_; };
    bool isMapInitialized() { return global_map_initialized_; };

private:

    void initializeMaps(const grid_map::Position &a_center);
    void updateStepAtIter(const grid_map::GridMapIterator &it);
    void updateSlopeAtIter(const grid_map::GridMapIterator &it);
    void updateOccupancyAtIter(const grid_map::GridMapIterator &it);

    // User-defined attirbutes
    std::shared_ptr<grid_map::GridMap> global_map_;
    std::shared_ptr<grid_map::GridMap> local_map_;

    Eigen::Vector2d local_map_pose_;
    std::mutex map_mutex_;
    bool global_map_initialized_ = false;
    RobotBoundingBox robot_bb_;

}; // class TraversabilityMap

}; // namespace arena_demos
