#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Local
#include "husky/mapping/traversability_map.hpp"


class TraversabilityMappingNode : public rclcpp::Node
{
public:
    TraversabilityMappingNode()
    : Node("traversability_map_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        using std::placeholders::_1;

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/cloud_in", rclcpp::SensorDataQoS(), std::bind(&TraversabilityMappingNode::cloudCallback, this, _1));
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/groundTruth/poseStamped", rclcpp::SensorDataQoS(), std::bind(&TraversabilityMappingNode::poseCallback, this, _1));

        local_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("/local_grid_map", rclcpp::QoS(1));
        global_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("/global_grid_map", rclcpp::QoS(1).transient_local());

        // Slow global publishing
        global_timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&TraversabilityMappingNode::publishGlobalMap, this));
    }

private:

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr a_msg)
    {
        robot_position_ = {a_msg->pose.position.x, a_msg->pose.position.y};

        traversability_map_.moveLocalMap(robot_position_);

        if (traversability_map_.getLocalMap())
            publishLocalMap();
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr a_msg)
    {
        if (!traversability_map_.isMapInitialized())
            return;
        
        sensor_msgs::msg::PointCloud2 cloud_world;

        try
        {
            cloud_world = tf_buffer_.transform(*a_msg, "world", tf2::durationFromSec(0.1));
        }
        catch (tf2::TransformException & ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF failed: %s", ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(cloud_world, cloud);

        if (cloud.empty())
            return;

        traversability_map_.updateMap(cloud);
    }

    void publishLocalMap()
    {
        if (!traversability_map_.isMapInitialized())
            return;
        
        std::shared_ptr<grid_map::GridMap> local_map = traversability_map_.getLocalMap();

        if (!local_map)
            return;
        
        local_map->setTimestamp(this->now().nanoseconds());

        auto msg = grid_map::GridMapRosConverter::toMessage(*local_map);

        msg->header.frame_id = "world";

        local_pub_->publish(std::move(msg));   // ZERO COPY
    }

    void publishGlobalMap()
    {
        if (!traversability_map_.isMapInitialized())
            return;

        std::shared_ptr<grid_map::GridMap> global_map = traversability_map_.getGlobalMap();

        if (!global_map)
            return;

        global_map->setTimestamp(this->now().nanoseconds());

        auto msg = grid_map::GridMapRosConverter::toMessage(*global_map);

        global_pub_->publish(std::move(msg));  // ZERO COPY
    }

    arena_demos::TraversabilityMap traversability_map_;
    Eigen::Vector2d robot_position_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr local_pub_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr global_pub_;

    rclcpp::TimerBase::SharedPtr global_timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TraversabilityMappingNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
