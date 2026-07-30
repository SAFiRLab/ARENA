#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
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

        is_simulation_ = this->get_parameter("use_sim_time").as_bool();

        //ground_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/travel_node/ground", rclcpp::SensorDataQoS(), std::bind(&TraversabilityMappingNode::groundCloudCallback, this, _1));
        non_ground_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/travel_node/nonground", rclcpp::SensorDataQoS(), std::bind(&TraversabilityMappingNode::nonGroundCloudCallback, this, _1));
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/groundTruth/poseStamped", rclcpp::SensorDataQoS(), std::bind(&TraversabilityMappingNode::poseCallback, this, _1));

        if (is_simulation_)
            imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", rclcpp::SensorDataQoS(), std::bind(&TraversabilityMappingNode::imuCallback, this, _1));
        //else
            //imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", rclcpp::SensorDataQoS(), std::bind(&TraversabilityMappingNode::imuCallback, this, _1)); // TODO

        local_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("/local_grid_map", rclcpp::QoS(1));
        global_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("/global_grid_map", rclcpp::QoS(1).transient_local());

        // Slow global publishing
        global_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TraversabilityMappingNode::publishGlobalMap, this));
        local_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&TraversabilityMappingNode::publishLocalMap, this));
        // 10 Hz update rate for traversability map
        update_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TraversabilityMappingNode::updateMap, this));
    }

private:

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr a_msg)
    {
        robot_position_ = {a_msg->pose.position.x, a_msg->pose.position.y};

        traversability_map_.moveLocalMap(robot_position_);
    }

    void nonGroundCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr a_msg)
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

        non_ground_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(cloud);
    }

    void groundCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr a_msg)
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

        ground_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(cloud);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr a_msg)
    {
        imu_data_ = a_msg;
    }

    void updateMap()
    {
        if (!traversability_map_.isMapInitialized())
            return;

        // Check if angle velocity is above threshold
        if (imu_data_)
        {
            double angular_velocity_threshold = 0.5; // rad/s
            double angular_velocity_magnitude = std::sqrt(std::pow(imu_data_->angular_velocity.x, 2) +
                                                        std::pow(imu_data_->angular_velocity.y, 2) +
                                                        std::pow(imu_data_->angular_velocity.z, 2));

            if (angular_velocity_magnitude > angular_velocity_threshold)
            {
                RCLCPP_WARN(this->get_logger(), "Angular velocity is above threshold: %f rad/s", angular_velocity_magnitude);
                return; // Skip updating the map
            }
        }

        if (!ground_cloud_ && !non_ground_cloud_)
            return;
        
        std::unordered_map<std::string, std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> clouds;
        //clouds["ground"] = ground_cloud_;
        clouds["non_ground"] = non_ground_cloud_;

        traversability_map_.updateMap(clouds);
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

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ground_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr non_ground_cloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr local_pub_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr global_pub_;

    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::TimerBase::SharedPtr global_timer_;
    rclcpp::TimerBase::SharedPtr local_timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> ground_cloud_;
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> non_ground_cloud_;
    std::shared_ptr<sensor_msgs::msg::Imu> imu_data_;

    bool is_simulation_ = false;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TraversabilityMappingNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
