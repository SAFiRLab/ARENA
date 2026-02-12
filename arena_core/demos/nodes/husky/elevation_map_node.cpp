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

#include <mutex>

class ElevationMappingNode : public rclcpp::Node
{
public:
    ElevationMappingNode()
    : Node("elevation_mapping_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        initializeROS();
    }

private:

    /* =======================
            INITIALIZATION
        ======================= */

    void initializeMaps(grid_map::Position &center)
    {
        // ----- GLOBAL MAP -----
        global_map_ = grid_map::GridMap({"elevation"});
        global_map_.add("count");
        global_map_.add("sum");
        global_map_.setFrameId("world");

        global_map_.setGeometry(grid_map::Length(200.0, 200.0), 0.3, center);
        global_map_["elevation"].setConstant(std::numeric_limits<float>::quiet_NaN());
        global_map_["count"].setZero();
        global_map_["sum"].setZero();

        // ----- LOCAL MAP -----
        local_map_ = grid_map::GridMap({"elevation"});
        local_map_.setFrameId("world");
        local_map_.setGeometry(grid_map::Length(30.0, 30.0), 0.3);

        global_map_initialized_ = true;
    }

    void initializeROS()
    {
        using std::placeholders::_1;

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/cloud_in", rclcpp::SensorDataQoS(), std::bind(&ElevationMappingNode::cloudCallback, this, _1));
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/groundTruth/poseStamped", rclcpp::SensorDataQoS(), std::bind(&ElevationMappingNode::poseCallback, this, _1));

        local_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("/local_grid_map", rclcpp::QoS(1));
        global_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("/global_grid_map", rclcpp::QoS(1).transient_local());

        // Slow global publishing
        global_timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&ElevationMappingNode::publishGlobalMap, this));
    }

    /* =======================
            CALLBACKS
        ======================= */

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);

        robot_position_ = {msg->pose.position.x, msg->pose.position.y};
        grid_map::Position center(robot_position_.x(), robot_position_.y());

        if (!global_map_initialized_)
        {
            initializeMaps(center);
            RCLCPP_INFO(this->get_logger(), "Global map initialized at (%.2f, %.2f)", center.x(), center.y());
        }

        // Move local map window
        local_map_.setPosition(robot_position_);

        // Extract submap from global
        bool is_success = false;
        local_map_ = global_map_.getSubmap(center, local_map_.getLength(), is_success);

        if (!is_success)
        {
            RCLCPP_WARN(get_logger(), "Failed to extract local map submap at position (%.2f, %.2f)", center.x(), center.y());
            return;
        }

        publishLocalMap();
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!global_map_initialized_)
            return;
        
        sensor_msgs::msg::PointCloud2 cloud_world;

        try
        {
            cloud_world = tf_buffer_.transform(*msg, "world", tf2::durationFromSec(0.1));
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

        std::lock_guard<std::mutex> lock(map_mutex_);

        for (const auto & p : cloud.points)
        {
            if (!std::isfinite(p.z)) continue;

            grid_map::Position pos(p.x, p.y);
            if (!global_map_.isInside(pos)) continue;

            grid_map::Index index;
            global_map_.getIndex(pos, index);

            global_map_.at("sum", index) += p.z;
            global_map_.at("count", index) += 1.0f;
        }

        // Update elevation layer
        for (grid_map::GridMapIterator it(global_map_); !it.isPastEnd(); ++it)
        {
            float c = global_map_.at("count", *it);

            if (c > 0.0f)
                global_map_.at("elevation", *it) = global_map_.at("sum", *it) / c;
        }
    }

    /* =======================
            PUBLISHING
        ======================= */

    void publishLocalMap()
    {
        if (!global_map_initialized_)
            return;
        
        local_map_.setTimestamp(this->now().nanoseconds());

        auto msg = grid_map::GridMapRosConverter::toMessage(local_map_);

        msg->header.frame_id = "world";

        local_pub_->publish(std::move(msg));   // ZERO COPY
    }

    void publishGlobalMap()
    {
        if (!global_map_initialized_)
            return;
        
        std::lock_guard<std::mutex> lock(map_mutex_);

        global_map_.setTimestamp(this->now().nanoseconds());

        auto msg = grid_map::GridMapRosConverter::toMessage(global_map_);

        global_pub_->publish(std::move(msg));  // ZERO COPY
    }

    /* =======================
        MEMBERS
    ======================= */

    grid_map::GridMap global_map_;
    grid_map::GridMap local_map_;

    Eigen::Vector2d robot_position_;

    std::mutex map_mutex_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr local_pub_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr global_pub_;

    rclcpp::TimerBase::SharedPtr global_timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    bool global_map_initialized_ = false;
};

/* =======================
            MAIN
   ======================= */

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ElevationMappingNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
