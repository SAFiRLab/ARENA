// ROS 2
#include "rclcpp/rclcpp.hpp"

// ROS Messages
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <nav_msgs/msg/odometry.hpp>

// Octomap
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/OcTree.h>

// System
#include <string>
#include <chrono>

// Local
#include "linedrone/testbench_config.hpp"


#define RATE 10

namespace arena_testbench
{

namespace linedrone
{

class TestbenchUtilitiesPublisherNode : public rclcpp::Node
{
public:

    TestbenchUtilitiesPublisherNode();
    ~TestbenchUtilitiesPublisherNode()
    {
        delete octree_;
        delete color_octree_;
    }

    // Getters

    // Setters

private:

    // Methods
    void run();
    void publishOctomap();
    void publishColorOctomap();
    void publishDroneOdom();

    // Callbacks

    // Timers
    rclcpp::TimerBase::SharedPtr run_timer_;

    // Publishers
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr color_octomap_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr drone_odom_pub_;

    // Subscribers

    // User-defined attributes
    octomap::OcTree* octree_;
    octomap::ColorOcTree* color_octree_;
    std::string map_file_path_;

    TestbenchConfig testbench_config_;

}; // class TestbenchUtilitiesPublisherNode


void TestbenchUtilitiesPublisherNode::publishDroneOdom()
{
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "map";
    odom_msg.child_frame_id = "offset_base_link";

    // Set position
    odom_msg.pose.pose.position.x = testbench_config_.x;
    odom_msg.pose.pose.position.y = testbench_config_.y;
    odom_msg.pose.pose.position.z = testbench_config_.z;

    // Set orientation
    odom_msg.pose.pose.orientation.x = testbench_config_.o_x;
    odom_msg.pose.pose.orientation.y = testbench_config_.o_y;
    odom_msg.pose.pose.orientation.z = testbench_config_.o_z;
    odom_msg.pose.pose.orientation.w = testbench_config_.o_w;

    drone_odom_pub_->publish(odom_msg);
}

void TestbenchUtilitiesPublisherNode::publishOctomap()
{
    // Publish octomap
    octomap_msgs::msg::Octomap octomap_msg;
    octomap_msg.header.stamp = this->now();
    octomap_msg.header.frame_id = "map";
    octomap_msgs::fullMapToMsg(*octree_, octomap_msg);
    octomap_pub_->publish(octomap_msg);
}

void TestbenchUtilitiesPublisherNode::publishColorOctomap()
{
    // Publish color octomap
    octomap_msgs::msg::Octomap color_octomap_msg;
    color_octomap_msg.header.stamp = this->now();
    color_octomap_msg.header.frame_id = "map";
    octomap_msgs::fullMapToMsg(*color_octree_, color_octomap_msg);
    color_octomap_pub_->publish(color_octomap_msg);
}

void TestbenchUtilitiesPublisherNode::run()
{
    publishOctomap();
    publishColorOctomap();
    publishDroneOdom();
}

TestbenchUtilitiesPublisherNode::TestbenchUtilitiesPublisherNode()
: Node("testbench_utilities_publisher_node")
{
    // Parameters
    map_file_path_ = this->declare_parameter<std::string>("octomap_file", "map.bt");
    std::string testbench_config_file = this->declare_parameter<std::string>("testbench_config_file", "config.yaml");

    // Publishers
    color_octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("/colored_filtered_map", 1);
    octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("/filtered_map", 1);
    drone_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/testbench/drone/odom", 1);

    // Subscribers
    // Load octomap from binary tree at map_file_path_
    octree_ = new octomap::OcTree(1.0);
    if (octree_->readBinary(map_file_path_))
    {
        RCLCPP_INFO(get_logger(), "Octomap loaded from %s", map_file_path_.c_str());
    }
    else
        RCLCPP_ERROR(get_logger(), "Failed to load octomap from %s", map_file_path_.c_str());


    color_octree_ = new octomap::ColorOcTree(1.0);
    if (color_octree_->readBinary(map_file_path_))
    {
        RCLCPP_INFO(get_logger(), "Color octomap loaded from %s", map_file_path_.c_str());
    }
    else
        RCLCPP_ERROR(get_logger(), "Failed to load octomap from %s", map_file_path_.c_str());

    for (octomap::ColorOcTree::leaf_iterator it = color_octree_->begin_leafs(), end = color_octree_->end_leafs(); it != end; ++it)
    {
        octomap::OcTreeKey key = it.getKey();

        // Change the color of the node to gray
        color_octree_->setNodeColor(key, 128, 128, 128); // RGB values for gray
    }

    try
    {
        testbench_config_.parseConfig(testbench_config_file);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), "Failed to parse testbench config file: %s", e.what());

        // Set default values
        testbench_config_.x = 0.0;
        testbench_config_.y = 0.0;
        testbench_config_.z = 0.0;
        testbench_config_.o_x = 0.0;
        testbench_config_.o_y = 0.0;
        testbench_config_.o_z = 0.0;
        testbench_config_.o_w = 1.0;
    }

    // Main loop
    run_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / RATE), std::bind(&TestbenchUtilitiesPublisherNode::run, this));
}

}; // namespace linedrone

}; // namespace arena_testbench


int main(int argc, char** argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);

    // Run node
    rclcpp::spin(std::make_shared<arena_testbench::linedrone::TestbenchUtilitiesPublisherNode>());

    rclcpp::shutdown();
    return 0;
}
