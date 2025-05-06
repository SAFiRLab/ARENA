// Local
#include "arena_core/mapping/costmap_3D.h"

// ROS
#include "rclcpp/rclcpp.hpp"
#include <octomap_ros/conversions.hpp>
#include "octomap_msgs/conversions.h"

// ROS Messages
#include "octomap_msgs/msg/octomap.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

// System
#include <string>
#include <memory>


#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_RESET   "\x1b[0m"

#define RATE 1


namespace arena_core
{

class ARENACostmapNode : public rclcpp::Node
{

public:

    ARENACostmapNode(const std::string& node_name);

    void updateParams();
    void printStatus();

    // Getters

    // Setters


private:

    // Subscribers
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octree_sub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr inflated_octomap_sub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr color_octomap_sub_;

    // Publishers
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr color_octomap_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr inflated_octomap_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr color_octomap_marker_pub_;

    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time loop_duration_;
    double octomap_duration_ = 0.0;

    // Callbacks
    void octreeCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
    void inflatedOctomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
    void colorOctomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
    void timerCallback();

    // User-defined attributes
    std::unique_ptr<Costmap3D> costmap_3d_;

}; // class ARENACostmapNode


void ARENACostmapNode::octreeCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received octomap message");
    rclcpp::Time begin = this->get_clock()->now();
    auto octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*msg));

    if (!octree) {
        RCLCPP_WARN(this->get_logger(), "Received non-OcTree message, skipping");
        return;
    }

    costmap_3d_->setOctree(octree);

    // Inflated octomap
    octomap_msgs::msg::Octomap inflated_msg;
    inflated_msg.header.frame_id = msg->header.frame_id;
    inflated_msg.header.stamp = this->now();
    octomap_msgs::binaryMapToMsg(*costmap_3d_->getOctree(), inflated_msg);
    octomap_pub_->publish(inflated_msg);

    // Colored octomap
    inflated_msg.header.stamp = this->now();
    octomap_msgs::fullMapToMsg(*costmap_3d_->getColorOctree(), inflated_msg);
    color_octomap_pub_->publish(inflated_msg);

    octomap_duration_ = (this->get_clock()->now() - begin).seconds();
}


void ARENACostmapNode::inflatedOctomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received inflated octomap message");

    auto octree = dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(*msg));

    if (!octree)
    {
        RCLCPP_WARN(this->get_logger(), "Received non-OcTree message, skipping");
        return;
    }

    // Publish the inflated octomap as a marker array to visualize it
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = msg->header.frame_id;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "inflated_octomap";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = octree->getResolution();
    marker.color.r = 0.0f;
    marker.color.g = 0.5f;
    marker.color.b = 1.0f;
    marker.color.a = 0.8f;

    for (auto it = octree->begin_leafs(); it != octree->end_leafs(); ++it)
    {
        if (octree->isNodeOccupied(*it))
        {
            /*octomap::point3d center = it.getCoordinate();
            geometry_msgs::msg::Point p;
            p.x = center.x();
            p.y = center.y();
            p.z = center.z();
            marker.points.push_back(p);*/
            geometry_msgs::msg::Point p;
            p.x = it.getX();
            p.y = it.getY();
            p.z = it.getZ();
            marker.points.push_back(p);
        }
    }
    marker_array.markers.push_back(marker);
    inflated_octomap_marker_pub_->publish(marker_array);

    delete octree; // Clean up the octree after processing
}


void ARENACostmapNode::colorOctomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received color octomap message");

    auto octree = dynamic_cast<octomap::ColorOcTree*>(octomap_msgs::fullMsgToMap(*msg));

    if (!octree)
    {
        RCLCPP_WARN(this->get_logger(), "Received non-ColorOcTree message, skipping");
        return;
    }
    // Publish the color octomap as a marker array to visualize it

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = msg->header.frame_id;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "color_octomap";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = marker.scale.z = octree->getResolution();
    marker.color.a = 0.8f;

    for (auto it = octree->begin_leafs(); it != octree->end_leafs(); ++it)
    {
        if (octree->isNodeOccupied(*it))
        {
            octomap::ColorOcTreeNode* node = octree->search(it.getKey());
            if (node)
            {
                std_msgs::msg::ColorRGBA color;
                color.r = node->getColor().r / 255.0f;
                color.g = node->getColor().g / 255.0f;
                color.b = node->getColor().b / 255.0f;
                color.a = 1.0f; // Set alpha to 1 for full opacity

                geometry_msgs::msg::Point p;
                p.x = it.getX();
                p.y = it.getY();
                p.z = it.getZ();
                marker.points.push_back(p);
                marker.colors.push_back(color);
            }
        }
    }

    marker_array.markers.push_back(marker);
    color_octomap_marker_pub_->publish(marker_array);

    delete octree; // Clean up the octree after processing
}


void ARENACostmapNode::timerCallback()
{
    updateParams();
    printStatus();
}


void ARENACostmapNode::updateParams()
{
    float val;
    if (this->get_parameter("navigation.3d.inflation_radius", val))
        costmap_3d_->setInflationRadius(val);

    if (this->get_parameter("navigation.3d.obstacle_influence_distance.min", val))
        costmap_3d_->setMinInfluenceRadius(val);

    if (this->get_parameter("navigation.3d.obstacle_influence_distance.max", val))
        costmap_3d_->setMaxInfluenceRadius(val);
}


void ARENACostmapNode::printStatus()
{
    double frequency = 1.0 / (this->now() - loop_duration_).seconds();
    loop_duration_ = this->now();

    //std::cout << "\033[2J\033[1;1H";
    printf("------------------- COSTMAP 3D -------------------\n");
    printf("Frequency: %s\t%4.2f %s\n",
        fabs(frequency - RATE) < 1.5 ? ANSI_COLOR_GREEN : ANSI_COLOR_RED,
        frequency,
        ANSI_COLOR_RESET);
    printf("Octomap inflation | radius: %.1fm | min influence: %.1f | max influence: %.1f | time: %.3fs\n",
        costmap_3d_->getInflationRadius(),
        costmap_3d_->getMinInfluenceRadius(),
        costmap_3d_->getMaxInfluenceRadius(),
        octomap_duration_);
    printf("Octree depth: %d\n", costmap_3d_->getColorOctree()->getTreeDepth());
    printf("Octree node size: %f\n", costmap_3d_->getColorOctree()->getNodeSize(
        costmap_3d_->getColorOctree()->getTreeDepth()));
    printf("---------------------------------------------------\n");
}


ARENACostmapNode::ARENACostmapNode(const std::string& node_name)
: Node(node_name), costmap_3d_(std::make_unique<Costmap3D>())
{
    using std::placeholders::_1;

    // Parameters
    this->declare_parameter("navigation.3d.inflation_radius", costmap_3d_->getInflationRadius());
    this->declare_parameter("navigation.3d.obstacle_influence_distance.min", costmap_3d_->getMinInfluenceRadius());
    this->declare_parameter("navigation.3d.obstacle_influence_distance.max", costmap_3d_->getMaxInfluenceRadius());

    costmap_3d_->setInflationRadius(this->get_parameter("navigation.3d.inflation_radius").as_double());
    costmap_3d_->setMinInfluenceRadius(this->get_parameter("navigation.3d.obstacle_influence_distance.min").as_double());
    costmap_3d_->setMaxInfluenceRadius(this->get_parameter("navigation.3d.obstacle_influence_distance.max").as_double());

    // Subscribers
    octree_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>("/octomap_binary", 10, std::bind(&ARENACostmapNode::octreeCallback, this, _1));
    inflated_octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>("/navigation/inflated_octomap/full", 10, std::bind(&ARENACostmapNode::inflatedOctomapCallback, this, _1));
    color_octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>("/navigation/sdf_octomap/full", 10, std::bind(&ARENACostmapNode::colorOctomapCallback, this, _1));


    // Publishers
    octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("/navigation/inflated_octomap/full", 10);
    color_octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("/navigation/sdf_octomap/full", 10);
    inflated_octomap_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/navigation/inflated_octomap/viz", 10);
    color_octomap_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/navigation/sdf_octomap/viz", 10);

    // Timers
    loop_duration_ = this->get_clock()->now();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / RATE)), std::bind(&ARENACostmapNode::timerCallback, this));
}

}; // namespace arena_core


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<arena_core::ARENACostmapNode>("costmap_3d_node");

    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
