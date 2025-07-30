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

    ARENACostmapNode(rclcpp::NodeOptions options = rclcpp::NodeOptions());

    /** @brief Update the parameters of the costmap node.
     * 
     * This function updates the parameters of the costmap node based on the current configuration.
     * 
     */
    void updateParams();

    /** @brief Method to print the status of the costmap node.
     * 
     * This function prints the current status of the costmap node, including the resolution,
     * frame ID, and whether the costmap is initialized.
     *
     */
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
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr inflated_octomap_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr color_octomap_marker_pub_;


    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time loop_duration_;
    double octomap_duration_ = 0.0;

    // Callbacks
    void octreeCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
    void inflatedOctomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
    void colorOctomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
    void timerCallback();

    // User-defined methods
    void publishInflatedOctomap(const std::shared_ptr<octomap::OcTree> octree);
    void publishColorOctomap(const std::shared_ptr<octomap::ColorOcTree> octree);

    // User-defined attributes
    std::unique_ptr<Costmap3D> costmap_3d_;
    bool has_loaded_bt_file_ = false;
    std::shared_ptr<octomap::OcTree> loaded_octree_;

}; // class ARENACostmapNode


void ARENACostmapNode::octreeCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Received octomap message");
    rclcpp::Time begin = this->get_clock()->now();
    auto octree = std::dynamic_pointer_cast<octomap::OcTree>(std::shared_ptr<octomap::AbstractOcTree>(octomap_msgs::fullMsgToMap(*msg)));

    if (!octree)
    {
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
    if (has_loaded_bt_file_)
        return;
    
    auto octree = std::dynamic_pointer_cast<octomap::OcTree>(std::shared_ptr<octomap::AbstractOcTree>(octomap_msgs::binaryMsgToMap(*msg)));

    if (!octree)
    {
        RCLCPP_WARN(this->get_logger(), "Received non-OcTree message, skipping");
        return;
    }

    publishInflatedOctomap(octree);
}


void ARENACostmapNode::publishInflatedOctomap(const std::shared_ptr<octomap::OcTree> octree)
{
    if (!octree)
        return;
    
    // Publish the inflated octomap as a marker array to visualize it
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "map";
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
            geometry_msgs::msg::Point p;
            p.x = it.getX();
            p.y = it.getY();
            p.z = it.getZ();
            marker.points.push_back(p);
        }
    }
    inflated_octomap_marker_pub_->publish(marker);

    // Publish octomap as a message
    octomap_msgs::msg::Octomap octomap_msg;
    octomap_msg.header.frame_id = "map";
    octomap_msg.header.stamp = this->get_clock()->now();
    octomap_msgs::fullMapToMsg(*octree, octomap_msg);
    octomap_pub_->publish(octomap_msg);
}


void ARENACostmapNode::colorOctomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
    if (has_loaded_bt_file_)
        return;
    
    auto octree = std::dynamic_pointer_cast<octomap::ColorOcTree>(std::shared_ptr<octomap::AbstractOcTree>(octomap_msgs::binaryMsgToMap(*msg)));

    if (!octree)
    {
        RCLCPP_WARN(this->get_logger(), "Received non-ColorOcTree message, skipping");
        return;
    }

    publishColorOctomap(octree);
}


void ARENACostmapNode::publishColorOctomap(const std::shared_ptr<octomap::ColorOcTree> octree)
{
    if (!octree)
        return;

    // Publish the color octomap as a marker array to visualize it
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "map";
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

    color_octomap_marker_pub_->publish(marker);

    // Publish color octomap as a message
    octomap_msgs::msg::Octomap octomap_msg;
    octomap_msg.header.frame_id = "map";
    octomap_msg.header.stamp = this->get_clock()->now();
    octomap_msgs::fullMapToMsg(*octree, octomap_msg);
    color_octomap_pub_->publish(octomap_msg);
}


void ARENACostmapNode::timerCallback()
{
    // Re-publish loaded octomap if present
    if (has_loaded_bt_file_ && loaded_octree_)
    {
        if (!costmap_3d_->getOctree())
            costmap_3d_->setOctree(loaded_octree_);

        publishInflatedOctomap(costmap_3d_->getOctree());
        publishColorOctomap(costmap_3d_->getColorOctree());
        RCLCPP_INFO(this->get_logger(), "Re-published loaded octomap from .bt file");
    }

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


ARENACostmapNode::ARENACostmapNode(rclcpp::NodeOptions options)
: Node("costmap_3D_node", options), costmap_3d_(std::make_unique<Costmap3D>())
{
    // Parameters
    costmap_3d_->setInflationRadius(this->get_parameter("navigation.3d.inflation_radius").as_double());
    costmap_3d_->setMinInfluenceRadius(this->get_parameter("navigation.3d.obstacle_influence_distance.min").as_double());
    costmap_3d_->setMaxInfluenceRadius(this->get_parameter("navigation.3d.obstacle_influence_distance.max").as_double());
    std::string bt_file_path = this->get_parameter("saved_map.bt_file").as_string();
    
    if (!bt_file_path.empty())
    {
        RCLCPP_INFO(this->get_logger(), "Loading .bt file: %s", bt_file_path.c_str());
        loaded_octree_ = std::make_shared<octomap::OcTree>(1.0); // resolution will be overridden by file
        if (loaded_octree_->readBinary(bt_file_path))
        {
            has_loaded_bt_file_ = true;
        }
        else
        {
            loaded_octree_ = nullptr;
            RCLCPP_WARN(this->get_logger(), "Failed to load the specified .bt file: %s", bt_file_path.c_str());
        }
    }

    // Subscribers
    using std::placeholders::_1;
    octree_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>("/octomap_binary", 10, std::bind(&ARENACostmapNode::octreeCallback, this, _1));
    inflated_octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>("/navigation/inflated_octomap/full", 10, std::bind(&ARENACostmapNode::inflatedOctomapCallback, this, _1));
    color_octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>("/navigation/sdf_octomap/full", 10, std::bind(&ARENACostmapNode::colorOctomapCallback, this, _1));

    // Publishers
    octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("/navigation/inflated_octomap/full", 10);
    color_octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("/navigation/sdf_octomap/full", 10);

    inflated_octomap_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/navigation/inflated_octomap/viz", 10);
    color_octomap_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/navigation/sdf_octomap/viz", 10);

    // Timers
    loop_duration_ = this->get_clock()->now();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / RATE)), std::bind(&ARENACostmapNode::timerCallback, this));
}

}; // namespace arena_core


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
    rclcpp::spin(std::make_shared<arena_core::ARENACostmapNode>(options));
    rclcpp::shutdown();
    
    return 0;
}
