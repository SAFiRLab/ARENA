#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "arena_core/math/nurbs.h"

#include <Eigen/Dense>


using namespace std::chrono_literals;

class NurbsVisualizerNode : public rclcpp::Node {
public:
    NurbsVisualizerNode()
    : Node("nurbs_visualizer_node")
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("nurbs_marker", 10);
        control_points_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("control_points_marker", 10);

        // Example NURBS control points
        Eigen::VectorXd p1(3);
        p1 << 0, 0, 0;
        Eigen::VectorXd p2(3);
        p2 << 1, 2, 3;
        Eigen::VectorXd p3(3);
        p3 << 2, 0, 3;
        Eigen::VectorXd p4(3);
        p4 << 3, 2, -5;
        Eigen::VectorXd p5(3);
        p5 << 4, 0, 2;

        control_points_.push_back(arena_core::ControlPoint<double>(p1));
        control_points_.push_back(arena_core::ControlPoint<double>(p2));
        control_points_.push_back(arena_core::ControlPoint<double>(p3));
        control_points_.push_back(arena_core::ControlPoint<double>(p4));
        control_points_.push_back(arena_core::ControlPoint<double>(p5));

        // Example weights
        control_points_[1].setW(5.0);
        control_points_[2].setW(10.0);
        control_points_[3].setW(7.0);

        int degree = 3;
        sample_size_ = 50;
        nurbs_ = std::make_shared<arena_core::Nurbs>(control_points_, sample_size_, degree);

        timer_ = this->create_wall_timer(500ms, std::bind(&NurbsVisualizerNode::publishAll, this));
    }

private:
    void publish_marker()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "nurbs";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = 0.05;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        // Evalute the time it takes to evaluate the NURBS curve
        auto start = std::chrono::high_resolution_clock::now();
        Eigen::VectorXd* pt = nurbs_->evaluate();
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = end - start;
        RCLCPP_INFO(this->get_logger(), "NURBS evaluation took: %f ms", elapsed.count());
        // Sample the curve and add points
        for (int i = 0; i < sample_size_; ++i)
        {
            geometry_msgs::msg::Point p;
            p.x = pt[i][0];
            p.y = pt[i][1];
            p.z = pt[i][2];
            marker.points.push_back(p);
        }

        marker_pub_->publish(marker);
    }

    void publishControlPoints()
    {
        visualization_msgs::msg::Marker control_points_marker;
        control_points_marker.header.frame_id = "map";
        control_points_marker.header.stamp = this->now();
        control_points_marker.ns = "control_points";
        control_points_marker.id = 1;
        control_points_marker.type = visualization_msgs::msg::Marker::POINTS;
        control_points_marker.action = visualization_msgs::msg::Marker::ADD;

        control_points_marker.scale.x = 0.25;
        control_points_marker.scale.y = 0.25;
        control_points_marker.scale.z = 0.25;
        control_points_marker.color.r = 1.0;
        control_points_marker.color.g = 0.0;
        control_points_marker.color.b = 0.0;
        control_points_marker.color.a = 1.0;

        for (const auto& cp : control_points_) {
            geometry_msgs::msg::Point p;
            p.x = cp[0];
            p.y = cp[1];
            p.z = cp[2];
            control_points_marker.points.push_back(p);
        }

        control_points_pub_->publish(control_points_marker);
    }

    void publishAll()
    {
        publish_marker();
        publishControlPoints();
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr control_points_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<arena_core::ControlPoint<double>> control_points_;
    std::shared_ptr<arena_core::Nurbs> nurbs_;
    int sample_size_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NurbsVisualizerNode>());
    rclcpp::shutdown();
    return 0;
}
