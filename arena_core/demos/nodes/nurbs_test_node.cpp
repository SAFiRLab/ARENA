// ROS 2
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

// Local
#include "arena_core/math/nurbs.h"
#include "arena_core/math/control_point.h"

// External Libraries
// Eigen
#include <Eigen/Dense>


using namespace std::chrono_literals;

class NurbsVisualizerNode : public rclcpp::Node {
public:
    NurbsVisualizerNode()
    : Node("nurbs_visualizer_node")
    {
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("nurbs_marker", 10);
        fitted_curve_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("fitted_curve_marker", 10);
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
        Eigen::VectorXd p6(3);
        p6 << 5, 2, 1;
        Eigen::VectorXd p7(3);
        p7 << 6, 0, 0;
        Eigen::VectorXd p8(3);
        p8 << 7, 2, -1;
        Eigen::VectorXd p9(3);
        p9 << 8, 0, -2;
        Eigen::VectorXd p10(3);
        p10 << 9, 2, -3;
        Eigen::VectorXd p11(3);
        p11 << 10, 0, -4;
        Eigen::VectorXd p12(3);
        p12 << 11, 2, -5;
        Eigen::VectorXd p13(3);
        p13 << 12, 0, -6;

        control_points_.push_back(arena_core::ControlPoint<double>(p1));
        control_points_.push_back(arena_core::ControlPoint<double>(p2));
        control_points_.push_back(arena_core::ControlPoint<double>(p3));
        control_points_.push_back(arena_core::ControlPoint<double>(p4));
        control_points_.push_back(arena_core::ControlPoint<double>(p5));
        control_points_.push_back(arena_core::ControlPoint<double>(p6));
        control_points_.push_back(arena_core::ControlPoint<double>(p7));
        control_points_.push_back(arena_core::ControlPoint<double>(p8));
        control_points_.push_back(arena_core::ControlPoint<double>(p9));
        control_points_.push_back(arena_core::ControlPoint<double>(p10));
        control_points_.push_back(arena_core::ControlPoint<double>(p11));
        control_points_.push_back(arena_core::ControlPoint<double>(p12));
        control_points_.push_back(arena_core::ControlPoint<double>(p13));

        // Example weights
        control_points_[1].setW(5.0);
        control_points_[2].setW(10.0);
        control_points_[3].setW(7.0);

        int degree = 5;
        sample_size_ = 50;
        nurbs_ = std::make_shared<arena_core::Nurbs>(control_points_, sample_size_, degree);

        timer_ = this->create_wall_timer(500ms, std::bind(&NurbsVisualizerNode::publishAll, this));
        derivatives_timer_ = this->create_wall_timer(1000ms, std::bind(&NurbsVisualizerNode::publishDerivatives, this));
    }

private:
    void publish_marker()
    {
        visualization_msgs::msg::Marker marker, fitted_curve_marker;
        marker.header.frame_id = "map";
        fitted_curve_marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        fitted_curve_marker.header.stamp = this->now();
        marker.ns = "nurbs";
        fitted_curve_marker.ns = "fitted_curve";
        marker.id = 0;
        fitted_curve_marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        fitted_curve_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        fitted_curve_marker.action = visualization_msgs::msg::Marker::ADD;

        marker.scale.x = 0.05;
        fitted_curve_marker.scale.x = 0.05;
        marker.color.r = 0.0;
        fitted_curve_marker.color.r = 1.0;
        marker.color.g = 1.0;
        fitted_curve_marker.color.g = 0.0;
        marker.color.b = 0.0;
        fitted_curve_marker.color.b = 0.0;
        marker.color.a = 1.0;
        fitted_curve_marker.color.a = 1.0;

        // Evalute the time it takes to evaluate the NURBS curve
        auto start = std::chrono::high_resolution_clock::now();
        Eigen::VectorXd* pt = nurbs_->evaluate();
        auto end = std::chrono::high_resolution_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start);
        RCLCPP_INFO(this->get_logger(), "NURBS evaluation took: %f ms", elapsed_ms.count());

        // Fit a polynomial curve on the points
        start = std::chrono::high_resolution_clock::now();
        Eigen::MatrixXd fitted_curve_coeffs = nurbs_->fitPolynomialCurve(pt);
        end = std::chrono::high_resolution_clock::now();
        elapsed_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start);
        RCLCPP_INFO(this->get_logger(), "Polynomial curve fitting took: %f ms", elapsed_ms.count());

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

        // Sample the fitted curve and add points
        Eigen::VectorXd fitted_curve_sample = nurbs_->estimateParamByArcLength(pt);
        for (int i = 0; i < fitted_curve_sample.size(); ++i)
        {
            geometry_msgs::msg::Point p;
            p.x = nurbs_->evaluatePolynomial(fitted_curve_coeffs, fitted_curve_sample(i))[0];
            p.y = nurbs_->evaluatePolynomial(fitted_curve_coeffs, fitted_curve_sample(i))[1];
            p.z = nurbs_->evaluatePolynomial(fitted_curve_coeffs, fitted_curve_sample(i))[2];
            fitted_curve_marker.points.push_back(p);
        }

        fitted_curve_pub_->publish(fitted_curve_marker);
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

    void publishDerivatives()
    {
        // Evaluate the NURBS curve and its derivatives
        auto start = std::chrono::high_resolution_clock::now();
        std::vector<std::vector<std::vector<double>>> ders = nurbs_->derivatives(3);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed = end - start;
        RCLCPP_INFO(this->get_logger(), "NURBS derivative evaluation took: %f ms", elapsed.count());
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fitted_curve_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr control_points_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr derivatives_timer_;
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
