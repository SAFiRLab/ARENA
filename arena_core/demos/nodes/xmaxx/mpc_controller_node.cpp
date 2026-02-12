/**
 * BSD 3-Clause License
 * 
 * Copyright (c) 2025, David-Alexandre Poissant, Université de Sherbrooke
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// ROS2
#include "rclcpp/rclcpp.hpp"
// ROS2 message types
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Eigen
#include <Eigen/Dense>

// System
#include <memory>


class DynamicModel
{
public:
    DynamicModel(double dt)
    : dt_(dt)
    {}

    virtual Eigen::VectorXd fx(const Eigen::VectorXd &state, const Eigen::VectorXd &control_input) = 0;
    
protected:
    // User-defined attributes
    double dt_; // Time step


}; // class DynamicModel


class BicycleModel : public DynamicModel
{
public:
    BicycleModel(double steering_ratio, double bicycle_length, double dt)
    : steering_ratio_(steering_ratio), bicycle_length_(bicycle_length), DynamicModel(dt)
    {}

    Eigen::VectorXd fx(const Eigen::VectorXd &state, const Eigen::VectorXd &control_input) override
    {
        // x <- [X, Y, yaw]
        // control_input <- [velocity, steering angle]

        Eigen::VectorXd next_state(3); // [x, y, yaw]

        double R = bicycle_length_ / (tan(control_input[1])); // Radius of curvature

        double x_dot = control_input[0] * cos(state[2]);
        double y_dot = control_input[0] * sin(state[2]);
        double yaw_dot = control_input[0] * tan(control_input[1]) / bicycle_length_;

        // Update the state
        next_state[0] = state[0] + x_dot * dt_; // x position
        next_state[1] = state[1] + y_dot * dt_; // y position
        next_state[2] = state[2] + yaw_dot * dt_; // yaw

        return next_state;
    }

private:

    // User-defined attributes
    double steering_ratio_; // Steering wheel angle / wheel angle
    double bicycle_length_; // Distance between front and rear wheels

}; // class BicycleModel


class MPCController
{
public:
    MPCController(double dt, unsigned int horizon)
    : dt_(dt), horizon_(horizon), dynamic_model_(std::make_shared<BicycleModel>(1.0, 1.0, dt))
    {}

    Eigen::MatrixXd solve(const Eigen::VectorXd &state, const Eigen::MatrixXd &reference)
    {
        const int N = horizon_;
        const int nx = 3;  // [x, y, yaw]
        const int nu = 2;  // [v, steering angle]

        const int n_vars = N * (nx + nu);  // decision vector size

        Eigen::VectorXd x0 = state;

        // QP matrices
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(n_vars, n_vars);
        Eigen::VectorXd f = Eigen::VectorXd::Zero(n_vars);

        // Cost weights
        Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(nx, nx);
        Q(0, 0) = 1.0; // x
        Q(1, 1) = 1.0; // y
        Q(2, 2) = 0.5; // yaw

        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(nu, nu);
        R(1, 1) = 0.01; // steering effort

        Eigen::VectorXd x_t = x0;

        for (int t = 0; t < N; ++t)
        {
            int x_idx = t * (nx + nu);
            int u_idx = x_idx + nx;

            // Tracking reference
            Eigen::VectorXd x_ref = Eigen::VectorXd::Zero(nx);
            if (t < reference.rows())
            {
                x_ref(0) = reference(t, 0);
                x_ref(1) = reference(t, 1);
                x_ref(2) = reference(t, 5);  // yaw
            }

            // Cost: (x - x_ref)^T Q (x - x_ref)
            H.block(x_idx, x_idx, nx, nx) = Q;
            f.segment(x_idx, nx) = -Q * x_ref;

            // Cost: u^T R u
            H.block(u_idx, u_idx, nu, nu) = R;
            // f.segment(u_idx, nu) = 0 already
        }

        // Solve: H * z = -f, where z = [x₀, u₀, x₁, u₁, ..., xₙ]
        Eigen::VectorXd z = H.ldlt().solve(-f);

        // Extract optimal states (not needed for control) and controls
        std::vector<Eigen::VectorXd> best_trajectory;
        Eigen::VectorXd a_state = x0;

        for (int t = 0; t < N; ++t)
        {
            int x_idx = t * (nx + nu);
            int u_idx = x_idx + nx;

            Eigen::VectorXd u_t = z.segment(u_idx, nu);

            // Simulate to get full trajectory (real dynamics)
            a_state = dynamic_model_->fx(a_state, u_t);

            Eigen::VectorXd full_state(6);
            full_state << a_state(0), a_state(1), 0.0, 0.0, 0.0, a_state(2);
            best_trajectory.push_back(full_state);
        }

        Eigen::MatrixXd predicted_path(N, 6);
        for (int i = 0; i < N; ++i)
            predicted_path.row(i) = best_trajectory[i].transpose();

        return predicted_path;
    }


private:

    // User-defined attributes
    double dt_; // Time step for the MPC controller
    unsigned int horizon_; // Prediction horizon for the MPC controller
    std::shared_ptr<DynamicModel> dynamic_model_; // Dynamic model of the system, e.g., BicycleModel

}; // class MPCController


class MPCControllernode : public rclcpp::Node
{
public:
    MPCControllernode()
    : Node("mpc_controller_node"), mpc_controller_(std::make_shared<MPCController>(0.1, 20))
    {
        // Initialize the publisher for visualization markers
        reference_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("mpc_reference_path", 10);
        predicted_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("mpc_predicted_path", 10);

        // Subscribers
        auto reference_path_callback = std::bind(&MPCControllernode::referencePathCallback, this, std::placeholders::_1);
        reference_path_sub_ = this->create_subscription<nav_msgs::msg::Path>("mpc_reference_path", 10, reference_path_callback);
        auto state_callback = std::bind(&MPCControllernode::currentStateCallback, this, std::placeholders::_1);
        current_state_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("groundTruth/poseStamped", 10, state_callback);

        // Timers
        using namespace std::chrono_literals;
        send_reference_path_timer_ = this->create_wall_timer(1000ms, std::bind(&MPCControllernode::sendReferencePath, this));
        send_predicted_path_timer_ = this->create_wall_timer(200ms, std::bind(&MPCControllernode::publishPredictedPath, this));
        mpc_timer_ = this->create_wall_timer(100ms, std::bind(&MPCControllernode::solveMPC, this));
    };

private:

    void solveMPC()
    {
        // Check if the current state is initialized
        if (current_state_.size() == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Current state not initialized.");
            return;
        }

        //nav_msgs::path to Eigen::MatrixXd conversion
        if (reference_path_.poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Reference path is empty, cannot solve MPC.");
            return;
        }

        Eigen::MatrixXd reference(reference_path_.poses.size(), 6);
        for (size_t i = 0; i < reference_path_.poses.size(); ++i)
        {
            const auto &pose = reference_path_.poses[i].pose;
            const auto &orientation = reference_path_.poses[i].pose.orientation;
            Eigen::Quaterniond quat(orientation.w, orientation.x, orientation.y, orientation.z);
            Eigen::Vector3d euler_angles = quat.toRotationMatrix().eulerAngles(0, 1, 2); // Roll, Pitch, Yaw
            reference(i, 0) = pose.position.x; // x position
            reference(i, 1) = pose.position.y; // y position
            reference(i, 2) = pose.position.z; // z position
            reference(i, 3) = euler_angles(0); // roll
            reference(i, 4) = euler_angles(1); // pitch
            reference(i, 5) = euler_angles(2); // yaw
        }

        // Solve the MPC problem
        Eigen::VectorXd initial_state(3); // x, y, yaw
        initial_state << current_state_[0], current_state_[1], current_state_[5];
        predicted_path_ = mpc_controller_->solve(initial_state, reference);
    }

    void sendReferencePath()
    {
        // Publish the reference path
        if (reference_path_.poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Reference path is empty, not publishing.");
            return;
        }
        
        reference_path_.header.stamp = this->now();
        reference_path_pub_->publish(reference_path_);
    }

    void publishPredictedPath()
    {
        // Publish the predicted path
        if (predicted_path_.rows() == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Predicted path is empty, not publishing.");
            return;
        }

        nav_msgs::msg::Path predicted_path_msg;
        predicted_path_msg.header.frame_id = "world";
        predicted_path_msg.header.stamp = this->now();

        for (int i = 0; i < predicted_path_.rows(); ++i)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = predicted_path_msg.header;
            pose_stamped.pose.position.x = predicted_path_(i, 0);
            pose_stamped.pose.position.y = predicted_path_(i, 1);
            pose_stamped.pose.position.z = predicted_path_(i, 2);
            Eigen::Quaterniond quat = Eigen::AngleAxisd(predicted_path_(i, 5), Eigen::Vector3d::UnitZ()) *
                                      Eigen::AngleAxisd(predicted_path_(i, 4), Eigen::Vector3d::UnitY()) *
                                      Eigen::AngleAxisd(predicted_path_(i, 3), Eigen::Vector3d::UnitX());
            pose_stamped.pose.orientation.x = quat.x();
            pose_stamped.pose.orientation.y = quat.y();
            pose_stamped.pose.orientation.z = quat.z();
            pose_stamped.pose.orientation.w = quat.w();
            predicted_path_msg.poses.push_back(pose_stamped);
        }

        predicted_path_pub_->publish(predicted_path_msg);
    }

    // Callback for the MPC controller
    void referencePathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        // Extract the reference path from the message
        Eigen::MatrixXd reference;
        if (msg->poses.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty reference path.");
            return;
        }
    }

    // Callback for the current state of the robot
    void currentStateCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Check the size of the current state vector
        if (current_state_.size() != 6)
        {
            current_state_.resize(6); // Resize to fit the state vector: x, y, z, roll, pitch, yaw
        }
        // Get orientation from quaternion
        Eigen::Quaterniond orientation(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
        // Convert quaternion to Euler angles (roll, pitch, yaw)
        Eigen::Vector3d euler_angles = orientation.toRotationMatrix().eulerAngles(0, 1, 2); // Roll, Pitch, Yaw
        // Update the current state of the robot
        current_state_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, 
                          euler_angles(0), euler_angles(1), euler_angles(2); // x, y, z, roll, pitch, yaw

        if (!reference_initialized_)
        {
            // Initialize the reference path
            reference_path_.header.frame_id = "world";
            reference_path_.header.stamp = this->now();
            reference_path_.poses.clear(); // Clear previous poses
            for (size_t i = 0; i < 10; ++i) // Example: Add 10 poses to the reference path
            {
                geometry_msgs::msg::PoseStamped pose_stamped;
                pose_stamped.header = reference_path_.header;
                pose_stamped.pose.position.x = current_state_[0] + 2.0 * i;
                pose_stamped.pose.position.y = current_state_[1];// + i * 0.1;
                pose_stamped.pose.position.z = current_state_[2];// + i * 0.1; // Example positions
                pose_stamped.pose.orientation.x = msg->pose.orientation.x;
                pose_stamped.pose.orientation.y = msg->pose.orientation.y;
                pose_stamped.pose.orientation.z = msg->pose.orientation.z;
                pose_stamped.pose.orientation.w = msg->pose.orientation.w;
                reference_path_.poses.push_back(pose_stamped);
            }
            reference_initialized_ = true; // Set the flag to true after initialization
        }
    }
    
    // ROS2 publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predicted_path_pub_;

    // ROS2 subscribers
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr reference_path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_state_sub_;

    // ROS2 Timers
    rclcpp::TimerBase::SharedPtr send_reference_path_timer_;
    rclcpp::TimerBase::SharedPtr send_predicted_path_timer_;
    rclcpp::TimerBase::SharedPtr mpc_timer_;

    // User-defined attributes
    std::shared_ptr<MPCController> mpc_controller_;
    Eigen::VectorXd current_state_; // Current position and orientation of the robot
    Eigen::MatrixXd predicted_path_; // Predicted path from the MPC controller
    nav_msgs::msg::Path reference_path_; // Reference path for the MPC controller 
    bool reference_initialized_ = false; // Flag to check if the reference path is initialized

}; // class MPCControllernode


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MPCControllernode>());
    rclcpp::shutdown();
    return 0;
}
