#pragma once

// External Libraries
// Eigen
#include <Eigen/Dense>

// Local
#include "xmax/dynamic_model.h"



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
