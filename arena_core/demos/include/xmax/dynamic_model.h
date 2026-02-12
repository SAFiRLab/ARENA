#pragma once

// External Libraries
// Eigen
#include <Eigen/Dense>


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
