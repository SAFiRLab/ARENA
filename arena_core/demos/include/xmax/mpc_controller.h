#pragma once

// External Libraries
// Eigen
#include <Eigen/Dense>

// Local
#include "xmax/bicycle_model.h"


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
