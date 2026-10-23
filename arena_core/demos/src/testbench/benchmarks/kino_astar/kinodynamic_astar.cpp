/**
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, David-Alexandre Poissant, Université de Sherbrooke
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

#include "testbench/benchmarks/kino_astar/kinodynamic_astar.h"

// System
#include <cmath>


namespace arena_benchmarks
{

namespace
{
constexpr char IN_CLOSE_SET = 'a';
constexpr char IN_OPEN_SET = 'b';
constexpr char NOT_EXPAND = 'c';
}; // namespace

KinodynamicAstar::KinodynamicAstar(const KinodynamicAstarConfig& a_config, OccupancyFn a_occupied)
: config_(a_config), occupied_(std::move(a_occupied))
{
    // setParam()
    max_tau_ = config_.max_tau;
    init_max_tau_ = config_.init_max_tau;
    max_vel_ = config_.max_vel + config_.vel_margin;
    max_acc_ = config_.max_acc;
    w_time_ = config_.w_time;
    horizon_ = config_.horizon;
    resolution_ = config_.resolution_astar;
    lambda_heu_ = config_.lambda_heu;
    allocate_num_ = config_.allocate_num;
    check_num_ = config_.check_num;
    time_res_ = config_.time_res;
    norm_limits_ = config_.norm_limits;
    tie_breaker_ = 1.0 + 1.0 / 10000;

    // init()
    inv_resolution_ = 1.0 / resolution_;
    origin_ = config_.origin;
    map_size_3d_ = config_.map_size;
    phi_ = Eigen::MatrixXd::Identity(6, 6);
    use_node_num_ = 0;
    iter_num_ = 0;
}

KinodynamicAstar::~KinodynamicAstar() = default;

KinodynamicAstar::PathNodePtr KinodynamicAstar::newNode()
{
    // Nodes allocated when needed (preallocated pool of allocate_num nodes in Fast-Planner)
    if (use_node_num_ >= static_cast<int>(path_node_pool_.size()))
        path_node_pool_.push_back(std::make_unique<PathNode>());
    PathNodePtr node = path_node_pool_[use_node_num_].get();
    return node;
}

int KinodynamicAstar::search(Eigen::Vector3d start_pt, Eigen::Vector3d start_v, Eigen::Vector3d start_a, Eigen::Vector3d end_pt,
                             Eigen::Vector3d end_v, bool init)
{
    start_vel_ = start_v;
    start_acc_ = start_a;

    PathNodePtr cur_node = newNode();
    cur_node->parent = NULL;
    cur_node->state.head(3) = start_pt;
    cur_node->state.tail(3) = start_v;
    cur_node->index = posToIndex(start_pt);
    cur_node->g_score = 0.0;

    Eigen::VectorXd end_state(6);
    Eigen::Vector3i end_index;
    double time_to_goal;

    end_state.head(3) = end_pt;
    end_state.tail(3) = end_v;
    end_index = posToIndex(end_pt);
    cur_node->f_score = lambda_heu_ * estimateHeuristic(cur_node->state, end_state, time_to_goal);
    cur_node->node_state = IN_OPEN_SET;
    open_set_.push(cur_node);
    use_node_num_ += 1;

    expanded_nodes_.insert(std::make_pair(cur_node->index, cur_node));

    PathNodePtr terminate_node = NULL;
    bool init_search = init;
    const int tolerance = ceil(1 / resolution_);

    while (!open_set_.empty())
    {
        cur_node = open_set_.top();

        // Terminate?
        bool reach_horizon = (cur_node->state.head(3) - start_pt).norm() >= horizon_;
        bool near_end = abs(cur_node->index(0) - end_index(0)) <= tolerance &&
                        abs(cur_node->index(1) - end_index(1)) <= tolerance &&
                        abs(cur_node->index(2) - end_index(2)) <= tolerance;

        if (reach_horizon || near_end)
        {
            terminate_node = cur_node;
            retrievePath(terminate_node);
            if (near_end)
            {
                // Check whether shot traj exist
                estimateHeuristic(cur_node->state, end_state, time_to_goal);
                computeShotTraj(cur_node->state, end_state, time_to_goal);
            }
        }
        if (reach_horizon)
            return is_shot_succ_ ? REACH_END : REACH_HORIZON;

        if (near_end)
        {
            if (is_shot_succ_)
                return REACH_END;
            else if (norm_limits_)
                path_nodes_.clear(); // Shot beyond the limits (checked with norm_limits only): the search goes on
            else if (cur_node->parent != NULL)
                return NEAR_END;
            else
                return NO_PATH;
        }
        open_set_.pop();
        cur_node->node_state = IN_CLOSE_SET;
        iter_num_ += 1;

        double res = 1 / 2.0, time_res = time_res_, time_res_init = 1 / 20.0;
        Eigen::Matrix<double, 6, 1> cur_state = cur_node->state;
        Eigen::Matrix<double, 6, 1> pro_state;
        std::vector<PathNodePtr> tmp_expand_nodes;
        Eigen::Vector3d um;
        std::vector<Eigen::Vector3d> inputs;
        std::vector<double> durations;
        if (init_search)
        {
            inputs.push_back(start_acc_);
            for (double tau = time_res_init * init_max_tau_; tau <= init_max_tau_ + 1e-3; tau += time_res_init * init_max_tau_)
                durations.push_back(tau);
            init_search = false;
        }
        else
        {
            for (double ax = -max_acc_; ax <= max_acc_ + 1e-3; ax += max_acc_ * res)
                for (double ay = -max_acc_; ay <= max_acc_ + 1e-3; ay += max_acc_ * res)
                    for (double az = -max_acc_; az <= max_acc_ + 1e-3; az += max_acc_ * res)
                    {
                        um << ax, ay, az;
                        if (norm_limits_ && um.norm() > max_acc_ + 1e-6)
                            continue;
                        inputs.push_back(um);
                    }
            for (double tau = time_res * max_tau_; tau <= max_tau_ + 1e-6; tau += time_res * max_tau_)
                durations.push_back(tau);
        }

        for (size_t i = 0; i < inputs.size(); ++i)
            for (size_t j = 0; j < durations.size(); ++j)
            {
                um = inputs[i];
                double tau = durations[j];
                stateTransit(cur_state, pro_state, um, tau);

                Eigen::Vector3d pro_pos = pro_state.head(3);

                // Check if in close set
                Eigen::Vector3i pro_id = posToIndex(pro_pos);
                auto found = expanded_nodes_.find(pro_id);
                PathNodePtr pro_node = found == expanded_nodes_.end() ? NULL : found->second;
                if (pro_node != NULL && pro_node->node_state == IN_CLOSE_SET)
                    continue;

                // Check maximal velocity
                Eigen::Vector3d pro_v = pro_state.tail(3);
                if (norm_limits_ ? pro_v.norm() > max_vel_ :
                    fabs(pro_v(0)) > max_vel_ || fabs(pro_v(1)) > max_vel_ || fabs(pro_v(2)) > max_vel_)
                    continue;

                // Check not in the same voxel
                Eigen::Vector3i diff = pro_id - cur_node->index;
                if (diff.norm() == 0)
                    continue;

                // Check safety
                Eigen::Vector3d pos;
                Eigen::Matrix<double, 6, 1> xt;
                bool is_occ = false;
                for (int k = 1; k <= check_num_; ++k)
                {
                    double dt = tau * double(k) / double(check_num_);
                    stateTransit(cur_state, xt, um, dt);
                    pos = xt.head(3);
                    if (occupied_(pos))
                    {
                        is_occ = true;
                        break;
                    }
                }
                if (is_occ)
                    continue;

                double tmp_g_score, tmp_f_score;
                tmp_g_score = (um.squaredNorm() + w_time_) * tau + cur_node->g_score;
                tmp_f_score = tmp_g_score + lambda_heu_ * estimateHeuristic(pro_state, end_state, time_to_goal);

                // Compare nodes expanded from the same parent
                bool prune = false;
                for (size_t j = 0; j < tmp_expand_nodes.size(); ++j)
                {
                    PathNodePtr expand_node = tmp_expand_nodes[j];
                    if ((pro_id - expand_node->index).norm() == 0)
                    {
                        prune = true;
                        if (tmp_f_score < expand_node->f_score)
                        {
                            expand_node->f_score = tmp_f_score;
                            expand_node->g_score = tmp_g_score;
                            expand_node->state = pro_state;
                            expand_node->input = um;
                            expand_node->duration = tau;
                        }
                        break;
                    }
                }

                // This node end up in a voxel different from others
                if (!prune)
                {
                    if (pro_node == NULL)
                    {
                        pro_node = newNode();
                        pro_node->index = pro_id;
                        pro_node->state = pro_state;
                        pro_node->f_score = tmp_f_score;
                        pro_node->g_score = tmp_g_score;
                        pro_node->input = um;
                        pro_node->duration = tau;
                        pro_node->parent = cur_node;
                        pro_node->node_state = IN_OPEN_SET;
                        open_set_.push(pro_node);

                        expanded_nodes_.insert(std::make_pair(pro_id, pro_node));

                        tmp_expand_nodes.push_back(pro_node);

                        use_node_num_ += 1;
                        if (use_node_num_ == allocate_num_)
                            return NO_PATH; // Run out of memory
                    }
                    else if (pro_node->node_state == IN_OPEN_SET)
                    {
                        if (tmp_g_score < pro_node->g_score)
                        {
                            pro_node->state = pro_state;
                            pro_node->f_score = tmp_f_score;
                            pro_node->g_score = tmp_g_score;
                            pro_node->input = um;
                            pro_node->duration = tau;
                            pro_node->parent = cur_node;
                        }
                    }
                }
            }
    }

    return NO_PATH; // Open set empty, no path
}

void KinodynamicAstar::retrievePath(PathNodePtr end_node)
{
    PathNodePtr cur_node = end_node;
    path_nodes_.push_back(cur_node);

    while (cur_node->parent != NULL)
    {
        cur_node = cur_node->parent;
        path_nodes_.push_back(cur_node);
    }

    std::reverse(path_nodes_.begin(), path_nodes_.end());
}

double KinodynamicAstar::estimateHeuristic(Eigen::VectorXd x1, Eigen::VectorXd x2, double& optimal_time)
{
    const Eigen::Vector3d dp = x2.head(3) - x1.head(3);
    const Eigen::Vector3d v0 = x1.segment(3, 3);
    const Eigen::Vector3d v1 = x2.segment(3, 3);

    double c1 = -36 * dp.dot(dp);
    double c2 = 24 * (v0 + v1).dot(dp);
    double c3 = -4 * (v0.dot(v0) + v0.dot(v1) + v1.dot(v1));
    double c4 = 0;
    double c5 = w_time_;

    std::vector<double> ts = quartic(c5, c4, c3, c2, c1);

    double v_max = max_vel_ * 0.5;
    double t_bar = (x1.head(3) - x2.head(3)).lpNorm<Eigen::Infinity>() / v_max;
    ts.push_back(t_bar);

    double cost = 100000000;
    double t_d = t_bar;

    for (auto t : ts)
    {
        if (t < t_bar)
            continue;
        double c = -c1 / (3 * t * t * t) - c2 / (2 * t * t) - c3 / t + w_time_ * t;
        if (c < cost)
        {
            cost = c;
            t_d = t;
        }
    }

    optimal_time = t_d;

    return 1.0 * (1 + tie_breaker_) * cost;
}

bool KinodynamicAstar::computeShotTraj(Eigen::VectorXd state1, Eigen::VectorXd state2, double time_to_goal)
{
    /* ---------- get coefficient ---------- */
    const Eigen::Vector3d p0 = state1.head(3);
    const Eigen::Vector3d dp = state2.head(3) - p0;
    const Eigen::Vector3d v0 = state1.segment(3, 3);
    const Eigen::Vector3d v1 = state2.segment(3, 3);
    const Eigen::Vector3d dv = v1 - v0;
    double t_d = time_to_goal;
    Eigen::MatrixXd coef(3, 4);
    end_vel_ = v1;

    Eigen::Vector3d a = 1.0 / 6.0 * (-12.0 / (t_d * t_d * t_d) * (dp - v0 * t_d) + 6 / (t_d * t_d) * dv);
    Eigen::Vector3d b = 0.5 * (6.0 / (t_d * t_d) * (dp - v0 * t_d) - 2 / t_d * dv);
    Eigen::Vector3d c = v0;
    Eigen::Vector3d d = p0;

    // 1/6 * alpha * t^3 + 1/2 * beta * t^2 + v0
    // a*t^3 + b*t^2 + v0*t + p0
    coef.col(3) = a, coef.col(2) = b, coef.col(1) = c, coef.col(0) = d;

    Eigen::Vector3d coord;
    Eigen::VectorXd poly1d, t;

    /* ---------- forward checking of trajectory ---------- */
    // The velocity and acceleration limits aren't checked in Fast-Planner (its check is commented out, the B-spline
    // optimization makes the trajectory feasible). With norm_limits, the shot is the end of the returned trajectory: its
    // acceleration (linear in time, largest norm at an end) and its velocity (every 0.01 s) must respect the limits
    if (norm_limits_)
    {
        if ((2.0 * b).norm() > max_acc_ + 1e-9 || (2.0 * b + 6.0 * a * t_d).norm() > max_acc_ + 1e-9)
            return false;
        for (double time = 0.0; time <= t_d + 0.01; time += 0.01)
        {
            const double tc = std::min(time, t_d);
            if ((c + 2.0 * b * tc + 3.0 * a * tc * tc).norm() > max_vel_ + 1e-9)
                return false;
        }
    }
    double t_delta = t_d / 10;
    for (double time = t_delta; time <= t_d; time += t_delta)
    {
        t = Eigen::VectorXd::Zero(4);
        for (int j = 0; j < 4; j++)
            t(j) = pow(time, j);

        for (int dim = 0; dim < 3; dim++)
        {
            poly1d = coef.row(dim);
            coord(dim) = poly1d.dot(t);
        }

        if (coord(0) < origin_(0) || coord(0) >= map_size_3d_(0) || coord(1) < origin_(1) || coord(1) >= map_size_3d_(1) ||
            coord(2) < origin_(2) || coord(2) >= map_size_3d_(2))
        {
            return false;
        }

        if (occupied_(coord))
            return false;
    }
    coef_shot_ = coef;
    t_shot_ = t_d;
    is_shot_succ_ = true;
    return true;
}

std::vector<double> KinodynamicAstar::cubic(double a, double b, double c, double d)
{
    std::vector<double> dts;

    double a2 = b / a;
    double a1 = c / a;
    double a0 = d / a;

    double Q = (3 * a1 - a2 * a2) / 9;
    double R = (9 * a1 * a2 - 27 * a0 - 2 * a2 * a2 * a2) / 54;
    double D = Q * Q * Q + R * R;
    if (D > 0)
    {
        double S = std::cbrt(R + sqrt(D));
        double T = std::cbrt(R - sqrt(D));
        dts.push_back(-a2 / 3 + (S + T));
        return dts;
    }
    else if (D == 0)
    {
        double S = std::cbrt(R);
        dts.push_back(-a2 / 3 + S + S);
        dts.push_back(-a2 / 3 - S);
        return dts;
    }
    else
    {
        double theta = acos(R / sqrt(-Q * Q * Q));
        dts.push_back(2 * sqrt(-Q) * cos(theta / 3) - a2 / 3);
        dts.push_back(2 * sqrt(-Q) * cos((theta + 2 * M_PI) / 3) - a2 / 3);
        dts.push_back(2 * sqrt(-Q) * cos((theta + 4 * M_PI) / 3) - a2 / 3);
        return dts;
    }
}

std::vector<double> KinodynamicAstar::quartic(double a, double b, double c, double d, double e)
{
    std::vector<double> dts;

    double a3 = b / a;
    double a2 = c / a;
    double a1 = d / a;
    double a0 = e / a;

    std::vector<double> ys = cubic(1, -a2, a1 * a3 - 4 * a0, 4 * a2 * a0 - a1 * a1 - a3 * a3 * a0);
    double y1 = ys.front();
    double r = a3 * a3 / 4 - a2 + y1;
    if (r < 0)
        return dts;

    double R = sqrt(r);
    double D, E;
    if (R != 0)
    {
        D = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 + 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
        E = sqrt(0.75 * a3 * a3 - R * R - 2 * a2 - 0.25 * (4 * a3 * a2 - 8 * a1 - a3 * a3 * a3) / R);
    }
    else
    {
        D = sqrt(0.75 * a3 * a3 - 2 * a2 + 2 * sqrt(y1 * y1 - 4 * a0));
        E = sqrt(0.75 * a3 * a3 - 2 * a2 - 2 * sqrt(y1 * y1 - 4 * a0));
    }

    if (!std::isnan(D))
    {
        dts.push_back(-a3 / 4 + R / 2 + D / 2);
        dts.push_back(-a3 / 4 + R / 2 - D / 2);
    }
    if (!std::isnan(E))
    {
        dts.push_back(-a3 / 4 - R / 2 + E / 2);
        dts.push_back(-a3 / 4 - R / 2 - E / 2);
    }

    return dts;
}

void KinodynamicAstar::reset()
{
    expanded_nodes_.clear();
    path_nodes_.clear();

    std::priority_queue<PathNodePtr, std::vector<PathNodePtr>, NodeComparator> empty_queue;
    open_set_.swap(empty_queue);

    for (int i = 0; i < use_node_num_; i++)
    {
        PathNodePtr node = path_node_pool_[i].get();
        node->parent = NULL;
        node->node_state = NOT_EXPAND;
    }

    use_node_num_ = 0;
    iter_num_ = 0;
    is_shot_succ_ = false;
}

NativeTrajectory KinodynamicAstar::getTrajectory(double a_delta_t) const
{
    NativeTrajectory trajectory;
    if (path_nodes_.empty())
        return trajectory;

    // Primitives of the searched path (path_nodes_ from the start to the end node)
    TrajectoryPoint point;
    point.position = path_nodes_.front()->state.head(3);
    point.velocity = path_nodes_.front()->state.tail(3);
    point.time = 0.0;
    trajectory.push_back(point);
    for (size_t i = 1; i < path_nodes_.size(); ++i)
    {
        const TrajectoryPoint start = trajectory.back();
        appendConstantAcceleration(trajectory, start, path_nodes_[i]->input, path_nodes_[i]->duration, a_delta_t);
    }

    // Shot trajectory: p(t) = d + c t + b t^2 + a t^3
    if (is_shot_succ_)
    {
        const double t0 = trajectory.back().time;
        const int nb_of_steps = std::max(1, static_cast<int>(std::ceil(t_shot_ / a_delta_t - 1.0e-9)));
        for (int k = 1; k <= nb_of_steps; ++k)
        {
            const double t = k == nb_of_steps ? t_shot_ : k * a_delta_t;
            TrajectoryPoint shot;
            shot.position = coef_shot_.col(0) + coef_shot_.col(1) * t + coef_shot_.col(2) * t * t + coef_shot_.col(3) * t * t * t;
            shot.velocity = coef_shot_.col(1) + 2.0 * coef_shot_.col(2) * t + 3.0 * coef_shot_.col(3) * t * t;
            shot.time = t0 + t;
            trajectory.push_back(shot);
        }
    }
    return trajectory;
}

Eigen::Vector3i KinodynamicAstar::posToIndex(Eigen::Vector3d pt)
{
    Eigen::Vector3i idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
    return idx;
}

void KinodynamicAstar::stateTransit(Eigen::Matrix<double, 6, 1>& state0, Eigen::Matrix<double, 6, 1>& state1,
                                    Eigen::Vector3d um, double tau)
{
    for (int i = 0; i < 3; ++i)
        phi_(i, i + 3) = tau;

    Eigen::Matrix<double, 6, 1> integral;
    integral.head(3) = 0.5 * pow(tau, 2) * um;
    integral.tail(3) = tau * um;

    state1 = phi_ * state0 + integral;
}

}; // namespace arena_benchmarks
