// ROS 2
#include "rclcpp/rclcpp.hpp"

// ROS Messages
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <arena_msgs/msg/optimizer_path_info.hpp>
#include <arena_msgs/msg/mission.hpp>
#include <arena_msgs/msg/coordinate.hpp>
#include <arena_msgs/msg/mission_risks.hpp>

// Octomap
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

// System
#include <string>
#include <functional>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <iomanip>
#include <map>
#include <chrono>
#include <thread>
#include <cmath>
#include <limits>


#define RATE 10

using namespace std::chrono_literals;

namespace arena_testbench
{

namespace linedrone
{

// Planner parameters modified by the tests
const std::string PLANNER_PARAM_NB_OF_GENERATIONS = "optimization.NSGA-II.generations";
const std::string PLANNER_PARAM_POPULATION_SIZE = "optimization.NSGA-II.population_size";
const std::string PLANNER_PARAM_NURBS_SAMPLE_SIZE = "optimization.sample_size";
const std::string PLANNER_PARAM_RRT_RANGE = "optimization.initialization.rrt_range";
const std::string PLANNER_PARAM_COST_TIME = "optimization.adaptive_costs_weights.time";
const std::string PLANNER_PARAM_COST_SAFETY = "optimization.adaptive_costs_weights.safety";
const std::string PLANNER_PARAM_COST_ENERGY = "optimization.adaptive_costs_weights.energy";

class TestbenchNode : public rclcpp::Node
{
public:

    TestbenchNode();
    ~TestbenchNode()
    {
        if (costmap_)
            delete costmap_;
    }

    void run();

    // Getters

    // Setters

private:

    // Methods
    void initCSVFile();
    void writePlanningReport(const arena_msgs::msg::OptimizerPathInfo& msg);
    void writeRisksReport(const arena_msgs::msg::OptimizerPathInfo& msg);
    void writeParetoFront(const arena_msgs::msg::OptimizerPathInfo& msg);
    void runStepsVariationTests(rclcpp::Rate& rate);
    void runHyperparametersVariationTests(rclcpp::Rate& rate);
    void runOptimalSolutionPerObjectiveTests(rclcpp::Rate& rate);
    void runRisksVarationTests(rclcpp::Rate& rate);

    void publishMission();
    void publishMissionRisksPaths();
    void publishRisksVariationPaths();

    double getClosestObstacleDistance(const octomap::point3d&);

    // Planner management
    bool isPlannerRunning();
    void startPlanner();
    void killPlanner();
    void requestPlanning();
    void setPlannerParameter(const std::string& name, const rclcpp::ParameterValue& value);
    void pushPlannerParameters(const std::map<std::string, rclcpp::ParameterValue>& params);
    template <typename PublisherT>
    void waitForSubscribers(const PublisherT& pub, std::chrono::milliseconds timeout);
    void spinOnce();

    // Callbacks
    void pathPlanningFinishedCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void nurbsInfosCallback(const arena_msgs::msg::OptimizerPathInfo::SharedPtr msg);
    void costmapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
    void missionRisksCallback(const arena_msgs::msg::MissionRisks::SharedPtr msg);

    // Executor used to spin the node from the tests loops
    rclcpp::executors::SingleThreadedExecutor executor_;

    // Node used to set the planner parameters
    rclcpp::Node::SharedPtr param_client_node_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr path_planning_finished_counter_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr planning_activated_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr planning_goal_pub_;
    rclcpp::Publisher<arena_msgs::msg::Mission>::SharedPtr mission_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr nurbs_from_risks_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr risks_variation_paths_pub_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr path_planning_finished_sub_;
    rclcpp::Subscription<arena_msgs::msg::OptimizerPathInfo>::SharedPtr nurbs_infos_sub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr costmap_sub_;
    rclcpp::Subscription<arena_msgs::msg::MissionRisks>::SharedPtr mission_risks_sub_;

    // User-defined attributes
    int path_planning_finished_counter_;
    std::string folder_name_;
    std::string csv_filename_;
    std::string pareto_front_filename_;
    // Steps and hyperparameters variation tests write the planning report, the other tests the risks report
    bool planning_report_;
    int nurbs_infos_counter_;
    bool steps_variation_tests_;
    bool hyperparameters_variation_tests_;
    bool optimal_solution_per_objective_tests_;
    bool risks_variation_tests_;
    bool costmap_received_;
    bool planning_requested_;
    geometry_msgs::msg::PointStamped goal_msg_;
    std::vector<std::vector<double>> mission_risks_;
    std::vector<visualization_msgs::msg::Marker> mission_risks_paths_;
    std::vector<std::vector<double>> mission_risks_paths_costs_;
    std::vector<visualization_msgs::msg::Marker> risks_variation_paths_;

    // Planner attributes
    std::string planner_node_name_;
    std::string planner_start_command_;
    std::string planner_kill_command_;
    std::chrono::milliseconds planner_startup_timeout_;
    // Parameters set on the planner, re-applied every time the planner is restarted
    std::map<std::string, rclcpp::ParameterValue> planner_params_;

    octomap::OcTree* costmap_;

}; // class TestbenchNode


void TestbenchNode::initCSVFile()
{
    std::string world_name = this->get_parameter("world_name").as_string();

    // Create the data_output folder if it doesn't exist
    std::error_code error;
    std::filesystem::create_directories(folder_name_, error);
    if (error)
    {
        RCLCPP_ERROR(get_logger(), "Failed to create folder: %s (%s)", folder_name_.c_str(), error.message().c_str());
        return;
    }

    // Get the current system time
    std::time_t raw_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm* time_info = std::localtime(&raw_time);

    // Format the time as a string
    std::stringstream ss;
    ss << std::put_time(time_info, "%Y-%m-%d_%H-%M-%S");
    std::string time_string = ss.str();

    csv_filename_ = folder_name_;
    csv_filename_.append("/report_3d_");
    csv_filename_.append(world_name.c_str());
    csv_filename_.append("_");
    csv_filename_.append(time_string);
    csv_filename_.append(".csv");

    // Open the CSV file
    std::ofstream csv_file;
    csv_file.open(csv_filename_, std::ios::out | std::ios::app);
    if (!csv_file.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open CSV file: %s", csv_filename_.c_str());
        return;
    }

    RCLCPP_INFO(get_logger(), "Writing report to %s", csv_filename_.c_str());

    // Write the header of the CSV file
    if (planning_report_)
    {
        // One row per pose of the chosen path, the planning values are repeated on every row of a same Id.
        // "Time cost", "Security cost" and "Energy cost" are the costs of the chosen solution (same as "Chosen ... cost"),
        // they are set to the max double when no feasible solution has been found.
        csv_file <<
        "Id" << "," <<
        "Planing Time" << "," <<
        "Number of generations" << "," <<
        "Population size" << "," <<
        "Nurbs sample size" << "," <<
        "Time cost" << "," <<
        "Security cost" << "," <<
        "Energy cost" << "," <<
        "Best time cost" << "," <<
        "Best security cost" << "," <<
        "Best energy cost" << "," <<
        "Chosen time cost" << "," <<
        "Chosen security cost" << "," <<
        "Chosen energy cost" << "," <<
        "Drone Position X" << "," <<
        "Drone Position Y" << "," <<
        "Drone Position Z" << "," <<
        "Drone Orientation X" << "," <<
        "Drone Orientation Y" << "," <<
        "Drone Orientation Z" << "," <<
        "Drone Orientation W" << "," <<
        "Drone Velocity Norm" << "," <<
        "Closest Obstacle Distance" << "," <<
        "Time coefficient" << "," <<
        "Security coefficient" << "," <<
        "Energy coefficient" << "," <<
        "Feasible" << "," <<
        "Pareto front size" << "," <<
        "RRT range" << "," <<
        "Initialization time" << "," <<
        "Optimization time" << "," <<
        "Number of control points" << std::endl;
    }
    else
    {
        csv_file <<
        "Id" << "," <<
        "Path Duration" << "," <<
        "Average Cloest Obstacle Distance" << "," <<
        "Energy Cost" << "," <<
        "Baterry risk" << "," <<
        "Location risk" << "," <<
        "Wind risk" <<
        std::endl;
    }

    csv_file.close();

    // Pareto front of every planning, one row per non-dominated solution
    pareto_front_filename_ = csv_filename_.substr(0, csv_filename_.size() - 4) + "_pareto_front.csv";
    std::ofstream pareto_file(pareto_front_filename_, std::ios::out | std::ios::app);
    if (!pareto_file.is_open())
    {
        RCLCPP_ERROR(get_logger(), "Failed to open CSV file: %s", pareto_front_filename_.c_str());
        return;
    }

    RCLCPP_INFO(get_logger(), "Writing Pareto fronts to %s", pareto_front_filename_.c_str());

    pareto_file <<
    "Id" << "," <<
    "Number of generations" << "," <<
    "Population size" << "," <<
    "Nurbs sample size" << "," <<
    "Time coefficient" << "," <<
    "Security coefficient" << "," <<
    "Energy coefficient" << "," <<
    "Time cost" << "," <<
    "Security cost" << "," <<
    "Energy cost" << "," <<
    "RRT range" << std::endl;

    pareto_file.close();
}

void TestbenchNode::spinOnce()
{
    executor_.spin_some();
}

template <typename PublisherT>
void TestbenchNode::waitForSubscribers(const PublisherT& pub, std::chrono::milliseconds timeout)
{
    // ROS 2 discovery is asynchronous, make sure the planner is connected before publishing to it
    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && pub->get_subscription_count() == 0)
    {
        if (std::chrono::steady_clock::now() - start > timeout)
        {
            RCLCPP_WARN(get_logger(), "No subscriber on %s", pub->get_topic_name());
            return;
        }
        std::this_thread::sleep_for(50ms);
    }
}

bool TestbenchNode::isPlannerRunning()
{
    std::vector<std::string> node_list = this->get_node_names();
    return std::find(node_list.begin(), node_list.end(), planner_node_name_) != node_list.end();
}

void TestbenchNode::pushPlannerParameters(const std::map<std::string, rclcpp::ParameterValue>& params)
{
    if (params.empty())
        return;

    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(param_client_node_, planner_node_name_);
    if (!param_client->wait_for_service(5s))
    {
        RCLCPP_ERROR(get_logger(), "Parameter service of %s not available", planner_node_name_.c_str());
        return;
    }

    std::vector<rclcpp::Parameter> parameters;
    for (const auto& [name, value] : params)
        parameters.emplace_back(name, value);

    std::vector<rcl_interfaces::msg::SetParametersResult> results = param_client->set_parameters(parameters);
    if (results.size() != parameters.size())
    {
        RCLCPP_ERROR(get_logger(), "Failed to set the parameters of %s", planner_node_name_.c_str());
        return;
    }

    for (size_t i = 0; i < results.size(); i++)
    {
        if (!results[i].successful)
            RCLCPP_ERROR(get_logger(), "Failed to set %s on %s: %s", parameters[i].get_name().c_str(),
                         planner_node_name_.c_str(), results[i].reason.c_str());
    }
}

void TestbenchNode::setPlannerParameter(const std::string& name, const rclcpp::ParameterValue& value)
{
    planner_params_[name] = value;

    if (isPlannerRunning())
        pushPlannerParameters({{name, value}});
}

void TestbenchNode::startPlanner()
{
    if (system(planner_start_command_.c_str()) != 0)
        RCLCPP_ERROR(get_logger(), "Planner start command failed: %s", planner_start_command_.c_str());

    // Wait for the planner to appear in the ROS graph
    std::vector<std::string> node_list;
    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !isPlannerRunning() && std::chrono::steady_clock::now() - start < planner_startup_timeout_)
        std::this_thread::sleep_for(500ms);

    // print the list of nodes
    node_list = this->get_node_names();
    for (size_t i = 0; i < node_list.size(); i++)
        RCLCPP_INFO(get_logger(), "Node: %s", node_list[i].c_str());

    if (!isPlannerRunning())
    {
        RCLCPP_ERROR(get_logger(), "Failed to start %s", planner_node_name_.c_str());
        return;
    }

    // Parameters live on the planner node in ROS 2, so they are lost when it restarts
    pushPlannerParameters(planner_params_);

    publishMission();

    // The planner died while planning, the request has been lost with it
    if (planning_requested_)
    {
        RCLCPP_WARN(get_logger(), "%s stopped while planning, sending the planning request again", planner_node_name_.c_str());
        requestPlanning();
    }
}

void TestbenchNode::killPlanner()
{
    // Kill the planner because there is a memory leak and we can't plan too many times in a row
    // Publish planning activated
    std_msgs::msg::Bool planning_activated_msg;
    planning_activated_msg.data = false;
    planning_activated_pub_->publish(planning_activated_msg);

    // Let the time to the path planning node to deactivate the planning
    rclcpp::sleep_for(200ms);

    if (system(planner_kill_command_.c_str()) != 0)
        RCLCPP_ERROR(get_logger(), "Planner kill command failed: %s", planner_kill_command_.c_str());

    // Wait for the planner to leave the ROS graph
    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && isPlannerRunning())
    {
        if (std::chrono::steady_clock::now() - start > 10s)
        {
            RCLCPP_ERROR(get_logger(), "%s is still running after being killed", planner_node_name_.c_str());
            break;
        }
        std::this_thread::sleep_for(200ms);
    }
}

void TestbenchNode::requestPlanning()
{
    waitForSubscribers(planning_activated_pub_, 5s);
    waitForSubscribers(planning_goal_pub_, 5s);

    // Publish planning activated
    std_msgs::msg::Bool planning_activated_msg;
    planning_activated_msg.data = true;
    planning_activated_pub_->publish(planning_activated_msg);

    // Let the time to the path planning node to activate the planning
    rclcpp::sleep_for(200ms);

    // Publish planning goal
    goal_msg_.header.stamp = this->now();
    planning_goal_pub_->publish(goal_msg_);

    planning_requested_ = true;
}

void TestbenchNode::runStepsVariationTests(rclcpp::Rate& rate)
{
    // Get the number of steps to vary
    double coeff_steps = this->get_parameter("step_variation_tests.coeff_steps").as_double();
    if (coeff_steps == 0.0)
    {
        RCLCPP_ERROR(get_logger(), "Number of steps is 0");
        return;
    }

    // Get the number of iterations for every sets of coeff
    int nb_of_iterations = static_cast<int>(this->get_parameter("step_variation_tests.nb_of_iter").as_double());
    if (nb_of_iterations == 0.0)
    {
        RCLCPP_ERROR(get_logger(), "Number of iterations is 0.0");
        return;
    }

    bool first_iteration = true;
    int counter = 0;
    int max_pagmo_optimizer_node_calls = 5;
    int pagmo_calls = 0;
    int iteration_counter = 0;

    double time_coeff = 0.0;
    double security_coeff = 0.0;
    double energy_coeff = 0.0;

    // Set the coefficients parameters
    setPlannerParameter(PLANNER_PARAM_COST_TIME, rclcpp::ParameterValue(time_coeff));
    setPlannerParameter(PLANNER_PARAM_COST_SAFETY, rclcpp::ParameterValue(security_coeff));
    setPlannerParameter(PLANNER_PARAM_COST_ENERGY, rclcpp::ParameterValue(energy_coeff));

    // Main loop
    while (rclcpp::ok())
    {
        // Check if the planner is running
        if (isPlannerRunning())
        {
            if (!costmap_received_)
            {
                // Sleep
                rate.sleep();
                spinOnce();
                continue;
            }

            if (first_iteration)
            {
                requestPlanning();

                first_iteration = false;
            }
            else
            {
                if (path_planning_finished_counter_ > counter)
                {
                    if (iteration_counter >= nb_of_iterations)
                    {
                        // Change the coefficients, the sum of the coefficients must be less or equal to 1
                        energy_coeff += coeff_steps;
                        if (energy_coeff > 1.0 || energy_coeff + security_coeff + time_coeff > 1.0)
                        {
                            energy_coeff = 0.0;
                            security_coeff += coeff_steps;
                            if (security_coeff > 1.0 || energy_coeff + security_coeff + time_coeff > 1.0)
                            {
                                security_coeff = 0.0;
                                time_coeff += coeff_steps;
                                if (time_coeff > 1.0)
                                {
                                    time_coeff = 0.0;
                                    RCLCPP_INFO(get_logger(), "End of the steps variation tests");
                                    spinOnce();
                                    return;
                                }
                            }
                        }

                        // Set the coefficients parameters
                        setPlannerParameter(PLANNER_PARAM_COST_TIME, rclcpp::ParameterValue(time_coeff));
                        setPlannerParameter(PLANNER_PARAM_COST_SAFETY, rclcpp::ParameterValue(security_coeff));
                        setPlannerParameter(PLANNER_PARAM_COST_ENERGY, rclcpp::ParameterValue(energy_coeff));

                        iteration_counter = 0;
                    }

                    if (pagmo_calls >= max_pagmo_optimizer_node_calls)
                    {
                        killPlanner();

                        pagmo_calls = 0;
                        costmap_received_ = false;
                        continue;
                    }

                    requestPlanning();

                    RCLCPP_INFO(get_logger(), "Coefficients: time: %f, security: %f, energy: %f", time_coeff, security_coeff, energy_coeff);
                    RCLCPP_INFO(get_logger(), "Iteration: %i / %i", iteration_counter, nb_of_iterations);

                    pagmo_calls++;
                    counter++;
                    iteration_counter++;
                }

            }
        }
        else
        {
            startPlanner();
        }

        // Sleep
        rate.sleep();
        spinOnce();
    }
}

void TestbenchNode::runHyperparametersVariationTests(rclcpp::Rate& rate)
{
    // Get the number of steps to vary
    double hyperparameter_steps = this->get_parameter("hyperparameters_variation_tests.hyperparameter_steps").as_double();
    if (hyperparameter_steps == 0.0)
    {
        RCLCPP_ERROR(get_logger(), "Number of steps is 0");
        return;
    }

    double hyperparameter_current = this->get_parameter("hyperparameters_variation_tests.hyperparameter_min").as_double();
    double hyperparameter_max = this->get_parameter("hyperparameters_variation_tests.hyperparameter_max").as_double();

    // Get which hyperparameter to vary
    std::string hyperparameter_name = this->get_parameter("hyperparameters_variation_tests.hyperparameter_name").as_string();

    // Define witch updateHyperparameter lambda function to call
    // The planner hyperparameters are integers
    auto updateNbOfGenerations = [this](double value) -> void
    { setPlannerParameter(PLANNER_PARAM_NB_OF_GENERATIONS, rclcpp::ParameterValue(static_cast<int64_t>(std::lround(value)))); };

    auto updatePopSize = [this](double value) -> void
    { setPlannerParameter(PLANNER_PARAM_POPULATION_SIZE, rclcpp::ParameterValue(static_cast<int64_t>(std::lround(value)))); };

    auto updateNurbsSampleSize = [this](double value) -> void
    { setPlannerParameter(PLANNER_PARAM_NURBS_SAMPLE_SIZE, rclcpp::ParameterValue(static_cast<int64_t>(std::lround(value)))); };

    // The distance between the RRT nodes is a double
    auto updateRRTRange = [this](double value) -> void
    { setPlannerParameter(PLANNER_PARAM_RRT_RANGE, rclcpp::ParameterValue(value)); };

    if (hyperparameter_name != "nb_of_generations" && hyperparameter_name != "population_size" &&
        hyperparameter_name != "nurbs_sample_size" && hyperparameter_name != "rrt_range")
    {
        RCLCPP_ERROR(get_logger(), "Unknown hyperparameter %s (nb_of_generations, population_size, nurbs_sample_size or rrt_range)",
                     hyperparameter_name.c_str());
        return;
    }

    // Get the number of iterations for every hyperparameter value
    int nb_of_iterations = static_cast<int>(this->get_parameter("hyperparameters_variation_tests.nb_of_iter").as_double());
    if (nb_of_iterations == 0.0)
    {
        RCLCPP_ERROR(get_logger(), "Number of iterations is 0.0");
        return;
    }

    bool first_iteration = true;
    int counter = 0;
    int max_pagmo_optimizer_node_calls = 5;
    int pagmo_calls = 0;
    int iteration_counter = 0;

    // Fixed hyperparameters, set before the varied one so the varied one wins
    auto fixedValue = [this](const std::string& name) -> double
    { return this->get_parameter("fixed_hyperparameters." + name).as_double(); };

    if (fixedValue("nb_of_generations") >= 0.0)
        updateNbOfGenerations(fixedValue("nb_of_generations"));
    if (fixedValue("population_size") >= 0.0)
        updatePopSize(fixedValue("population_size"));
    if (fixedValue("nurbs_sample_size") >= 0.0)
        updateNurbsSampleSize(fixedValue("nurbs_sample_size"));
    if (fixedValue("rrt_range") >= 0.0)
        updateRRTRange(fixedValue("rrt_range"));
    if (fixedValue("cost_time") >= 0.0)
        setPlannerParameter(PLANNER_PARAM_COST_TIME, rclcpp::ParameterValue(fixedValue("cost_time")));
    if (fixedValue("cost_safety") >= 0.0)
        setPlannerParameter(PLANNER_PARAM_COST_SAFETY, rclcpp::ParameterValue(fixedValue("cost_safety")));
    if (fixedValue("cost_energy") >= 0.0)
        setPlannerParameter(PLANNER_PARAM_COST_ENERGY, rclcpp::ParameterValue(fixedValue("cost_energy")));

    if (hyperparameter_name == "nb_of_generations")
    {
        updateNbOfGenerations(hyperparameter_current);
    }
    else if (hyperparameter_name == "population_size")
    {
        updatePopSize(hyperparameter_current);
    }
    else if (hyperparameter_name == "nurbs_sample_size")
    {
        updateNurbsSampleSize(hyperparameter_current);
    }
    else if (hyperparameter_name == "rrt_range")
    {
        updateRRTRange(hyperparameter_current);
    }

    // Main loop
    while (rclcpp::ok())
    {
        // Check if the planner is running
        if (isPlannerRunning())
        {
            if (!costmap_received_)
            {
                // Sleep
                rate.sleep();
                spinOnce();
                continue;
            }

            if (first_iteration)
            {
                requestPlanning();

                first_iteration = false;
                iteration_counter++;
            }
            else
            {
                if (path_planning_finished_counter_ > counter)
                {
                    if (iteration_counter >= nb_of_iterations)
                    {
                        // Change the hyperparameter
                        if (hyperparameter_current >= hyperparameter_max)
                        {
                            RCLCPP_INFO(get_logger(), "End of the hyperparameters variation tests");
                            spinOnce();
                            return;
                        }

                        hyperparameter_current += hyperparameter_steps;

                        // Set the hyperparameter parameter
                        if (hyperparameter_name == "nb_of_generations")
                        {
                            updateNbOfGenerations(hyperparameter_current);
                        }
                        else if (hyperparameter_name == "population_size")
                        {
                            updatePopSize(hyperparameter_current);
                        }
                        else if (hyperparameter_name == "nurbs_sample_size")
                        {
                            updateNurbsSampleSize(hyperparameter_current);
                        }
                        else if (hyperparameter_name == "rrt_range")
                        {
                            updateRRTRange(hyperparameter_current);
                        }

                        iteration_counter = 0;
                    }

                    if (pagmo_calls >= max_pagmo_optimizer_node_calls)
                    {
                        killPlanner();

                        costmap_received_ = false;
                        pagmo_calls = 0;
                        continue;
                    }

                    requestPlanning();

                    RCLCPP_INFO(get_logger(), "Hyperparameter: %s, current value: %f, max value: %f", hyperparameter_name.c_str(), hyperparameter_current, hyperparameter_max);
                    RCLCPP_INFO(get_logger(), "Iteration: %i / %i", iteration_counter, nb_of_iterations);

                    pagmo_calls++;
                    counter++;
                    iteration_counter++;
                }
            }
        }
        else
        {
            startPlanner();
        }

        // Sleep
        rate.sleep();
        spinOnce();
    }
}

void TestbenchNode::runOptimalSolutionPerObjectiveTests(rclcpp::Rate& rate)
{
    // Get the number of iterations for every sets of coeff
    int nb_of_iterations = static_cast<int>(this->get_parameter("optimal_solution_per_objective_tests.nb_of_iter").as_double());
    if (nb_of_iterations == 0.0)
    {
        RCLCPP_ERROR(get_logger(), "Number of iterations is 0.0");
        return;
    }

    bool first_iteration = true;
    int counter = 0;
    int max_pagmo_optimizer_node_calls = 105;
    int pagmo_calls = 0;

    // Main loop
    while (rclcpp::ok())
    {
        if (counter >= nb_of_iterations)
        {
            RCLCPP_INFO(get_logger(), "End of the optimal solution per objective tests");
            //publishMissionRisksPaths();

            spinOnce();
            return;
        }

        // Check if the planner is running
        if (isPlannerRunning())
        {
            if (!costmap_received_)
            {
                // Sleep
                rate.sleep();
                spinOnce();
                continue;
            }

            if (first_iteration)
            {
                requestPlanning();

                first_iteration = false;
            }
            else
            {
                if (path_planning_finished_counter_ > counter)
                {
                    pagmo_calls++;

                    if (pagmo_calls >= max_pagmo_optimizer_node_calls)
                    {
                        killPlanner();

                        pagmo_calls = 0;
                        costmap_received_ = false;
                        continue;
                    }

                    counter++;
                    if (counter >= nb_of_iterations)
                        continue;

                    RCLCPP_INFO(get_logger(), "Iteration: %i / %i", counter, nb_of_iterations);

                    requestPlanning();
                }
            }
        }
        else
        {
            startPlanner();
        }

        // Sleep
        rate.sleep();
        spinOnce();
    }
}

void TestbenchNode::runRisksVarationTests(rclcpp::Rate& rate)
{
    // Get the number of iterations for every sets of risks
    int nb_of_iterations = static_cast<int>(this->get_parameter("optimal_solution_per_objective_tests.nb_of_iter").as_double());
    if (nb_of_iterations == 0.0)
    {
        RCLCPP_ERROR(get_logger(), "Number of iterations is 0.0");
        return;
    }

    bool first_iteration = true;
    int counter = 0;
    int max_pagmo_optimizer_node_calls = 65;
    int pagmo_calls = 0;

    std::vector<std::vector<double>> risks = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0},
                                              {1.0, 0.0, 1.0}, {0.0, 1.0, 1.0}, {1.0, 1.0, 0.0}};
    std::vector<std::vector<double>> initial_risks = {{0.33, 0.33, 0.33}, {0.33, 0.33, 0.33}, {0.25, 0.5, 0.25},
                                                      {0.25, 0.5, 0.25}, {0.33, 0.33, 0.33}, {0.25, 0.5, 0.25}};
    size_t risk_id = 0;

    // Lambda function calculating coeffs
    auto calculateCoeffs = [](std::vector<double> current_risks, std::vector<double> initial_risks) -> std::vector<double>
    {
        // Risks order: battery, wind, location
        double time_coeff = initial_risks[0] * (1.0 - ((1.0/2.0*current_risks[1]) + (1.0/4.0*current_risks[2]) + (1.0/4.0*0.0) - current_risks[0]));
        double safety_coeff = initial_risks[1] * (1.0 + ((1.0/2.0*current_risks[1]) + (1.0/4.0*current_risks[2]) + (1.0/4.0*0.0) - current_risks[0]));
        double energy_coeff = initial_risks[2] * (1.0 + ((1.0/2.0*current_risks[1]) + (1.0/2.0*current_risks[0])));

        return {time_coeff, safety_coeff, energy_coeff};
    };

    std::vector<double> coeffs = calculateCoeffs(risks[risk_id], initial_risks[risk_id]);
    setPlannerParameter(PLANNER_PARAM_COST_TIME, rclcpp::ParameterValue(coeffs[0]));
    setPlannerParameter(PLANNER_PARAM_COST_SAFETY, rclcpp::ParameterValue(coeffs[1]));
    setPlannerParameter(PLANNER_PARAM_COST_ENERGY, rclcpp::ParameterValue(coeffs[2]));
    RCLCPP_INFO(get_logger(), "Coeffs: time: %f, safety: %f, energy: %f", coeffs[0], coeffs[1], coeffs[2]);

    // Main loop
    while (rclcpp::ok())
    {
        if (counter >= nb_of_iterations)
        {
            //RCLCPP_INFO(get_logger(), "End of the optimal solution per objective tests");


            risk_id++;

            if (risk_id >= risks.size())
            {
                RCLCPP_INFO(get_logger(), "End of the risks variation tests");

                publishRisksVariationPaths();
                //publishMissionRisksPaths();
                spinOnce();

                return;
            }

            // Set the coefficients parameters
            coeffs = calculateCoeffs(risks[risk_id], initial_risks[risk_id]);
            RCLCPP_INFO(get_logger(), "Coeffs: time: %f, safety: %f, energy: %f", coeffs[0], coeffs[1], coeffs[2]);
            setPlannerParameter(PLANNER_PARAM_COST_TIME, rclcpp::ParameterValue(coeffs[0]));
            setPlannerParameter(PLANNER_PARAM_COST_SAFETY, rclcpp::ParameterValue(coeffs[1]));
            setPlannerParameter(PLANNER_PARAM_COST_ENERGY, rclcpp::ParameterValue(coeffs[2]));

            // Sleep
            rclcpp::sleep_for(200ms);

            counter = 0;
            path_planning_finished_counter_ = 0;
            first_iteration = true;

            spinOnce();
        }

        // Check if the planner is running
        if (isPlannerRunning())
        {
            if (!costmap_received_)
            {
                // Sleep
                rate.sleep();
                spinOnce();
                continue;
            }

            if (first_iteration)
            {
                requestPlanning();

                first_iteration = false;
            }
            else
            {
                if (path_planning_finished_counter_ > counter)
                {
                    pagmo_calls++;

                    if (pagmo_calls >= max_pagmo_optimizer_node_calls)
                    {
                        killPlanner();

                        pagmo_calls = 0;
                        costmap_received_ = false;
                        continue;
                    }

                    counter++;
                    if (counter >= nb_of_iterations)
                        continue;

                    RCLCPP_INFO(get_logger(), "Iteration: %i / %i", counter, nb_of_iterations);

                    requestPlanning();
                }
            }
        }
        else
        {
            startPlanner();
        }

        // Sleep
        rate.sleep();
        spinOnce();
    }
}

void TestbenchNode::publishMission()
{
    arena_msgs::msg::Mission mission_msg;

    arena_msgs::msg::Coordinate source_pylons;
    arena_msgs::msg::Coordinate charge_pylons;

    source_pylons.latitude = 45.33068555300065;
    source_pylons.longitude = -72.63555643998323;
    source_pylons.altitude = 71.46475178140408;

    charge_pylons.latitude = 45.33258602056009;
    charge_pylons.longitude = -72.63382723535231;
    charge_pylons.altitude = 71.46475178140408;

    mission_msg.source_pylons.push_back(source_pylons);
    mission_msg.charge_pylons.push_back(charge_pylons);

    mission_msg.corridors_width.push_back(35.0);
    mission_msg.corridors_margin.push_back(10.0);
    mission_msg.inspection_position_ratios.push_back(0.5);

    mission_pub_->publish(mission_msg);
}

void TestbenchNode::pathPlanningFinishedCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Path planning finished: %d", msg->data);
    if (msg->data)
    {
        planning_requested_ = false;
        path_planning_finished_counter_++;
        std_msgs::msg::Int32 counter_msg;
        counter_msg.data = path_planning_finished_counter_;
        path_planning_finished_counter_pub_->publish(counter_msg);
    }
}

namespace
{

// Costs of infeasible plannings are the max double, write them in scientific notation like the ROS 1 reports
std::string formatValue(double value)
{
    std::ostringstream ss;
    if (std::fabs(value) >= 1.0e15)
        ss << std::setprecision(6) << value;
    else
        ss << std::fixed << std::setprecision(6) << value;
    return ss.str();
}

} // namespace

void TestbenchNode::nurbsInfosCallback(const arena_msgs::msg::OptimizerPathInfo::SharedPtr msg)
{
    if (planning_report_)
        writePlanningReport(*msg);
    else
        writeRisksReport(*msg);

    writeParetoFront(*msg);

    nurbs_infos_counter_++;
}

void TestbenchNode::writePlanningReport(const arena_msgs::msg::OptimizerPathInfo& msg)
{
    std::ofstream csv_file(csv_filename_, std::ios::out | std::ios::app);
    if (!csv_file.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open CSV file: %s", csv_filename_.c_str());
        return;
    }

    // Values shared by every row of this planning
    std::ostringstream planning_values;
    planning_values <<
    formatValue(msg.planning_time * 1.0e9) << "," << // In nanoseconds like the ROS 1 reports
    msg.nb_of_generations << "," <<
    msg.population_size << "," <<
    msg.nurbs_sample_size << "," <<
    formatValue(msg.chosen_time_cost) << "," <<
    formatValue(msg.chosen_security_cost) << "," <<
    formatValue(msg.chosen_energy_cost) << "," <<
    formatValue(msg.best_time_cost) << "," <<
    formatValue(msg.best_security_cost) << "," <<
    formatValue(msg.best_energy_cost) << "," <<
    formatValue(msg.chosen_time_cost) << "," <<
    formatValue(msg.chosen_security_cost) << "," <<
    formatValue(msg.chosen_energy_cost);

    std::ostringstream coefficients;
    coefficients <<
    formatValue(msg.time_coefficient) << "," <<
    formatValue(msg.security_coefficient) << "," <<
    formatValue(msg.energy_coefficient) << "," <<
    (msg.feasible ? 1 : 0) << "," <<
    msg.pareto_front_time_costs.size() << "," <<
    formatValue(msg.rrt_range) << "," <<
    formatValue(msg.initialization_time * 1.0e9) << "," << // In nanoseconds like the planning time
    formatValue(msg.optimization_time * 1.0e9) << "," <<
    msg.nb_of_control_points;

    if (msg.path.poses.empty())
    {
        // Infeasible planning, a single row without path
        csv_file << nurbs_infos_counter_ << "," << planning_values.str() << "," <<
        "0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000," << coefficients.str() << std::endl;
        return;
    }

    for (size_t i = 0; i < msg.path.poses.size(); i++)
    {
        const geometry_msgs::msg::Pose& pose = msg.path.poses[i].pose;
        double velocity = i < msg.velocities.size() ? msg.velocities[i] : 0.0;
        double closest_distance = getClosestObstacleDistance(octomap::point3d(pose.position.x, pose.position.y, pose.position.z));

        csv_file << nurbs_infos_counter_ << "," << planning_values.str() << "," <<
        formatValue(pose.position.x) << "," <<
        formatValue(pose.position.y) << "," <<
        formatValue(pose.position.z) << "," <<
        formatValue(pose.orientation.x) << "," <<
        formatValue(pose.orientation.y) << "," <<
        formatValue(pose.orientation.z) << "," <<
        formatValue(pose.orientation.w) << "," <<
        formatValue(std::fabs(velocity)) << "," <<
        formatValue(closest_distance) << "," <<
        coefficients.str() << std::endl;
    }
}

void TestbenchNode::writeParetoFront(const arena_msgs::msg::OptimizerPathInfo& msg)
{
    std::ofstream pareto_file(pareto_front_filename_, std::ios::out | std::ios::app);
    if (!pareto_file.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open CSV file: %s", pareto_front_filename_.c_str());
        return;
    }

    for (size_t i = 0; i < msg.pareto_front_time_costs.size(); i++)
    {
        pareto_file <<
        nurbs_infos_counter_ << "," <<
        msg.nb_of_generations << "," <<
        msg.population_size << "," <<
        msg.nurbs_sample_size << "," <<
        formatValue(msg.time_coefficient) << "," <<
        formatValue(msg.security_coefficient) << "," <<
        formatValue(msg.energy_coefficient) << "," <<
        formatValue(msg.pareto_front_time_costs[i]) << "," <<
        formatValue(msg.pareto_front_security_costs[i]) << "," <<
        formatValue(msg.pareto_front_energy_costs[i]) << "," <<
        formatValue(msg.rrt_range) << std::endl;
    }
}

void TestbenchNode::writeRisksReport(const arena_msgs::msg::OptimizerPathInfo& msg)
{
    if (msg.path.poses.empty())
    {
        RCLCPP_WARN(get_logger(), "Infeasible planning, it is not written in the risks report");
        return;
    }

    // Open the CSV file in append mode or create a new file
    std::ofstream csv_file;
    csv_file.open(csv_filename_, std::ios::out | std::ios::app);
    if (!csv_file.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open CSV file: %s", csv_filename_.c_str());
        return;
    }

    // Write the data to the CSV file
    const nav_msgs::msg::Path& path_msg = msg.path;

    double path_duration = 0.0;
    double avg_closest_obstacle_distance = 0.0;
    geometry_msgs::msg::Point old_point;
    old_point.x = path_msg.poses[0].pose.position.x;
    old_point.y = path_msg.poses[0].pose.position.y;
    old_point.z = path_msg.poses[0].pose.position.z;

    // Add path to risks_variation_paths_
    visualization_msgs::msg::Marker risks_variation_path_marker;
    risks_variation_path_marker.header.frame_id = "map";
    risks_variation_path_marker.header.stamp = this->now();
    risks_variation_path_marker.ns = "risks_variation_paths";
    risks_variation_path_marker.id = risks_variation_paths_.size();
    risks_variation_path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    risks_variation_path_marker.action = visualization_msgs::msg::Marker::ADD;
    risks_variation_path_marker.scale.x = 0.1;
    risks_variation_path_marker.color.a = 1.0;

    for (size_t i = 0; i < path_msg.poses.size(); i++)
    {
        /*csv_file << std::fixed << std::setprecision(6) <<
        nurbs_infos_counter_ << "," <<
        msg.planning_time << "," <<
        msg.nb_of_generations << "," <<
        msg.population_size << "," <<
        msg.nurbs_sample_size << "," <<
        msg.best_time_cost << "," <<
        msg.best_security_cost << "," <<
        msg.best_energy_cost << "," <<
        msg.chosen_time_cost << "," <<
        msg.chosen_security_cost << "," <<
        msg.chosen_energy_cost << "," <<
        path_msg.poses[i].pose.position.x << "," <<
        path_msg.poses[i].pose.position.y << "," <<
        path_msg.poses[i].pose.position.z << "," <<
        path_msg.poses[i].pose.orientation.x << "," <<
        path_msg.poses[i].pose.orientation.y << "," <<
        path_msg.poses[i].pose.orientation.z << "," <<
        path_msg.poses[i].pose.orientation.w << "," <<
        msg.velocities[i] << "," <<
        0.0 << "," <<
        msg.time_coefficient << "," <<
        msg.security_coefficient << "," <<
        msg.energy_coefficient << std::endl;*/
        geometry_msgs::msg::Point point;
        point.x = path_msg.poses[i].pose.position.x;
        point.y = path_msg.poses[i].pose.position.y;
        point.z = path_msg.poses[i].pose.position.z;

        if (risks_variation_tests_)
            risks_variation_path_marker.points.push_back(point);

        // Calculate the distance between the two points
        double distance = sqrt(pow(point.x - old_point.x, 2) + pow(point.y - old_point.y, 2) + pow(point.z - old_point.z, 2));
        double duration = distance / 2.0;
        path_duration += duration;

        // Calculate the closest obstacle distance
        octomap::point3d point3d(point.x, point.y, point.z);
        double closest_distance = getClosestObstacleDistance(point3d);
        avg_closest_obstacle_distance += closest_distance;

        old_point = point;
    }
    avg_closest_obstacle_distance /= path_msg.poses.size();

    if (risks_variation_tests_)
        risks_variation_paths_.push_back(risks_variation_path_marker);

    csv_file << std::fixed << std::setprecision(6) <<
    nurbs_infos_counter_ << "," <<
    path_duration << "," <<
    avg_closest_obstacle_distance << "," <<
    msg.chosen_energy_cost << "," <<
    0.0 << "," <<
    0.0 << "," <<
    0.0 << std::endl;

    // Close the CSV file
    csv_file.close();
}

void TestbenchNode::costmapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
    octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
    if (tree)
    {
        costmap_received_ = true;

        if (costmap_)
            delete costmap_;
        costmap_ = tree;
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Failed to convert the Octomap message to an OcTree");
    }
}

void TestbenchNode::missionRisksCallback(const arena_msgs::msg::MissionRisks::SharedPtr msg)
{
    for (size_t i = 0; i < msg->battery_risks.size(); i++)
    {
        std::vector<double> risks;
        risks.push_back(msg->battery_risks[i]);
        risks.push_back(msg->wind_risks[i]);
        risks.push_back(msg->localization_risks[i]);
        risks.push_back(msg->communication_risks[i]);
        mission_risks_.push_back(risks);
    }

    for (size_t i = 0; i < msg->nurbs_paths.size(); i++)
    {
        mission_risks_paths_.push_back(msg->nurbs_paths[i]);
        std::vector<double> costs;
        costs.push_back(msg->time_costs[i]);
        costs.push_back(msg->safety_costs[i]);
        costs.push_back(msg->energy_costs[i]);
        mission_risks_paths_costs_.push_back(costs);
    }
}

void TestbenchNode::publishMissionRisksPaths()
{
    // Open the CSV file in append mode or create a new file
    std::ofstream csv_file;
    csv_file.open(csv_filename_, std::ios::out | std::ios::app);
    if (!csv_file.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open CSV file: %s", csv_filename_.c_str());
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;

    // Define the color of every path bys ranking them by the costs
    std::vector<std::vector<double>> objective_score_ranks;
    for (size_t i = 0; i < mission_risks_paths_costs_[0].size(); i++)
    {
        std::vector<double> objective_score_rank (mission_risks_paths_costs_.size(), std::numeric_limits<double>::max());

        for (size_t j = 0; j < mission_risks_paths_costs_.size(); j++)
        {
            int rank = 0;
            for (size_t k = 0; k < mission_risks_paths_costs_.size(); k++)
            {
                if (j == k)
                    continue;

                if (mission_risks_paths_costs_[j][i] > mission_risks_paths_costs_[k][i])
                    rank++;
            }
            objective_score_rank[j] = rank;
        }

        double max_rank = *std::max_element(objective_score_rank.begin(), objective_score_rank.end());
        if (max_rank > 0)
        {
            for (size_t j = 0; j < objective_score_rank.size(); j++)
            {
                objective_score_rank[j] /= max_rank;
                // Revert the rank to have the best solution with the highest rank
                //objective_score_rank[j] = 1.0 - objective_score_rank[j];
            }
        }
        objective_score_ranks.push_back(objective_score_rank);
    }

    std::vector<std::vector<double>> final_ranks;
    for (size_t i = 0; i < mission_risks_.size(); i++)
    {
        std::vector<double> final_rank (mission_risks_paths_.size(), 0.0);
        double time_coeff = 0.33 * (1.0 - ((0.5 * mission_risks_[i][1]) + (0.25 * mission_risks_[i][2]) + 0.0 - (1.0 * mission_risks_[i][0])));
        double safety_coeff = 0.33 * (1.0 + ((0.5 * mission_risks_[i][1]) + (0.25 * mission_risks_[i][2]) + 0.0 - (1.0 * mission_risks_[i][0])));
        double energy_coeff = 0.33 * (1.0 + ((0.5 * mission_risks_[i][1]) + (0.5 * mission_risks_[i][0])));

        // Normalize the coefficients
        double sum = time_coeff + safety_coeff + energy_coeff;
        time_coeff /= sum;
        safety_coeff /= sum;
        energy_coeff /= sum;

        for (size_t j = 0; j < mission_risks_paths_.size(); j++)
            final_rank[j] = (time_coeff * objective_score_ranks[0][j]) + (safety_coeff * objective_score_ranks[1][j]) + (energy_coeff * objective_score_ranks[2][j]);

        // Rescale the final ranks between 0 and 1
        /*double max_final_rank = *std::max_element(final_rank.begin(), final_rank.end());
        double min_final_rank = *std::min_element(final_rank.begin(), final_rank.end());
        for (int j = 0; j < final_rank.size(); j++)
            final_rank[j] = (final_rank[j] - min_final_rank) / (max_final_rank - min_final_rank);*/

        final_ranks.push_back(final_rank);
    }

    for (size_t i = 0; i < mission_risks_paths_.size(); i++)
    {
        visualization_msgs::msg::Marker pose_marker;
        pose_marker.header.frame_id = "map";
        pose_marker.header.stamp = this->now();
        pose_marker.ns = "mission_risks";
        pose_marker.id = i;
        pose_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        pose_marker.action = visualization_msgs::msg::Marker::ADD;

        double path_duration = 0.0;
        double avg_closest_obstacle_distance = 0.0;
        geometry_msgs::msg::Point old_point = mission_risks_paths_[i].points[0];
        for (size_t j = 0; j < mission_risks_paths_[i].points.size(); j++)
        {
            geometry_msgs::msg::Point point = mission_risks_paths_[i].points[j];

            // Calculate the distance between the two points
            double distance = sqrt(pow(point.x - old_point.x, 2) + pow(point.y - old_point.y, 2) + pow(point.z - old_point.z, 2));
            double duration = distance / 2.0;
            path_duration += duration;

            // Calculate the closest obstacle distance
            octomap::point3d point3d(point.x, point.y, point.z);
            double closest_distance = getClosestObstacleDistance(point3d);
            avg_closest_obstacle_distance += closest_distance;

            pose_marker.points.push_back(point);
            old_point = point;
        }
        avg_closest_obstacle_distance /= double(mission_risks_paths_[i].points.size());
        double energy_cost = mission_risks_paths_costs_[i][2];

        pose_marker.scale.x = 0.1;
        pose_marker.scale.y = 0.1;
        pose_marker.scale.z = 0.1;
        pose_marker.color.a = 0.5;

        // Set color based on the risk values (red:=battery low, green:=localization/communication lost, blue:=high wind) and final_ranks
        // Battery low iter=0, high wind iter=1, localization/communication lost iter=2-3
        std::vector<double> reds, greens, blues;
        for (size_t j = 0; j < mission_risks_.size(); j++)
        {
            double red = final_ranks[j][i] * mission_risks_[j][0];
            reds.push_back(red);

            double green = final_ranks[j][i] * mission_risks_[j][2];
            greens.push_back(green);

            double blue = final_ranks[j][i] * mission_risks_[j][1];
            blues.push_back(blue);
        }
        // Bring the values between 0 and 1
        double max_red = *std::max_element(reds.begin(), reds.end());
        double min_red = *std::min_element(reds.begin(), reds.end());
        double max_green = *std::max_element(greens.begin(), greens.end());
        double min_green = *std::min_element(greens.begin(), greens.end());
        double max_blue = *std::max_element(blues.begin(), blues.end());
        double min_blue = *std::min_element(blues.begin(), blues.end());

        for (size_t j = 0; j < reds.size(); j++)
        {
            reds[j] = ((reds[j] - min_red) / (max_red - min_red)) - min_red;
            greens[j] = ((greens[j] - min_green) / (max_green - min_green)) - min_green;
            blues[j] = ((blues[j] - min_blue) / (max_blue - min_blue)) - min_blue;
        }

        double total_red = 0.0;
        double total_green = 0.0;
        double total_blue = 0.0;
        for (size_t j = 0; j < reds.size(); j++)
        {
            total_red += reds[j];
            total_green += greens[j];
            total_blue += blues[j];
        }

        // Get the average of each color
        double mean_red = total_red / reds.size();
        double mean_green = total_green / greens.size();
        double mean_blue = total_blue / blues.size();

        pose_marker.color.r = mean_red;
        pose_marker.color.g = mean_green;
        pose_marker.color.b = mean_blue;
        /*if (mean_red > mean_green && mean_red > mean_blue)
        {pose_marker.color.g -= 0.2; pose_marker.color.b -= 0.2;}
        else if (mean_green > mean_red && mean_green > mean_blue)
        {pose_marker.color.r -= 0.2; pose_marker.color.b -= 0.2;}
        else if (mean_blue > mean_red && mean_blue > mean_green)
        {pose_marker.color.r -= 0.2; pose_marker.color.g -= 0.2;}*/

        // Add the path duration and the closest obstacle distance to the CSV file
        csv_file << std::fixed << std::setprecision(6) <<
        i << "," <<
        path_duration << "," <<
        avg_closest_obstacle_distance << "," <<
        energy_cost << "," <<
        pose_marker.color.r << "," <<
        pose_marker.color.g << "," <<
        pose_marker.color.b << std::endl;

        pose_marker.color.a = 1.0;

        marker_array.markers.push_back(pose_marker);
    }

    // Close the CSV file
    csv_file.close();

    nurbs_from_risks_pub_->publish(marker_array);
}

void TestbenchNode::publishRisksVariationPaths()
{
    int nb_of_iterations = static_cast<int>(this->get_parameter("optimal_solution_per_objective_tests.nb_of_iter").as_double());
    std::vector<std::vector<double>> risks = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0},
                                              {1.0, 0.0, 1.0}, {0.0, 1.0, 1.0}, {1.0, 1.0, 0.0}};

    int risk_id = 0;
    int counter = 0;
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < risks_variation_paths_.size(); i++)
    {
        if (counter >= nb_of_iterations)
        {
            counter = 0;
            risk_id++;
        }

        visualization_msgs::msg::Marker pose_marker = risks_variation_paths_[i];
        pose_marker.id = i;
        pose_marker.color.r = risks[risk_id][0];
        pose_marker.color.g = risks[risk_id][2];
        pose_marker.color.b = risks[risk_id][1];
        pose_marker.color.a = 0.8;

        marker_array.markers.push_back(pose_marker);
        counter++;
    }

    risks_variation_paths_pub_->publish(marker_array);
}

double TestbenchNode::getClosestObstacleDistance(const octomap::point3d& point)
{
    if (!costmap_) // Check if the octree is initialized
        return 0.0;

    double resolution = costmap_->getResolution();
    double search_radius = resolution; // Start with the minimum search radius
    double old_search_radius = 0.0;
    double max_search_radius = 100.0;  // Define a maximum search radius (to avoid infinite loops)
    double min_distance = std::numeric_limits<double>::max();

    bool found = false; // Flag to indicate if an obstacle is found

    while (search_radius <= max_search_radius)
    {
        // Iterate over all nodes within the spherical region
        for (auto it = costmap_->begin_leafs_bbx(point - octomap::point3d(search_radius, search_radius, search_radius),
                             point + octomap::point3d(search_radius, search_radius, search_radius));
             it != costmap_->end_leafs_bbx();
             ++it)
        {
            double distance = point.distance(it.getCoordinate());
            if (distance > old_search_radius && costmap_->isNodeOccupied(*it))
            {
                if (distance < min_distance)
                {
                    min_distance = distance;
                    found = true;
                }
            }
        }

        if (found) // If a node is found, stop searching further
            break;

        old_search_radius = search_radius; // Save the old search radius
        search_radius += resolution; // Expand the search radius
    }

    return found ? min_distance : 0.0; // Return the minimum distance if found, otherwise 0.0
}

void TestbenchNode::run()
{
    executor_.add_node(this->get_node_base_interface());

    // Rate
    rclcpp::Rate rate(RATE);

    if (steps_variation_tests_)
    {
        runStepsVariationTests(rate);
    }
    else if (hyperparameters_variation_tests_)
    {
        runHyperparametersVariationTests(rate);
    }
    else if (optimal_solution_per_objective_tests_)
    {
        runOptimalSolutionPerObjectiveTests(rate);
    }
    else if (risks_variation_tests_)
    {
        runRisksVarationTests(rate);
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "No tests to run");
    }

    executor_.remove_node(this->get_node_base_interface());
}

TestbenchNode::TestbenchNode()
: Node("testbench_node"), path_planning_finished_counter_(0), nurbs_infos_counter_(0),
steps_variation_tests_(false), hyperparameters_variation_tests_(false),
optimal_solution_per_objective_tests_(false), risks_variation_tests_(false),
costmap_received_(false), planning_requested_(false), goal_msg_(), costmap_(nullptr)
{
    // Parameters
    this->declare_parameter<std::string>("world_name", "undefined");
    this->declare_parameter<std::string>("output_folder", "/home/dev_ws/data_output/navigation_3d");

    this->declare_parameter<bool>("is_step_variation_tests", true);
    this->declare_parameter<bool>("is_hyperparameter_variation_tests", false);
    this->declare_parameter<bool>("is_optimal_solution_per_objective_tests", false);
    this->declare_parameter<bool>("is_risks_variation_tests", false);

    this->declare_parameter<double>("planning_goal.position.x", 30.0);
    this->declare_parameter<double>("planning_goal.position.y", 20.0);
    this->declare_parameter<double>("planning_goal.position.z", 35.0);

    this->declare_parameter<double>("step_variation_tests.coeff_steps", 0.0);
    this->declare_parameter<double>("step_variation_tests.nb_of_iter", 0.0);

    this->declare_parameter<std::string>("hyperparameters_variation_tests.hyperparameter_name", "nb_of_generations");
    this->declare_parameter<double>("hyperparameters_variation_tests.hyperparameter_steps", 0.0);
    this->declare_parameter<double>("hyperparameters_variation_tests.hyperparameter_min", 0.0);
    this->declare_parameter<double>("hyperparameters_variation_tests.hyperparameter_max", 0.0);
    this->declare_parameter<double>("hyperparameters_variation_tests.nb_of_iter", 0.0);

    this->declare_parameter<double>("optimal_solution_per_objective_tests.nb_of_iter", 0.0);

    // Planner hyperparameters kept fixed during the hyperparameters variation tests, -1 keeps the planner's own value
    this->declare_parameter<double>("fixed_hyperparameters.nb_of_generations", -1.0);
    this->declare_parameter<double>("fixed_hyperparameters.population_size", -1.0);
    this->declare_parameter<double>("fixed_hyperparameters.nurbs_sample_size", -1.0);
    this->declare_parameter<double>("fixed_hyperparameters.rrt_range", -1.0);
    this->declare_parameter<double>("fixed_hyperparameters.cost_time", -1.0);
    this->declare_parameter<double>("fixed_hyperparameters.cost_safety", -1.0);
    this->declare_parameter<double>("fixed_hyperparameters.cost_energy", -1.0);

    this->declare_parameter<std::string>("planner.node_name", "/linedrone_test_node/linedrone_test_node");
    this->declare_parameter<std::string>("planner.start_command", "");
    this->declare_parameter<std::string>("planner.kill_command", "pkill -INT -f 'lib/arena_core/[l]inedrone_test_node'");
    this->declare_parameter<double>("planner.startup_timeout", 30.0);

    steps_variation_tests_ = this->get_parameter("is_step_variation_tests").as_bool();
    hyperparameters_variation_tests_ = this->get_parameter("is_hyperparameter_variation_tests").as_bool();
    optimal_solution_per_objective_tests_ = this->get_parameter("is_optimal_solution_per_objective_tests").as_bool();
    risks_variation_tests_ = this->get_parameter("is_risks_variation_tests").as_bool();

    goal_msg_.header.frame_id = "map";
    goal_msg_.point.x = this->get_parameter("planning_goal.position.x").as_double();
    goal_msg_.point.y = this->get_parameter("planning_goal.position.y").as_double();
    goal_msg_.point.z = this->get_parameter("planning_goal.position.z").as_double();

    folder_name_ = this->get_parameter("output_folder").as_string();
    planner_node_name_ = this->get_parameter("planner.node_name").as_string();
    planner_start_command_ = this->get_parameter("planner.start_command").as_string();
    planner_kill_command_ = this->get_parameter("planner.kill_command").as_string();
    planner_startup_timeout_ = std::chrono::milliseconds(static_cast<int64_t>(this->get_parameter("planner.startup_timeout").as_double() * 1000.0));

    if (planner_start_command_.empty())
        RCLCPP_WARN(get_logger(), "planner.start_command is empty, the planner must be started manually");

    // Don't use the global arguments, the launch file's __node remap would give it the same name as this node
    param_client_node_ = std::make_shared<rclcpp::Node>(std::string(this->get_name()) + "_param_client",
                                                        rclcpp::NodeOptions().use_global_arguments(false));

    // Publishers (latched)
    rclcpp::QoS latched_qos = rclcpp::QoS(1).transient_local();
    path_planning_finished_counter_pub_ = this->create_publisher<std_msgs::msg::Int32>("/testbench/path_planning_finished_counter", latched_qos);
    planning_activated_pub_ = this->create_publisher<std_msgs::msg::Bool>("/navigation/planning_activated", latched_qos);
    planning_goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/navigation/goal", latched_qos);
    mission_pub_ = this->create_publisher<arena_msgs::msg::Mission>("/interface/mission", latched_qos);
    nurbs_from_risks_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/navigation/nurbs_from_risks", latched_qos);
    risks_variation_paths_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/navigation/risks_variation_paths", latched_qos);

    // Subscribers
    path_planning_finished_sub_ = this->create_subscription<std_msgs::msg::Bool>("/navigation/path_planning_finished", 1,
                                    std::bind(&TestbenchNode::pathPlanningFinishedCallback, this, std::placeholders::_1));
    nurbs_infos_sub_ = this->create_subscription<arena_msgs::msg::OptimizerPathInfo>("/navigation/nurbs_infos", 1,
                                    std::bind(&TestbenchNode::nurbsInfosCallback, this, std::placeholders::_1));
    costmap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>("/navigation/inflated_octomap", 1,
                                    std::bind(&TestbenchNode::costmapCallback, this, std::placeholders::_1));
    mission_risks_sub_ = this->create_subscription<arena_msgs::msg::MissionRisks>("/navigation/mission_risks_infos", 1,
                                    std::bind(&TestbenchNode::missionRisksCallback, this, std::placeholders::_1));

    std_msgs::msg::Int32 counter_msg;
    counter_msg.data = path_planning_finished_counter_;
    path_planning_finished_counter_pub_->publish(counter_msg);

    planning_report_ = steps_variation_tests_ || hyperparameters_variation_tests_;
    initCSVFile();
}

}; // namespace linedrone

}; // namespace arena_testbench


int main(int argc, char** argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);

    // Create TestbenchNode
    using namespace arena_testbench::linedrone;
    auto testbench_node = std::make_shared<TestbenchNode>();

    // Run node
    testbench_node->run();

    rclcpp::shutdown();
    return 0;
}
