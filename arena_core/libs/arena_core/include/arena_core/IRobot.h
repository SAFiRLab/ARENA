#ifndef __ARENA_CORE_IROBOT_H__
#define __ARENA_CORE_IROBOT_H__

// System
#include <string>
#include <Eigen/Dense>


namespace arena_core
{

class IRobot
{
public:

    //IRobot(std::string robot_namespace, std::string robot_frame_id);

    IRobot() : armed_(false), drone_position_(Eigen::Vector3f(0., 0., 0.)), yaw_(0.)
    {};

    // User-defined methods
    // Setters
    void arm() { armed_ = true; };
    void disarm() { armed_ = false; };
    void setPosition(Eigen::Vector3f pos) { position_ = pos; };
    void setYaw(double yaw) { yaw_ = yaw; };

    // Getters
    bool isArmed() { return armed_; };
    /*std::string getRobotNamespace() { return robot_namespace_; };
    std::string getRobotFrameId() { return robot_frame_id_; };*/
    Eigen::Vector3f getPosition() { return position_; };
    double getYaw() { return yaw_; };


private:

    // User-defined methods

    // User-defined attributes
    Eigen::Vector3f position_;
    double yaw_;
    bool armed_;

    /*std::string robot_namespace_;
    std::string robot_frame_id_;*/
}; // class IRobot

}; // namespace arena_core

#endif // __ARENA_CORE_IROBOT_H__
