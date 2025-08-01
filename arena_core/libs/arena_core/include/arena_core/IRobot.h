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

    IRobot() : armed_(false), position_(Eigen::Vector3f(0., 0., 0.)), heading_(0.)
    {};

    // User-defined methods
    // Setters
    void arm() { armed_ = true; };
    void disarm() { armed_ = false; };
    void setPosition(Eigen::Vector3f pos) { position_ = pos; };
    void setheading(double heading) { heading_ = heading; };

    // Getters
    bool isArmed() { return armed_; };
    /*std::string getRobotNamespace() { return robot_namespace_; };
    std::string getRobotFrameId() { return robot_frame_id_; };*/
    Eigen::Vector3f getPosition() { return position_; };
    double getHeading() { return heading_; };


private:

    // User-defined methods

    // User-defined attributes
    Eigen::Vector3f position_;
    double heading_;
    bool armed_;

    /*std::string robot_namespace_;
    std::string robot_frame_id_;*/
}; // class IRobot

}; // namespace arena_core

#endif // __ARENA_CORE_IROBOT_H__
