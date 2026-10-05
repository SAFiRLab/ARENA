#pragma once

#include <string>


namespace arena_testbench
{
namespace linedrone
{

/**
 * \brief Pipeline config container
 */
struct TestbenchConfig
{
    // Drone position configs
    float x;
    float y;
    float z;

    // Drone orientation configs
    float o_x;
    float o_y;
    float o_z;
    float o_w;

    void parseConfig(std::string config_file_path);
}; // struct TestbenchConfig

}; // namespace linedrone
}; // namespace arena_testbench
