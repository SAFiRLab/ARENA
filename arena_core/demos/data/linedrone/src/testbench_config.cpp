#include "linedrone/testbench_config.hpp"

#define YAML_CPP_STATIC_DEFINE
#include <yaml-cpp/yaml.h>


namespace arena_testbench
{

namespace linedrone
{

void TestbenchConfig::parseConfig(std::string config_file_path)
{
    YAML::Node config = YAML::LoadFile(config_file_path);

    if (config["drone"]["position"]["x"])
        x = config["drone"]["position"]["x"].as<float>();

    if (config["drone"]["position"]["y"])
        y = config["drone"]["position"]["y"].as<float>();

    if (config["drone"]["position"]["z"])
        z = config["drone"]["position"]["z"].as<float>();

    if (config["drone"]["orientation"]["x"])
        o_x = config["drone"]["orientation"]["x"].as<float>();

    if (config["drone"]["orientation"]["y"])
        o_y = config["drone"]["orientation"]["y"].as<float>();

    if (config["drone"]["orientation"]["z"])
        o_z = config["drone"]["orientation"]["z"].as<float>();

    if (config["drone"]["orientation"]["w"])
        o_w = config["drone"]["orientation"]["w"].as<float>();
}

}; // namespace linedrone

}; // namespace arena_testbench
