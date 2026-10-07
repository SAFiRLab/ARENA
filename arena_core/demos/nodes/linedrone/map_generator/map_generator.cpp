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

// Local
#include "linedrone/map_generator/map_generator.hpp"

// System
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>


#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_RESET   "\x1b[0m"

#define DEFAULT_CONFIG_FILE "/home/dev_ws/src/arena_core/demos/config/linedrone/map_generator/map_generator_params.yaml"


namespace
{

void printUsage(const char* program)
{
    std::cout
        << "Generates random cluttered 3D maps and saves them as octomaps (.bt).\n\n"
        << "Usage: " << program << " [options]\n\n"
        << "Options:\n"
        << "  -c, --config <file>      Generator config (default: " << DEFAULT_CONFIG_FILE << ")\n"
        << "  -s, --seed <seed>        Seed of the map, overrides map_generator.seed\n"
        << "  -n, --count <count>      Number of maps to generate, with the seeds seed, seed + 1, ... (default: 1)\n"
        << "      --name <name>        Name of the map, overrides map_generator.name\n"
        << "  -o, --output-dir <dir>   Output directory, overrides map_generator.output_directory\n"
        << "      --no-testbench       Don't write the testbench config, even if map_generator.testbench.enabled is true\n"
        << "  -h, --help               Show this message\n\n"
        << "Every map is saved as <output_dir>/seed_<seed>/<name>_seed<seed>.bt, with its metadata (config, statistics\n"
        << "and obstacles) in <output_dir>/seed_<seed>/<name>_seed<seed>.yaml. The same config and seed always give the\n"
        << "same map.\n";
}

} // namespace


int main(int argc, char** argv)
{
    using arena_demos::map_generator::MapGenerator;
    using arena_demos::map_generator::MapGeneratorConfig;

    std::string config_file = DEFAULT_CONFIG_FILE;
    std::string seed_override;
    std::string name_override;
    std::string output_dir_override;
    int count = 1;
    bool no_testbench = false;

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        auto nextValue = [&]() -> std::string
        {
            if (i + 1 >= argc)
            {
                std::cerr << ANSI_COLOR_RED << "Missing value for " << arg << ANSI_COLOR_RESET << std::endl;
                std::exit(EXIT_FAILURE);
            }
            return argv[++i];
        };

        if (arg == "-h" || arg == "--help")
        {
            printUsage(argv[0]);
            return EXIT_SUCCESS;
        }
        else if (arg == "-c" || arg == "--config")
            config_file = nextValue();
        else if (arg == "-s" || arg == "--seed")
            seed_override = nextValue();
        else if (arg == "-n" || arg == "--count")
            count = std::stoi(nextValue());
        else if (arg == "--name")
            name_override = nextValue();
        else if (arg == "-o" || arg == "--output-dir")
            output_dir_override = nextValue();
        else if (arg == "--no-testbench")
            no_testbench = true;
        else
        {
            std::cerr << ANSI_COLOR_RED << "Unknown argument: " << arg << ANSI_COLOR_RESET << "\n\n";
            printUsage(argv[0]);
            return EXIT_FAILURE;
        }
    }

    if (count < 1)
    {
        std::cerr << ANSI_COLOR_RED << "--count must be at least 1" << ANSI_COLOR_RESET << std::endl;
        return EXIT_FAILURE;
    }

    try
    {
        MapGeneratorConfig config = MapGeneratorConfig::fromYamlFile(config_file);
        if (!seed_override.empty())
            config.seed = std::stoull(seed_override);
        if (!name_override.empty())
            config.name = name_override;
        if (!output_dir_override.empty())
            config.output_directory = output_dir_override;
        if (no_testbench)
            config.testbench_enabled = false;

        std::cout << "Config: " << config_file << std::endl;
        const uint64_t first_seed = config.seed;

        for (int i = 0; i < count; ++i)
        {
            config.seed = first_seed + static_cast<uint64_t>(i);

            MapGenerator generator(config);
            generator.generate();

            // One folder per seed, for the files of the map (scripts, data...) that depend on it
            const std::filesystem::path output_dir = std::filesystem::path(config.output_directory) / ("seed_" + std::to_string(config.seed));
            const std::string bt_file = (output_dir / (config.getMapName() + ".bt")).string();
            const std::string metadata_file = (output_dir / (config.getMapName() + ".yaml")).string();

            if (!generator.saveBinary(bt_file))
            {
                std::cerr << ANSI_COLOR_RED << "Failed to write " << bt_file << ANSI_COLOR_RESET << std::endl;
                return EXIT_FAILURE;
            }
            generator.saveMetadata(metadata_file, bt_file);

            std::cout << "\n" << generator.getSummary() << "\n";
            std::cout << ANSI_COLOR_GREEN << "  Saved:         " << bt_file << ANSI_COLOR_RESET << "\n";
            std::cout << "  Metadata:      " << metadata_file << "\n";

            if (config.testbench_enabled)
            {
                const std::string testbench_file =
                    (std::filesystem::path(config.testbench_config_directory) / (config.getMapName() + ".yaml")).string();
                generator.saveTestbenchConfig(testbench_file);
                std::cout << "  Testbench:     " << testbench_file << "\n";
            }

            if (generator.getStats().nb_of_failed_obstacles > 0)
                std::cout << ANSI_COLOR_YELLOW << "  Warning: the map has fewer obstacles than requested" << ANSI_COLOR_RESET << "\n";
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << ANSI_COLOR_RED << e.what() << ANSI_COLOR_RESET << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
