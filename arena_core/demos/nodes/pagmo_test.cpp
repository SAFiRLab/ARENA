#include <iostream>
#include <functional>

#include <pagmo/algorithm.hpp>
#include <pagmo/algorithms/sade.hpp>
#include <pagmo/archipelago.hpp>
#include <pagmo/problem.hpp>
#include <pagmo/problems/schwefel.hpp>
#include <pagmo/algorithms/nsga2.hpp>

// LineDrone_navigation
#include "linedrone/linedrone_problem.hpp"

int total_fitness_evals = 0;

int main()
{
    pagmo::problem prob{pagmo::schwefel(30)};


    // 2 - Instantiate a pagmo algorithm (self-adaptive differential

    // evolution, 100 generations).

    pagmo::algorithm algo{pagmo::sade(100)};


    // 3 - Instantiate an archipelago with 16 islands having each 20 individuals.

    pagmo::archipelago archi{16u, algo, prob, 20u};


    // 4 - Run the evolution in parallel on the 16 separate islands 10 times.

    archi.evolve(10);


    // 5 - Wait for the evolutions to finish.

    archi.wait_check();


    // 6 - Print the fitness of the best solution in each island.

    for (const auto &isl : archi) {

        std::cout << isl.get_population().champion_f()[0] << '\n';

    }

}
