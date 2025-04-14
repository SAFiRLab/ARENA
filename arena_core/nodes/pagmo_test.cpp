#include <iostream>
#include <functional>

#include <pagmo/algorithm.hpp>
#include <pagmo/algorithms/sade.hpp>
#include <pagmo/archipelago.hpp>
#include <pagmo/problem.hpp>
#include <pagmo/problems/schwefel.hpp>
#include <pagmo/algorithms/nsga2.hpp>

// LineDrone_navigation
#include "arena_core/math/optimization/problems/pagmo_problems/nadile_problem.hpp"

int total_fitness_evals = 0;

pagmo::vector_double nadile_fitness(const pagmo::vector_double &dv)
{
    std::cout << "nadile_fitness nb: " << total_fitness_evals << std::endl;
    total_fitness_evals++;
    pagmo::vector_double ret(4u, 0.0);
    for (size_t i = 0; i < dv.size(); ++i) {
        ret[0] += 0;
        ret[1] += dv[i];
        ret[2] += dv[i] * dv[i];
        ret[3] += dv[i] * dv[i] * dv[i];
        ret[4] += dv[i] * dv[i] * dv[i] * dv[i];
    }
    return ret;
}

int main()
{

    // 1 - Instantiate a pagmo problem constructing it from a UDP

    // (i.e., a user-defined problem, in this case the 30-dimensional

    // generalised Schwefel test function).

    std::function<pagmo::vector_double(const pagmo::vector_double &)> fitness_eval = &nadile_fitness;

    double* x_bounds = new double[2];
    x_bounds[0] = -10.0;
    x_bounds[1] = 10.0;

    double* y_bounds = new double[2];
    y_bounds[0] = -10.0;
    y_bounds[1] = 10.0;

    double* z_bounds = new double[2];
    z_bounds[0] = -10.0;
    z_bounds[1] = 10.0;

    pagmo::problem prob_nadile{pagmo::nadile_problem(22u, 5u, &fitness_eval, x_bounds, y_bounds, z_bounds)};
    prob_nadile.get_extra_info();

    // 2 - Instantiate a pagmo algorithm (NSGA-II).
    pagmo::algorithm nsga2{pagmo::nsga2(25u, 0.95, 10.0, 0.01, 50.0, pagmo::random_device::next(), nullptr, 0)};

    pagmo::population pop_nadile{prob_nadile, 100u};

    pop_nadile = nsga2.evolve(pop_nadile);

    std::vector<pagmo::vector_double> fitness = pop_nadile.get_f();
    for (size_t i = 0; i < fitness.size(); ++i) {
        std::cout << "Individual " << i << " fitness: " << std::endl;
        for (size_t j = 0; j < fitness[i].size(); ++j) {
            std::cout << "Objective " << j << ": ";
            std::cout << fitness[i][j] << " ";
        }
        std::cout << std::endl;
    }

    /*pagmo::problem prob{pagmo::schwefel(30)};


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

    }*/

}
