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

#include "linedrone/linedrone_problem.hpp"

// System
#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

// External libraries
// Pagmo2
#include <pagmo/detail/constants.hpp>
#include <pagmo/exceptions.hpp>
#include <pagmo/population.hpp>
#include <pagmo/problem.hpp>
#include <pagmo/s11n.hpp>
#include <pagmo/types.hpp>


// MINGW-specific warnings.
#if defined(__GNUC__) && defined(__MINGW32__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsuggest-attribute=pure"
#endif


namespace pagmo
{

    linedrone_problem::linedrone_problem(vector_double::size_type dim, vector_double::size_type fdim, 
                   fitness_eval_callback* fitness_eval, double* x_bounds, 
                   double* y_bounds, double* z_bounds, double drone_speed)
    : m_dim(dim), m_fdim(fdim), m_fitness_eval(fitness_eval), m_x_bounds(x_bounds), 
      m_y_bounds(y_bounds), m_z_bounds(z_bounds), m_drone_speed(drone_speed)
{
    if (fdim < 2u) {
        pagmo_throw(std::invalid_argument,
                    "Linedrone problem have a minimum of 2 objectives: fdim=" + std::to_string(fdim) + " was detected");
    }
    // We conservatively limit these dimensions to avoid checking overflows later
    if (fdim > std::numeric_limits<decltype(fdim)>::max() / 3u) {
        pagmo_throw(std::invalid_argument, "The number of objectives is too large");
    }
    if (dim > std::numeric_limits<decltype(dim)>::max() / 3u) {
        pagmo_throw(std::invalid_argument, "The problem dimension is too large");
    }
    if (dim <= fdim) {
        pagmo_throw(std::invalid_argument, "The problem dimension has to be larger than the number of objectives.");
    }
    if (m_fitness_eval == nullptr) {
        pagmo_throw(std::invalid_argument, "The fitness evaluation callback cannot be null.");
    }
    if (m_x_bounds == nullptr || m_y_bounds == nullptr || m_z_bounds == nullptr) {
        pagmo_throw(std::invalid_argument, "The bounds cannot be null.");
    }
}

linedrone_problem::~linedrone_problem()
{}

/// Fitness computation
/**
 * Computes the fitness for this UDP
 *
 * @param x the decision vector.
 *
 * @return the fitness of \p x.
 */
vector_double linedrone_problem::fitness(const vector_double &x) const
{
    return m_fitness_eval->operator()(x);
}

vector_double linedrone_problem::fitness_non_const(vector_double &x) const
{
    return vector_double(4u, std::numeric_limits<double>::max());
}

/**
 *
 * It returns the bounds for this UDP, bounds are set according to the type of variable
 *
 * @return the lower and upper bounds for each of the decision vector components
 */
std::pair<vector_double, vector_double> linedrone_problem::get_bounds() const
{
    vector_double lb(m_dim);
    vector_double ub(m_dim);

    lb[0] = 0.0;
    ub[0] = 10.0;

    for (vector_double::size_type i = 1; i < m_dim - 1; i+=5) {
        lb[i] = m_x_bounds[0];
        ub[i] = m_x_bounds[1];
        lb[i+1] = m_y_bounds[0];
        ub[i+1] = m_y_bounds[1];
        lb[i+2] = m_z_bounds[0];
        ub[i+2] = m_z_bounds[1];
        lb[i+3] = 0.0;
        ub[i+3] = m_drone_speed;
        lb[i+4] = 0.0;
        ub[i+4] = 10.0;
    }

    lb[m_dim - 1] = 0.0;
    ub[m_dim - 1] = 10.0;

    return {lb, ub};
}

/// Problem name
/**
 * @return a string containing the problem name
 */
std::string linedrone_problem::get_name() const
{
    return "LINEDRONE";
}

// Object serialization
template <typename Archive>
void linedrone_problem::serialize(Archive &ar, unsigned)
{
    detail::archive(ar, m_dim, m_fdim);
}

// Extra information
std::string linedrone_problem::get_extra_info() const
{
    std::ostringstream ss;
    ss << "Dimension: " << m_dim << "\nNumber of objectives: " << m_fdim << "\n";
    std::pair<vector_double, vector_double> bounds = get_bounds();
    ss << "\nBounds:\n";
    for (vector_double::size_type i = 0; i < m_dim; i++) {
        ss << "x[" << i << "]: [" << bounds.first[i] << ", " << bounds.second[i] << "]\n";
    }
    
    return ss.str();
}

}; // namespace pagmo
