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

#ifndef PAGMO_PROBLEMS_LINEDRONE_HPP
#define PAGMO_PROBLEMS_LINEDRONE_HPP

// System
#include <string>
#include <utility>
#include <functional>

// External libraries
// Pagmo2
#include <pagmo/detail/visibility.hpp>
#include <pagmo/population.hpp>
#include <pagmo/problem.hpp>
#include <pagmo/s11n.hpp>
#include <pagmo/types.hpp>


namespace pagmo
{

class PAGMO_DLL_PUBLIC linedrone_problem
{
public:

    typedef std::function<vector_double(const vector_double &)> fitness_eval_callback;
    typedef std::function<void(const population &)> show_population_callback;

    linedrone_problem(vector_double::size_type dim = 5u, vector_double::size_type fdim = 3u, 
                   fitness_eval_callback* fitness_eval = nullptr, double* x_bounds = nullptr, 
                   double* y_bounds = nullptr, double* z_bounds = nullptr, double drone_speed = 1.0);

    ~linedrone_problem();

    // Fitness computation
    vector_double fitness(const vector_double &) const;
    vector_double fitness_non_const(vector_double &) const;
    /// Number of objectives
    /**
     *
     * It returns the number of objectives.
     *
     * @return the number of objectives
     */
    vector_double::size_type get_nobj() const
    {
        return m_fdim;
    }
    // Box-bounds
    std::pair<vector_double, vector_double> get_bounds() const;

    vector_double::size_type get_nec() const
    {
        return 0u;
    }

    // Problem name
    std::string get_name() const;

    // Extra information
    std::string get_extra_info() const;

private:

    // Object serialization
    friend class boost::serialization::access;
    template <typename Archive>
    void serialize(Archive &, unsigned);

    // dimension parameter
    vector_double::size_type m_dim;
    // number of objectives
    vector_double::size_type m_fdim;
    // fitness evaluation callback
    fitness_eval_callback* m_fitness_eval;

    // bounds
    double* m_x_bounds;
    double* m_y_bounds;
    double* m_z_bounds;
    double m_drone_speed;

}; // class linedrone_problem

}; // namespace pagmo

PAGMO_S11N_PROBLEM_EXPORT_KEY(pagmo::linedrone_problem);

#endif // PAGMO_PROBLEMS_LINEDRONE_HPP
