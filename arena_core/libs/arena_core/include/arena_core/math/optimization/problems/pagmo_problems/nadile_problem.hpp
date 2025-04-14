#ifndef PAGMO_PROBLEMS_NADILE_HPP
#define PAGMO_PROBLEMS_NADILE_HPP

#include <string>
#include <utility>
#include <functional>

#include <pagmo/detail/visibility.hpp>
#include <pagmo/population.hpp>
#include <pagmo/problem.hpp>
#include <pagmo/s11n.hpp>
#include <pagmo/types.hpp>


namespace pagmo
{

class PAGMO_DLL_PUBLIC nadile_problem
{
public:

    typedef std::function<vector_double(const vector_double &)> fitness_eval_callback;
    typedef std::function<void(const population &)> show_population_callback;

    nadile_problem(vector_double::size_type dim = 5u, vector_double::size_type fdim = 3u, 
                   fitness_eval_callback* fitness_eval = nullptr, double* x_bounds = nullptr, 
                   double* y_bounds = nullptr, double* z_bounds = nullptr, double drone_speed = 1.0);

    ~nadile_problem();

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

}; // class nadile_problem

}; // namespace pagmo

PAGMO_S11N_PROBLEM_EXPORT_KEY(pagmo::nadile_problem);

#endif // PAGMO_PROBLEMS_NADILE_HPP
