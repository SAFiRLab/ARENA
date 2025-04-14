#include "arena_core/math/nurbs.h"
#include "arena_core/math/linalg.h"

#include <iostream>
#include <algorithm>


namespace arena_core
{

Nurbs::Nurbs(const std::vector<Point3D<double>>& control_points, int sample_size,
std::vector<double> weights, int degree, int dimension)
: control_points_(control_points), sample_size_(sample_size), weights_(weights),
degree_(degree), dimension_(dimension), adequate_conf_(false)
{
    initialize();
}


bool Nurbs::initialize()
{
    if (degree_ >= control_points_.size()) {
        std::cout << "You've got an inadequate configuration of curve degree VS number of control points on your hands"
                  << std::endl;
        adequate_conf_ = false;
        return adequate_conf_;
    }
    adequate_conf_ = true;

    if (control_points_.size() != weights_.size())
    {
        std::cout << "The amount of weights doesn't correspond the amount of control points" << std::endl;
        weights_.clear();
        for (size_t i = 0; i < control_points_.size(); ++i)
            weights_.push_back(1.0);
    }

    knot_vector_ = generateKnotVector();
    double start = knot_vector_[degree_];
    double stop = knot_vector_[knot_vector_.size() - (degree_ + 1)];
    parameters_space_ = linalg::linspace<double>(start, stop, sample_size_);
    spans_ = findSpans();
    return adequate_conf_;
}


bool Nurbs::setDegree(int degree)
{
    if (1 <= degree <= 5)
    {
        degree_ = degree;
        return initialize();
    }
    else
    {
        std::cout << "Curve order needs to be in the interval [1, 5]" << std::endl;
        return false;
    }
}


bool Nurbs::setSampleSize(int sample_size)
{
    if (0 < sample_size)
    {
        sample_size_ = sample_size;
        return initialize();
    }
    else
    {
        std::cout << "Sample size needs to be a positive non-zero number";
    }
    return false;
}


bool Nurbs::setControlPoints(const std::vector<Point3D<double>>& control_points)
{
    if (!control_points.empty())
    {
        control_points_ = control_points;
        return initialize();
    }
    else
    {
        std::cout << "Set of control points can't be empty";
    }
    return false;
}


bool Nurbs::setControlPoint(const Point3D<double>& control_point, int index)
{
    if (0 <= index < control_points_.size())
    {
        control_points_[index] = control_point;
        return initialize();
    }
    else
    {
        std::cout << "The index: " << index << " needs to be above 0 and below " << control_points_.size() << std::endl;
    }

    return false;
}

bool Nurbs::setWeightedControlPoints(const std::vector<Point3D<double>>& control_points, 
const std::vector<double>& weights)
{
    if (!control_points.empty() && !weights.empty())
    {
        control_points_ = control_points;
        weights_ = weights;
        return initialize();
    }
    else
    {
        std::cout << "Sets of control points or weights can't be empty";
    }
    return false;
}


std::vector<double> Nurbs::generateKnotVector() const
{
    int num_control_points = control_points_.size();
    int num_segments = num_control_points - degree_ + 1;
    std::vector<double> knot_vector(num_control_points + degree_ + 1, 0.0);

    // Set up the knots at the start and end to create a clamped curve
    for (int i = 0; i <= degree_; i++) {
        knot_vector[i] = 0.0;
        knot_vector[knot_vector.size() - (i + 1)] = 1.0;
    }

    // Generate the interior knots
    for (int i = 1; i < num_segments - 1; i++) {
        knot_vector[i + degree_] = static_cast<double>(i) / (num_segments - 1);
    }

    return knot_vector;
}


std::vector<int> Nurbs::findSpans() const
{
    std::vector<int> spans;
    for (double parameter : parameters_space_) {
        spans.push_back(findSpan(parameter));
    }
    return spans;
}


int Nurbs::findSpan(double parameter) const
{
    int span = degree_ + 1;
    while (span < control_points_.size() && knot_vector_[span] <= parameter) {
        span++;
    }
    return span - 1;
}


std::vector<std::vector<double>> Nurbs::basisFunctions() const
{
    std::vector<std::vector<double>> basis;
    for (size_t i = 0; i < spans_.size(); i++) {
        basis.push_back(basisFunction(spans_[i], parameters_space_[i]));
    }
    return basis;
}


std::vector<double> Nurbs::basisFunction(int span, double parameter) const
{
    std::vector<double> left(degree_ + 1, 0.0);
    std::vector<double> right(degree_ + 1, 0.0);
    std::vector<double> N(degree_ + 1, 1.0);

    for (int j = 1; j <= degree_; j++) {
        left[j] = parameter - knot_vector_[span + 1 - j];
        right[j] = knot_vector_[span + j] - parameter;
        double saved = 0.0;
        for (int r = 0; r < j; r++) {
            double temp = N[r] / (right[r + 1] + left[j - r]);
            N[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        N[j] = saved;
    }
    return N;
}


std::vector<Point3D<double>> Nurbs::evaluate() const
{
    std::vector<std::vector<double>> crvpt_w;
    std::vector<Point3D<double>> eval_points;

    std::vector<std::vector<double>> weighted_control_points;
    for (int i = 0; i < control_points_.size(); ++i)
    {
        std::vector<double> tmp_pt(dimension_ + 1, 0.0);
        for (int j = 0; j < dimension_; ++j)
            tmp_pt[j] = control_points_[i].getCoordinate(j) * double(weights_[i]);

        tmp_pt[dimension_] = weights_[i];
        weighted_control_points.push_back(tmp_pt);
    }

    if (adequate_conf_)
    {
        auto basis = basisFunctions();
        for (size_t idx = 0; idx < parameters_space_.size(); idx++)
        {
            std::vector<double> crvpt(dimension_ + 1, 0.0);
            for (int i = 0; i <= degree_; i++)
            {
                int index = spans_[idx] - degree_ + i;
                for (size_t j = 0; j < crvpt.size(); j++)
                    crvpt[j] += basis[idx][i] * weighted_control_points[index][j];
            }
            crvpt_w.push_back(crvpt);
        }

        for (const std::vector<double>& pt : crvpt_w)
        {
            Point3D<double> cpt{};
            for (int i = 0; i < dimension_; i++)
                cpt.setCoordinate(i, pt[i] / pt[dimension_]);
            eval_points.push_back(cpt);
        }
    }
    else
    {
        std::cout << "You've got an inadequate configuration of curve degree VS number of control points on your hands"
                  << std::endl;
    }
    return eval_points;
}


}; // namespace arena_core
