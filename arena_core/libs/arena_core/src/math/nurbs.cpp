#include "arena_core/math/nurbs.h"
#include "arena_core/math/utils/linalg.h"

#include <iostream>
#include <algorithm>


namespace arena_core
{

Nurbs::Nurbs(const std::vector<ControlPoint<double>>& control_points, int sample_size, int degree)
: control_points_(control_points), sample_size_(sample_size), degree_(degree), adequate_conf_(false)
{
    initialize();
}

bool Nurbs::initialize()
{
    if (control_points_.empty())
    {
        std::cout << "Set of control points can't be empty" << std::endl;
        return false;
    }

    // Check if the control points are in the same dimension
    size_t dimension = control_points_[0].size();
    for (const auto& cp : control_points_)
    {
        if (cp.size() != dimension)
        {
            std::cout << "Control points must be in the same dimension" << std::endl;
            return false;
        }
    }

    if (degree_ >= control_points_.size()) {
        std::cout << "You've got an inadequate configuration of curve degree VS number of control points on your hands"
                  << std::endl;
        adequate_conf_ = false;
        return adequate_conf_;
    }
    adequate_conf_ = true;

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
        std::cout << "Sample size needs to be a positive non-zero number";
    
    return false;
}

bool Nurbs::setControlPoints(const std::vector<ControlPoint<double>>& control_points)
{    
    control_points_ = control_points;
    return initialize();
}

bool Nurbs::setControlPoint(const ControlPoint<double>& control_point, int index)
{
    if (0 <= index < control_points_.size())
    {
        control_points_[index] = control_point;
        return initialize();
    }
    else
        std::cout << "The index: " << index << " needs to be above 0 and below " << control_points_.size() << std::endl;

    return false;
}

bool Nurbs::setWeight(double weight, unsigned int index)
{
    if (0 <= index < control_points_.size())
    {
        if (weight > 0)
        {
            control_points_[index].setW(weight);
            return initialize();
        }
        else
            std::cout << "Weight needs to be a positive non-zero number" << std::endl;
    }
    else
        std::cout << "The index: " << index << " needs to be above 0 and below " << control_points_.size() << std::endl;
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
    for (double parameter : parameters_space_)
        spans.push_back(findSpan(parameter));

    return spans;
}

int Nurbs::findSpan(double parameter) const
{
    int span = degree_ + 1;

    while (span < control_points_.size() && knot_vector_[span] <= parameter)
        span++;
    
    return span - 1;
}

void Nurbs::basisFunction(int span, double parameter, double* N) const
{
    double left[degree_ + 1] = {0.0};
    double right[degree_ + 1] = {0.0};

    N[0] = 1.0;

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
}

void Nurbs::basisFunctions(double** basis) const
{
    for (size_t i = 0; i < spans_.size(); i++)
    {
        basis[i] = new double[degree_ + 1];
        basisFunction(spans_[i], parameters_space_[i], basis[i]);
    }
}

std::vector<std::vector<double>> Nurbs::basisFunctionDerivatives(int order, int span, double parameter) const
{
    std::vector<std::vector<double>> ders(std::min(degree_, order) + 1, std::vector<double>(degree_ + 1, 0.0));
    std::vector<double> left(degree_ + 1, 1.0);
    std::vector<double> right(degree_ + 1, 1.0);
    std::vector<std::vector<double>> ndu(degree_ + 1, std::vector<double>(degree_ + 1, 1.0));

    for (int j = 1; j < degree_ + 1; j++)
    {
        left[j] = parameter - knot_vector_[span + 1 - j];
        right[j] = knot_vector_[span + j] - parameter;
        double saved = 0.0;
        for (int r = 0; r < j; r++)
        {
            ndu[j][r] = right[r + 1] + left[j - r];
            double temp = ndu[r][j - 1] / ndu[j][r];
            ndu[r][j] = saved + (right[r + 1] * temp);
            saved = left[j - r] * temp;
        }
        ndu[j][j] = saved;
    }

    for (int j = 0; j < degree_ + 1; j++)
        ders[0][j] = ndu[j][degree_];

    std::vector<std::vector<double>> a(2, std::vector<double>(degree_ + 1, 1.0));
    for (int r = 0; r < degree_ + 1; r++)
    {
        int s1 = 0;
        int s2 = 1;
        int j1 = 0;
        int j2 = 0;
        a[0][0] = 1.0;

        for (int k = 1; k < order + 1; k++)
        {
            float d = 0.0;
            int rk = r - k;
            int pk = degree_ - k;
            if (r >= k)
            {
                a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
                d = a[s2][0] * ndu[rk][pk];
            }

            if (rk >= -1)
                j1 = 1;
            else
                j1 = -rk;

            if (r - 1 <= pk)
                j2 = k - 1;
            else
                j2 = degree_ - r;

            for (int j = j1; j <= j2; j++)
            {
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j];
                d += a[s2][j] * ndu[rk + j][pk];
            }

            if (r <= pk)
            {
                a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][r];
                d += a[s2][k] * ndu[r][pk];
            }

            ders[k][r] = d;

            int j = s1;
            s1 = s2;
            s2 = j;
        }
    }

    float r = float(degree_);
    for (int k = 1; k <= order; k++)
    {
        for (int j = 0; j <= degree_; j++)
            ders[k][j] *= r;
        r *= (degree_ - k);
    }

    return ders;
}

std::vector<std::vector<std::vector<double>>> Nurbs::basisFunctionsDerivatives(int order) const
{
    std::vector<std::vector<std::vector<double>>> ders;
    int du = std::min(degree_, order);
    for (size_t i = 0; i < spans_.size(); i++)
        ders.push_back(basisFunctionDerivatives(du, spans_[i], parameters_space_[i]));
    
    return ders;
}

Eigen::VectorXd* Nurbs::evaluate() const
{
    if (adequate_conf_)
    {
        size_t dimension = control_points_[0].size();

        double** crvpt_w = new double*[parameters_space_.size()];
        Eigen::VectorXd* eval_points = new Eigen::VectorXd[parameters_space_.size()];

        std::vector<std::vector<double>> weighted_control_points;
        for (int i = 0; i < control_points_.size(); ++i)
        {
            std::vector<double> tmp_pt(dimension + 1, 0.0);
            for (int j = 0; j < dimension; ++j)
                tmp_pt[j] = control_points_[i][j] * double(control_points_[i].w());

            tmp_pt[dimension] = control_points_[i].w();
            weighted_control_points.push_back(tmp_pt);
        }

        double** basis = new double*[spans_.size()];
        basisFunctions(basis);
        for (size_t idx = 0; idx < parameters_space_.size(); idx++)
        {
            double* crvpt = new double[dimension + 1];
            for (int i = 0; i <= dimension; i++)
                crvpt[i] = 0.0;
            
            for (int i = 0; i <= degree_; i++)
            {
                int index = spans_[idx] - degree_ + i;
                for (size_t j = 0; j < (dimension + 1); j++)
                    crvpt[j] += basis[idx][i] * weighted_control_points[index][j];
            }
            crvpt_w[idx] = crvpt;
        }

        for (int i = 0; i < parameters_space_.size(); i++)
        {
            double* pt = crvpt_w[i];
            Eigen::VectorXd cpt(dimension);
            for (int j = 0; j < dimension; j++)
                cpt[j] = pt[j] / pt[dimension];
            eval_points[i] = cpt;
            delete[] pt;
        }

        // Free memory for basis
        for (size_t i = 0; i < spans_.size(); ++i)
        {
            delete[] basis[i];
        }
        delete[] basis;

        // Free memory for crvpt_w
        delete[] crvpt_w;

        return eval_points;
    }
    else
    {
        std::cout << "You've got an inadequate configuration of curve degree VS number of control points on your hands"
                  << std::endl;
        
        return nullptr;
    }
}

std::vector<std::vector<std::vector<double>>> Nurbs::derivatives(int order) const
{
    if (!adequate_conf_) 
    {
        std::cout << "You've got an inadequate configuration of curve degree VS number of control points on your hands"
                  << std::endl;
        return {};
    }

    size_t dimension = control_points_[0].size();

    int du = std::min(degree_, order);
    std::vector<std::vector<std::vector<double>>> CK(parameters_space_.size(), 
                                                     std::vector<std::vector<double>>(order + 1, 
                                                     std::vector<double>(dimension + 1, 0.0)));
    std::vector<std::vector<std::vector<double>>> v = basisFunctionsDerivatives(du);

    std::vector<std::vector<double>> weighted_control_points;
    for (int i = 0; i < control_points_.size(); ++i)
    {
        std::vector<double> tmp_pt(dimension + 1, 0.0);
        for (int j = 0; j < dimension; ++j)
            tmp_pt[j] = control_points_[i][j] * double(control_points_[i].w());

        tmp_pt[dimension] = control_points_[i].w();
        weighted_control_points.push_back(tmp_pt);
    }
    
    for (int i = 0; i < parameters_space_.size(); i++)
    {
        for (int k = 0; k <= du; k++)
        {
            for (int j = 0; j <= degree_; j++)
            {
                for (int r = 0; r < dimension; r++)
                    CK[i][k][r] += v[i][k][j] * weighted_control_points[spans_[i] - degree_ + j][r];
            }
        }
    }

    return CK;
}

}; // namespace arena_core
