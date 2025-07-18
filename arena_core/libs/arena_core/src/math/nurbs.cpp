// Local
#include "arena_core/math/nurbs.h"
#include "arena_core/math/utils/linalg.h"

// System
#include <iostream>
#include <algorithm>


namespace arena_core
{

Nurbs::Nurbs(const std::vector<ControlPoint<double>>& control_points, int sample_size, int degree)
: control_points_(control_points), sample_size_(sample_size), degree_(degree), adequate_conf_(false)
{
    adequate_conf_ = initialize();
}

bool Nurbs::initialize()
{
    if (control_points_.empty())
    {
        std::cout << "Set of control points can't be empty" << std::endl;
        return false;
    }

    if (degree_ >= control_points_.size())
    {
        std::cout << "You've got an inadequate configuration of curve degree VS number of control points on your hands"
                  << std::endl;
        return false;
    }

    unsigned int dimension = control_points_[0].getDimension();
    if (!std::all_of(control_points_.begin(), control_points_.end(),
        [dimension](const auto& cp) { return cp.getDimension() == dimension; }))
    {
        std::cout << "All control points must have the same dimension." << std::endl;
        return false;
    }
    dimension_ = dimension;

    knot_vector_ = generateKnotVector();
    double start = knot_vector_[degree_];
    double stop = knot_vector_[knot_vector_.size() - (degree_ + 1)];
    parameters_space_ = linalg::linspace<double>(start, stop, sample_size_);
    spans_ = findSpans();
    return true;
}

bool Nurbs::setDegree(int degree)
{
    adequate_conf_ = false;
    if (1 <= degree && degree <= 5)
    {
        degree_ = degree;
        adequate_conf_ = initialize();
    }
    else
        std::cout << "Curve order needs to be in the interval [1, 5]" << std::endl;

    return adequate_conf_;
}

bool Nurbs::setSampleSize(int sample_size)
{
    adequate_conf_ = false;
    if (0 < sample_size)
    {
        sample_size_ = sample_size;
        adequate_conf_ = initialize();
    }
    else
        std::cout << "Sample size needs to be a positive non-zero number";
    
    return adequate_conf_;
}

bool Nurbs::setControlPoints(const std::vector<ControlPoint<double>>& control_points)
{
    adequate_conf_ = false;
    if (!control_points.empty())
    {
        control_points_ = std::move(control_points);
        adequate_conf_ = initialize();
    }
    else
        std::cout << "Set of control points can't be empty" << std::endl;
    
    return adequate_conf_;
}

bool Nurbs::setControlPoint(const ControlPoint<double>& control_point, int index)
{
    adequate_conf_ = false;
    if (0 <= index && index < control_points_.size())
    {
        control_points_[index] = control_point;
        adequate_conf_ = initialize();
    }
    else
        std::cout << "The index: " << index << " needs to be above 0 and below " << control_points_.size() << std::endl;

    return adequate_conf_;
}

std::vector<double> Nurbs::generateKnotVector() const
{
    int num_control_points = control_points_.size();
    
    int num_segments = num_control_points - degree_ + 1;
    std::vector<double> knot_vector(num_control_points + degree_ + 1, 0.0);

    // Set up the knots at the start and end to create a clamped curve
    for (int i = 0; i <= degree_; i++)
    {
        knot_vector[i] = 0.0;
        knot_vector[knot_vector.size() - (i + 1)] = 1.0;
    }

    // Generate the interior knots
    for (int i = 1; i < num_segments - 1; i++)
        knot_vector[i + degree_] = static_cast<double>(i) / (num_segments - 1);

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

    for (int j = 1; j <= degree_; j++)
    {
        left[j] = parameter - knot_vector_[span + 1 - j];
        right[j] = knot_vector_[span + j] - parameter;
        double saved = 0.0;
        for (int r = 0; r < j; r++)
        {
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
    double** crvpt_w = new double*[parameters_space_.size()];
    Eigen::VectorXd* eval_points = new Eigen::VectorXd[parameters_space_.size()];

    std::vector<std::vector<double>> weighted_control_points;
    for (int i = 0; i < control_points_.size(); ++i)
    {
        std::vector<double> tmp_pt(dimension_ + 1, 0.0);
        for (int j = 0; j < dimension_; ++j)
            tmp_pt[j] = control_points_[i][j] * double(control_points_[i].W());

        tmp_pt[dimension_] = control_points_[i].W();
        weighted_control_points.push_back(tmp_pt);
    }

    if (adequate_conf_)
    {
        double** basis = new double*[spans_.size()];
        basisFunctions(basis);
        for (size_t idx = 0; idx < parameters_space_.size(); idx++)
        {
            double* crvpt = new double[dimension_ + 1];
            for (int i = 0; i <= dimension_; i++)
                crvpt[i] = 0.0;
            
            for (int i = 0; i <= degree_; i++)
            {
                int index = spans_[idx] - degree_ + i;
                for (size_t j = 0; j < (dimension_ + 1); j++)
                    crvpt[j] += basis[idx][i] * weighted_control_points[index][j];
            }
            crvpt_w[idx] = crvpt;
        }

        for (int i = 0; i < parameters_space_.size(); i++)
        {
            double* pt = crvpt_w[i];
            Eigen::VectorXd cpt(dimension_);
            for (int j = 0; j < dimension_; j++)
                cpt[j] = pt[j] / pt[dimension_];
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
    }
    else
    {
        std::cout << "You've got an inadequate configuration of curve degree VS number of control points on your hands"
                  << std::endl;
        delete[] eval_points;  // Free memory for eval_points in case of inadequate configuration
        eval_points = nullptr;  // Return nullptr in case of error
    }
    
    return eval_points;
}

std::vector<std::vector<std::vector<double>>> Nurbs::derivatives(int order) const
{
    int du = std::min(degree_, order);
    std::vector<std::vector<std::vector<double>>> CK(parameters_space_.size(), 
                                                     std::vector<std::vector<double>>(order + 1, 
                                                     std::vector<double>(dimension_ + 1, 0.0)));
    std::vector<std::vector<std::vector<double>>> v = basisFunctionsDerivatives(du);

    std::vector<std::vector<double>> weighted_control_points;
    for (int i = 0; i < control_points_.size(); ++i)
    {
        std::vector<double> tmp_pt(dimension_ + 1, 0.0);
        for (int j = 0; j < dimension_; ++j)
            tmp_pt[j] = control_points_[i][j] * double(control_points_[i].W());

        tmp_pt[dimension_] = control_points_[i].W();
        weighted_control_points.push_back(tmp_pt);
    }
    
    for (int i = 0; i < parameters_space_.size(); i++)
    {
        for (int k = 0; k <= du; k++)
        {
            for (int j = 0; j <= degree_; j++)
            {
                for (int r = 0; r < dimension_; r++)
                    CK[i][k][r] += v[i][k][j] * weighted_control_points[spans_[i] - degree_ + j][r];
            }
        }
    }

    return CK;
}


Eigen::VectorXd Nurbs::estimateParamByArcLength(const Eigen::VectorXd* points) const
{
    int N = sample_size_;
    Eigen::VectorXd t(N);
    t(0) = 0.0;

    for (int i = 1; i < N; ++i)
        t(i) = t(i-1) + (points[i] - points[i-1]).norm();

    // Optional: normalize to [0, 1]
    t /= t(N-1);
    return t;
}


Eigen::MatrixXd Nurbs::fitPolynomialCurve(Eigen::VectorXd* points)
{
    int degree = 15;  // Degree of the polynomial
    if (!adequate_conf_)
        throw std::runtime_error("Number of points must be greater than polynomial degree.");

    const int D = points[0].size();  // Dimensionality of the data
    Eigen::VectorXd t = estimateParamByArcLength(points);

    // Create Vandermonde matrix V (N x (degree+1))
    Eigen::MatrixXd V(sample_size_, degree + 1);
    for (int i = 0; i < sample_size_; ++i)
    {
        double val = 1.0;
        for (int j = 0; j <= degree; ++j)
        {
            V(i, j) = val;
            val *= t(i);
        }
    }

    // Create Y matrix (N x D) from input curve points
    Eigen::MatrixXd Y(sample_size_, D);
    for (int i = 0; i < sample_size_; ++i)
        Y.row(i) = points[i].transpose();

    // Solve for coefficients using QR decomposition: minimize ||V * C - Y||
    // Result: C is (degree+1) x D, transpose to get (D x degree+1)
    Eigen::MatrixXd C = V.colPivHouseholderQr().solve(Y);

    return C.transpose();  // Each row corresponds to one dimension (e.g., x(t), y(t), z(t))
}


Eigen::VectorXd Nurbs::evaluatePolynomial(const Eigen::MatrixXd& coeffs,  double t)
{
    const int D = coeffs.rows();
    const int degree = coeffs.cols() - 1;

    Eigen::VectorXd result = Eigen::VectorXd::Zero(D);
    double t_pow = 1.0;
    for (int i = 0; i <= degree; ++i)
    {
        result += coeffs.col(i) * t_pow;
        t_pow *= t;
    }
    return result;
}


}; // namespace arena_core
