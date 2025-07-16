#pragma once

#include <vector>
#include <cmath>

// Eigen
#include <Eigen/Dense>


namespace arena_core
{

class Nurbs
{
public:
    Nurbs(const std::vector<Eigen::VectorXd>& control_points, int sample_size, 
          std::vector<double> weights = std::vector<double>(), int degree = 5);

    Eigen::VectorXd* evaluate() const;
    std::vector<std::vector<std::vector<double>>> derivatives(int order) const;
    Eigen::MatrixXd fitPolynomialCurve(Eigen::VectorXd* points);
    Eigen::VectorXd estimateParamByArcLength(const Eigen::VectorXd* points) const;
    Eigen::VectorXd evaluatePolynomial(const Eigen::MatrixXd& coeffs,  double t);

    // Setters
    bool setDegree(int degree);
    bool setSampleSize(int sample_size);
    bool setControlPoints(const std::vector<Eigen::VectorXd>& control_points);
    bool setControlPoint(const Eigen::VectorXd& control_point, int index);
    bool setWeightedControlPoints(const std::vector<Eigen::VectorXd>& control_points, 
                                    const std::vector<double>& weights);

    // Getters
    int getDegree() const { return degree_; };
    int getSampleSize() const { return sample_size_; };
    std::vector<Eigen::VectorXd> getControlPoints() const { return control_points_; };

private:

    // User defined methods
    bool initialize();
    std::vector<double> generateKnotVector() const;

    std::vector<int> findSpans() const;
    int findSpan(double parameter) const;

    void basisFunctions(double** basis) const;
    void basisFunction(int span, double parameter, double* N) const;
    std::vector<std::vector<std::vector<double>>> basisFunctionsDerivatives(int order) const;
    std::vector<std::vector<double>> basisFunctionDerivatives(int order, int span, double parameter) const;

    // User defined attributes
    std::vector<Eigen::VectorXd> control_points_;
    std::vector<double> weights_;
    std::vector<double> knot_vector_;
    int sample_size_;
    int degree_;
    int dimension_;
    bool adequate_conf_;
    std::vector<double> parameters_space_;
    std::vector<int> spans_;

}; // class Nurbs

}; // namespace arena_core
