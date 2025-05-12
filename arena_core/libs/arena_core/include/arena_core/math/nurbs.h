#pragma once

#include <vector>
#include <cmath>

// Eigen
#include <Eigen/Dense>

// Arena Core
#include "arena_core/spaces/control_point.h"


namespace arena_core
{

class Nurbs
{
public:
    Nurbs(const std::vector<ControlPoint<double>>& control_points, int sample_size, int degree = 5);

    Eigen::VectorXd* evaluate() const;
    std::vector<std::vector<std::vector<double>>> derivatives(int order) const;

    // Setters
    bool setDegree(int degree);
    bool setSampleSize(int sample_size);
    bool setControlPoints(const std::vector<ControlPoint<double>>& control_points);
    bool setControlPoint(const ControlPoint<double>& control_point, int index);
    bool setWeight(double weight, unsigned int index);

    // Getters
    int getDegree() const { return degree_; };
    int getSampleSize() const { return sample_size_; };
    std::vector<ControlPoint<double>> getControlPoints() const { return control_points_; };

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
    std::vector<ControlPoint<double>> control_points_;
    std::vector<double> knot_vector_;
    int sample_size_;
    int degree_;
    bool adequate_conf_;
    std::vector<double> parameters_space_;
    std::vector<int> spans_;

}; // class Nurbs

}; // namespace arena_core
