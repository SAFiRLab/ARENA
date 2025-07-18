#pragma once

// External Libraries
// Eigen
#include <Eigen/Dense>


namespace arena_core
{

/**
 * @brief ControlPoint class template.
 * This class is a placeholder for control points in NURBS or other mathematical constructs.
 * It can be specialized for different types of control points as needed.
 *
 * @tparam T The type of the control point 
 */
template<typename T, int _Dim>
class ControlPoint
{
public:

    /**
     * @brief Constructor with values.
     * Initializes the control point with specified values and weight.
     *
     * @param a_values The values of the control point.
     * @param a_weight The weight of the control point.
     * @param a_id The unique identifier for the control point.
     */
    ControlPoint(const Eigen::Matrix<T, _Dim, 1>& a_values,
                 const unsigned int a_id = 0, const double a_weight = 1.0)
        : values_(std::move(a_values)), weight_(a_weight), id_(a_id)
    {}

    /**
     * @brief Move constructor.
     * Initializes the control point with values from another control point.
     */
    ControlPoint(ControlPoint&& other) noexcept
        : values_(std::move(other.values_)), weight_(other.weight_), id_(other.id_)
    {}

    /**
     * @brief Copy constructor.
     * Initializes the control point with values from another control point.
     */
    ControlPoint(const ControlPoint& other)
        : values_(other.values_), weight_(other.weight_), id_(other.id_)
    {}

    /************* Setters *************/
    /**
     * @brief Set the weight of the control point.
     * 
     * @param a_weight The new weight to set.
     */
    void setW(const double a_weight)
    {
        if (a_weight <= 0.0)
            throw std::invalid_argument("Weight must be positive.");
        
        weight_ = a_weight;
    }

    /**
     * @brief Set the values of the control point.
     * 
     * @param a_values The new values to set.
     */
    void setValues(const Eigen::Matrix<T, _Dim, 1>& a_values)
    {
        if (a_values.rows() != values_.rows())
            throw std::invalid_argument("Values size does not match the dimension of the control point.");

        if (a_values.rows() == 0)
            throw std::invalid_argument("Values cannot be empty.");
        
        values_ = std::move(a_values);
    }

    /**
     * @brief Set a specific value of the control point.
     * 
     * @param a_value The value to set.
     * @param a_index The index of the value to set.
     */
    void setValue(const T& a_value, const unsigned int a_index)
    {
        if (a_index >= values_.rows() || a_index < 0)
            throw std::out_of_range("Index out of range for control point values.");

        values_(a_index) = a_value;
    }

    /************* Getters *************/
    /**
     * @brief Get the id of the control point.
     * 
     * @return The id of the control point.
     */
    unsigned int getId() const noexcept { return id_; }

    /**
     * @brief Get the weight of the control point.
     * 
     * @return The weight of the control point.
     */
    double W() const noexcept { return weight_; }

    /**
     * @brief Get the dimension of the control point.
     * 
     * @return The dimension of the control point.
     */
    unsigned int getDimension() const noexcept { return values_.rows(); }

    /**
     * @brief Get the values of the control point.
     * 
     * @return The values of the control point as an Eigen matrix.
     */
    Eigen::Matrix<T, _Dim, 1> getValues() const noexcept { return values_; }

    /************* Operators overload *************/
    /**
     * @brief Overload the equality operator.
     * 
     * @param other The other control point to compare with.
     * @return True if the control points are equal, false otherwise.
     */
    bool operator==(const ControlPoint& other) const
    {
        return (id_ == other.id_) && (values_ == other.values_) && (weight_ == other.weight_);
    }

    /**
     * @brief Overload the inequality operator.
     * 
     * @param other The other control point to compare with.
     * @return True if the control points are not equal, false otherwise.
     */
    bool operator!=(const ControlPoint& other) const
    {
        return !(*this == other);
    }

    /**
     * @brief Overload the assignment operator.
     * @param other The other control point to assign from.
     * @return A reference to this control point.
     */
    ControlPoint& operator=(const ControlPoint& other)
    {
        if (other.values_.rows() != values_.rows())
            throw std::invalid_argument("Cannot assign control point with different dimension.");
        
        if (this != &other)
        {
            id_ = other.id_;
            values_ = other.values_;
            weight_ = other.weight_;
        }
        return *this;
    }

    /**
     * @brief Overload the [] operator to access values.
     * @param index The index of the value to access.
     * @return A reference to the value at the specified index.
     */
    T& operator[](const unsigned int index)
    {
        if (index >= values_.rows() || index < 0)
            throw std::out_of_range("Index out of range for control point values.");
        return values_(index);
    }

    /**
     * @brief Overload the const [] operator to access values.
     * @param index The index of the value to access.
     * @return A const reference to the value at the specified index.
     */
    const T& operator[](const unsigned int index) const
    {
        if (index >= values_.rows() || index < 0)
            throw std::out_of_range("Index out of range for control point values.");
        return values_(index);
    }

private:
    
    /************* User-defined attributes *************/
    Eigen::Matrix<T, _Dim, 1> values_; // Eigen Matrix of n values, where n is the dimension of the control point
    double weight_; // Weight of the control point, used in NURBS calculations
    unsigned int id_; // Unique identifier for the control point

}; // ControlPoint

}; // arena_core
