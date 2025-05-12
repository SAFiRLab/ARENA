#ifndef ARENA_CORE_COMPOUND_STATE_
#define ARENA_CORE_COMPOUND_STATE_

#include <stdexcept>
#include <memory>
#include <Eigen/Dense>

#include "arena_core/state.h"


namespace arena_core
{

/** \brief Definition of a ControlPoint, inheriting from State */
template <typename T>
class ControlPoint : public State
{
public:
    using ValueType = Eigen::Matrix<T, Eigen::Dynamic, 1>;

    /** \brief Default constructor */
    ControlPoint() = delete;

    /** \brief Constructor */
    /** \param dim The dimension of the control point vector */
    /** \param weight The weight of the control point */
    /** \throws std::invalid_argument if dim is 0 or weight is negative */
    ControlPoint(size_t dim, double weight = 1.0)
        : values_(nullptr), weight_(weight), dim_(dim)
    {
        if (dim_ == 0) throw std::invalid_argument("Dimension of control point cannot be zero");
        if (weight_ < 0) throw std::invalid_argument("Weight of control point cannot be negative");
        values_ = std::make_unique<T[]>(dim_);
    }

    /** \brief Constructor from Eigen::Matrix<T, Eigen::Dynamic, 1> */
    /** \param vec The Eigen::Matrix<T, Eigen::Dynamic, 1> to convert */
    /** \param weight The weight of the control point */
    /** \throws std::invalid_argument if vec is empty or weight is negative */
    /** \returns ControlPoint with the values of the Eigen::Matrix<T, Eigen::Dynamic, 1> */
    ControlPoint(const ValueType& vec, double weight = 1.0)
        : values_(nullptr), weight_(weight), dim_(vec.size())
    {
        if (dim_ == 0) throw std::invalid_argument("Dimension of control point cannot be zero");
        if (weight_ < 0) throw std::invalid_argument("Weight of control point cannot be negative");
        values_ = std::make_unique<T[]>(dim_);
        for (size_t i = 0; i < dim_; ++i)
            values_[i] = static_cast<T>(vec[i]);
    }

    /** \brief Copy constructor (deep copy) */
    /** \param other The control point to copy */
    /** \throws std::invalid_argument if dim is 0 or weight is negative */
    ControlPoint(const ControlPoint& other)
        : dim_(other.dim_), values_(std::make_unique<T[]>(other.dim_)), weight_(other.weight_)
    {
        if (dim_ == 0) throw std::invalid_argument("Dimension of control point cannot be zero");
        if (weight_ < 0) throw std::invalid_argument("Weight of control point cannot be negative");
        std::copy(other.values_.get(), other.values_.get() + dim_, values_.get());
    }

    /** \brief Move constructor */
    /** \param other The control point to move */
    ControlPoint(ControlPoint&& other) noexcept
        : dim_(other.dim_), values_(std::move(other.values_)), weight_(other.weight_)
    {}

    /** \brief Copy assignment (deep copy) */
    /** \param other The control point to copy */
    /** \throws std::invalid_argument if dim is 0 or weight is negative */
    ControlPoint& operator=(const ControlPoint& other)
    {
        if (this != &other)
        {
            dim_ = other.dim_;
            weight_ = other.weight_;
            if (dim_ == 0) throw std::invalid_argument("Dimension of control point cannot be zero");
            if (weight_ < 0) throw std::invalid_argument("Weight of control point cannot be negative");
            values_ = std::make_unique<T[]>(dim_);
            std::copy(other.values_.get(), other.values_.get() + dim_, values_.get());
        }
        return *this;
    }

    /** \brief Move assignment */
    /** \param other The control point to move */
    ControlPoint& operator=(ControlPoint&& other) noexcept
    {
        if (this != &other)
        {
            dim_ = other.dim_;
            weight_ = other.weight_;
            values_ = std::move(other.values_);
            other.dim_ = 0;
        }
        return *this;
    }

    ~ControlPoint() override = default;

    /** \brief Access element i of values. */
    /** \param i The index of the element to access */
    /** \throws std::out_of_range if i is out of bounds */
    /** \returns Const value of the element at index i */
    T operator[](unsigned int i) const
    {
        if (i >= dim_) throw std::out_of_range("Index out of bounds");
        return values_[i];
    }

    /** \brief Access element i of values. */
    /** \param i The index of the element to access */
    /** \throws std::out_of_range if i is out of bounds */
    /** \returns Reference to the element at index i */
    T &operator[](unsigned int i)
    {
        if (i >= dim_) throw std::out_of_range("Index out of bounds");
        return values_[i];
    }

    /** \brief Conversion to Eigen::Matrix<T, Eigen::Dynamic, 1> */
    /** \returns Eigen::Matrix<T, Eigen::Dynamic, 1> with the values of the control point */
    ValueType toEigen() const
    {
        ValueType v(dim_);
        for (size_t i = 0; i < dim_; ++i)
            v[i] = values_[i];
        return v;
    }

    /** \brief Conversion from Eigen::Matrix<T, Eigen::Dynamic, 1>, set weight to 1.0 */
    /** \param vec The Eigen::Matrix<T, Eigen::Dynamic, 1> to convert */
    /** \throws std::invalid_argument if vec is empty */
    /** \returns ControlPoint with the values of the Eigen::Matrix<T, Eigen::Dynamic, 1> */
    static ControlPoint<T> fromEigen(const ValueType& vec)
    {
        if (vec.size() == 0) throw std::invalid_argument("Vector is empty");

        ControlPoint<T> cp(vec.size());
        for (int i = 0; i < vec.size(); ++i)
            cp[i] = static_cast<T>(vec[i]);
        return cp;
    }

    /** \brief Weight getter */
    double w() const { return weight_; }

    /** \brief Weight setter */
    /** \param w The new weight */
    /** \throws std::invalid_argument if w is negative */
    void setW(double w)
    {
        if (w < 0) throw std::invalid_argument("Weight of control point cannot be negative");
        weight_ = w;
    }

    /** \brief Dimension getter */
    size_t size() const { return dim_; }


private:

    /** \brief The value of the control point vector */
    std::unique_ptr<T[]> values_;
    /** \brief The weight of the control point */
    double weight_;
    /** \brief The dimension of the control point vector */
    size_t dim_;

}; // class ControlPoint

} // namespace arena_core

#endif // ARENA_CORE_COMPOUND_STATE_
