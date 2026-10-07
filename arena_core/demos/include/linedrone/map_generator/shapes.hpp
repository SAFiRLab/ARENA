/**
 * BSD 3-Clause License
 * 
 * Copyright (c) 2026, David-Alexandre Poissant, Université de Sherbrooke
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

#pragma once


// Local
#include "linedrone/map_generator/random.hpp"

// System
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

// External Libraries
#include <Eigen/Dense>
#include <Eigen/Geometry>


namespace arena_demos
{

namespace map_generator
{

/**
 * @brief Closed volume primitives that can be used as obstacles.
 */
enum class ShapeType
{
    Box,
    Sphere,
    Ellipsoid,
    Cylinder,
    Cone,
    Capsule,
    Torus,
    Prism
}; // enum class ShapeType

/** @brief All the shape types, in a fixed order (used to keep the generation deterministic). */
const std::vector<ShapeType>& allShapeTypes();

std::string shapeTypeToString(ShapeType type);

/** @brief Throws std::invalid_argument if the name is not a known shape type. */
ShapeType shapeTypeFromString(const std::string& name);


/**
 * @brief Closed volume placed in the world.
 *
 * Every shape is defined in a local frame centered on its bounding box, then placed in the world with
 * a center and a rotation. The local z axis is the "up" axis of the shape (axis of the cylinder, cone,
 * capsule, torus and prism), so an upright shape (OrientationMode::Yaw) keeps its axis vertical.
 */
class Shape
{
public:

    virtual ~Shape() = default;

    virtual ShapeType getType() const = 0;

    /** @brief Analytic volume in m^3. */
    virtual double getVolume() const = 0;

    /** @brief Half extents of the axis aligned bounding box in the local frame. */
    virtual Eigen::Vector3d getLocalHalfExtents() const = 0;

    /** @brief Thinnest dimension of the shape, used to reject shapes thinner than the map resolution. */
    virtual double getMinFeatureSize() const = 0;

    /** @brief Named dimensions of the shape (in meters), saved in the map metadata. */
    virtual std::map<std::string, double> getDimensions() const = 0;

    /** @brief True if the point (local frame) is inside the closed volume. */
    virtual bool containsLocal(const Eigen::Vector3d& point) const = 0;

    /** @brief True if the point (world frame) is inside the closed volume. */
    bool contains(const Eigen::Vector3d& point) const
    {
        return containsLocal(rotation_.transpose() * (point - center_));
    }

    /** @brief Axis aligned bounding box of the shape in the world frame. */
    Eigen::AlignedBox3d getWorldBoundingBox() const;

    /** @brief Half extents of the world axis aligned bounding box. */
    Eigen::Vector3d getWorldHalfExtents() const;

    // Getters
    const Eigen::Vector3d& getCenter() const { return center_; }
    const Eigen::Matrix3d& getRotation() const { return rotation_; }

    // Setters
    void setCenter(const Eigen::Vector3d& center) { center_ = center; }
    void setRotation(const Eigen::Matrix3d& rotation) { rotation_ = rotation; }

protected:

    Eigen::Vector3d center_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rotation_ = Eigen::Matrix3d::Identity();

}; // class Shape


/**
 * @brief Creates a shape of the given type with the given volume and random proportions.
 *
 * @param type Type of the shape.
 * @param volume Volume of the shape in m^3.
 * @param max_aspect_ratio Maximum ratio between the largest and the smallest dimension (>= 1).
 * @param rng Random number generator.
 * @return Shape centered at the origin with an identity rotation.
 */
std::unique_ptr<Shape> makeRandomShape(ShapeType type, double volume, double max_aspect_ratio, Random& rng);


class BoxShape : public Shape
{
public:
    explicit BoxShape(const Eigen::Vector3d& size) : size_(size) {}

    ShapeType getType() const override { return ShapeType::Box; }
    double getVolume() const override;
    Eigen::Vector3d getLocalHalfExtents() const override { return size_ / 2.0; }
    double getMinFeatureSize() const override { return size_.minCoeff(); }
    std::map<std::string, double> getDimensions() const override;
    bool containsLocal(const Eigen::Vector3d& point) const override;

private:
    Eigen::Vector3d size_;
}; // class BoxShape


class SphereShape : public Shape
{
public:
    explicit SphereShape(double radius) : radius_(radius) {}

    ShapeType getType() const override { return ShapeType::Sphere; }
    double getVolume() const override;
    Eigen::Vector3d getLocalHalfExtents() const override { return Eigen::Vector3d::Constant(radius_); }
    double getMinFeatureSize() const override { return 2.0 * radius_; }
    std::map<std::string, double> getDimensions() const override;
    bool containsLocal(const Eigen::Vector3d& point) const override;

private:
    double radius_;
}; // class SphereShape


class EllipsoidShape : public Shape
{
public:
    explicit EllipsoidShape(const Eigen::Vector3d& semi_axes) : semi_axes_(semi_axes) {}

    ShapeType getType() const override { return ShapeType::Ellipsoid; }
    double getVolume() const override;
    Eigen::Vector3d getLocalHalfExtents() const override { return semi_axes_; }
    double getMinFeatureSize() const override { return 2.0 * semi_axes_.minCoeff(); }
    std::map<std::string, double> getDimensions() const override;
    bool containsLocal(const Eigen::Vector3d& point) const override;

private:
    Eigen::Vector3d semi_axes_;
}; // class EllipsoidShape


class CylinderShape : public Shape
{
public:
    CylinderShape(double radius, double height) : radius_(radius), height_(height) {}

    ShapeType getType() const override { return ShapeType::Cylinder; }
    double getVolume() const override;
    Eigen::Vector3d getLocalHalfExtents() const override { return Eigen::Vector3d(radius_, radius_, height_ / 2.0); }
    double getMinFeatureSize() const override { return std::min(2.0 * radius_, height_); }
    std::map<std::string, double> getDimensions() const override;
    bool containsLocal(const Eigen::Vector3d& point) const override;

private:
    double radius_;
    double height_;
}; // class CylinderShape


/** @brief Cone with its base at the bottom (local -z) and its apex at the top (local +z). */
class ConeShape : public Shape
{
public:
    ConeShape(double radius, double height) : radius_(radius), height_(height) {}

    ShapeType getType() const override { return ShapeType::Cone; }
    double getVolume() const override;
    Eigen::Vector3d getLocalHalfExtents() const override { return Eigen::Vector3d(radius_, radius_, height_ / 2.0); }
    double getMinFeatureSize() const override { return std::min(2.0 * radius_, height_); }
    std::map<std::string, double> getDimensions() const override;
    bool containsLocal(const Eigen::Vector3d& point) const override;

private:
    double radius_;
    double height_;
}; // class ConeShape


/** @brief Cylinder of the given length (along local z) capped with two hemispheres. */
class CapsuleShape : public Shape
{
public:
    CapsuleShape(double radius, double length) : radius_(radius), length_(length) {}

    ShapeType getType() const override { return ShapeType::Capsule; }
    double getVolume() const override;
    Eigen::Vector3d getLocalHalfExtents() const override { return Eigen::Vector3d(radius_, radius_, length_ / 2.0 + radius_); }
    double getMinFeatureSize() const override { return 2.0 * radius_; }
    std::map<std::string, double> getDimensions() const override;
    bool containsLocal(const Eigen::Vector3d& point) const override;

private:
    double radius_;
    double length_;
}; // class CapsuleShape


/** @brief Torus lying in the local xy plane. */
class TorusShape : public Shape
{
public:
    TorusShape(double major_radius, double minor_radius) : major_radius_(major_radius), minor_radius_(minor_radius) {}

    ShapeType getType() const override { return ShapeType::Torus; }
    double getVolume() const override;
    Eigen::Vector3d getLocalHalfExtents() const override;
    double getMinFeatureSize() const override { return 2.0 * minor_radius_; }
    std::map<std::string, double> getDimensions() const override;
    bool containsLocal(const Eigen::Vector3d& point) const override;

private:
    double major_radius_;
    double minor_radius_;
}; // class TorusShape


/** @brief Regular polygon (3 to 8 sides) extruded along local z. */
class PrismShape : public Shape
{
public:
    PrismShape(int nb_of_sides, double circumradius, double height);

    ShapeType getType() const override { return ShapeType::Prism; }
    double getVolume() const override;
    Eigen::Vector3d getLocalHalfExtents() const override { return Eigen::Vector3d(circumradius_, circumradius_, height_ / 2.0); }
    double getMinFeatureSize() const override { return std::min(2.0 * apothem_, height_); }
    std::map<std::string, double> getDimensions() const override;
    bool containsLocal(const Eigen::Vector3d& point) const override;

private:
    int nb_of_sides_;
    double circumradius_;
    double height_;
    double apothem_;
    std::vector<Eigen::Vector2d> edge_normals_;
}; // class PrismShape

}; // namespace map_generator

}; // namespace arena_demos
