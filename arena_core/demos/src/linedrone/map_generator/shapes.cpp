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

// Local
#include "linedrone/map_generator/shapes.hpp"

// System
#include <algorithm>
#include <cmath>
#include <stdexcept>


namespace arena_demos
{

namespace map_generator
{

const std::vector<ShapeType>& allShapeTypes()
{
    static const std::vector<ShapeType> types = {
        ShapeType::Box,
        ShapeType::Sphere,
        ShapeType::Ellipsoid,
        ShapeType::Cylinder,
        ShapeType::Cone,
        ShapeType::Capsule,
        ShapeType::Torus,
        ShapeType::Prism
    };
    return types;
}

std::string shapeTypeToString(ShapeType type)
{
    switch (type)
    {
        case ShapeType::Box:        return "box";
        case ShapeType::Sphere:     return "sphere";
        case ShapeType::Ellipsoid:  return "ellipsoid";
        case ShapeType::Cylinder:   return "cylinder";
        case ShapeType::Cone:       return "cone";
        case ShapeType::Capsule:    return "capsule";
        case ShapeType::Torus:      return "torus";
        case ShapeType::Prism:      return "prism";
    }
    return "unknown";
}

ShapeType shapeTypeFromString(const std::string& name)
{
    for (ShapeType type : allShapeTypes())
    {
        if (shapeTypeToString(type) == name)
            return type;
    }
    throw std::invalid_argument("Unknown shape type: \"" + name + "\"");
}


/*********** Shape ***********/

Eigen::Vector3d Shape::getWorldHalfExtents() const
{
    // Exact for the bounding box of the rotated local bounding box
    return rotation_.cwiseAbs() * getLocalHalfExtents();
}

Eigen::AlignedBox3d Shape::getWorldBoundingBox() const
{
    Eigen::Vector3d half_extents = getWorldHalfExtents();
    return Eigen::AlignedBox3d(center_ - half_extents, center_ + half_extents);
}


/*********** Factory ***********/

std::unique_ptr<Shape> makeRandomShape(ShapeType type, double volume, double max_aspect_ratio, Random& rng)
{
    if (volume <= 0.0)
        throw std::invalid_argument("makeRandomShape => volume must be positive.");

    const double aspect = std::max(1.0, max_aspect_ratio);

    // Ratio between the height and the diameter of axisymmetric shapes, in [1 / aspect, aspect]
    auto elongation = [&rng, aspect]() { return rng.logUniform(1.0 / aspect, aspect); };

    switch (type)
    {
        case ShapeType::Box:
        {
            // Side ratios in [1, aspect], scaled to the volume
            Eigen::Vector3d ratios(rng.logUniform(1.0, aspect), rng.logUniform(1.0, aspect), rng.logUniform(1.0, aspect));
            double scale = std::cbrt(volume / ratios.prod());
            return std::make_unique<BoxShape>(scale * ratios);
        }

        case ShapeType::Sphere:
            return std::make_unique<SphereShape>(std::cbrt(3.0 * volume / (4.0 * M_PI)));

        case ShapeType::Ellipsoid:
        {
            Eigen::Vector3d ratios(rng.logUniform(1.0, aspect), rng.logUniform(1.0, aspect), rng.logUniform(1.0, aspect));
            double scale = std::cbrt(3.0 * volume / (4.0 * M_PI * ratios.prod()));
            return std::make_unique<EllipsoidShape>(scale * ratios);
        }

        case ShapeType::Cylinder:
        {
            // V = pi r^2 h with h = 2 r k
            double k = elongation();
            double radius = std::cbrt(volume / (2.0 * M_PI * k));
            return std::make_unique<CylinderShape>(radius, 2.0 * radius * k);
        }

        case ShapeType::Cone:
        {
            // V = pi r^2 h / 3 with h = 2 r k
            double k = elongation();
            double radius = std::cbrt(3.0 * volume / (2.0 * M_PI * k));
            return std::make_unique<ConeShape>(radius, 2.0 * radius * k);
        }

        case ShapeType::Capsule:
        {
            // V = pi r^2 L + 4/3 pi r^3 with L = 2 r k (length of the cylindrical part)
            double k = rng.uniform(0.25, std::max(0.5, aspect));
            double radius = std::cbrt(volume / (M_PI * (2.0 * k + 4.0 / 3.0)));
            return std::make_unique<CapsuleShape>(radius, 2.0 * radius * k);
        }

        case ShapeType::Torus:
        {
            // V = 2 pi^2 R r^2 with R = q r, q > 1 keeps the hole open
            double q = rng.uniform(1.5, std::max(2.0, std::min(4.0, 1.5 * aspect)));
            double minor_radius = std::cbrt(volume / (2.0 * M_PI * M_PI * q));
            return std::make_unique<TorusShape>(q * minor_radius, minor_radius);
        }

        case ShapeType::Prism:
        {
            // V = n/2 rho^2 sin(2 pi / n) h with h = 2 rho k
            int n = rng.uniformInt(3, 8);
            double k = elongation();
            double circumradius = std::cbrt(volume / (n * k * std::sin(2.0 * M_PI / n)));
            return std::make_unique<PrismShape>(n, circumradius, 2.0 * circumradius * k);
        }
    }

    throw std::invalid_argument("makeRandomShape => unknown shape type.");
}


/*********** Box ***********/

double BoxShape::getVolume() const
{
    return size_.prod();
}

std::map<std::string, double> BoxShape::getDimensions() const
{
    return {{"size_x", size_.x()}, {"size_y", size_.y()}, {"size_z", size_.z()}};
}

bool BoxShape::containsLocal(const Eigen::Vector3d& point) const
{
    return (point.cwiseAbs().array() <= (size_ / 2.0).array()).all();
}


/*********** Sphere ***********/

double SphereShape::getVolume() const
{
    return 4.0 / 3.0 * M_PI * radius_ * radius_ * radius_;
}

std::map<std::string, double> SphereShape::getDimensions() const
{
    return {{"radius", radius_}};
}

bool SphereShape::containsLocal(const Eigen::Vector3d& point) const
{
    return point.squaredNorm() <= radius_ * radius_;
}


/*********** Ellipsoid ***********/

double EllipsoidShape::getVolume() const
{
    return 4.0 / 3.0 * M_PI * semi_axes_.prod();
}

std::map<std::string, double> EllipsoidShape::getDimensions() const
{
    return {{"semi_axis_x", semi_axes_.x()}, {"semi_axis_y", semi_axes_.y()}, {"semi_axis_z", semi_axes_.z()}};
}

bool EllipsoidShape::containsLocal(const Eigen::Vector3d& point) const
{
    return point.cwiseQuotient(semi_axes_).squaredNorm() <= 1.0;
}


/*********** Cylinder ***********/

double CylinderShape::getVolume() const
{
    return M_PI * radius_ * radius_ * height_;
}

std::map<std::string, double> CylinderShape::getDimensions() const
{
    return {{"radius", radius_}, {"height", height_}};
}

bool CylinderShape::containsLocal(const Eigen::Vector3d& point) const
{
    return std::abs(point.z()) <= height_ / 2.0 && point.head<2>().squaredNorm() <= radius_ * radius_;
}


/*********** Cone ***********/

double ConeShape::getVolume() const
{
    return M_PI * radius_ * radius_ * height_ / 3.0;
}

std::map<std::string, double> ConeShape::getDimensions() const
{
    return {{"radius", radius_}, {"height", height_}};
}

bool ConeShape::containsLocal(const Eigen::Vector3d& point) const
{
    if (std::abs(point.z()) > height_ / 2.0)
        return false;

    // Radius of the cross-section: radius_ at the base (z = -h/2), 0 at the apex (z = h/2)
    double section_radius = radius_ * (0.5 - point.z() / height_);
    return point.head<2>().squaredNorm() <= section_radius * section_radius;
}


/*********** Capsule ***********/

double CapsuleShape::getVolume() const
{
    return M_PI * radius_ * radius_ * length_ + 4.0 / 3.0 * M_PI * radius_ * radius_ * radius_;
}

std::map<std::string, double> CapsuleShape::getDimensions() const
{
    return {{"radius", radius_}, {"length", length_}};
}

bool CapsuleShape::containsLocal(const Eigen::Vector3d& point) const
{
    // Distance to the segment [-L/2, L/2] on the z axis
    Eigen::Vector3d closest(0.0, 0.0, std::clamp(point.z(), -length_ / 2.0, length_ / 2.0));
    return (point - closest).squaredNorm() <= radius_ * radius_;
}


/*********** Torus ***********/

double TorusShape::getVolume() const
{
    return 2.0 * M_PI * M_PI * major_radius_ * minor_radius_ * minor_radius_;
}

Eigen::Vector3d TorusShape::getLocalHalfExtents() const
{
    double outer_radius = major_radius_ + minor_radius_;
    return Eigen::Vector3d(outer_radius, outer_radius, minor_radius_);
}

std::map<std::string, double> TorusShape::getDimensions() const
{
    return {{"major_radius", major_radius_}, {"minor_radius", minor_radius_}};
}

bool TorusShape::containsLocal(const Eigen::Vector3d& point) const
{
    double ring_distance = point.head<2>().norm() - major_radius_;
    return ring_distance * ring_distance + point.z() * point.z() <= minor_radius_ * minor_radius_;
}


/*********** Prism ***********/

PrismShape::PrismShape(int nb_of_sides, double circumradius, double height)
: nb_of_sides_(nb_of_sides), circumradius_(circumradius), height_(height)
{
    if (nb_of_sides_ < 3)
        throw std::invalid_argument("PrismShape => a prism needs at least 3 sides.");

    apothem_ = circumradius_ * std::cos(M_PI / nb_of_sides_);

    // Vertices at angles 2 pi i / n, so the outward normal of edge i is at angle 2 pi (i + 0.5) / n
    for (int i = 0; i < nb_of_sides_; ++i)
    {
        double angle = 2.0 * M_PI * (i + 0.5) / nb_of_sides_;
        edge_normals_.emplace_back(std::cos(angle), std::sin(angle));
    }
}

double PrismShape::getVolume() const
{
    return 0.5 * nb_of_sides_ * circumradius_ * circumradius_ * std::sin(2.0 * M_PI / nb_of_sides_) * height_;
}

std::map<std::string, double> PrismShape::getDimensions() const
{
    return {{"nb_of_sides", static_cast<double>(nb_of_sides_)}, {"circumradius", circumradius_}, {"height", height_}};
}

bool PrismShape::containsLocal(const Eigen::Vector3d& point) const
{
    if (std::abs(point.z()) > height_ / 2.0)
        return false;

    Eigen::Vector2d planar = point.head<2>();
    for (const Eigen::Vector2d& normal : edge_normals_)
    {
        if (planar.dot(normal) > apothem_)
            return false;
    }
    return true;
}

}; // namespace map_generator

}; // namespace arena_demos
