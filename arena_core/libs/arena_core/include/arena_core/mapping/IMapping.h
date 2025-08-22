/**
 * BSD 3-Clause License
 * 
 * Copyright (c) 2025, David-Alexandre Poissant, Université de Sherbrooke
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

// System
#include <memory>
#include <unordered_map>
#include <typeindex>

// External Libraries
// Eigen
#include <Eigen/Dense>


namespace arena_core
{

/**
 * @brief Interface for capabilities in the mapping system.
 * 
 * This interface defines a capability that can be registered and retrieved from the mapping algorithm.
 * It allows for extending the functionalities of the mapping algorithm with specific capabilities.
 */
class ICapability
{
public:
    virtual ~ICapability() = default;
}; // class ICapability


/**
 * @brief Interface for mapping functionalities.
 * 
 * This interface defines the core methods for mapping functionalities and provides a capability registry for extending functionalities.
 */
class IMapping
{
public:
    virtual ~IMapping() = default;

    /***************** CORE Methods *******************/
    /**
     * @brief Check if the given point is occupied in the mapping.
     * 
     * This method checks if a point is occupied in the mapping, which can be used for collision checking.
     * 
     * @param point The point to check for occupancy.
     * @return true if the point is occupied, false otherwise.
     */
    virtual bool isOccupied(const Eigen::VectorXd& point) const = 0;

    virtual bool isOccupiedRayTracing(const Eigen::VectorXd& point1, const Eigen::VectorXd& point2) const
    {
        // Default implementation, can be overridden by derived classes
        return isOccupied(point1) || isOccupied(point2);
    }

    /**
     * @brief Check if the given point is clear of obstacles within a specified clearance.
     * 
     * This method checks if a point is clear of obstacles within a specified clearance distance.
     * This is useful for ensuring that a point is not only unoccupied but also has a safe distance from obstacles.
     * 
     * @param point The point to check for clearance.
     * @param clearance The clearance distance to check against obstacles.
     * @return true if the point is clear of obstacles, false otherwise.
     */
    virtual bool isClearance(const Eigen::VectorXd& point, double clearance) const = 0;

    /**
     * @brief Get the bounds of the mapping.
     * 
     * This method retrieves the bounds of the mapping, which can be used for various purposes such as visualization or planning.
     * 
     * @return A matrix representing the bounds of the mapping.
     */
    virtual Eigen::MatrixXd getMapBounds() const = 0;


    /***************** CAPABILITY Registry *******************/
    /**
     * @brief Register a capability in the mapping system.
     * 
     * This method allows for registering a capability that can be used to extend the functionalities of the mapping system.
     * 
     * @tparam T The type of the capability to register.
     * @param cap A shared pointer to the capability to register.
     */
    template <typename T>
    void registerCapability(std::shared_ptr<T> cap)
    {
        capabilities_[std::type_index(typeid(T))] = cap;
    }

    /**
     * @brief Get a capability from the mapping system.
     * 
     * This method retrieves a capability of the specified type from the mapping system.
     * If the capability is not registered, it returns a null pointer.
     * 
     * @tparam T The type of the capability to retrieve.
     * @return A shared pointer to the capability if it exists, or nullptr if it does not.
     */
    template <typename T>
    std::shared_ptr<T> getCapability() const
    {
        auto it = capabilities_.find(std::type_index(typeid(T)));
        if (it != capabilities_.end())
            return std::static_pointer_cast<T>(it->second);
        
        return nullptr;
    }

private:
    std::unordered_map<std::type_index, std::shared_ptr<ICapability>> capabilities_;

}; // class IMapping

}; // namespace arena_core
