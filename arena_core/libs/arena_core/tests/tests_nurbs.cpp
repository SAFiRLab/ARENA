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

#include <gtest/gtest.h>
#include "arena_core/math/nurbs.h"
#include "arena_core/spaces/control_point.h"

using namespace arena_core;

// Fixture for setting up Nurbs tests
class NurbsTest : public ::testing::Test
{
protected:
    std::vector<ControlPoint<double>> control_points;

    void SetUp() override
    {
        Eigen::VectorXd p1(2);
        p1 << 0, 0;
        Eigen::VectorXd p2(2);
        p2 << 1, 2;
        Eigen::VectorXd p3(2);
        p3 << 2, 0;
        Eigen::VectorXd p4(2);
        p4 << 3, -1;
        Eigen::VectorXd p5(2);
        p5 << 4, 0;
        control_points.push_back(ControlPoint<double>(p1));
        control_points.push_back(ControlPoint<double>(p2));
        control_points.push_back(ControlPoint<double>(p3));
        control_points.push_back(ControlPoint<double>(p4));
        control_points.push_back(ControlPoint<double>(p5));
    }
};

TEST_F(NurbsTest, InitializationWithValidData)
{
    Nurbs nurbs(control_points, 50, 3);
    EXPECT_EQ(nurbs.getControlPoints().size(), control_points.size());
    EXPECT_EQ(nurbs.getDegree(), 3);
    EXPECT_EQ(nurbs.getSampleSize(), 50);
}

TEST_F(NurbsTest, InitializationFailsWithEmptyControlPoints)
{
    std::vector<ControlPoint<double>> empty;
    Nurbs nurbs(empty, 10, 3);
    // Can't assert directly, but check configuration flag indirectly through evaluate
    EXPECT_EQ(nurbs.evaluate(), nullptr);
}

TEST_F(NurbsTest, DegreeOutOfRange)
{
    Nurbs nurbs(control_points, 50, 3);
    EXPECT_FALSE(nurbs.setDegree(0)); // Invalid degree
    EXPECT_FALSE(nurbs.setDegree(6)); // Invalid degree
}

TEST_F(NurbsTest, SampleSizeInvalid)
{
    Nurbs nurbs(control_points, 50, 3);
    EXPECT_FALSE(nurbs.setSampleSize(0));
    EXPECT_FALSE(nurbs.setSampleSize(-10));
}

TEST_F(NurbsTest, SetControlPointAtValidIndex)
{
    Nurbs nurbs(control_points, 50, 3);
    Eigen::VectorXd p(2);
    p << 10.0, 10.0;
    ControlPoint<double> cp(p);
    EXPECT_TRUE(nurbs.setControlPoint(cp, 2));
    EXPECT_EQ(nurbs.getControlPoints()[2][0], 10.0);
    EXPECT_EQ(nurbs.getControlPoints()[2][1], 10.0);
}

TEST_F(NurbsTest, SetControlPointInvalidIndex)
{
    Nurbs nurbs(control_points, 50, 3);
    Eigen::VectorXd p(2);
    p << 1.0, 1.0;
    ControlPoint<double> cp(p);
    EXPECT_EQ(nurbs.getControlPoints().size(), control_points.size());
    EXPECT_FALSE(nurbs.setControlPoint(cp, -1));
    EXPECT_FALSE(nurbs.setControlPoint(cp, control_points.size()));
    EXPECT_EQ(nurbs.getControlPoints().size(), control_points.size());
}

TEST_F(NurbsTest, SetWeightValid)
{
    Nurbs nurbs(control_points, 50, 3);
    EXPECT_EQ(nurbs.getControlPoints()[0].w(), 1.0); // Default weight
    EXPECT_TRUE(nurbs.setWeight(2.0, 0));
    EXPECT_EQ(nurbs.getControlPoints()[0].w(), 2.0);
}

TEST_F(NurbsTest, SetWeightInvalid)
{
    Nurbs nurbs(control_points, 50, 3);
    EXPECT_EQ(nurbs.getControlPoints()[0].w(), 1.0); // Default weight
    EXPECT_FALSE(nurbs.setWeight(0.0, 0));
    EXPECT_FALSE(nurbs.setWeight(1.0, control_points.size()));
    EXPECT_EQ(nurbs.getControlPoints()[0].w(), 1.0); // Weight should not change
}

TEST_F(NurbsTest, EvaluateReturnsValidPointer)
{
    Nurbs nurbs(control_points, 20, 3);
    Eigen::VectorXd* result = nurbs.evaluate();
    EXPECT_NE(result, nullptr);
    delete[] result;
}

TEST_F(NurbsTest, DerivativesReturnCorrectShape)
{
    Nurbs nurbs(control_points, 10, 3);
    auto ders = nurbs.derivatives(1);
    EXPECT_EQ(ders.size(), 10);
    for (const auto& level : ders) {
        EXPECT_GE(level.size(), 1);
        for (const auto& vec : level) {
            EXPECT_GE(vec.size(), 1);
        }
    }
}

// You can add more detailed tests for basis function evaluation and derivatives as needed.