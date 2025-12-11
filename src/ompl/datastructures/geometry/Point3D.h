/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Servet Bora Bayraktar */

#ifndef OMPL_DATASTRUCTURES_GEOMETRY_POINT3D_H
#define OMPL_DATASTRUCTURES_GEOMETRY_POINT3D_H

#include <cmath>
#include <stdexcept>

namespace ompl
{
    namespace geometry
    {
        /** @brief 3D point with vector operations and metadata for sampling */
        struct Point3D
        {
            double x, y, z;
            
            // Sampling metadata
            double sampledAtRadius{0.0};
            bool valid{false};
            bool sampledByCylinder{false};
            double sampledAtKappa{0.0};
            int componentID{-1};

            // Constructors
            Point3D() : x(0.0), y(0.0), z(0.0) {}
            Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
            Point3D(double x_, double y_, double z_, double radius_) 
                : x(x_), y(y_), z(z_), sampledAtRadius(radius_) {}
            Point3D(double x_, double y_, double z_, double radius_, bool cylinderSample) 
                : x(x_), y(y_), z(z_), sampledAtRadius(radius_), sampledByCylinder(cylinderSample) {}
            Point3D(double x_, double y_, double z_, double radius_, double kappa_, int componentID_) 
                : x(x_), y(y_), z(z_), sampledAtRadius(radius_), sampledAtKappa(kappa_), componentID(componentID_) {}

            // Arithmetic operators
            Point3D operator+(const Point3D& other) const { return{x + other.x, y + other.y, z + other.z}; }
            Point3D operator-(const Point3D& other) const { return {x - other.x, y - other.y, z - other.z}; }
            Point3D operator*(double scalar) const { return {x * scalar, y * scalar, z * scalar}; }
            Point3D operator/(double scalar) const { return {x / scalar, y / scalar, z / scalar}; }

            // Assignment operator
            Point3D(const Point3D&) = default;
            Point3D& operator=(const Point3D& other)
            {
                if (this != &other)
                {
                    x = other.x;
                    y = other.y;
                    z = other.z;
                    sampledAtRadius = other.sampledAtRadius;
                    sampledByCylinder = other.sampledByCylinder;
                    sampledAtKappa = other.sampledAtKappa;
                    componentID = other.componentID;
                    valid = other.valid;
                }
                return *this;
            }

            // Subscript operators
            double operator[](std::size_t index) const
            {
                if (index == 0) return x;
                if (index == 1) return y;
                if (index == 2) return z;
                throw std::out_of_range("Index out of range. Point3D only has 3 components (x, y, z).");
            }

            double& operator[](std::size_t index)
            {
                if (index == 0) return x;
                if (index == 1) return y;
                if (index == 2) return z;
                throw std::out_of_range("Index out of range. Point3D only has 3 components (x, y, z).");
            }

            // Geometric operations
            double magnitude() const { return std::sqrt(x * x + y * y + z * z); }
            double dot(const Point3D& other) const { return x * other.x + y * other.y + z * other.z; }
            double distanceTo(const Point3D& other) const { return (*this - other).magnitude(); }
            
            Point3D normalized() const
            {
                double mag = magnitude();
                if (mag < 1e-12) {
                    return Point3D{0.0, 0.0, 0.0}; // Return zero vector for degenerate case
                }
                return {x / mag, y / mag, z / mag};
            }
        };
    }
}

#endif // OMPL_DATASTRUCTURES_GEOMETRY_POINT3D_H
