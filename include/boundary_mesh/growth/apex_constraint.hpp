#pragma once

#include <vector>

#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh
{
    struct ApexConstraint
    {
        Vector3 normal{Vector3::Zero()};
        Scalar offset{};
    };

    struct ApexFeasibleRegion
    {
        std::vector<ApexConstraint> constraints;

        bool contains(const Point3 &point, Scalar tolerance = 1e-10) const;
        Scalar minimumSlack(const Point3 &point) const;
    };
}
