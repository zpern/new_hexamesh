#pragma once

#include <array>
#include <optional>

#include <boundary_mesh/growth/apex_constraint.hpp>

namespace boundary_mesh
{
    struct ApexSolverInput
    {
        std::array<Point3, 4> quad{};
        Point3 target{Point3::Zero()};
        Scalar minimum_height_ratio{1e-5};
        Scalar maximum_height_ratio{2};
        Scalar lateral_radius_ratio{2};
        std::vector<ApexConstraint> additional_constraints;
    };

    enum class ApexSolverStatus
    {
        Valid,
        InvalidInput,
        EmptyFeasibleRegion,
        QualityRejected
    };

    struct ApexSolverResult
    {
        ApexSolverStatus status{ApexSolverStatus::InvalidInput};
        ApexFeasibleRegion region;
        std::optional<Point3> apex;
        Scalar minimum_slack{};
    };

    ApexSolverResult solvePyramidApex(const ApexSolverInput &input);
    std::vector<Point3> projectPyramidApexToPlaneSides(
        const ApexSolverInput &input,
        const Point3 &target,
        const std::array<Point3,3> &plane,
        std::size_t maximum_candidates = 2);
}
