#pragma once

#include <array>
#include <cstdint>
#include <map>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/spatial/spatial_error.hpp>
#include <boundary_mesh/spatial/triangle_contact.hpp>

namespace boundary_mesh
{
    struct SlidingContactPermission
    {
        std::uint8_t vertex_mask{};
        std::uint8_t edge_mask{};
        bool complete_face_exemption{};
    };

    enum class SlidingIntersectionKind
    {
        Empty,
        Points,
        Segments,
        CoplanarArea
    };

    struct SlidingIntersectionGeometry
    {
        SlidingIntersectionKind kind{SlidingIntersectionKind::Empty};
        std::vector<Point3> points;
        std::vector<std::array<Point3, 2>> segments;
    };

    SlidingIntersectionGeometry classifySlidingIntersection(
        const TrianglePoints &candidate,
        const TrianglePoints &surface);

    Result<bool, SpatialError> hasInvalidSlidingIntersection(
        const TrianglePoints &candidate,
        const TrianglePoints &surface,
        const SlidingContactPermission &permission);

    std::map<std::uint32_t, SlidingContactPermission>
    buildSlidingContactPermissions(
        const std::array<std::vector<std::uint32_t>, 3> &node_associations,
        std::uint8_t physical_edge_mask,
        const std::vector<std::uint32_t> &complete_exemptions);

    bool slidingSideValuesStayOnOneSide(
        const std::vector<Scalar> &values,
        Scalar tolerance);
}
