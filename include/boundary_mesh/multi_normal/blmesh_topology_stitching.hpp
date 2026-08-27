#pragma once

#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/multi_normal/multi_normal_error.hpp>

namespace boundary_mesh
{
    struct BlmeshDirectedTriangle
    {
        int triangle_index{};
        int start_point{};
        int end_point{};
    };

    Result<std::vector<BlmeshDirectedTriangle>, MultiNormalError>
    orderBlmeshDirectedTriangleChain(
        const std::vector<BlmeshDirectedTriangle> &triangles);

    Result<std::vector<int>, MultiNormalError>
    findBlmeshSmoothestInterleaving(
        const std::vector<Vector3> &left_normals,
        const std::vector<Vector3> &right_normals);
}
