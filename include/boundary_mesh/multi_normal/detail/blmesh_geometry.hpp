#pragma once

#include <vector>

#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh::detail
{
    Scalar blmeshMinCos(
        const Vector3 &direction,
        const std::vector<Vector3> &neighbor_normals);

    Vector3 blmeshCircleCenter(
        const Vector3 &first,
        const Vector3 &second,
        const Vector3 &third);

    Vector3 blmeshMostNormal(
        const std::vector<Vector3> &neighbor_normals);
}
