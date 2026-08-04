#pragma once

#include <cstdint>
#include <Eigen/Core>

namespace boundary_mesh
{
    using Scalar = double;
    using Point3 = Eigen::Vector3d;
    using Vector3 = Eigen::Vector3d;

    using VertexId = std::uint32_t;
    using EdgeId = std::uint32_t;
    using SurfaceFaceId = std::uint32_t;
    using VolumeCellId = std::uint32_t;
}