#pragma once

#include <cstdint>
#include <filesystem>
#include <map>

#include <boundary_mesh/mesh/mesh_surface.hpp>

namespace boundary_mesh::test
{
    struct FaceGrowthParameters
    {
        std::uint32_t layer_count{};
        Scalar first_height{};
        Scalar growth_ratio{};
    };

    struct BoundaryMeshingFixture
    {
        SurfaceMesh mesh;
        std::uint32_t layer_count{};
        Scalar first_height{};
        Scalar growth_ratio{};
        Scalar maximum_prism_skewness{};
        Scalar maximum_pyramid_skewness{};
        Scalar maximum_ratio_difference{};
        Scalar isotropic_stop{};
        bool use_multiple_normals{};
        std::map<std::uint32_t, FaceGrowthParameters> face_parameters;
    };

    BoundaryMeshingFixture readBoundaryMeshingFixture(
        const std::filesystem::path &path);
}
