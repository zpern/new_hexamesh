#pragma once

#include <cstddef>

#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh
{
    struct SkewnessNormalOptimizationOptions
    {
        bool enabled{true};
        Scalar activation_skewness{0.8};
        Scalar first_angle_degrees{5};
        Scalar second_angle_degrees{2};
        std::size_t azimuth_samples{6};
        std::size_t maximum_levels{2};
        Scalar improvement_tolerance{1e-12};
    };

    struct GrowthFieldSmoothingOptions
    {
        SkewnessNormalOptimizationOptions skewness;
    };

    struct GrowthFieldSmoothingDiagnostics
    {
        std::size_t activated_vertices{};
        std::size_t updated_vertices{};
        Scalar maximum_skewness_before{};
        Scalar maximum_skewness_after{};
    };
}
