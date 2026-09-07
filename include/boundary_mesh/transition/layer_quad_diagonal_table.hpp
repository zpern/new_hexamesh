#pragma once

#include <cstdint>
#include <optional>
#include <variant>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/transition/quad_diagonal.hpp>

namespace boundary_mesh
{
    struct LayerQuadFaceKey
    {
        SurfaceFaceId source_face_id{};
        std::uint32_t layer{};
    };

    struct ConflictingLayerQuadDiagonal
    {
        LayerQuadFaceKey key;
        QuadDiagonal registered{};
        QuadDiagonal requested{};
    };

    using LayerQuadDiagonalError = std::variant<
        ConflictingLayerQuadDiagonal,
        FaceEvaluationError>;

    class LayerQuadDiagonalTable
    {
    public:
        Result<QuadDiagonal, LayerQuadDiagonalError> resolve(
            LayerQuadFaceKey key,
            const OrientedQuad &quad,
            std::optional<QuadDiagonal> forced,
            Scalar length_tolerance);

    private:
        struct Entry
        {
            LayerQuadFaceKey key;
            QuadDiagonal diagonal{};
        };
        std::vector<Entry> entries_;
    };
}
