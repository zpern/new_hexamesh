#pragma once

#include <cstdint>
#include <variant>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/mesh/mesh_surface.hpp>
#include <boundary_mesh/mesh/mesh_volume.hpp>
#include <boundary_mesh/surface/face_evaluation.hpp>
#include <boundary_mesh/transition/quad_diagonal.hpp>

namespace boundary_mesh
{
    struct TransitionTemplateOutput
    {
        SurfaceFaceId source_face_id{};
        QuadDiagonal low_diagonal{};
        std::vector<Point3> created_vertices;
        std::vector<VolumeCell> volume_cells;
        std::vector<CellMetadata> metadata;
        std::vector<Triangle> top_faces;
    };

    struct InvalidTransitionTemplateInput
    {
        SurfaceFaceId source_face_id{};
        std::uint32_t stage{};
    };

    using TransitionTemplateError = std::variant<
        InvalidTransitionTemplateInput,
        FaceEvaluationError>;

    using TransitionTemplateResult = Result<
        TransitionTemplateOutput,
        TransitionTemplateError>;
}
