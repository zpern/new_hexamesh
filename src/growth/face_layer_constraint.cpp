#include <boundary_mesh/growth/face_layer_constraint.hpp>

#include <algorithm>
#include <limits>
#include <variant>

namespace boundary_mesh
{
    namespace
    {
        std::vector<VertexId> faceVertexIds(const SurfaceFace &face)
        {
            return std::visit(
                [](const auto &value)
                {
                    return std::vector<VertexId>{
                        value.vertex_ids.begin(),
                        value.vertex_ids.end()};
                },
                face);
        }
    }

    const FaceLayerConstraint *FaceLayerConstraintTable::find(
        SurfaceFaceId source_face_id) const noexcept
    {
        const auto found = std::lower_bound(
            entries_.begin(),
            entries_.end(),
            source_face_id,
            [](const FaceLayerConstraint &entry, SurfaceFaceId id)
            {
                return entry.source_face_id < id;
            });
        return found != entries_.end() &&
                       found->source_face_id == source_face_id
            ? &*found
            : nullptr;
    }

    FaceLayerConstraint *FaceLayerConstraintTable::find(
        SurfaceFaceId source_face_id) noexcept
    {
        return const_cast<FaceLayerConstraint *>(
            static_cast<const FaceLayerConstraintTable *>(this)->find(
                source_face_id));
    }

    const std::vector<FaceLayerConstraint> &
    FaceLayerConstraintTable::entries() const noexcept
    {
        return entries_;
    }

    Result<FaceLayerConstraintTable, InvalidFaceConstraintState>
    buildFaceLayerConstraints(
        const GrowthPatch &patch,
        const GrowthFront &initial_front,
        const GrowthProfileTable &profiles)
    {
        using ConstraintResult = Result<
            FaceLayerConstraintTable,
            InvalidFaceConstraintState>;
        if (initial_front.layer != 0 ||
            initial_front.faces.size() !=
                initial_front.source_face_ids.size())
        {
            return ConstraintResult::failure({0, 0});
        }

        FaceLayerConstraintTable table;
        for (std::size_t face_index = 0;
             face_index < initial_front.faces.size();
             ++face_index)
        {
            const SurfaceFaceId source_face_id =
                initial_front.source_face_ids[face_index];
            if (std::find(
                    patch.sourceFaceIds().begin(),
                    patch.sourceFaceIds().end(),
                    source_face_id) == patch.sourceFaceIds().end())
            {
                return ConstraintResult::failure({source_face_id, 0});
            }
            std::uint32_t requested =
                std::numeric_limits<std::uint32_t>::max();
            for (const VertexId local_id :
                 faceVertexIds(initial_front.faces[face_index]))
            {
                const std::size_t local_index =
                    static_cast<std::size_t>(local_id);
                if (local_index >= initial_front.vertices.size())
                {
                    return ConstraintResult::failure({source_face_id, 0});
                }
                const VertexGrowthProfile *profile = profiles.find(
                    initial_front.vertices[local_index].source_vertex_id);
                if (profile == nullptr)
                {
                    return ConstraintResult::failure({source_face_id, 0});
                }
                requested = std::min(requested, profile->layer_count);
            }
            const auto existing = std::find_if(
                table.entries_.begin(), table.entries_.end(),
                [&](const FaceLayerConstraint &entry)
                {
                    return entry.source_face_id == source_face_id;
                });
            if (existing == table.entries_.end())
            {
                table.entries_.push_back(
                    {source_face_id,
                     requested,
                     requested,
                     FaceLayerLimitKind::Requested,
                     FaceStopReason::None});
            }
            else
            {
                existing->requested_layer_count = std::min(
                    existing->requested_layer_count, requested);
                existing->allowed_layer_count = std::min(
                    existing->allowed_layer_count, requested);
            }
        }
        std::sort(
            table.entries_.begin(),
            table.entries_.end(),
            [](const FaceLayerConstraint &left,
               const FaceLayerConstraint &right)
            {
                return left.source_face_id < right.source_face_id;
            });
        return ConstraintResult::success(std::move(table));
    }
}
