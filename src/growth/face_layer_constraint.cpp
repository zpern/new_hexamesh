#include <boundary_mesh/growth/face_layer_constraint.hpp>

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <variant>

namespace boundary_mesh
{
    namespace
    {
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
        std::unordered_set<SurfaceFaceId> patch_faces;
        patch_faces.reserve(patch.sourceFaceIds().size());
        for (const SurfaceFaceId id : patch.sourceFaceIds())
            patch_faces.insert(id);

        table.entries_.reserve(initial_front.faces.size());
        std::unordered_map<SurfaceFaceId, std::size_t> entry_indices;
        entry_indices.reserve(initial_front.faces.size());
        for (std::size_t face_index = 0;
             face_index < initial_front.faces.size();
             ++face_index)
        {
            const SurfaceFaceId source_face_id =
                initial_front.source_face_ids[face_index];
            if (patch_faces.find(source_face_id) == patch_faces.end())
            {
                return ConstraintResult::failure({source_face_id, 0});
            }
            std::uint32_t requested =
                std::numeric_limits<std::uint32_t>::max();
            bool valid = true;
            std::visit(
                [&](const auto &face)
                {
                    for (const VertexId local_id : face.vertex_ids)
                    {
                        const std::size_t local_index =
                            static_cast<std::size_t>(local_id);
                        if (local_index >= initial_front.vertices.size())
                        {
                            valid = false;
                            return;
                        }
                        const VertexGrowthProfile *profile = profiles.find(
                            initial_front.vertices[local_index]
                                .source_vertex_id);
                        if (profile == nullptr)
                        {
                            valid = false;
                            return;
                        }
                        requested = std::min(
                            requested, profile->layer_count);
                    }
                },
                initial_front.faces[face_index]);
            if (!valid)
                return ConstraintResult::failure({source_face_id, 0});

            const auto existing = entry_indices.find(source_face_id);
            if (existing == entry_indices.end())
            {
                entry_indices.emplace(source_face_id, table.entries_.size());
                table.entries_.push_back(
                    {source_face_id,
                     requested,
                     requested,
                     FaceLayerLimitKind::Requested,
                     FaceStopReason::None});
            }
            else
            {
                FaceLayerConstraint &entry =
                    table.entries_[existing->second];
                entry.requested_layer_count = std::min(
                    entry.requested_layer_count, requested);
                entry.allowed_layer_count = std::min(
                    entry.allowed_layer_count, requested);
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
