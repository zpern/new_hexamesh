#include <boundary_mesh/growth/termination_propagator.hpp>

#include <algorithm>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <variant>

namespace boundary_mesh
{
    namespace
    {
        const std::vector<SurfaceFaceId> empty_neighbors;

        bool containsFace(
            const std::vector<SurfaceFaceId> &faces,
            SurfaceFaceId id)
        {
            return std::binary_search(faces.begin(), faces.end(), id);
        }

        std::vector<SurfaceFaceId> faceNeighbors(
            const FaceNeighborIds &neighbors)
        {
            return std::visit(
                [](const auto &value)
                {
                    return std::vector<SurfaceFaceId>{
                        value.begin(), value.end()};
                },
                neighbors);
        }
    }

    Result<TerminationPropagator, InvalidFaceConstraintState>
    TerminationPropagator::build(
        const GrowthPatch &patch,
        const SurfaceTopology &topology)
    {
        TerminationPropagator propagator;
        const std::vector<SurfaceFaceId> &patch_faces =
            patch.sourceFaceIds();
        for (const SurfaceFaceId face_id : patch_faces)
        {
            const std::size_t face_index =
                static_cast<std::size_t>(face_id);
            if (face_index >= topology.faceNeighbors().size())
            {
                return Result<TerminationPropagator, InvalidFaceConstraintState>::failure(
                    {face_id, 0});
            }
            NeighborEntry entry;
            entry.source_face_id = face_id;
            for (const SurfaceFaceId neighbor : faceNeighbors(
                     topology.faceNeighbors()[face_index]))
            {
                if (neighbor != face_id &&
                    containsFace(patch_faces, neighbor))
                {
                    entry.neighbors.push_back(neighbor);
                }
            }
            std::sort(entry.neighbors.begin(), entry.neighbors.end());
            entry.neighbors.erase(
                std::unique(entry.neighbors.begin(), entry.neighbors.end()),
                entry.neighbors.end());
            propagator.entries_.push_back(std::move(entry));
        }
        std::sort(
            propagator.entries_.begin(),
            propagator.entries_.end(),
            [](const NeighborEntry &left, const NeighborEntry &right)
            {
                return left.source_face_id < right.source_face_id;
            });
        return Result<TerminationPropagator, InvalidFaceConstraintState>::success(
            std::move(propagator));
    }

    const std::vector<SurfaceFaceId> &TerminationPropagator::neighbors(
        SurfaceFaceId source_face_id) const
    {
        const auto found = std::lower_bound(
            entries_.begin(),
            entries_.end(),
            source_face_id,
            [](const NeighborEntry &entry, SurfaceFaceId id)
            {
                return entry.source_face_id < id;
            });
        return found != entries_.end() &&
                       found->source_face_id == source_face_id
            ? found->neighbors
            : empty_neighbors;
    }

    Result<std::vector<SurfaceFaceId>, InvalidFaceConstraintState>
    TerminationPropagator::propagateInitial(
        FaceLayerConstraintTable &constraints,
        std::uint32_t max_difference) const
    {
        return propagate(constraints, max_difference);
    }

    Result<std::vector<SurfaceFaceId>, InvalidFaceConstraintState>
    TerminationPropagator::propagate(
        FaceLayerConstraintTable &constraints,
        std::uint32_t max_difference) const
    {
        using QueueValue = std::pair<std::uint32_t, SurfaceFaceId>;
        std::priority_queue<
            QueueValue,
            std::vector<QueueValue>,
            std::greater<QueueValue>> queue;

        for (const NeighborEntry &entry : entries_)
        {
            const FaceLayerConstraint *constraint =
                constraints.find(entry.source_face_id);
            if (constraint == nullptr)
            {
                return Result<std::vector<SurfaceFaceId>, InvalidFaceConstraintState>::failure(
                    {entry.source_face_id, 0});
            }
            queue.push({
                constraint->allowed_layer_count,
                entry.source_face_id});
        }

        std::vector<SurfaceFaceId> changed;
        while (!queue.empty())
        {
            const auto [queued_limit, face_id] = queue.top();
            queue.pop();
            const FaceLayerConstraint *current = constraints.find(face_id);
            if (current == nullptr ||
                current->allowed_layer_count != queued_limit)
            {
                continue;
            }
            const std::uint64_t candidate =
                static_cast<std::uint64_t>(queued_limit) +
                static_cast<std::uint64_t>(max_difference);
            const std::uint32_t bounded = candidate >
                    std::numeric_limits<std::uint32_t>::max()
                ? std::numeric_limits<std::uint32_t>::max()
                : static_cast<std::uint32_t>(candidate);

            for (const SurfaceFaceId neighbor_id : neighbors(face_id))
            {
                FaceLayerConstraint *neighbor =
                    constraints.find(neighbor_id);
                if (neighbor == nullptr)
                {
                    return Result<std::vector<SurfaceFaceId>, InvalidFaceConstraintState>::failure(
                        {neighbor_id, 0});
                }
                if (neighbor->allowed_layer_count > bounded)
                {
                    neighbor->allowed_layer_count = bounded;
                    if (neighbor->limit_kind != FaceLayerLimitKind::DirectStop)
                    {
                        neighbor->limit_kind =
                            FaceLayerLimitKind::NeighborConstraint;
                    }
                    changed.push_back(neighbor_id);
                    queue.push({bounded, neighbor_id});
                }
            }
        }

        std::sort(changed.begin(), changed.end());
        changed.erase(
            std::unique(changed.begin(), changed.end()),
            changed.end());
        return Result<std::vector<SurfaceFaceId>, InvalidFaceConstraintState>::success(
            std::move(changed));
    }
}
