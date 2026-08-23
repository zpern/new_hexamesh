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

        std::vector<VertexId> faceVertexIds(const SurfaceFace &face)
        {
            return std::visit(
                [](const auto &value)
                {
                    return std::vector<VertexId>{
                        value.vertex_ids.begin(), value.vertex_ids.end()};
                },
                face);
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
    TerminationPropagator::applyDirectStops(
        FaceLayerConstraintTable &constraints,
        const std::vector<FaceStopEvent> &events,
        std::uint32_t max_difference) const
    {
        for (const FaceStopEvent &event : events)
        {
            if (event.layer == 0 ||
                event.reason == FaceStopReason::None ||
                constraints.find(event.source_face_id) == nullptr)
            {
                return Result<std::vector<SurfaceFaceId>, InvalidFaceConstraintState>::failure(
                    {event.source_face_id, event.layer});
            }
        }

        std::vector<SurfaceFaceId> changed;
        for (const FaceStopEvent &event : events)
        {
            FaceLayerConstraint *constraint =
                constraints.find(event.source_face_id);
            const std::uint32_t limit = event.layer - 1;
            if (limit <= constraint->allowed_layer_count)
            {
                if (limit < constraint->allowed_layer_count)
                {
                    constraint->allowed_layer_count = limit;
                    changed.push_back(event.source_face_id);
                }
                constraint->limit_kind = FaceLayerLimitKind::DirectStop;
                constraint->direct_reason = event.reason;
            }
        }

        const auto propagated = propagate(constraints, max_difference);
        if (!propagated.hasValue())
        {
            return propagated;
        }
        changed.insert(
            changed.end(),
            propagated.value().begin(),
            propagated.value().end());
        std::sort(changed.begin(), changed.end());
        changed.erase(
            std::unique(changed.begin(), changed.end()),
            changed.end());
        return Result<std::vector<SurfaceFaceId>, InvalidFaceConstraintState>::success(
            std::move(changed));
    }

    Result<LayerStepResult, InvalidFaceConstraintState>
    TerminationPropagator::filterCandidates(
        const GrowthFront &current_front,
        const LayerStepResult &step,
        const FaceLayerConstraintTable &constraints) const
    {
        using FilterResult =
            Result<LayerStepResult, InvalidFaceConstraintState>;
        if (step.next_front.faces.size() !=
                step.previous_front_face_indices.size() ||
            step.next_front.vertices.size() !=
                step.previous_front_vertex_indices.size())
        {
            return FilterResult::failure({0, step.layer});
        }

        LayerStepResult output;
        output.layer = step.layer;
        output.next_front.layer = step.next_front.layer;
        output.stopped_faces = step.stopped_faces;
        output.completed_faces = step.completed_faces;
        std::vector<bool> keep(step.next_front.faces.size(), true);
        std::vector<bool> used(step.next_front.vertices.size(), false);

        for (std::size_t face_index = 0;
             face_index < step.next_front.faces.size();
             ++face_index)
        {
            const SurfaceFaceId source_face_id =
                step.next_front.source_face_ids[face_index];
            const FaceLayerConstraint *constraint =
                constraints.find(source_face_id);
            if (constraint == nullptr)
            {
                return FilterResult::failure(
                    {source_face_id, step.layer});
            }
            if (constraint->allowed_layer_count < step.layer)
            {
                keep[face_index] = false;
                if (constraint->limit_kind ==
                    FaceLayerLimitKind::NeighborConstraint)
                {
                    const bool already_present = std::any_of(
                        output.stopped_faces.begin(),
                        output.stopped_faces.end(),
                        [&](const FaceStopEvent &event)
                        {
                            return event.source_face_id == source_face_id;
                        });
                    if (!already_present)
                    {
                        output.stopped_faces.push_back(
                            {step.previous_front_face_indices[face_index],
                             source_face_id,
                             step.layer,
                             FaceStopReason::NeighborLayerConstraint});
                    }
                }
                continue;
            }
            for (const VertexId id :
                 faceVertexIds(step.next_front.faces[face_index]))
            {
                const std::size_t vertex_index =
                    static_cast<std::size_t>(id);
                if (vertex_index >= used.size())
                {
                    return FilterResult::failure(
                        {source_face_id, step.layer});
                }
                used[vertex_index] = true;
            }
        }

        std::vector<VertexId> remap(
            step.next_front.vertices.size(), VertexId{});
        for (std::size_t vertex = 0;
             vertex < step.next_front.vertices.size();
             ++vertex)
        {
            if (!used[vertex]) continue;
            remap[vertex] = static_cast<VertexId>(
                output.next_front.vertices.size());
            output.next_front.vertices.push_back(
                step.next_front.vertices[vertex]);
            output.previous_front_vertex_indices.push_back(
                step.previous_front_vertex_indices[vertex]);
        }

        for (std::size_t face_index = 0;
             face_index < step.next_front.faces.size();
             ++face_index)
        {
            if (!keep[face_index]) continue;
            output.next_front.faces.push_back(std::visit(
                [&](const auto &value) -> SurfaceFace
                {
                    auto face = value;
                    for (VertexId &id : face.vertex_ids)
                    {
                        id = remap[static_cast<std::size_t>(id)];
                    }
                    return face;
                },
                step.next_front.faces[face_index]));
            output.next_front.source_face_ids.push_back(
                step.next_front.source_face_ids[face_index]);
            output.previous_front_face_indices.push_back(
                step.previous_front_face_indices[face_index]);
        }
        for (const FaceStopEvent &event :
             step.accepted_stopped_faces)
        {
            if (std::find(
                    output.next_front.source_face_ids.begin(),
                    output.next_front.source_face_ids.end(),
                    event.source_face_id) !=
                output.next_front.source_face_ids.end())
            {
                output.accepted_stopped_faces.push_back(event);
            }
        }
        std::sort(
            output.stopped_faces.begin(),
            output.stopped_faces.end(),
            [](const FaceStopEvent &left, const FaceStopEvent &right)
            {
                return left.source_face_id < right.source_face_id;
            });
        return FilterResult::success(std::move(output));
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
