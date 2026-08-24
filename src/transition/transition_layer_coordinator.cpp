#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include <boundary_mesh/transition/transition_layer_coordinator.hpp>

namespace boundary_mesh
{
    namespace
    {
        struct Neighbor
        {
            std::size_t local_edge{};
            SurfaceFaceId source_face_id{};
        };

        struct WorkingFace
        {
            FaceLayerState layers;
            std::vector<Neighbor> neighbors;
        };

        WorkingFace *findFace(
            std::vector<WorkingFace> &faces,
            SurfaceFaceId id)
        {
            const auto found = std::lower_bound(
                faces.begin(), faces.end(), id,
                [](const WorkingFace &face, SurfaceFaceId value)
                { return face.layers.source_face_id < value; });
            return found != faces.end() &&
                           found->layers.source_face_id == id
                       ? &*found
                       : nullptr;
        }

        const WorkingFace *findFace(
            const std::vector<WorkingFace> &faces,
            SurfaceFaceId id)
        {
            const auto found = std::lower_bound(
                faces.begin(), faces.end(), id,
                [](const WorkingFace &face, SurfaceFaceId value)
                { return face.layers.source_face_id < value; });
            return found != faces.end() &&
                           found->layers.source_face_id == id
                       ? &*found
                       : nullptr;
        }

        void updateCounts(WorkingFace &face)
        {
            face.layers.regular_layers =
                regularLayerCount(face.layers.trial_layers);
            face.layers.occupied_layers =
                occupiedLayerCount(face.layers.trial_layers);
        }

        std::vector<SurfaceFaceId> neighborIds(
            const FaceNeighborIds &neighbors)
        {
            return std::visit(
                [](const auto &ids)
                {
                    return std::vector<SurfaceFaceId>(
                        ids.begin(), ids.end());
                },
                neighbors);
        }
    }

    Result<std::vector<CoordinatedTransitionFace>,
           TransitionCoordinationError>
    TransitionLayerCoordinator::coordinate(
        const GrowthPatch &patch,
        const SurfaceTopology &topology,
        const std::vector<std::pair<SurfaceFaceId, std::uint32_t>>
            &trial_layers) const
    {
        using CoordinationResult = Result<
            std::vector<CoordinatedTransitionFace>,
            TransitionCoordinationError>;

        auto sorted = trial_layers;
        std::sort(sorted.begin(), sorted.end());
        for (std::size_t index = 1; index < sorted.size(); ++index)
        {
            if (sorted[index - 1].first == sorted[index].first)
            {
                return CoordinationResult::failure(
                    DuplicateTransitionFaceState{sorted[index].first});
            }
        }

        std::vector<WorkingFace> faces;
        faces.reserve(patch.sourceFaceIds().size());
        for (const SurfaceFaceId source_id : patch.sourceFaceIds())
        {
            const auto found = std::lower_bound(
                sorted.begin(), sorted.end(), source_id,
                [](const auto &entry, SurfaceFaceId value)
                { return entry.first < value; });
            if (found == sorted.end() || found->first != source_id)
            {
                return CoordinationResult::failure(
                    MissingTransitionFaceState{source_id});
            }
            WorkingFace face;
            face.layers.source_face_id = source_id;
            face.layers.trial_layers = found->second;
            updateCounts(face);
            faces.push_back(std::move(face));
        }

        for (WorkingFace &face : faces)
        {
            const std::size_t face_index =
                static_cast<std::size_t>(face.layers.source_face_id);
            if (face_index >= topology.faceNeighbors().size())
            {
                return CoordinationResult::failure(
                    MissingTransitionFaceState{
                        face.layers.source_face_id});
            }
            const auto ids = neighborIds(
                topology.faceNeighbors()[face_index]);
            for (std::size_t local = 0; local < ids.size(); ++local)
            {
                if (findFace(faces, ids[local]) != nullptr)
                {
                    face.neighbors.push_back(Neighbor{local, ids[local]});
                }
            }
            std::sort(
                face.neighbors.begin(), face.neighbors.end(),
                [](const Neighbor &left, const Neighbor &right)
                { return left.source_face_id < right.source_face_id; });
        }

        bool changed = true;
        while (changed)
        {
            changed = false;
            for (WorkingFace &face : faces)
            {
                for (const Neighbor &neighbor_entry : face.neighbors)
                {
                    WorkingFace *neighbor = findFace(
                        faces, neighbor_entry.source_face_id);
                    if (neighbor == nullptr ||
                        face.layers.occupied_layers >=
                            neighbor->layers.occupied_layers)
                    {
                        continue;
                    }
                    if (neighbor->layers.occupied_layers >
                        face.layers.occupied_layers + 1)
                    {
                        neighbor->layers.trial_layers =
                            face.layers.occupied_layers + 2;
                        updateCounts(*neighbor);
                        changed = true;
                    }
                }
            }

            for (WorkingFace &face : faces)
            {
                std::vector<WorkingFace *> high;
                for (const Neighbor &neighbor_entry : face.neighbors)
                {
                    WorkingFace *neighbor = findFace(
                        faces, neighbor_entry.source_face_id);
                    if (neighbor != nullptr &&
                        neighbor->layers.occupied_layers ==
                            face.layers.occupied_layers + 1)
                    {
                        high.push_back(neighbor);
                    }
                }
                for (std::size_t index = 1; index < high.size(); ++index)
                {
                    high[index]->layers.trial_layers =
                        face.layers.occupied_layers + 1;
                    updateCounts(*high[index]);
                    changed = true;
                }
            }
        }

        std::vector<CoordinatedTransitionFace> result;
        result.reserve(faces.size());
        for (const WorkingFace &face : faces)
        {
            CoordinatedTransitionFace output;
            output.layers = face.layers;
            for (const Neighbor &neighbor_entry : face.neighbors)
            {
                const WorkingFace *neighbor = findFace(
                    faces, neighbor_entry.source_face_id);
                if (neighbor != nullptr &&
                    neighbor->layers.occupied_layers ==
                        face.layers.occupied_layers + 1)
                {
                    output.high_edge_local_index =
                        neighbor_entry.local_edge;
                    break;
                }
            }
            result.push_back(output);
        }
        return CoordinationResult::success(std::move(result));
    }
}
