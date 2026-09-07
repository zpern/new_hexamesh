#include <algorithm>
#include <cstdint>
#include <unordered_map>

#include <boundary_mesh/transition/accepted_stopped_front_carry.hpp>

namespace boundary_mesh
{
    namespace
    {
        std::uint64_t vertexKey(const GrowthFrontVertex &vertex)
        {
            return (static_cast<std::uint64_t>(vertex.source_vertex_id) << 32) |
                   vertex.branch_id;
        }

        std::vector<VertexId> faceVertices(const SurfaceFace &face)
        {
            return std::visit([](const auto &value)
            {
                return std::vector<VertexId>(
                    value.vertex_ids.begin(), value.vertex_ids.end());
            }, face);
        }

        StopOrigin stopOrigin(FaceStopReason reason)
        {
            switch (reason)
            {
            case FaceStopReason::Collision:
                return StopOrigin::Collision;
            case FaceStopReason::SlidingProjectionFailure:
                return StopOrigin::SlidingProjection;
            case FaceStopReason::IsotropicHeightReached:
                return StopOrigin::IsotropicStop;
            case FaceStopReason::VertexLayerLimit:
                return StopOrigin::RequestedLimit;
            default:
                return StopOrigin::Quality;
            }
        }

        SurfaceFace remapFace(
            const SurfaceFace &face,
            const std::vector<VertexId> &remap)
        {
            return std::visit([&](const auto &value) -> SurfaceFace
            {
                auto output = value;
                for (VertexId &id : output.vertex_ids)
                    id = remap[static_cast<std::size_t>(id)];
                return output;
            }, face);
        }
    }

    AcceptedStoppedFrontCarry carryAcceptedStoppedFaces(
        const GrowthFront &accepted_front,
        const std::vector<FaceStopEvent> &accepted_stops,
        const std::vector<SurfaceFaceId> &retained_faces)
    {
        AcceptedStoppedFrontCarry output;
        output.front.layer = accepted_front.layer;
        std::unordered_map<SurfaceFaceId, const FaceStopEvent *> stops;
        for (const FaceStopEvent &event : accepted_stops)
            stops[event.source_face_id] = &event;

        std::vector<bool> used(accepted_front.vertices.size(), false);
        std::vector<std::size_t> kept_faces;
        for (std::size_t index = 0;
             index < accepted_front.faces.size(); ++index)
        {
            const SurfaceFaceId id = accepted_front.source_face_ids[index];
            const auto event = stops.find(id);
            if (event == stops.end() ||
                !std::binary_search(retained_faces.begin(),
                                    retained_faces.end(), id))
                continue;
            kept_faces.push_back(index);
            output.stops.push_back({
                id, accepted_front.layer, stopOrigin(event->second->reason)});
            for (const VertexId vertex : faceVertices(
                     accepted_front.faces[index]))
                used[static_cast<std::size_t>(vertex)] = true;
        }

        std::vector<VertexId> remap(
            accepted_front.vertices.size(), VertexId{});
        for (std::size_t index = 0; index < used.size(); ++index)
            if (used[index])
            {
                remap[index] = static_cast<VertexId>(
                    output.front.vertices.size());
                output.front.vertices.push_back(
                    accepted_front.vertices[index]);
            }
        for (const std::size_t index : kept_faces)
        {
            output.front.faces.push_back(remapFace(
                accepted_front.faces[index], remap));
            output.front.source_face_ids.push_back(
                accepted_front.source_face_ids[index]);
        }
        return output;
    }

    AcceptedStoppedFrontCarry retainAcceptedFaces(
        const GrowthFront &accepted_front,
        const std::vector<FaceStopEvent> &accepted_stops,
        const std::vector<SurfaceFaceId> &retained_faces)
    {
        AcceptedStoppedFrontCarry output;
        output.front.layer = accepted_front.layer;
        std::unordered_map<SurfaceFaceId, const FaceStopEvent *> stops;
        for (const FaceStopEvent &event : accepted_stops)
            stops[event.source_face_id] = &event;
        std::vector<bool> used(accepted_front.vertices.size(), false);
        std::vector<std::size_t> kept_faces;
        for (std::size_t index = 0;
             index < accepted_front.faces.size(); ++index)
        {
            const SurfaceFaceId id = accepted_front.source_face_ids[index];
            if (!std::binary_search(
                    retained_faces.begin(), retained_faces.end(), id))
                continue;
            kept_faces.push_back(index);
            const auto event = stops.find(id);
            if (event != stops.end())
                output.stops.push_back({
                    id, accepted_front.layer,
                    stopOrigin(event->second->reason)});
            for (const VertexId vertex : faceVertices(
                     accepted_front.faces[index]))
                used[static_cast<std::size_t>(vertex)] = true;
        }
        std::vector<VertexId> remap(
            accepted_front.vertices.size(), VertexId{});
        for (std::size_t index = 0; index < used.size(); ++index)
            if (used[index])
            {
                remap[index] = static_cast<VertexId>(
                    output.front.vertices.size());
                output.front.vertices.push_back(
                    accepted_front.vertices[index]);
            }
        for (const std::size_t index : kept_faces)
        {
            output.front.faces.push_back(remapFace(
                accepted_front.faces[index], remap));
            output.front.source_face_ids.push_back(
                accepted_front.source_face_ids[index]);
        }
        return output;
    }

    AcceptedStoppedFrontCarry carryFacesMissingFromActive(
        const AcceptedStoppedFrontCarry &previously_accepted,
        const GrowthFront &active_front)
    {
        std::vector<SurfaceFaceId> active = active_front.source_face_ids;
        std::sort(active.begin(), active.end());
        std::vector<SurfaceFaceId> missing;
        for (const SurfaceFaceId id :
             previously_accepted.front.source_face_ids)
            if (!std::binary_search(active.begin(), active.end(), id))
                missing.push_back(id);
        std::sort(missing.begin(), missing.end());

        AcceptedStoppedFrontCarry output = retainAcceptedFaces(
            previously_accepted.front, {}, missing);
        for (const SurfaceFaceId id : missing)
        {
            const auto known = std::find_if(
                previously_accepted.stops.begin(),
                previously_accepted.stops.end(),
                [id](const LayerStopState &state)
                { return state.source_face_id == id; });
            output.stops.push_back(known == previously_accepted.stops.end()
                ? LayerStopState{id, previously_accepted.front.layer,
                                 StopOrigin::AcceptedStop}
                : *known);
        }
        std::sort(output.stops.begin(), output.stops.end(),
            [](const LayerStopState &left, const LayerStopState &right)
            { return left.source_face_id < right.source_face_id; });
        return output;
    }

    GrowthFront mergeWithCarriedStoppedFaces(
        const GrowthFront &active_front,
        const AcceptedStoppedFrontCarry &carry)
    {
        GrowthFront output = active_front;
        std::unordered_map<std::uint64_t, VertexId> vertices;
        for (std::size_t index = 0; index < output.vertices.size(); ++index)
            vertices.emplace(
                vertexKey(output.vertices[index]),
                static_cast<VertexId>(index));

        std::vector<VertexId> remap(carry.front.vertices.size());
        for (std::size_t index = 0;
             index < carry.front.vertices.size(); ++index)
        {
            const GrowthFrontVertex &vertex = carry.front.vertices[index];
            const auto found = vertices.find(vertexKey(vertex));
            if (found != vertices.end())
                remap[index] = found->second;
            else
            {
                remap[index] = static_cast<VertexId>(output.vertices.size());
                output.vertices.push_back(vertex);
                vertices.emplace(vertexKey(vertex), remap[index]);
            }
        }
        for (std::size_t index = 0;
             index < carry.front.faces.size(); ++index)
        {
            output.faces.push_back(remapFace(
                carry.front.faces[index], remap));
            output.source_face_ids.push_back(
                carry.front.source_face_ids[index]);
        }
        return output;
    }

    void addCarriedStops(
        LayerFaceSets &sets,
        const AcceptedStoppedFrontCarry &carry)
    {
        for (const LayerStopState &stop : carry.stops)
            addInitialStop(sets, stop);
    }
}
