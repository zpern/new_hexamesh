#include <algorithm>
#include <optional>
#include <unordered_map>
#include <utility>

#include <boundary_mesh/transition/provisional_transition_builder.hpp>
#include <boundary_mesh/transition/quad_high_neighbor_selector.hpp>
#include <boundary_mesh/transition/triangle_side_transition.hpp>

namespace boundary_mesh
{
    namespace
    {
        std::uint64_t cellKey(
            SurfaceFaceId id,
            std::uint32_t layer)
        {
            return (static_cast<std::uint64_t>(id) << 32) | layer;
        }

        std::uint64_t edgeKey(VertexId first, VertexId second)
        {
            if (second < first) std::swap(first, second);
            return (static_cast<std::uint64_t>(first) << 32) | second;
        }

        OrientedQuad oriented(
            const std::array<VertexId, 4> &ids,
            const std::vector<Point3> &points)
        {
            OrientedQuad quad;
            quad.vertex_ids = ids;
            for (std::size_t index = 0; index < 4; ++index)
                quad.points[index] = points[ids[index]];
            return quad;
        }

        void appendTriangles(
            SurfaceMesh &top,
            const std::vector<Triangle> &triangles,
            std::uint32_t region)
        {
            for (const Triangle &triangle : triangles)
            {
                top.faces.push_back(triangle);
                top.face_tags.push_back({
                    SurfaceBoundaryKind::BoundaryLayerInterface, region});
            }
        }

        TransitionTemplateError atStage(
            TransitionTemplateError error, std::uint32_t stage)
        {
            if (auto *invalid =
                    std::get_if<InvalidTransitionTemplateInput>(&error))
                invalid->stage = stage;
            return error;
        }
        std::vector<VertexId> faceIds(const SurfaceFace &face)
        {
            return std::visit([](const auto &value)
            {
                return std::vector<VertexId>(
                    value.vertex_ids.begin(), value.vertex_ids.end());
            }, face);
        }

        bool retainedFace(
            const std::vector<SurfaceFaceId> &retained, SurfaceFaceId id)
        {
            return std::binary_search(retained.begin(), retained.end(), id);
        }

        void appendOwnedTriangle(
            TransitionBoundaryInput &boundary,
            const Triangle &triangle,
            const std::vector<Point3> &points,
            const std::vector<CollisionVertexKey> &keys,
            const LayerBoundaryOwner &owner)
        {
            OwnedBoundaryTriangle output;
            output.owner = owner;
            for (std::size_t local = 0; local < 3; ++local)
            {
                const std::size_t id = triangle.vertex_ids[local];
                output.points[local] = points[id];
                output.vertex_keys[local] = keys[id];
            }
            boundary.candidate_triangles.push_back(std::move(output));
        }

        void appendFrontFace(
            TransitionBoundaryInput &boundary,
            const GrowthFront &front,
            std::size_t face_index,
            const LayerBoundaryOwner &owner)
        {
            const SurfaceFace &face = front.faces[face_index];
            const auto ids = faceIds(face);
            std::vector<Point3> points;
            std::vector<CollisionVertexKey> keys;
            points.reserve(ids.size());
            keys.reserve(ids.size());
            for (const VertexId local : ids)
            {
                const auto &vertex = front.vertices[local];
                points.push_back(vertex.position);
                keys.push_back({vertex.source_vertex_id,
                                front.layer, vertex.branch_id});
            }
            if (ids.size() == 3)
                appendOwnedTriangle(
                    boundary, Triangle{{0,1,2}}, points, keys, owner);
            else if (owner.role == BoundaryOwnerRole::RegularCandidate)
            {
                // A retained high face is only a collision proxy for an
                // unsplit regular cell. Match the regular-layer collision
                // checker; canonical/quality diagonals are resolved only
                // after the face becomes a transition low face or final cap.
                appendOwnedTriangle(
                    boundary, Triangle{{0,1,2}}, points, keys, owner);
                appendOwnedTriangle(
                    boundary, Triangle{{0,2,3}}, points, keys, owner);
            }
            else
            {
                const auto diagonal = chooseQuadDiagonal(
                    oriented({0,1,2,3}, points), 1e-12);
                if (!diagonal.hasValue()) return;
                for (const Triangle &triangle : diagonal.value().triangles)
                    appendOwnedTriangle(
                        boundary, triangle, points, keys, owner);
            }
        }
    }

        ProvisionalLayerTransitionResult buildProvisionalTransition(
            const GrowthFront &current,
            const GrowthFront &candidate,
            const std::vector<SurfaceFaceId> &retained,
            const LayerFaceSets &face_sets)
        {
            ProvisionalLayerTransition provisional;
            provisional.all_top_faces_are_triangles = true;
            std::unordered_map<SurfaceFaceId, std::size_t> current_faces;
            std::unordered_map<std::uint64_t, std::vector<SurfaceFaceId>>
                current_edge_faces;
            for (std::size_t index = 0;
                 index < current.faces.size(); ++index)
            {
                const SurfaceFaceId id = current.source_face_ids[index];
                current_faces[id] = index;
                const auto ids = faceIds(current.faces[index]);
                for (std::size_t edge = 0; edge < ids.size(); ++edge)
                    current_edge_faces[edgeKey(
                        ids[edge], ids[(edge+1)%ids.size()])]
                        .push_back(id);
            }
            std::unordered_map<std::uint64_t, std::size_t>
                candidate_vertices;
            for (std::size_t index = 0;
                 index < candidate.vertices.size(); ++index)
            {
                const auto &vertex = candidate.vertices[index];
                candidate_vertices[cellKey(
                    vertex.source_vertex_id, vertex.branch_id)] = index;
            }
            for (std::size_t index = 0;
                 index < candidate.faces.size(); ++index)
            {
                const SurfaceFaceId id = candidate.source_face_ids[index];
                if (!retainedFace(retained, id)) continue;
                appendFrontFace(provisional.boundary, candidate, index,
                    {id, candidate.layer,
                     BoundaryOwnerRole::RegularCandidate, {id}});
            }

            for (const SurfaceFaceId low_id :
                 face_sets.transition_low_faces)
            {
                const auto low_position = current_faces.find(low_id);
                if (low_position == current_faces.end()) continue;
                const SurfaceFace *low_face = &current.faces[
                    low_position->second];
                const auto low_source_ids = faceIds(*low_face);
                if (low_source_ids.size() == 3)
                {
                    std::optional<std::size_t> high_edge;
                    std::vector<SurfaceFaceId> dependencies;
                    for (std::size_t edge = 0;
                         edge < 3 && !high_edge; ++edge)
                    {
                        const auto uses = current_edge_faces.find(edgeKey(
                            low_source_ids[edge],
                            low_source_ids[(edge+1)%3]));
                        if (uses == current_edge_faces.end()) continue;
                        for (const SurfaceFaceId other_id : uses->second)
                            if (other_id != low_id &&
                                retainedFace(retained, other_id))
                            {
                                high_edge = edge;
                                dependencies.push_back(other_id);
                                break;
                            }
                    }
                    if (!high_edge.has_value()) continue;
                    std::vector<Point3> points(6);
                    std::vector<CollisionVertexKey> keys(6);
                    std::array<VertexId,3> low{{0,1,2}};
                    std::array<VertexId,3> high{{0,1,2}};
                    for (std::size_t local = 0; local < 3; ++local)
                    {
                        const auto &vertex = current.vertices[
                            low_source_ids[local]];
                        points[local] = vertex.position;
                        keys[local] = {vertex.source_vertex_id,
                                       current.layer,vertex.branch_id};
                        points[3+local] = vertex.position;
                        keys[3+local] = keys[local];
                    }
                    for (const std::size_t local : {
                             *high_edge, (*high_edge+1)%3})
                    {
                        const auto &vertex = current.vertices[
                            low_source_ids[local]];
                        const auto found = candidate_vertices.find(cellKey(
                            vertex.source_vertex_id,vertex.branch_id));
                        if (found != candidate_vertices.end())
                        {
                            high[local] = static_cast<VertexId>(3+local);
                            points[3+local] = candidate.vertices[
                                found->second].position;
                            keys[3+local] = {vertex.source_vertex_id,
                                            candidate.layer,
                                            vertex.branch_id};
                        }
                    }
                    const auto side = buildTriangleSideTransition({
                        low_id,current.layer,low,high,*high_edge});
                    if (!side.hasValue())
                        return ProvisionalLayerTransitionResult::failure(
                            LayerTransitionError{
                                atStage(side.error(),11)});
                    const LayerBoundaryOwner owner{
                        low_id,current.layer,
                        BoundaryOwnerRole::SideTransition,dependencies};
                    for (const Triangle &triangle : side.value().top_faces)
                        appendOwnedTriangle(
                            provisional.boundary,triangle,
                            points,keys,owner);
                    continue;
                }
                if (low_source_ids.size() != 4) continue;

                std::vector<Point3> points(8);
                std::vector<CollisionVertexKey> keys(8);
                std::array<VertexId,4> low{{0,1,2,3}};
                std::array<VertexId,4> high{{0,1,2,3}};
                for (std::size_t local = 0; local < 4; ++local)
                {
                    const auto &vertex = current.vertices[
                        low_source_ids[local]];
                    points[local] = vertex.position;
                    keys[local] = {vertex.source_vertex_id,
                                   current.layer, vertex.branch_id};
                    points[4+local] = vertex.position;
                    keys[4+local] = keys[local];
                    const auto candidate_vertex = candidate_vertices.find(
                        cellKey(vertex.source_vertex_id,
                                vertex.branch_id));
                    if (candidate_vertex != candidate_vertices.end())
                    {
                        points[4+local] = candidate.vertices[
                            candidate_vertex->second].position;
                        keys[4+local] = {vertex.source_vertex_id,
                                        candidate.layer,
                                        vertex.branch_id};
                    }
                }

                std::vector<QuadHighNeighbor> neighbors;
                std::vector<SurfaceFaceId> dependencies;
                for (std::size_t edge = 0; edge < 4; ++edge)
                {
                    const auto uses = current_edge_faces.find(edgeKey(
                        low_source_ids[edge],
                        low_source_ids[(edge+1)%4]));
                    if (uses == current_edge_faces.end()) continue;
                    for (const SurfaceFaceId other_id : uses->second)
                        if (other_id != low_id &&
                            retainedFace(retained, other_id))
                        {
                            neighbors.push_back({edge,other_id});
                            dependencies.push_back(other_id);
                        }
                }
                if (neighbors.empty()) continue;
                for (const auto &neighbor : neighbors)
                    for (const std::size_t local : {
                             neighbor.local_edge,
                             (neighbor.local_edge+1)%4})
                        high[local] = static_cast<VertexId>(4+local);

                const auto selection = selectQuadHighNeighbors({
                    low_id, current.layer, neighbors, low, high,
                    &points, 1e-12});
                if (!selection.hasValue())
                    return ProvisionalLayerTransitionResult::failure(
                        LayerTransitionError{
                            atStage(selection.error(),12)});
                dependencies.clear();
                for (const auto &neighbor : neighbors)
                    if (std::find(
                            selection.value().retained_local_edges.begin(),
                            selection.value().retained_local_edges.end(),
                            neighbor.local_edge) !=
                        selection.value().retained_local_edges.end())
                        dependencies.push_back(neighbor.neighbor_face_id);
                std::sort(dependencies.begin(), dependencies.end());
                dependencies.erase(std::unique(
                    dependencies.begin(), dependencies.end()),
                    dependencies.end());

                QuadDiagonal diagonal{};
                if (selection.value().required_low_diagonal.has_value())
                    diagonal = *selection.value().required_low_diagonal;
                else
                {
                    const auto chosen = chooseQuadDiagonal(
                        oriented(low, points), 1e-12);
                    if (!chosen.hasValue())
                        return ProvisionalLayerTransitionResult::failure(
                            LayerTransitionError{
                                TransitionTemplateError{chosen.error()}});
                    diagonal = chosen.value().diagonal;
                }
                provisional.boundary.diagonal_requirements.push_back(
                    {{low_id,current.layer},diagonal});
                std::vector<Triangle> low_cap = diagonal ==
                        QuadDiagonal::ZeroTwo
                    ? std::vector<Triangle>{
                        Triangle{{0,1,2}}, Triangle{{0,2,3}}}
                    : std::vector<Triangle>{
                        Triangle{{1,2,3}}, Triangle{{1,3,0}}};
                const LayerBoundaryOwner owner{
                    low_id, current.layer, BoundaryOwnerRole::TopCap,
                    dependencies};
                for (const Triangle &triangle : low_cap)
                    appendOwnedTriangle(
                        provisional.boundary, triangle,
                        points, keys, owner);

                const auto side =
                    selection.value().retained_local_edges.size() == 1
                    ? buildQuadSideTransition({
                        low_id,current.layer,low,high,
                        selection.value().retained_local_edges[0],diagonal})
                    : buildQuadAdjacentSideTransition({
                        low_id,current.layer,low,high,
                        selection.value().retained_local_edges[0],
                        selection.value().retained_local_edges[1],
                        diagonal,&points,1e-12});
                if (!side.hasValue())
                    return ProvisionalLayerTransitionResult::failure(
                        LayerTransitionError{
                            atStage(side.error(),13)});
                const LayerBoundaryOwner side_owner{
                    low_id, current.layer,
                    BoundaryOwnerRole::SideTransition, dependencies};
                for (const Triangle &triangle : side.value().top_faces)
                    appendOwnedTriangle(
                        provisional.boundary, triangle,
                        points, keys, side_owner);
            }
            return ProvisionalLayerTransitionResult::success(
                std::move(provisional));
        }
}
