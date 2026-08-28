#include <algorithm>
#include <cstddef>
#include <iostream>
#include <type_traits>
#include <variant>
#include <vector>

#include <boundary_mesh/growth/growth_front_builder.hpp>
#include <boundary_mesh/growth/growth_patch_builder.hpp>
#include <boundary_mesh/growth/regular_layer_generator.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

namespace
{
    using namespace boundary_mesh;

    SurfaceMesh makeCollisionPair()
    {
        SurfaceMesh mesh;
        mesh.vertices = {
            {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 0.0, 1.0}, {1.0, 1.0, 1.0}, {0.0, 1.0, 1.0}, {0.30, 0.10, 1.08}, {0.70, 0.10, 1.08}, {0.50, 0.40, 1.08}, {0.50, 0.20, 1.12}};
        mesh.faces = {
            Quad{{0, 3, 2, 1}},
            Triangle{{4, 5, 6}},
            Triangle{{4, 6, 7}},
            Quad{{0, 1, 5, 4}},
            Quad{{1, 2, 6, 5}},
            Quad{{2, 3, 7, 6}},
            Quad{{3, 0, 4, 7}},
            Triangle{{8, 10, 9}},
            Triangle{{8, 9, 11}},
            Triangle{{9, 10, 11}},
            Triangle{{10, 8, 11}}};
        mesh.face_tags = {
            {SurfaceBoundaryKind::Farfield, 20},
            {SurfaceBoundaryKind::Wall, 10},
            {SurfaceBoundaryKind::Wall, 11},
            {SurfaceBoundaryKind::Farfield, 21},
            {SurfaceBoundaryKind::Farfield, 22},
            {SurfaceBoundaryKind::Farfield, 23},
            {SurfaceBoundaryKind::Farfield, 24},
            {SurfaceBoundaryKind::Farfield, 25},
            {SurfaceBoundaryKind::Farfield, 25},
            {SurfaceBoundaryKind::Farfield, 25},
            {SurfaceBoundaryKind::Farfield, 25}};
        return mesh;
    }

    SurfaceMesh makeQualityPair()
    {
        SurfaceMesh mesh;
        mesh.vertices = {
            {0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {0, 1, 0}, {1, 1, 0}, {2, 1, 0}, {0, 0, 1}, {1, 0, 1}, {2, 0, 1}, {0, 1, 1}, {1, 1, 1}, {2, 1, 1}};
        mesh.faces = {
            Quad{{0, 3, 4, 1}}, Quad{{1, 4, 5, 2}},
            Quad{{6, 7, 10, 9}}, Quad{{7, 8, 11, 10}},
            Quad{{0, 1, 7, 6}}, Quad{{1, 2, 8, 7}},
            Quad{{2, 5, 11, 8}},
            Quad{{5, 4, 10, 11}}, Quad{{4, 3, 9, 10}},
            Quad{{3, 0, 6, 9}}};
        mesh.face_tags.resize(
            mesh.faces.size(),
            {SurfaceBoundaryKind::Farfield, 20});
        mesh.face_tags[2] = {SurfaceBoundaryKind::Wall, 10};
        mesh.face_tags[3] = {SurfaceBoundaryKind::Wall, 11};
        return mesh;
    }

    const FaceGrowthRecord *faceRecord(
        const RegularLayerGrowthResult &result,
        SurfaceFaceId source_face_id)
    {
        const auto found = std::find_if(
            result.faces.begin(),
            result.faces.end(),
            [&](const FaceGrowthRecord &record)
            {
                return record.source_face_id == source_face_id;
            });
        return found == result.faces.end() ? nullptr : &*found;
    }

    bool run(
        std::uint32_t maximum_difference,
        RegularLayerGrowthResult &output)
    {
        const SurfaceMesh mesh = makeCollisionPair();
        const auto topology = SurfaceTopologyBuilder{}.build(mesh);
        if (!topology.hasValue())
        {
            std::cerr << "topology failed\n";
            return false;
        }
        const auto patch = GrowthPatchBuilder{}.build(mesh, topology.value());
        if (!patch.hasValue())
        {
            std::cerr << "patch failed\n";
            return false;
        }
        const auto front = GrowthFrontBuilder{}.buildInitial(
            mesh, patch.value());
        if (!front.hasValue())
        {
            std::cerr << "front failed\n";
            return false;
        }

        std::vector<SourceVertexGrowthProfile> profiles;
        for (const PatchVertex &vertex : patch.value().vertices())
        {
            profiles.push_back(
                {vertex.source_vertex_id, {0.1, 1.0, 1}});
        }
        RegularLayerGrowthOptions options;
        options.max_layer_diff = maximum_difference;
        const auto result = generateRegularLayers(
            mesh,
            topology.value(),
            patch.value(),
            front.value(),
            profiles,
            options);
        if (!result.hasValue())
        {
            std::cerr << "growth failed\n";
            return false;
        }
        output = result.value();
        return true;
    }

    bool runQuality(
        std::uint32_t maximum_difference,
        RegularLayerGrowthResult &output)
    {
        const SurfaceMesh mesh = makeQualityPair();
        const auto topology = SurfaceTopologyBuilder{}.build(mesh);
        if (!topology.hasValue())
            return false;
        const auto patch = GrowthPatchBuilder{}.build(mesh, topology.value());
        if (!patch.hasValue())
            return false;
        const auto front = GrowthFrontBuilder{}.buildInitial(
            mesh, patch.value());
        if (!front.hasValue())
            return false;

        std::vector<SourceVertexGrowthProfile> profiles;
        for (const PatchVertex &vertex : patch.value().vertices())
        {
            const Scalar height = vertex.source_vertex_id == VertexId{6}
                                      ? Scalar{0.5}
                                      : Scalar{0.1};
            profiles.push_back(
                {vertex.source_vertex_id, {height, 1.0, 1}});
        }
        RegularLayerGrowthOptions options;
        options.cell_quality.maximum_skewness = 0.05;
        options.max_layer_diff = maximum_difference;
        const auto result = generateRegularLayers(
            mesh,
            topology.value(),
            patch.value(),
            front.value(),
            profiles,
            options);
        if (!result.hasValue())
            return false;
        output = result.value();
        return true;
    }

    bool runLayerDifference(
        bool reverse_profiles,
        RegularLayerGrowthResult &output)
    {
        const SurfaceMesh mesh = makeQualityPair();
        const auto topology = SurfaceTopologyBuilder{}.build(mesh);
        if (!topology.hasValue())
            return false;
        const auto patch = GrowthPatchBuilder{}.build(mesh, topology.value());
        if (!patch.hasValue())
            return false;
        const auto front = GrowthFrontBuilder{}.buildInitial(
            mesh, patch.value());
        if (!front.hasValue())
            return false;

        std::vector<SourceVertexGrowthProfile> profiles;
        for (const PatchVertex &vertex : patch.value().vertices())
        {
            const bool left_unique =
                vertex.source_vertex_id == VertexId{6} ||
                vertex.source_vertex_id == VertexId{9};
            profiles.push_back({vertex.source_vertex_id,
                                {0.1, 1.0, left_unique ? 1u : 2u}});
        }
        if (reverse_profiles)
        {
            std::reverse(profiles.begin(), profiles.end());
        }
        const auto result = generateRegularLayers(
            mesh,
            topology.value(),
            patch.value(),
            front.value(),
            profiles);
        if (!result.hasValue())
            return false;
        output = result.value();
        return true;
    }

    bool runIsotropicPropagation(
        RegularLayerGrowthResult &output)
    {
        SurfaceMesh mesh = makeQualityPair();
        for (Point3 &point : mesh.vertices)
        {
            if (point.x() == Scalar{1})
            {
                point.x() = Scalar{0.5};
            }
        }
        const auto topology = SurfaceTopologyBuilder{}.build(mesh);
        if (!topology.hasValue())
            return false;
        const auto patch = GrowthPatchBuilder{}.build(
            mesh, topology.value());
        if (!patch.hasValue())
            return false;
        const auto front = GrowthFrontBuilder{}.buildInitial(
            mesh, patch.value());
        if (!front.hasValue())
            return false;

        std::vector<SourceVertexGrowthProfile> profiles;
        for (const PatchVertex &vertex : patch.value().vertices())
        {
            profiles.push_back(
                {vertex.source_vertex_id, {0.1, 1.0, 4}});
        }
        RegularLayerGrowthOptions options;
        options.isotropic_height = Scalar{0.12};
        options.max_layer_diff = 0;
        const auto result = generateRegularLayers(
            mesh,
            topology.value(),
            patch.value(),
            front.value(),
            profiles,
            options);
        if (!result.hasValue())
            return false;
        output = result.value();
        return true;
    }

    bool sameFace(const SurfaceFace &first, const SurfaceFace &second)
    {
        return std::visit(
            [&](const auto &value)
            {
                using Face = std::decay_t<decltype(value)>;
                const Face *other = std::get_if<Face>(&second);
                return other != nullptr &&
                       other->vertex_ids == value.vertex_ids;
            },
            first);
    }

    bool sameCell(const VolumeCell &first, const VolumeCell &second)
    {
        return std::visit(
            [&](const auto &value)
            {
                using Cell = std::decay_t<decltype(value)>;
                const Cell *other = std::get_if<Cell>(&second);
                return other != nullptr &&
                       other->vertex_ids == value.vertex_ids;
            },
            first);
    }

    bool sameSurface(const SurfaceMesh &first, const SurfaceMesh &second)
    {
        if (first.vertices.size() != second.vertices.size() ||
            first.faces.size() != second.faces.size() ||
            first.face_tags.size() != second.face_tags.size())
        {
            return false;
        }
        for (std::size_t index = 0; index < first.vertices.size(); ++index)
        {
            if ((first.vertices[index] - second.vertices[index]).norm() != 0.0)
            {
                return false;
            }
        }
        for (std::size_t index = 0; index < first.faces.size(); ++index)
        {
            if (!sameFace(first.faces[index], second.faces[index]) ||
                first.face_tags[index].kind != second.face_tags[index].kind ||
                first.face_tags[index].region_id !=
                    second.face_tags[index].region_id)
            {
                return false;
            }
        }
        return true;
    }

    bool sameGrowthResult(
        const RegularLayerGrowthResult &first,
        const RegularLayerGrowthResult &second)
    {
        if (first.mesh.vertices.size() != second.mesh.vertices.size() ||
            first.mesh.cells.size() != second.mesh.cells.size() ||
            first.mesh.metadata.size() != second.mesh.metadata.size() ||
            first.faces.size() != second.faces.size() ||
            !sameSurface(first.farfield_boundary, second.farfield_boundary))
        {
            return false;
        }
        for (std::size_t index = 0; index < first.mesh.vertices.size(); ++index)
        {
            if ((first.mesh.vertices[index] -
                 second.mesh.vertices[index])
                    .norm() != 0.0)
            {
                return false;
            }
        }
        for (std::size_t index = 0; index < first.mesh.cells.size(); ++index)
        {
            if (!sameCell(first.mesh.cells[index], second.mesh.cells[index]) ||
                first.mesh.metadata[index].role !=
                    second.mesh.metadata[index].role ||
                first.mesh.metadata[index].source_face_id !=
                    second.mesh.metadata[index].source_face_id ||
                first.mesh.metadata[index].layer !=
                    second.mesh.metadata[index].layer)
            {
                return false;
            }
        }
        for (std::size_t index = 0; index < first.faces.size(); ++index)
        {
            const FaceGrowthRecord &left = first.faces[index];
            const FaceGrowthRecord &right = second.faces[index];
            if (left.source_face_id != right.source_face_id ||
                left.accepted_layer_count != right.accepted_layer_count ||
                left.status != right.status ||
                left.stop_reason != right.stop_reason ||
                left.stop_layer != right.stop_layer)
            {
                return false;
            }
        }
        return true;
    }

    std::size_t countKind(
        const SurfaceMesh &mesh,
        SurfaceBoundaryKind kind)
    {
        return static_cast<std::size_t>(std::count_if(
            mesh.face_tags.begin(),
            mesh.face_tags.end(),
            [&](const SurfaceBoundaryTag &tag)
            {
                return tag.kind == kind;
            }));
    }

    bool hasNoUnreferencedVertices(const SurfaceMesh &mesh)
    {
        std::vector<bool> used(mesh.vertices.size(), false);
        for (const SurfaceFace &face : mesh.faces)
        {
            std::visit(
                [&](const auto &value)
                {
                    for (const VertexId vertex_id : value.vertex_ids)
                    {
                        const std::size_t index =
                            static_cast<std::size_t>(vertex_id);
                        if (index < used.size())
                            used[index] = true;
                    }
                },
                face);
        }
        return std::all_of(used.begin(), used.end(), [](bool value)
                           { return value; });
    }
}

int main()
{
    using namespace boundary_mesh;

    RegularLayerGrowthResult difference_zero;
    if (!run(0, difference_zero))
        return 1;
    const FaceGrowthRecord *collision_zero = faceRecord(
        difference_zero, SurfaceFaceId{1});
    const FaceGrowthRecord *neighbor_zero = faceRecord(
        difference_zero, SurfaceFaceId{2});
    if (collision_zero == nullptr || neighbor_zero == nullptr)
        return 2;
    if (collision_zero->accepted_layer_count != 0)
        return 3;
    if (collision_zero->status != FaceGrowthStatus::Stopped)
        return 4;
    if (collision_zero->stop_reason != FaceStopReason::Collision)
        return 5;
    if (neighbor_zero->accepted_layer_count != 0)
        return 6;
    if (neighbor_zero->status != FaceGrowthStatus::Stopped)
        return 7;
    if (neighbor_zero->stop_reason !=
        FaceStopReason::NeighborLayerConstraint)
    {
        std::cerr << "difference=0 neighbor reason="
                  << static_cast<int>(neighbor_zero->stop_reason) << '\n';
        return 8;
    }
    if (!difference_zero.mesh.cells.empty())
        return 9;

    RegularLayerGrowthResult difference_one;
    if (!run(1, difference_one))
        return 10;
    const FaceGrowthRecord *collision_one = faceRecord(
        difference_one, SurfaceFaceId{1});
    const FaceGrowthRecord *neighbor_one = faceRecord(
        difference_one, SurfaceFaceId{2});
    if (collision_one == nullptr || neighbor_one == nullptr)
        return 11;
    if (collision_one->accepted_layer_count != 0)
        return 12;
    if (collision_one->stop_reason != FaceStopReason::Collision)
        return 13;
    if (neighbor_one->accepted_layer_count != 1)
        return 14;
    if (neighbor_one->status != FaceGrowthStatus::Completed)
        return 15;
    if (neighbor_one->stop_reason != FaceStopReason::VertexLayerLimit)
    {
        return 16;
    }
    if (difference_one.mesh.cells.size() != 1)
        return 17;

    RegularLayerGrowthResult quality_zero;
    if (!runQuality(0, quality_zero))
        return 18;
    const FaceGrowthRecord *quality_failure = faceRecord(
        quality_zero, SurfaceFaceId{2});
    const FaceGrowthRecord *quality_neighbor = faceRecord(
        quality_zero, SurfaceFaceId{3});
    if (quality_failure == nullptr || quality_neighbor == nullptr)
        return 19;
    if (quality_failure->accepted_layer_count != 0 ||
        quality_failure->status != FaceGrowthStatus::Stopped ||
        quality_failure->stop_reason != FaceStopReason::SkewnessExceeded)
    {
        std::cerr << "quality reason="
                  << static_cast<int>(quality_failure->stop_reason) << '\n';
        return 20;
    }
    if (quality_neighbor->accepted_layer_count != 0 ||
        quality_neighbor->stop_reason !=
            FaceStopReason::NeighborLayerConstraint ||
        !quality_zero.mesh.cells.empty())
    {
        std::cerr << "quality neighbor accepted="
                  << quality_neighbor->accepted_layer_count
                  << " reason="
                  << static_cast<int>(quality_neighbor->stop_reason)
                  << " cells=" << quality_zero.mesh.cells.size() << '\n';
        return 21;
    }

    RegularLayerGrowthResult quality_one;
    if (!runQuality(1, quality_one))
        return 22;
    const FaceGrowthRecord *quality_one_neighbor = faceRecord(
        quality_one, SurfaceFaceId{3});
    if (quality_one_neighbor == nullptr ||
        quality_one_neighbor->accepted_layer_count != 1 ||
        quality_one_neighbor->status != FaceGrowthStatus::Completed ||
        quality_one.mesh.cells.size() != 1)
    {
        return 23;
    }

    RegularLayerGrowthResult layer_difference;
    if (!runLayerDifference(false, layer_difference))
        return 24;
    const FaceGrowthRecord *left = faceRecord(
        layer_difference, SurfaceFaceId{2});
    const FaceGrowthRecord *right = faceRecord(
        layer_difference, SurfaceFaceId{3});
    if (left == nullptr || right == nullptr ||
        left->accepted_layer_count != 1 ||
        right->accepted_layer_count != 2 ||
        layer_difference.mesh.cells.size() != 3)
    {
        std::cerr << "layer difference left="
                  << (left == nullptr ? 999u : left->accepted_layer_count)
                  << " left_reason="
                  << (left == nullptr
                          ? 999
                          : static_cast<int>(left->stop_reason))
                  << " right="
                  << (right == nullptr ? 999u : right->accepted_layer_count)
                  << " right_reason="
                  << (right == nullptr
                          ? 999
                          : static_cast<int>(right->stop_reason))
                  << " cells=" << layer_difference.mesh.cells.size()
                  << '\n';
        return 25;
    }
    const SurfaceMesh &boundary = layer_difference.farfield_boundary;
    if (boundary.faces.size() != boundary.face_tags.size() ||
        countKind(boundary, SurfaceBoundaryKind::Farfield) != 8 ||
        countKind(
            boundary,
            SurfaceBoundaryKind::BoundaryLayerInterface) != 12 ||
        !hasNoUnreferencedVertices(boundary))
    {
        std::cerr << "farfield="
                  << countKind(boundary, SurfaceBoundaryKind::Farfield)
                  << " interface="
                  << countKind(
                         boundary,
                         SurfaceBoundaryKind::BoundaryLayerInterface)
                  << " vertices=" << boundary.vertices.size()
                  << " faces=" << boundary.faces.size() << '\n';
        return 26;
    }
    for (const SurfaceBoundaryTag &tag : boundary.face_tags)
    {
        if (tag.kind == SurfaceBoundaryKind::BoundaryLayerInterface &&
            tag.region_id != 10 && tag.region_id != 11)
        {
            return 27;
        }
    }

    RegularLayerGrowthResult reversed_profiles;
    if (!runLayerDifference(true, reversed_profiles) ||
        !sameGrowthResult(layer_difference, reversed_profiles))
    {
        return 28;
    }

    RegularLayerGrowthResult isotropic_propagation;
    if (!runIsotropicPropagation(isotropic_propagation))
        return 29;
    const FaceGrowthRecord *isotropic_direct = faceRecord(
        isotropic_propagation, SurfaceFaceId{2});
    const FaceGrowthRecord *isotropic_neighbor = faceRecord(
        isotropic_propagation, SurfaceFaceId{3});
    if (isotropic_direct == nullptr || isotropic_neighbor == nullptr ||
        isotropic_direct->accepted_layer_count != 4 ||
        isotropic_direct->status != FaceGrowthStatus::Completed ||
        isotropic_direct->stop_reason !=
            FaceStopReason::VertexLayerLimit ||
        isotropic_direct->stop_layer != 5 ||
        isotropic_neighbor->accepted_layer_count != 4 ||
        isotropic_neighbor->status != FaceGrowthStatus::Completed ||
        isotropic_neighbor->stop_reason !=
            FaceStopReason::VertexLayerLimit ||
        isotropic_neighbor->stop_layer != 5 ||
        isotropic_propagation.mesh.cells.size() != 8)
    {
        return 30;
    }
}
