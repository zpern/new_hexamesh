#include <boundary_mesh/growth/sliding_surface_builder.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <type_traits>
#include <utility>
#include <variant>

namespace boundary_mesh
{
    namespace
    {
        struct RegionData
        {
            SurfaceBoundaryKind kind{SurfaceBoundaryKind::Symmetry};
            bool has_kind{};
            std::vector<std::size_t> face_indices;
        };

        std::vector<VertexId> faceVertices(const SurfaceFace &face)
        {
            return std::visit([](const auto &value)
            {
                return std::vector<VertexId>(
                    value.vertex_ids.begin(), value.vertex_ids.end());
            }, face);
        }
    }

    const SlidingSurface *SlidingSurfaceSet::find(
        std::uint32_t region_id) const noexcept
    {
        const auto found = std::lower_bound(
            surfaces_.begin(), surfaces_.end(), region_id,
            [](const SlidingSurface &surface, std::uint32_t id)
            { return surface.region_id < id; });
        return found != surfaces_.end() && found->region_id == region_id
            ? &*found : nullptr;
    }

    const std::vector<SlidingSurface> &
    SlidingSurfaceSet::surfaces() const noexcept { return surfaces_; }

    Result<SlidingSurfaceSet, GrowthDirectionError>
    SlidingSurfaceBuilder::build(const SurfaceMesh &mesh) const
    {
        using BuildResult = Result<SlidingSurfaceSet, GrowthDirectionError>;
        if (mesh.faces.size() != mesh.face_tags.size())
            return BuildResult::failure(SlidingInputMismatch{});

        std::map<std::uint32_t, RegionData> regions;
        for (std::size_t face_index = 0; face_index < mesh.faces.size(); ++face_index)
        {
            const SurfaceBoundaryTag tag = mesh.face_tags[face_index];
            if (!isSlidingBoundary(tag.kind)) continue;
            RegionData &region = regions[tag.region_id];
            if (region.has_kind && region.kind != tag.kind)
                return BuildResult::failure(SlidingInputMismatch{tag.region_id});
            region.kind = tag.kind;
            region.has_kind = true;
            region.face_indices.push_back(face_index);
        }

        SlidingSurfaceSet output;
        for (const auto &[region_id, region] : regions)
        {
            std::set<VertexId> vertex_ids;
            std::set<std::pair<VertexId, VertexId>> edges;
            std::vector<SurfaceTriangle> triangles;
            for (const std::size_t face_index : region.face_indices)
            {
                const std::vector<VertexId> ids = faceVertices(mesh.faces[face_index]);
                for (const VertexId id : ids)
                {
                    if (static_cast<std::size_t>(id) >= mesh.vertices.size() ||
                        !mesh.vertices[id].allFinite())
                        return BuildResult::failure(InvalidSlidingSurface{
                            region_id, static_cast<SurfaceFaceId>(face_index)});
                    vertex_ids.insert(id);
                }
                for (std::size_t i = 0; i < ids.size(); ++i)
                    edges.insert(std::minmax(ids[i], ids[(i + 1) % ids.size()]));

                const auto append = [&](VertexId a, VertexId b, VertexId c,
                                        std::uint32_t local)
                {
                    triangles.push_back({
                        {mesh.vertices[a], mesh.vertices[b], mesh.vertices[c]},
                        static_cast<SurfaceFaceId>(face_index), local});
                };
                if (ids.size() == 3) append(ids[0], ids[1], ids[2], 0);
                else
                {
                    append(ids[0], ids[1], ids[2], 0);
                    append(ids[0], ids[2], ids[3], 1);
                }
            }
            if (vertex_ids.empty() || edges.empty())
                return BuildResult::failure(InvalidSlidingSurface{region_id, 0});
            Scalar edge_sum{};
            for (const auto &[first, second] : edges)
                edge_sum += (mesh.vertices[first] - mesh.vertices[second]).norm();
            const Scalar average = edge_sum / static_cast<Scalar>(edges.size());
            if (!std::isfinite(average) || average <= Scalar{0})
                return BuildResult::failure(InvalidSlidingSurface{region_id, 0});
            const Scalar reference = Scalar{0.02} * average;
            const Scalar epsilon = Scalar{0.1} * reference;
            Point3 minimum = mesh.vertices[*vertex_ids.begin()];
            Point3 maximum = minimum;
            Point3 sum = Point3::Zero();
            for (const VertexId id : vertex_ids)
            {
                minimum = minimum.cwiseMin(mesh.vertices[id]);
                maximum = maximum.cwiseMax(mesh.vertices[id]);
                sum += mesh.vertices[id];
            }
            const Point3 mean = sum / static_cast<Scalar>(vertex_ids.size());
            SlidingSurface surface;
            surface.region_id = region_id;
            surface.boundary_kind = region.kind;
            surface.reference_length = reference;
            surface.projection_tolerance = std::max(Scalar{1e-12}, epsilon);
            const Vector3 span = maximum - minimum;
            if (span.x() < epsilon)
            { surface.kind = SlidingSurfaceKind::AxisX; surface.axis_value = mean.x(); }
            else if (span.y() < epsilon)
            { surface.kind = SlidingSurfaceKind::AxisY; surface.axis_value = mean.y(); }
            else if (span.z() < epsilon)
            { surface.kind = SlidingSurfaceKind::AxisZ; surface.axis_value = mean.z(); }
            else
            {
                surface.kind = SlidingSurfaceKind::Curved;
                auto index = TriangleSurfaceIndex::build(std::move(triangles));
                if (!index.hasValue())
                    return BuildResult::failure(InvalidSlidingSurface{
                        region_id, static_cast<SurfaceFaceId>(region.face_indices.front())});
                surface.curved_index = std::move(index.value());
            }
            output.surfaces_.push_back(std::move(surface));
        }
        return BuildResult::success(std::move(output));
    }
}
