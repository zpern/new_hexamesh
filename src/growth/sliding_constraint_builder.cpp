#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <Eigen/Geometry>

#include <boundary_mesh/growth/sliding_constraint_builder.hpp>
#include <boundary_mesh/surface/face_evaluation.hpp>

namespace boundary_mesh
{
    namespace
    {
        using ConstraintResult =
            Result<SlidingConstraints, GrowthDirectionError>;

        Result<FaceEvaluation, FaceEvaluationError>
        evaluateSurfaceFace(
            const SurfaceMesh &mesh,
            const SurfaceFace &surface_face,
            Scalar length_tolerance)
        {
            return std::visit(
                [&](const auto &face)
                    -> Result<FaceEvaluation, FaceEvaluationError>
                {
                    using Face = std::decay_t<decltype(face)>;
                    for (const VertexId vertex_id : face.vertex_ids)
                    {
                        if (static_cast<std::size_t>(vertex_id) >=
                            mesh.vertices.size())
                        {
                            return Result<FaceEvaluation, FaceEvaluationError>::
                                failure(FaceEvaluationError::NonFiniteCoordinate);
                        }
                    }
                    if constexpr (std::is_same_v<Face, Triangle>)
                    {
                        return evaluateTriangle(
                            mesh.vertices[face.vertex_ids[0]],
                            mesh.vertices[face.vertex_ids[1]],
                            mesh.vertices[face.vertex_ids[2]],
                            length_tolerance);
                    }
                    else
                    {
                        return evaluateQuad(
                            mesh.vertices[face.vertex_ids[0]],
                            mesh.vertices[face.vertex_ids[1]],
                            mesh.vertices[face.vertex_ids[2]],
                            mesh.vertices[face.vertex_ids[3]],
                            length_tolerance);
                    }
                },
                surface_face);
        }

        bool faceLiesOnPlane(
            const SurfaceMesh &mesh,
            const SurfaceFace &surface_face,
            const Point3 &plane_point,
            const Vector3 &plane_normal,
            Scalar distance_tolerance)
        {
            bool valid = true;
            std::visit(
                [&](const auto &face)
                {
                    for (const VertexId vertex_id : face.vertex_ids)
                    {
                        if (static_cast<std::size_t>(vertex_id) >=
                            mesh.vertices.size() ||
                            std::abs((mesh.vertices[vertex_id] - plane_point)
                                         .dot(plane_normal)) > distance_tolerance)
                        {
                            valid = false;
                            break;
                        }
                    }
                },
                surface_face);
            return valid;
        }
    }

    const std::vector<SlidingPlane> &
    SlidingConstraints::planes() const noexcept
    {
        return planes_;
    }

    const std::vector<VertexSlidingConstraint> &
    SlidingConstraints::vertices() const noexcept
    {
        return vertices_;
    }

    Result<Vector3, GrowthDirectionError>
    SlidingConstraints::apply(
        std::size_t front_vertex_index,
        const Vector3 &raw_direction) const
    {
        using ApplyResult = Result<Vector3, GrowthDirectionError>;
        if (front_vertex_index >= vertices_.size())
        {
            return ApplyResult::failure(SlidingInputMismatch{});
        }

        const VertexSlidingConstraint &constraint =
            vertices_[front_vertex_index];
        const Scalar raw_length = raw_direction.norm();
        if (!raw_direction.allFinite() || !std::isfinite(raw_length) ||
            raw_length <= direction_tolerance_)
        {
            return ApplyResult::failure(UndefinedConstrainedDirection{
                front_vertex_index, constraint.source_vertex_id, layer_});
        }

        Vector3 result = raw_direction;
        if (constraint.plane_indices.size() == 1)
        {
            const Vector3 &normal =
                planes_[constraint.plane_indices[0]].unit_normal;
            result -= result.dot(normal) * normal;
        }
        else if (constraint.plane_indices.size() == 2)
        {
            const Vector3 &first =
                planes_[constraint.plane_indices[0]].unit_normal;
            const Vector3 &second =
                planes_[constraint.plane_indices[1]].unit_normal;
            result = first.cross(second);
            if (result.dot(raw_direction) < Scalar{0})
            {
                result = -result;
            }
        }

        const Scalar result_length = result.norm();
        if (!result.allFinite() || !std::isfinite(result_length) ||
            result_length <= direction_tolerance_)
        {
            return ApplyResult::failure(UndefinedConstrainedDirection{
                front_vertex_index, constraint.source_vertex_id, layer_});
        }
        return ApplyResult::success(result / result_length);
    }

    namespace
    {
        Result<Point3, GrowthDirectionError> projectToSurface(
            const SlidingSurface &surface,
            const Point3 &point)
        {
            Point3 projected = point;
            if (!point.allFinite())
                return Result<Point3, GrowthDirectionError>::failure(
                    SlidingInputMismatch{surface.region_id});
            switch (surface.kind)
            {
            case SlidingSurfaceKind::AxisX: projected.x() = surface.axis_value; break;
            case SlidingSurfaceKind::AxisY: projected.y() = surface.axis_value; break;
            case SlidingSurfaceKind::AxisZ: projected.z() = surface.axis_value; break;
            case SlidingSurfaceKind::Curved:
            {
                if (!surface.curved_index)
                    return Result<Point3, GrowthDirectionError>::failure(
                        SlidingInputMismatch{surface.region_id});
                const auto closest = surface.curved_index->closestPoint(point);
                if (!closest.hasValue())
                    return Result<Point3, GrowthDirectionError>::failure(
                        InvalidSlidingSurface{
                            surface.region_id, SurfaceFaceId{0}});
                projected = closest.value().point;
                break;
            }
            }
            return Result<Point3, GrowthDirectionError>::success(projected);
        }
    }

    Result<Vector3, GrowthDirectionError>
    SlidingConstraints::constrainDirection(
        std::size_t front_vertex_index,
        const Point3 &current_position,
        const Vector3 &raw_direction) const
    {
        if (front_vertex_index >= vertices_.size() ||
            !current_position.allFinite() || !raw_direction.allFinite())
            return Result<Vector3, GrowthDirectionError>::failure(
                SlidingInputMismatch{});
        const VertexSlidingConstraint &constraint = vertices_[front_vertex_index];
        Point3 endpoint = current_position + raw_direction;
        for (std::size_t iteration = 0; iteration < 10; ++iteration)
            for (const std::size_t index : constraint.plane_indices)
            {
                const auto projected = projectToSurface(
                    surfaces_.surfaces()[index], endpoint);
                if (!projected.hasValue())
                    return Result<Vector3, GrowthDirectionError>::failure(
                        projected.error());
                endpoint = projected.value();
            }
        Vector3 direction = endpoint - current_position;
        const Scalar length = direction.norm();
        if (!direction.allFinite() || !std::isfinite(length) ||
            length <= direction_tolerance_)
            return Result<Vector3, GrowthDirectionError>::failure(
                UndefinedConstrainedDirection{front_vertex_index,
                    constraint.source_vertex_id, layer_});
        return Result<Vector3, GrowthDirectionError>::success(direction / length);
    }

    Result<SlidingPositionProjection, GrowthDirectionError>
    SlidingConstraints::projectPosition(
        std::size_t front_vertex_index,
        const Point3 &candidate) const
    {
        using ProjectionResult =
            Result<SlidingPositionProjection, GrowthDirectionError>;
        if (front_vertex_index >= vertices_.size() || !candidate.allFinite())
            return ProjectionResult::failure(SlidingInputMismatch{});
        const VertexSlidingConstraint &constraint = vertices_[front_vertex_index];
        SlidingPositionProjection result;
        result.position = candidate;
        if (constraint.plane_indices.empty())
            return ProjectionResult::success(result);
        Scalar tolerance = direction_tolerance_;
        for (const std::size_t index : constraint.plane_indices)
            tolerance = std::max(
                tolerance, surfaces_.surfaces()[index].projection_tolerance);
        const std::uint32_t maximum_iterations =
            constraint.plane_indices.size() == 1 ? 1u : 20u;
        for (std::uint32_t iteration = 1;
             iteration <= maximum_iterations; ++iteration)
        {
            const Point3 previous = result.position;
            for (const std::size_t index : constraint.plane_indices)
            {
                const auto projected = projectToSurface(
                    surfaces_.surfaces()[index], result.position);
                if (!projected.hasValue())
                    return ProjectionResult::failure(projected.error());
                result.position = projected.value();
            }
            result.iterations = iteration;
            result.position_change = (result.position - previous).norm();
            result.max_surface_residual = Scalar{0};
            for (const std::size_t index : constraint.plane_indices)
            {
                const auto on_surface = projectToSurface(
                    surfaces_.surfaces()[index], result.position);
                if (!on_surface.hasValue())
                    return ProjectionResult::failure(on_surface.error());
                result.max_surface_residual = std::max(
                    result.max_surface_residual,
                    (on_surface.value() - result.position).norm());
            }
            if ((constraint.plane_indices.size() == 1 ||
                 result.position_change <= tolerance) &&
                result.max_surface_residual <= tolerance)
                return ProjectionResult::success(result);
        }
        std::vector<std::uint32_t> regions;
        for (const std::size_t index : constraint.plane_indices)
            regions.push_back(surfaces_.surfaces()[index].region_id);
        return ProjectionResult::failure(SlidingProjectionNotConverged{
            front_vertex_index, constraint.source_vertex_id, layer_,
            std::move(regions), result.iterations,
            result.position_change, result.max_surface_residual});
    }

    Result<SlidingConstraints, GrowthDirectionError>
    SlidingConstraintBuilder::build(
        const SurfaceMesh &mesh,
        const GrowthFront &front,
        const FrontEvaluation &evaluation) const
    {
        if (front.layer != evaluation.layer ||
            mesh.faces.size() != mesh.face_tags.size() ||
            !std::isfinite(evaluation.characteristic_length) ||
            evaluation.characteristic_length <= Scalar{0} ||
            !std::isfinite(evaluation.effective_length_tolerance) ||
            evaluation.effective_length_tolerance <= Scalar{0})
        {
            return ConstraintResult::failure(SlidingInputMismatch{});
        }

        std::vector<std::uint32_t> referenced_regions;
        for (const GrowthFrontVertex &vertex : front.vertices)
        {
            const FrontVertexBoundary &boundary = vertex.boundary;
            referenced_regions.insert(
                referenced_regions.end(),
                boundary.sliding_region_ids.begin(),
                boundary.sliding_region_ids.end());
        }
        std::sort(referenced_regions.begin(), referenced_regions.end());
        referenced_regions.erase(
            std::unique(referenced_regions.begin(), referenced_regions.end()),
            referenced_regions.end());

        SlidingConstraints output;
        output.layer_ = front.layer;
        output.angular_tolerance_ = std::max(
            Scalar{1e-12},
            evaluation.effective_length_tolerance /
                evaluation.characteristic_length);
        output.direction_tolerance_ = output.angular_tolerance_;
        output.planes_.reserve(referenced_regions.size());

        for (const std::uint32_t region_id : referenced_regions)
        {
            std::vector<std::size_t> region_faces;
            for (std::size_t face_index = 0;
                 face_index < mesh.faces.size();
                 ++face_index)
            {
                const SurfaceBoundaryTag &tag = mesh.face_tags[face_index];
                const bool is_sliding_kind =
                    tag.kind == SurfaceBoundaryKind::Symmetry ||
                    tag.kind == SurfaceBoundaryKind::Internal;
                if (is_sliding_kind &&
                    tag.region_id == region_id)
                {
                    region_faces.push_back(face_index);
                }
            }
            if (region_faces.empty())
            {
                return ConstraintResult::failure(
                    SlidingInputMismatch{region_id});
            }

            const std::size_t reference_index = region_faces.front();
            const auto reference = evaluateSurfaceFace(
                mesh,
                mesh.faces[reference_index],
                evaluation.effective_length_tolerance);
            if (!reference.hasValue())
            {
                return ConstraintResult::failure(InvalidSlidingSurface{
                    region_id,
                    static_cast<SurfaceFaceId>(reference_index)});
            }

            const Point3 reference_point = reference.value().centroid;
            const Vector3 reference_normal = reference.value().unit_normal;
            for (const std::size_t face_index : region_faces)
            {
                const auto current = evaluateSurfaceFace(
                    mesh,
                    mesh.faces[face_index],
                    evaluation.effective_length_tolerance);
                const bool parallel = current.hasValue() &&
                    std::abs(reference_normal.dot(
                        current.value().unit_normal)) >=
                        Scalar{1} - output.angular_tolerance_;
                if (!parallel ||
                    !faceLiesOnPlane(
                        mesh,
                        mesh.faces[face_index],
                        reference_point,
                        reference_normal,
                        evaluation.effective_length_tolerance))
                {
                    return ConstraintResult::failure(InvalidSlidingSurface{
                        region_id,
                        static_cast<SurfaceFaceId>(face_index)});
                }
            }
            output.planes_.push_back(SlidingPlane{
                region_id, reference_point, reference_normal});
        }

        output.vertices_.reserve(front.vertices.size());
        for (std::size_t vertex_index = 0;
             vertex_index < front.vertices.size();
             ++vertex_index)
        {
            std::vector<std::uint32_t> regions =
                front.vertices[vertex_index].boundary.sliding_region_ids;
            std::sort(regions.begin(), regions.end());
            regions.erase(std::unique(regions.begin(), regions.end()), regions.end());

            VertexSlidingConstraint constraint;
            constraint.front_vertex_index = vertex_index;
            constraint.source_vertex_id =
                front.vertices[vertex_index].source_vertex_id;
            std::vector<Vector3> orthonormal_basis;

            for (const std::uint32_t region_id : regions)
            {
                const auto plane = std::lower_bound(
                    output.planes_.begin(),
                    output.planes_.end(),
                    region_id,
                    [](const SlidingPlane &value, std::uint32_t id)
                    {
                        return value.region_id < id;
                    });
                if (plane == output.planes_.end() ||
                    plane->region_id != region_id)
                {
                    return ConstraintResult::failure(
                        SlidingInputMismatch{region_id});
                }

                Vector3 residual = plane->unit_normal;
                for (const Vector3 &basis : orthonormal_basis)
                {
                    residual -= residual.dot(basis) * basis;
                }
                const Scalar residual_length = residual.norm();
                if (residual_length <= output.angular_tolerance_)
                {
                    continue;
                }
                if (orthonormal_basis.size() == 2)
                {
                    return ConstraintResult::failure(
                        OverConstrainedGrowthVertex{
                            vertex_index,
                            front.vertices[vertex_index].source_vertex_id,
                            front.layer});
                }
                orthonormal_basis.push_back(residual / residual_length);
                constraint.plane_indices.push_back(
                    static_cast<std::size_t>(
                        std::distance(output.planes_.begin(), plane)));
            }
            output.vertices_.push_back(std::move(constraint));
        }

        return ConstraintResult::success(std::move(output));
    }

    Result<SlidingConstraints, GrowthDirectionError>
    SlidingConstraintBuilder::build(
        const SlidingSurfaceSet &surfaces,
        const GrowthFront &front,
        const FrontEvaluation &evaluation) const
    {
        if (front.layer != evaluation.layer ||
            !std::isfinite(evaluation.characteristic_length) ||
            evaluation.characteristic_length <= Scalar{0})
            return ConstraintResult::failure(SlidingInputMismatch{});
        SlidingConstraints output;
        output.layer_ = front.layer;
        output.angular_tolerance_ = std::max(
            Scalar{1e-12}, evaluation.effective_length_tolerance /
                evaluation.characteristic_length);
        output.direction_tolerance_ = output.angular_tolerance_;
        output.surfaces_ = surfaces;
        for (const SlidingSurface &surface : surfaces.surfaces())
        {
            Vector3 normal = Vector3::Zero();
            if (surface.kind == SlidingSurfaceKind::AxisX) normal.x() = 1;
            if (surface.kind == SlidingSurfaceKind::AxisY) normal.y() = 1;
            if (surface.kind == SlidingSurfaceKind::AxisZ) normal.z() = 1;
            output.planes_.push_back({surface.region_id, Point3::Zero(), normal});
        }
        output.vertices_.reserve(front.vertices.size());
        for (std::size_t vertex_index = 0;
             vertex_index < front.vertices.size(); ++vertex_index)
        {
            VertexSlidingConstraint constraint;
            constraint.front_vertex_index = vertex_index;
            constraint.source_vertex_id = front.vertices[vertex_index].source_vertex_id;
            std::vector<Vector3> axis_basis;
            std::vector<std::uint32_t> regions =
                front.vertices[vertex_index].boundary.sliding_region_ids;
            std::sort(regions.begin(), regions.end());
            regions.erase(std::unique(regions.begin(), regions.end()), regions.end());
            for (const std::uint32_t region_id : regions)
            {
                const SlidingSurface *surface = surfaces.find(region_id);
                if (!surface)
                    return ConstraintResult::failure(
                        SlidingInputMismatch{region_id});
                const std::size_t index = static_cast<std::size_t>(
                    surface - surfaces.surfaces().data());
                constraint.plane_indices.push_back(index);
                Vector3 normal = output.planes_[index].unit_normal;
                if (normal.squaredNorm() > Scalar{0})
                {
                    bool independent = true;
                    for (const Vector3 &basis : axis_basis)
                        if (std::abs(normal.dot(basis)) >=
                            Scalar{1} - output.angular_tolerance_)
                            independent = false;
                    if (independent) axis_basis.push_back(normal);
                }
            }
            if (axis_basis.size() == 3)
                return ConstraintResult::failure(OverConstrainedGrowthVertex{
                    vertex_index, constraint.source_vertex_id, front.layer});
            output.vertices_.push_back(std::move(constraint));
        }
        return ConstraintResult::success(std::move(output));
    }
}
