#include <algorithm>
#include <cmath>
#include <filesystem>
#include <utility>
#include <vector>

#include <boundary_mesh/growth/front_evaluator.hpp>
#include <boundary_mesh/growth/incident_face_fan.hpp>
#include <boundary_mesh/growth/multi_normal_quad_triangulator.hpp>
#include <boundary_mesh/growth/multi_normal_intersection_resolver.hpp>
#include <boundary_mesh/growth/multi_normal_split_planner.hpp>
#include <boundary_mesh/growth/multi_normal_topology_builder.hpp>
#include <boundary_mesh/growth/multi_normal_transition_builder.hpp>
#include <boundary_mesh/growth/multi_normal_transition_generator.hpp>
#include <boundary_mesh/io/legacy_vtk_writer.hpp>

namespace boundary_mesh
{
    namespace
    {
        using GeneratorResult =
            Result<MultiNormalTransitionResult, MultiNormalError>;

        SurfaceMesh surfaceMesh(const GrowthFront &front)
        {
            SurfaceMesh mesh;
            mesh.vertices.reserve(front.vertices.size());
            for (const GrowthFrontVertex &vertex : front.vertices)
            {
                mesh.vertices.push_back(vertex.position);
            }
            mesh.faces = front.faces;
            return mesh;
        }

        GeneratorResult writeDebugOutput(
            MultiNormalTransitionResult result,
            const MultiNormalDebugOutput &debug)
        {
            if (!debug.enabled)
            {
                return GeneratorResult::success(std::move(result));
            }
            std::error_code directory_error;
            std::filesystem::create_directories(
                debug.directory, directory_error);
            if (directory_error)
            {
                return GeneratorResult::failure(
                    MultiNormalDebugOutputFailure{VtkWriteError{
                        VtkWriteErrorCode::FileOpenFailure,
                        debug.directory, 0, 0}});
            }

            const auto volume_status = writeLegacyVtk(
                debug.directory / debug.transition_filename,
                result.transition_cells);
            if (!volume_status.hasValue())
            {
                return GeneratorResult::failure(
                    MultiNormalDebugOutputFailure{volume_status.error()});
            }
            const auto front_status = writeLegacyVtk(
                debug.directory / debug.front_filename,
                surfaceMesh(result.transformed_front));
            if (!front_status.hasValue())
            {
                return GeneratorResult::failure(
                    MultiNormalDebugOutputFailure{front_status.error()});
            }
            return GeneratorResult::success(std::move(result));
        }
    }

    Result<MultiNormalTransitionResult, MultiNormalError>
    generateMultiNormalTransition(
        const GrowthFront &front,
        const MultiNormalOptions &options)
    {
        MultiNormalTransitionResult unchanged;
        unchanged.transformed_front = front;
        if (!options.enabled)
        {
            return writeDebugOutput(
                std::move(unchanged), options.debug_output);
        }
        if (!std::isfinite(options.transition_height) ||
            options.transition_height <= Scalar{0})
        {
            return GeneratorResult::failure(MultiNormalInputMismatch{
                front.vertices.size(), front.faces.size()});
        }

        const auto evaluation = FrontEvaluator{}.evaluate(front);
        if (!evaluation.hasValue())
        {
            return GeneratorResult::failure(MultiNormalInputMismatch{
                front.vertices.size(), front.faces.size()});
        }
        const auto fans = buildIncidentFaceFans(front, evaluation.value());
        if (!fans.hasValue())
        {
            return GeneratorResult::failure(fans.error());
        }
        const auto plans = planMultiNormalSplits(
            front, fans.value(), options);
        if (!plans.hasValue())
        {
            return GeneratorResult::failure(plans.error());
        }
        if (plans.value().empty())
        {
            return writeDebugOutput(
                std::move(unchanged), options.debug_output);
        }
        MultiNormalTopology marked;
        marked.front = front;
        for (const VertexSplitPlan &plan : plans.value())
            marked.front.vertices[plan.front_vertex_index]
                .multi_normal_branch = true;
        const auto local_triangles = triangulateMultiNormalQuads(marked);
        if (!local_triangles.hasValue())
            return GeneratorResult::failure(local_triangles.error());

        GrowthFront prepared_front = local_triangles.value().front;
        for (GrowthFrontVertex &vertex : prepared_front.vertices)
            vertex.multi_normal_branch = false;
        const auto prepared_evaluation = FrontEvaluator{}.evaluate(
            prepared_front);
        if (!prepared_evaluation.hasValue())
            return GeneratorResult::failure(MultiNormalInputMismatch{
                prepared_front.vertices.size(), prepared_front.faces.size()});
        const auto prepared_fans = buildIncidentFaceFans(
            prepared_front, prepared_evaluation.value());
        if (!prepared_fans.hasValue())
            return GeneratorResult::failure(prepared_fans.error());
        const auto prepared_plans = planMultiNormalSplits(
            prepared_front, prepared_fans.value(), options);
        if (!prepared_plans.hasValue())
            return GeneratorResult::failure(prepared_plans.error());

        std::vector<VertexSplitPlan> selected_plans;
        for (const VertexSplitPlan &prepared : prepared_plans.value())
            if (std::any_of(plans.value().begin(), plans.value().end(),
                    [&](const VertexSplitPlan &original) {
                        return original.front_vertex_index ==
                            prepared.front_vertex_index;
                    }))
                selected_plans.push_back(prepared);
        if (selected_plans.empty())
            return writeDebugOutput(
                std::move(unchanged), options.debug_output);

        const auto topology = buildMultiNormalTopology(
            prepared_front, selected_plans);
        if (!topology.hasValue())
        {
            return GeneratorResult::failure(topology.error());
        }
        const auto triangulated = triangulateMultiNormalQuads(
            topology.value());
        if (!triangulated.hasValue())
        {
            return GeneratorResult::failure(triangulated.error());
        }
        std::vector<Scalar> initial_lengths(
            triangulated.value().front.vertices.size(), Scalar{0});
        for (std::size_t index = 0;
             index < initial_lengths.size(); ++index)
        {
            if (triangulated.value().front.vertices[index]
                    .multi_normal_branch)
            {
                initial_lengths[index] = options.transition_height;
            }
        }
        const auto resolved = resolveMultiNormalLengths(
            triangulated.value(), std::move(initial_lengths), options);
        if (!resolved.hasValue())
        {
            return GeneratorResult::failure(resolved.error());
        }
        if (resolved.value().fallback_to_single_normal)
            return writeDebugOutput(
                std::move(unchanged), options.debug_output);
        MultiNormalOptions resolved_options = options;
        resolved_options.resolved_transition_lengths =
            resolved.value().lengths;
        auto transition = buildMultiNormalTransition(
            triangulated.value(), resolved_options);
        if (!transition.hasValue())
        {
            return GeneratorResult::failure(transition.error());
        }
        return writeDebugOutput(
            std::move(transition.value()), options.debug_output);
    }
}
