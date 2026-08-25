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
        const auto topology = buildMultiNormalTopology(
            front, plans.value());
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
