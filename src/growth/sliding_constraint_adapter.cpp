#include <boundary_mesh/growth/sliding_constraint_adapter.hpp>

#include <utility>
#include <variant>
#include <vector>

#include <boundary_mesh/sliding/sliding_constraint_builder.hpp>
#include <boundary_mesh/sliding/sliding_vertex_input.hpp>

namespace boundary_mesh
{
    GrowthDirectionError toGrowthDirectionError(
        const SlidingError &error)
    {
        return std::visit(
            [](const auto &value) -> GrowthDirectionError
            {
                return value;
            },
            error);
    }

    Result<SlidingConstraints, GrowthDirectionError>
    buildGrowthSlidingConstraints(
        const SlidingSurfaceSet &surfaces,
        const GrowthFront &front,
        const FrontEvaluation &evaluation)
    {
        using AdapterResult =
            Result<SlidingConstraints, GrowthDirectionError>;

        if (front.layer != evaluation.layer)
        {
            return AdapterResult::failure(DirectionInputMismatch{
                front.layer, evaluation.layer});
        }

        std::vector<SlidingVertexInput> inputs;
        inputs.reserve(front.vertices.size());
        for (std::size_t index = 0;
             index < front.vertices.size();
             ++index)
        {
            const GrowthFrontVertex &vertex = front.vertices[index];
            inputs.push_back(SlidingVertexInput{
                index,
                vertex.source_vertex_id,
                vertex.boundary.sliding_region_ids});
        }

        auto result = SlidingConstraintBuilder{}.build(
            surfaces,
            inputs,
            evaluation.characteristic_length,
            evaluation.effective_length_tolerance,
            front.layer);
        if (result.hasValue())
        {
            return AdapterResult::success(std::move(result.value()));
        }

        return AdapterResult::failure(
            toGrowthDirectionError(result.error()));
    }
}
