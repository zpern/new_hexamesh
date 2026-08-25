#include <cstddef>
#include <limits>
#include <optional>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <boundary_mesh/growth/multi_normal_mesh_merge.hpp>

namespace boundary_mesh
{
    Result<VolumeMesh, MultiNormalMergeError>
    mergeMultiNormalAndRegularMeshes(
        const MultiNormalTransitionResult &transition,
        const VolumeMesh &regular)
    {
        using MergeResult = Result<VolumeMesh, MultiNormalMergeError>;
        if (regular.cells.size() != regular.metadata.size() ||
            transition.transition_cells.cells.size() !=
                transition.transition_cells.metadata.size())
        {
            return MergeResult::failure(
                MultiNormalMergeInputMismatch{});
        }
        const auto invalidReference = [](
            const VolumeMesh &mesh)
            -> std::optional<MultiNormalMergeInvalidVertexReference>
        {
            for (std::size_t cell_index = 0;
                 cell_index < mesh.cells.size();
                 ++cell_index)
            {
                std::optional<VertexId> invalid;
                std::visit(
                    [&](const auto &cell)
                    {
                        for (const VertexId id : cell.vertex_ids)
                        {
                            if (static_cast<std::size_t>(id) >=
                                mesh.vertices.size())
                            {
                                invalid = id;
                                return;
                            }
                        }
                    },
                    mesh.cells[cell_index]);
                if (invalid.has_value())
                {
                    return MultiNormalMergeInvalidVertexReference{
                        cell_index, *invalid};
                }
            }
            return std::nullopt;
        };
        if (const auto invalid = invalidReference(
                transition.transition_cells))
        {
            return MergeResult::failure(*invalid);
        }
        if (const auto invalid = invalidReference(regular))
        {
            return MergeResult::failure(*invalid);
        }
        if (!transition.applied)
        {
            return MergeResult::success(regular);
        }

        const std::size_t interface_count =
            transition.transformed_front.vertices.size();
        if (transition.transformed_front_volume_vertex_ids.size() !=
                interface_count ||
            regular.vertices.size() < interface_count)
        {
            return MergeResult::failure(
                MultiNormalMergeInputMismatch{});
        }

        VolumeMesh output = transition.transition_cells;
        std::vector<VertexId> mapping(regular.vertices.size());
        for (std::size_t index = 0; index < interface_count; ++index)
        {
            const VertexId transition_id =
                transition.transformed_front_volume_vertex_ids[index];
            if (static_cast<std::size_t>(transition_id) >=
                    output.vertices.size() ||
                (regular.vertices[index] -
                 transition.transformed_front.vertices[index].position)
                        .norm() > Scalar{1e-12} ||
                (output.vertices[transition_id] - regular.vertices[index])
                        .norm() > Scalar{1e-12})
            {
                return MergeResult::failure(
                    MultiNormalMergeVertexMismatch{index});
            }
            mapping[index] = transition_id;
        }

        const std::size_t added_count =
            regular.vertices.size() - interface_count;
        const std::size_t maximum_count =
            static_cast<std::size_t>(
                std::numeric_limits<VertexId>::max()) + 1;
        if (output.vertices.size() > maximum_count ||
            added_count > maximum_count - output.vertices.size())
        {
            return MergeResult::failure(
                MultiNormalMergeVertexIdOverflow{
                    output.vertices.size() + added_count});
        }
        for (std::size_t index = interface_count;
             index < regular.vertices.size();
             ++index)
        {
            mapping[index] = static_cast<VertexId>(output.vertices.size());
            output.vertices.push_back(regular.vertices[index]);
        }

        for (std::size_t cell_index = 0;
             cell_index < regular.cells.size();
             ++cell_index)
        {
            VolumeCell cell = regular.cells[cell_index];
            bool valid = true;
            VertexId invalid_id{};
            std::visit(
                [&](auto &typed_cell)
                {
                    for (VertexId &id : typed_cell.vertex_ids)
                    {
                        if (static_cast<std::size_t>(id) >= mapping.size())
                        {
                            valid = false;
                            invalid_id = id;
                            return;
                        }
                        id = mapping[id];
                    }
                },
                cell);
            if (!valid)
            {
                return MergeResult::failure(
                    MultiNormalMergeInvalidVertexReference{
                        cell_index, invalid_id});
            }
            output.cells.push_back(std::move(cell));
            output.metadata.push_back(regular.metadata[cell_index]);
        }

        return MergeResult::success(std::move(output));
    }
}
