#include <array>
#include <variant>

#include <boundary_mesh/growth/multi_normal_mesh_merge.hpp>

int main()
{
    using namespace boundary_mesh;

    MultiNormalTransitionResult transition;
    transition.applied = true;
    transition.transition_cells.vertices = {
        Point3{0, 0, 0}, Point3{0, 0, 1},
        Point3{1, 0, 1}, Point3{0, 1, 1}};
    transition.transition_cells.cells = {Tetra{{0, 1, 2, 3}}};
    transition.transition_cells.metadata = {
        {CellRole::Transition, 10, 0}};
    transition.transformed_front.vertices = {
        {Point3{0, 0, 1}, 0}, {Point3{1, 0, 1}, 1},
        {Point3{0, 1, 1}, 2}};
    transition.transformed_front_volume_vertex_ids = {1, 2, 3};

    VolumeMesh regular;
    regular.vertices = {
        Point3{0, 0, 1}, Point3{1, 0, 1}, Point3{0, 1, 1},
        Point3{0, 0, 2}, Point3{1, 0, 2}, Point3{0, 1, 2}};
    regular.cells = {Prism{{0, 1, 2, 3, 4, 5}}};
    regular.metadata = {{CellRole::RegularLayer, 10, 1}};

    const auto merged = mergeMultiNormalAndRegularMeshes(
        transition, regular);
    if (!merged.hasValue() || merged.value().vertices.size() != 7 ||
        merged.value().cells.size() != 2 ||
        merged.value().metadata.size() != 2 ||
        merged.value().metadata[0].role != CellRole::Transition ||
        merged.value().metadata[1].role != CellRole::RegularLayer ||
        std::get<Prism>(merged.value().cells[1]).vertex_ids !=
            std::array<VertexId, 6>{1, 2, 3, 4, 5, 6})
    {
        return 1;
    }

    MultiNormalTransitionResult bad_mapping = transition;
    bad_mapping.transformed_front_volume_vertex_ids.pop_back();
    const auto mapping_failure = mergeMultiNormalAndRegularMeshes(
        bad_mapping, regular);
    if (mapping_failure.hasValue() ||
        !std::holds_alternative<MultiNormalMergeInputMismatch>(
            mapping_failure.error()))
    {
        return 2;
    }

    VolumeMesh bad_prefix = regular;
    bad_prefix.vertices[0] = Point3{9, 9, 9};
    const auto prefix_failure = mergeMultiNormalAndRegularMeshes(
        transition, bad_prefix);
    if (prefix_failure.hasValue() ||
        !std::holds_alternative<MultiNormalMergeVertexMismatch>(
            prefix_failure.error()))
    {
        return 3;
    }

    VolumeMesh bad_reference = regular;
    std::get<Prism>(bad_reference.cells[0]).vertex_ids[5] = 99;
    const auto reference_failure = mergeMultiNormalAndRegularMeshes(
        transition, bad_reference);
    if (reference_failure.hasValue() ||
        !std::holds_alternative<MultiNormalMergeInvalidVertexReference>(
            reference_failure.error()))
    {
        return 4;
    }

    VolumeMesh bad_metadata = regular;
    bad_metadata.metadata.clear();
    const auto metadata_failure = mergeMultiNormalAndRegularMeshes(
        transition, bad_metadata);
    if (metadata_failure.hasValue() ||
        !std::holds_alternative<MultiNormalMergeInputMismatch>(
            metadata_failure.error()))
    {
        return 5;
    }

    MultiNormalTransitionResult bad_transition = transition;
    std::get<Tetra>(bad_transition.transition_cells.cells[0]).vertex_ids[3] =
        99;
    const auto transition_reference_failure =
        mergeMultiNormalAndRegularMeshes(bad_transition, regular);
    if (transition_reference_failure.hasValue() ||
        !std::holds_alternative<MultiNormalMergeInvalidVertexReference>(
            transition_reference_failure.error()))
    {
        return 6;
    }

    return 0;
}
