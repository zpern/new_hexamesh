#include <cassert>

#include <boundary_mesh/transition/incremental_transition_collision_state.hpp>

using namespace boundary_mesh;

namespace
{
    OwnedBoundaryTriangle owned(
        SurfaceFaceId source,
        std::array<CollisionVertexKey, 3> keys,
        Scalar z = 0)
    {
        OwnedBoundaryTriangle value;
        value.points = {{{0,0,z},{1,0,z},{0,1,z}}};
        value.vertex_keys = keys;
        value.owner = {
            source, 2, BoundaryOwnerRole::SideTransition, {source}};
        return value;
    }

    bool sameExposed(
        const std::vector<OwnedBoundaryTriangle> &left,
        const std::vector<OwnedBoundaryTriangle> &right)
    {
        if (left.size() != right.size()) return false;
        for (std::size_t index = 0; index < left.size(); ++index)
            if (transitionTriangleKey(left[index]) !=
                    transitionTriangleKey(right[index]) ||
                layerBoundaryOwnerKey(left[index].owner) !=
                    layerBoundaryOwnerKey(right[index].owner) ||
                left[index].points != right[index].points)
                return false;
        return true;
    }
}

int main()
{
    const std::array<CollisionVertexKey, 3> shared{{
        {0,2,0},{1,2,0},{2,2,0}}};
    const std::array<CollisionVertexKey, 3> other{{
        {3,2,0},{4,2,0},{5,2,0}}};
    TransitionBoundaryChecker checker;

    TransitionBoundaryInput initial;
    initial.candidate_triangles = {
        owned(10, shared), owned(20, shared), owned(30, other)};
    auto state = IncrementalTransitionCollisionState::buildBoundary(initial);
    assert(state.hasValue());
    const auto initial_full = checker.assembleExposedBoundary(initial);
    assert(initial_full.hasValue());
    if (!sameExposed(state.value().exposedBoundary(), initial_full.value()))
        return 1;

    TransitionBoundaryInput reexposed;
    reexposed.candidate_triangles = {
        owned(10, shared), owned(30, other)};
    const auto remove_second = state.value().replaceBoundary(
        reexposed, {{20,2,BoundaryOwnerRole::SideTransition}});
    assert(remove_second.hasValue());
    const auto reexposed_full = checker.assembleExposedBoundary(reexposed);
    assert(reexposed_full.hasValue());
    if (!sameExposed(
            state.value().exposedBoundary(), reexposed_full.value()) ||
        state.value().diagnostics().affected_triangle_keys != 1)
        return 2;

    TransitionBoundaryInput hidden = reexposed;
    hidden.candidate_triangles.insert(
        hidden.candidate_triangles.begin(), owned(40, shared));
    const auto add_second = state.value().replaceBoundary(
        hidden, {{40,2,BoundaryOwnerRole::SideTransition}});
    assert(add_second.hasValue());
    const auto hidden_full = checker.assembleExposedBoundary(hidden);
    assert(hidden_full.hasValue());
    if (!sameExposed(state.value().exposedBoundary(), hidden_full.value()))
        return 3;

    TransitionBoundaryInput odd = hidden;
    odd.candidate_triangles.push_back(owned(60, shared));
    const auto make_odd = state.value().replaceBoundary(
        odd, {{60,2,BoundaryOwnerRole::SideTransition}});
    assert(make_odd.hasValue());
    const auto odd_full = checker.assembleExposedBoundary(odd);
    assert(odd_full.hasValue());
    if (!sameExposed(state.value().exposedBoundary(), odd_full.value()))
        return 4;

    TransitionBoundaryInput replaced = odd;
    replaced.candidate_triangles.erase(replaced.candidate_triangles.begin());
    replaced.candidate_triangles.insert(
        replaced.candidate_triangles.begin(), owned(50, shared, 0.5));
    const auto replace_owner = state.value().replaceBoundary(
        replaced,
        {{40,2,BoundaryOwnerRole::SideTransition},
         {50,2,BoundaryOwnerRole::SideTransition}});
    assert(replace_owner.hasValue());
    const auto replaced_full = checker.assembleExposedBoundary(replaced);
    assert(replaced_full.hasValue());
    if (!sameExposed(state.value().exposedBoundary(), replaced_full.value()))
        return 5;

    TransitionBoundaryInput conflict = replaced;
    conflict.diagonal_requirements = {
        {{7,2},QuadDiagonal::ZeroTwo},
        {{7,2},QuadDiagonal::OneThree}};
    const auto before_error = state.value().exposedBoundary();
    const auto failed = state.value().replaceBoundary(
        conflict, {{50,2,BoundaryOwnerRole::SideTransition}});
    if (failed.hasValue() ||
        !sameExposed(state.value().exposedBoundary(), before_error))
        return 6;

    const std::array<CollisionVertexKey, 3> crossing_a{{
        {10,2,0},{11,2,0},{12,2,0}}};
    const std::array<CollisionVertexKey, 3> crossing_b{{
        {20,2,0},{21,2,0},{22,2,0}}};
    auto first = owned(70, crossing_a);
    first.points = {{{0,0,0},{1,0,0},{0,1,0}}};
    auto second = owned(80, crossing_b);
    second.points = {{{0.25,0.25,-1},{0.25,0.25,1},{1,1,0}}};
    TransitionBoundaryInput crossing;
    crossing.candidate_triangles = {first, second};
    auto collision_state = IncrementalTransitionCollisionState::build(
        crossing);
    assert(collision_state.hasValue());
    const auto crossing_full = checker.inspect(crossing);
    assert(crossing_full.hasValue());
    if (collision_state.value().collisionReport().rollback_faces !=
            crossing_full.value().rollback_faces ||
        collision_state.value().collisionReport().colliding_owners.size() !=
            crossing_full.value().colliding_owners.size())
        return 7;

    TransitionBoundaryInput separated = crossing;
    for (Point3 &point : separated.candidate_triangles[0].points)
        point.z() += 5;
    const auto separated_update = collision_state.value().update(
        separated, {{70,2,BoundaryOwnerRole::SideTransition}});
    assert(separated_update.hasValue());
    const auto separated_full = checker.inspect(separated);
    assert(separated_full.hasValue());
    if (collision_state.value().collisionReport().rollback_faces !=
            separated_full.value().rollback_faces ||
        collision_state.value().collisionReport().colliding_owners.size() !=
            separated_full.value().colliding_owners.size() ||
        collision_state.value().diagnostics().full_collision_builds != 0 ||
        collision_state.value().diagnostics().self_collision_queries != 1)
        return 8;

    TransitionBoundaryInput changed_columns = separated;
    auto columns = std::make_shared<SlidingColumnContext>();
    columns->low_points = {{0,0,0}};
    columns->high_points = {{0,0,1}};
    columns->low_region_ids = {{3}};
    columns->high_region_ids = {{3}};
    changed_columns.candidate_triangles[0].sliding_columns = columns;
    const auto columns_update = collision_state.value().update(
        changed_columns, {{70,2,BoundaryOwnerRole::SideTransition}});
    assert(columns_update.hasValue());
    if (collision_state.value().diagnostics().self_collision_queries != 1)
        return 9;
}
