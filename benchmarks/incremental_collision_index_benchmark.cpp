#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <vector>

#include <boundary_mesh/spatial/collision_index.hpp>
#include <boundary_mesh/spatial/incremental_collision_index.hpp>

using namespace boundary_mesh;

namespace
{
    CollisionTriangle triangle(std::size_t index)
    {
        const Scalar x = static_cast<Scalar>(index % 300) * Scalar{2};
        const Scalar y = static_cast<Scalar>(index / 300) * Scalar{2};
        CollisionTriangle value;
        value.points = {{{x, y, 0}, {x + 0.4, y, 0}, {x, y + 0.4, 0}}};
        value.vertex_keys = {{{static_cast<VertexId>(index * 3), 0, 0},
                              {static_cast<VertexId>(index * 3 + 1), 0, 0},
                              {static_cast<VertexId>(index * 3 + 2), 0, 0}}};
        value.owner_kind = CollisionOwnerKind::ExposedBoundary;
        value.owner_id = static_cast<std::uint32_t>(index);
        value.boundary_vertex_count = 3;
        std::copy(value.points.begin(), value.points.end(),
                  value.boundary_points.begin());
        std::copy(value.vertex_keys.begin(), value.vertex_keys.end(),
                  value.boundary_vertex_keys.begin());
        return value;
    }

    CollisionTriangle probe(const CollisionTriangle &stored)
    {
        const Scalar x = stored.points[0].x() + Scalar{0.1};
        const Scalar y = stored.points[0].y() + Scalar{0.1};
        CollisionTriangle value;
        value.points = {{{x, y, -0.2}, {x, y, 0.2}, {x, y + 0.1, 0}}};
        value.vertex_keys = {{{900000, 1, 0}, {900001, 1, 0}, {900002, 1, 0}}};
        value.owner_kind = CollisionOwnerKind::LayerCandidate;
        value.boundary_vertex_count = 3;
        std::copy(value.points.begin(), value.points.end(),
                  value.boundary_points.begin());
        std::copy(value.vertex_keys.begin(), value.vertex_keys.end(),
                  value.boundary_vertex_keys.begin());
        return value;
    }
}

int main()
{
    constexpr std::size_t count = 60000;
    constexpr std::size_t churn = count / 10;
    std::vector<CollisionPrimitiveGroup> groups;
    std::vector<CollisionTriangle> active;
    groups.reserve(count);
    active.reserve(count);
    for (std::size_t index = 0; index < count; ++index)
    {
        active.push_back(triangle(index));
        groups.push_back({static_cast<CollisionGroupId>(index), {active.back()}});
    }

    const auto build_started = std::chrono::steady_clock::now();
    auto built = IncrementalCollisionIndex::build(std::move(groups));
    if (!built.hasValue()) return 1;
    auto index = std::move(built.value());
    const auto build_finished = std::chrono::steady_clock::now();

    for (std::size_t id = 0; id < churn; ++id)
        if (!index.eraseGroup(static_cast<CollisionGroupId>(id)).hasValue())
            return 2;
    for (std::size_t id = 0; id < churn; ++id)
    {
        CollisionTriangle replacement = triangle(id);
        replacement.points[0].z() = replacement.points[1].z() =
            replacement.points[2].z() = Scalar{0.01};
        std::copy(replacement.points.begin(), replacement.points.end(),
                  replacement.boundary_points.begin());
        active[id] = replacement;
        if (!index.insertGroup({static_cast<CollisionGroupId>(count + id),
                                {replacement}}).hasValue())
            return 3;
    }
    index.compactInactive();
    const auto update_finished = std::chrono::steady_clock::now();

    std::uint64_t hits{};
    for (const CollisionTriangle &stored : active)
        hits += index.queryIllegalContacts(probe(stored)).size();
    const auto query_finished = std::chrono::steady_clock::now();
    if (hits != count) return 4;

    auto fresh = CollisionIndex::build(active);
    if (!fresh.hasValue()) return 5;
    for (std::size_t sample = 0; sample < count; sample += 997)
        if (index.queryIllegalContacts(probe(active[sample])).size() !=
            fresh.value().queryIllegalContacts(probe(active[sample])).size())
            return 6;

    const auto ms = [](auto first, auto last)
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            last - first).count();
    };
    const auto &diagnostics = index.diagnostics();
    std::cout << "build_ms=" << ms(build_started, build_finished) << '\n'
              << "update_ms=" << ms(build_finished, update_finished) << '\n'
              << "query_ms=" << ms(update_finished, query_finished) << '\n'
              << "rebuilds=" << diagnostics.rebuilds << '\n'
              << "exact_tests=" << diagnostics.exact_tests << '\n'
              << "maximum_leaf_load=" << diagnostics.maximum_leaf_load << '\n';
}
