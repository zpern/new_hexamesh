#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>

#include <boundary_mesh/multi_normal/blmesh_topology_stitching.hpp>

namespace boundary_mesh
{
    Result<std::vector<BlmeshDirectedTriangle>, MultiNormalError>
    orderBlmeshDirectedTriangleChain(
        const std::vector<BlmeshDirectedTriangle> &triangles)
    {
        using ChainResult = Result<std::vector<BlmeshDirectedTriangle>, MultiNormalError>;
        if (triangles.empty()) return ChainResult::success({});
        std::map<int, int> starts, ends;
        for (int i = 0; i < static_cast<int>(triangles.size()); ++i)
        {
            const auto &triangle = triangles[i];
            if (triangle.start_point == triangle.end_point ||
                !starts.emplace(triangle.start_point, i).second ||
                !ends.emplace(triangle.end_point, i).second)
                return ChainResult::failure(InvalidMultiNormalTopology{});
        }
        std::vector<int> first;
        for (int i = 0; i < static_cast<int>(triangles.size()); ++i)
            if (ends.find(triangles[i].start_point) == ends.end())
                first.push_back(i);
        if (first.size() != 1)
            return ChainResult::failure(InvalidMultiNormalTopology{});
        std::vector<BlmeshDirectedTriangle> ordered;
        std::vector<bool> visited(triangles.size(), false);
        int current = first.front();
        while (true)
        {
            if (visited[current])
                return ChainResult::failure(InvalidMultiNormalTopology{});
            visited[current] = true;
            ordered.push_back(triangles[current]);
            const auto next = starts.find(triangles[current].end_point);
            if (next == starts.end()) break;
            current = next->second;
        }
        if (ordered.size() != triangles.size())
            return ChainResult::failure(InvalidMultiNormalTopology{});
        return ChainResult::success(std::move(ordered));
    }

    Result<std::vector<int>, MultiNormalError>
    findBlmeshSmoothestInterleaving(
        const std::vector<Vector3> &left,
        const std::vector<Vector3> &right)
    {
        using InterleaveResult = Result<std::vector<int>, MultiNormalError>;
        if (left.empty() || right.empty())
            return InterleaveResult::failure(InvalidMultiNormalTopology{});
        constexpr int LEFT = 0, RIGHT = 1;
        struct State { Scalar cost{std::numeric_limits<Scalar>::infinity()}; int previous_side{-1}; };
        const int lc = static_cast<int>(left.size()), rc = static_cast<int>(right.size());
        std::vector<std::vector<std::array<State, 2>>> states(
            lc + 1, std::vector<std::array<State, 2>>(rc + 1));
        states[1][0][LEFT].cost = 0;
        states[0][1][RIGHT].cost = 0;
        const auto last = [&](int lu, int ru, int side) -> const Vector3 & {
            return side == LEFT ? left[lu - 1] : right[rc - ru];
        };
        for (int lu = 0; lu <= lc; ++lu) for (int ru = 0; ru <= rc; ++ru)
            for (int side = LEFT; side <= RIGHT; ++side)
            {
                const State &state = states[lu][ru][side];
                if (!std::isfinite(state.cost)) continue;
                const Vector3 &previous = last(lu, ru, side);
                if (lu < lc)
                {
                    State &next = states[lu + 1][ru][LEFT];
                    const Scalar cost = state.cost + std::abs(previous.dot(left[lu]) - 1);
                    if (cost < next.cost) { next.cost = cost; next.previous_side = side; }
                }
                if (ru < rc)
                {
                    State &next = states[lu][ru + 1][RIGHT];
                    const Vector3 &normal = right[rc - ru - 1];
                    const Scalar cost = state.cost + std::abs(previous.dot(normal) - 1);
                    if (cost < next.cost) { next.cost = cost; next.previous_side = side; }
                }
            }
        int lu = lc, ru = rc;
        int side = states[lu][ru][LEFT].cost <= states[lu][ru][RIGHT].cost ? LEFT : RIGHT;
        std::vector<int> reversed;
        while (lu > 0 || ru > 0)
        {
            const int previous = states[lu][ru][side].previous_side;
            if (side == LEFT) { reversed.push_back(lu); --lu; }
            else { reversed.push_back(-(rc - ru + 1)); --ru; }
            side = previous;
        }
        std::reverse(reversed.begin(), reversed.end());
        return InterleaveResult::success(std::move(reversed));
    }
}
