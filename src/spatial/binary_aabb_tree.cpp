#include <boundary_mesh/spatial/binary_aabb_tree.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <numeric>
#include <queue>

namespace boundary_mesh
{
    namespace
    {
        constexpr std::size_t leaf_capacity = 16;

        bool valid(const Aabb &box)
        {
            return box.minimum.allFinite() &&
                   box.maximum.allFinite() &&
                   (box.minimum.array() <= box.maximum.array()).all();
        }

        Aabb unite(const Aabb &left, const Aabb &right)
        {
            return {
                left.minimum.cwiseMin(right.minimum),
                left.maximum.cwiseMax(right.maximum)};
        }

        Scalar squaredDistance(const Aabb &box, const Point3 &point)
        {
            return (point - point.cwiseMax(box.minimum).cwiseMin(box.maximum))
                .squaredNorm();
        }
    }

    Result<BinaryAabbTree, SpatialError> BinaryAabbTree::build(
        const std::vector<Aabb> &primitive_bounds)
    {
        if (primitive_bounds.size() >
            static_cast<std::size_t>(
                std::numeric_limits<std::uint32_t>::max()))
        {
            return Result<BinaryAabbTree, SpatialError>::failure(
                SpatialError::PrimitiveIdOverflow);
        }

        for (const Aabb &box : primitive_bounds)
        {
            if (!valid(box))
            {
                return Result<BinaryAabbTree, SpatialError>::failure(
                    box.minimum.allFinite() && box.maximum.allFinite()
                        ? SpatialError::InvalidAabb
                        : SpatialError::NonFiniteCoordinate);
            }
        }

        BinaryAabbTree tree;
        tree.primitive_bounds_ = primitive_bounds;
        tree.primitive_indices_.resize(primitive_bounds.size());
        std::iota(
            tree.primitive_indices_.begin(),
            tree.primitive_indices_.end(),
            std::size_t{0});

        if (!primitive_bounds.empty())
        {
            tree.nodes_.reserve(primitive_bounds.size() * 2);
            tree.appendNode(0, primitive_bounds.size());
        }

        return Result<BinaryAabbTree, SpatialError>::success(
            std::move(tree));
    }

    std::size_t BinaryAabbTree::appendNode(
        std::size_t first,
        std::size_t last)
    {
        Aabb bounds = primitive_bounds_[primitive_indices_[first]];
        Point3 center_min =
            (bounds.minimum + bounds.maximum) * Scalar{0.5};
        Point3 center_max = center_min;

        for (std::size_t index = first + 1; index < last; ++index)
        {
            const Aabb &box =
                primitive_bounds_[primitive_indices_[index]];
            bounds = unite(bounds, box);
            const Point3 center =
                (box.minimum + box.maximum) * Scalar{0.5};
            center_min = center_min.cwiseMin(center);
            center_max = center_max.cwiseMax(center);
        }

        const std::size_t node_index = nodes_.size();
        nodes_.push_back(Node{});

        const std::size_t count = last - first;
        if (count <= leaf_capacity)
        {
            nodes_[node_index] = Node{
                bounds,
                first,
                count,
                0,
                0,
                true};
            return node_index;
        }

        const Vector3 span = center_max - center_min;
        Eigen::Index axis = 0;
        span.maxCoeff(&axis);

        std::stable_sort(
            primitive_indices_.begin() +
                static_cast<std::ptrdiff_t>(first),
            primitive_indices_.begin() +
                static_cast<std::ptrdiff_t>(last),
            [&](std::size_t left, std::size_t right)
            {
                const Scalar left_center =
                    (primitive_bounds_[left].minimum[axis] +
                     primitive_bounds_[left].maximum[axis]) *
                    Scalar{0.5};
                const Scalar right_center =
                    (primitive_bounds_[right].minimum[axis] +
                     primitive_bounds_[right].maximum[axis]) *
                    Scalar{0.5};
                if (left_center != right_center)
                {
                    return left_center < right_center;
                }
                return left < right;
            });

        const std::size_t middle = first + count / 2;
        const std::size_t left = appendNode(first, middle);
        const std::size_t right = appendNode(middle, last);
        nodes_[node_index] = Node{
            bounds,
            0,
            0,
            left,
            right,
            false};
        return node_index;
    }

    std::vector<std::size_t> BinaryAabbTree::query(
        const Aabb &bounds) const
    {
        std::vector<std::size_t> result;
        if (nodes_.empty() || !valid(bounds))
        {
            return result;
        }

        std::vector<std::size_t> stack{0};
        while (!stack.empty())
        {
            const std::size_t node_index = stack.back();
            stack.pop_back();
            const Node &node = nodes_[node_index];
            if (!overlaps(node.bounds, bounds))
            {
                continue;
            }

            if (node.leaf)
            {
                for (std::size_t offset = 0; offset < node.count; ++offset)
                {
                    const std::size_t primitive =
                        primitive_indices_[node.first + offset];
                    if (overlaps(primitive_bounds_[primitive], bounds))
                    {
                        result.push_back(primitive);
                    }
                }
            }
            else
            {
                stack.push_back(node.right);
                stack.push_back(node.left);
            }
        }

        std::sort(result.begin(), result.end());
        result.erase(
            std::unique(result.begin(), result.end()),
            result.end());
        return result;
    }

    std::size_t BinaryAabbTree::nearest(
        const Point3 &point,
        const std::function<Scalar(std::size_t)> &squared_distance,
        Scalar &best_squared_distance) const
    {
        if (nodes_.empty() || !point.allFinite())
        {
            best_squared_distance = std::numeric_limits<Scalar>::infinity();
            return std::numeric_limits<std::size_t>::max();
        }
        using QueueEntry = std::pair<Scalar, std::size_t>;
        std::priority_queue<
            QueueEntry,
            std::vector<QueueEntry>,
            std::greater<QueueEntry>> queue;
        queue.push({squaredDistance(nodes_[0].bounds, point), 0});
        std::size_t best = std::numeric_limits<std::size_t>::max();
        best_squared_distance = std::numeric_limits<Scalar>::infinity();
        while (!queue.empty())
        {
            const auto [lower_bound, node_index] = queue.top();
            queue.pop();
            if (lower_bound > best_squared_distance) break;
            const Node &node = nodes_[node_index];
            if (node.leaf)
            {
                for (std::size_t offset = 0; offset < node.count; ++offset)
                {
                    const std::size_t primitive =
                        primitive_indices_[node.first + offset];
                    const Scalar distance = squared_distance(primitive);
                    if (distance < best_squared_distance ||
                        (distance == best_squared_distance && primitive < best))
                    {
                        best = primitive;
                        best_squared_distance = distance;
                    }
                }
            }
            else
            {
                for (const std::size_t child : {node.left, node.right})
                {
                    const Scalar distance =
                        squaredDistance(nodes_[child].bounds, point);
                    if (distance <= best_squared_distance)
                        queue.push({distance, child});
                }
            }
        }
        return best;
    }
}
