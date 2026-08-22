#include <algorithm>
#include <cassert>
#include <cstddef>
#include <vector>

#include <boundary_mesh/spatial/binary_aabb_tree.hpp>

using namespace boundary_mesh;

int main()
{
    const std::vector<Aabb> boxes{
        {{0.0, 0.0, 0.0}, {1.0, 1.0, 1.0}},
        {{3.0, 0.0, 0.0}, {4.0, 1.0, 1.0}},
        {{1.0, 1.0, 1.0}, {2.0, 2.0, 2.0}}};

    const auto tree = BinaryAabbTree::build(boxes);
    assert(tree.hasValue());
    assert((tree.value().query(
                {{0.5, 0.5, 0.5}, {1.0, 1.0, 1.0}}) ==
            std::vector<std::size_t>{0, 2}));
    assert(tree.value().query(
               {{10.0, 10.0, 10.0}, {11.0, 11.0, 11.0}})
               .empty());

    const auto empty = BinaryAabbTree::build({});
    assert(empty.hasValue());
    assert(empty.value().query(boxes.front()).empty());

    std::vector<Aabb> many;
    many.reserve(4096);
    for (std::size_t index = 0; index < 4096; ++index)
    {
        const double x = static_cast<double>(index);
        many.push_back({{x, 0.0, 0.0}, {x + 0.5, 1.0, 1.0}});
    }

    const auto large_tree = BinaryAabbTree::build(many);
    assert(large_tree.hasValue());

    for (std::size_t index = 0; index < many.size(); index += 31)
    {
        const Aabb query{
            {static_cast<double>(index), 0.5, 0.5},
            {static_cast<double>(index) + 1.0, 0.5, 0.5}};
        std::vector<std::size_t> expected;
        for (std::size_t primitive = 0; primitive < many.size(); ++primitive)
        {
            if (overlaps(many[primitive], query))
            {
                expected.push_back(primitive);
            }
        }
        assert(large_tree.value().query(query) == expected);
    }
}
