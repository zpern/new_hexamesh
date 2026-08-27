#include <cstddef>
#include <set>
#include <vector>

#include <boundary_mesh/multi_normal/detail/blmesh_splitter.hpp>

namespace
{
    using Set = std::set<std::size_t>;
    using Sets = std::vector<Set>;

    bool matches(
        boundary_mesh::detail::BlmeshSplitter &splitter,
        const Set &input,
        const Sets &expected)
    {
        const Sets first = splitter.combinations(input);
        const Sets cached = splitter.combinations(input);
        return first == expected && cached == expected;
    }
}

int main()
{
    boundary_mesh::detail::BlmeshSplitter splitter;
    if (!matches(splitter, {}, {})) return 1;
    if (!matches(splitter, {1}, {})) return 2;
    if (!matches(splitter, {1, 3}, {{1, 3}})) return 3;
    if (!matches(
            splitter,
            {0, 2, 4},
            {{0, 2}, {0, 2, 4}, {0, 4}, {2, 4}}))
    {
        return 4;
    }
    return 0;
}
