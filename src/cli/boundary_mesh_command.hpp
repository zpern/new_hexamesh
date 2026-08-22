#pragma once

#include <cstdint>
#include <filesystem>
#include <iosfwd>
#include <string>
#include <vector>

#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh
{
    struct BoundaryMeshCommandOptions
    {
        std::filesystem::path input; // CGNS 输入文件
        Scalar first_height{}; // 全部 Wall 顶点的第一层高度
        Scalar growth_ratio{}; // 全部 Wall 顶点的层间增长率
        std::uint32_t layer_count{}; // 全部 Wall 顶点的请求层数
        Scalar maximum_skewness{0.95}; // 候选体单元允许的最大 skewness
        std::uint32_t max_neighbor_layer_difference{1}; // 相邻源面的最大层数差
        std::filesystem::path output_prefix; // 两个 VTK 输出的公共前缀
    };

    int runBoundaryMeshCommand(
        const std::vector<std::string> &arguments,
        std::ostream &output,
        std::ostream &error);
}
