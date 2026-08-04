#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/surface/face_evaluation.hpp>

namespace boundary_mesh
{
    struct SurfaceEvaluationOptions
    {
        Scalar relative_length_tolerance{1e-12}; // 相对当前包围盒对角线的长度容差
    };

    struct FrontFaceEvaluation
    {
        std::size_t front_face_index{}; // 当前前沿面下标
        SurfaceFaceId source_face_id{}; // 对应输入 Wall 面编号
        FaceEvaluation value; // 当前坐标下重新计算的局部几何
    };

    struct FrontEvaluation
    {
        std::uint32_t layer{}; // 本次评价对应的层号
        Scalar characteristic_length{}; // 当前前沿包围盒对角线
        Scalar effective_length_tolerance{}; // 本次实际采用的绝对长度容差
        std::vector<FrontFaceEvaluation> faces; // 按前沿面顺序排列的完整结果
    };
}
