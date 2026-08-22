#pragma once

namespace boundary_mesh
{
    enum class SpatialError
    {
        NonFiniteCoordinate,      // 碰撞图元包含 NaN 或无穷坐标
        DegenerateTriangle,       // 碰撞三角形面积严格等于零
        InvalidAabb,              // 包围盒下界大于上界
        InvalidTopologyReference, // 碰撞图元引用的顶点或面不存在
        PrimitiveIdOverflow       // 空间图元数量无法由内部编号表达
    };
}
