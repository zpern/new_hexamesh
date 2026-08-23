# SurfaceMesh 全局绕向翻转设计

## 1. 目标

当调用者已经确认整个输入表面网格的绕向相反时，提供一个小型辅助函数，统一反转 `SurfaceMesh` 中所有 Triangle 和 Quad 的顶点绕序。

该函数只执行明确的全局翻转，不自动判断表面的内外方向，也不尝试修复局部绕向不一致。

## 2. 公共接口

新增头文件：

```text
include/boundary_mesh/mesh/mesh_surface_orientation.hpp
```

提供原地修改接口：

```cpp
namespace boundary_mesh
{
    void reverseSurfaceOrientation(
        SurfaceMesh &mesh) noexcept;
}
```

使用原地修改是为了避免复制大规模表面网格的顶点、面和标签数组。

## 3. 绕向变换

翻转时保留每个面的第一个顶点，只反转其余顶点的顺序：

```text
Triangle: [v0, v1, v2]     -> [v0, v2, v1]
Quad:     [v0, v1, v2, v3] -> [v0, v3, v2, v1]
```

该变换会反转由右手法则得到的面法向，同时保持面的起始顶点稳定。

## 4. 数据保持规则

函数只修改 `mesh.faces[*].vertex_ids`，以下数据保持不变：

- `mesh.vertices` 的数量、顺序和坐标；
- `mesh.faces` 的数量和顺序；
- 每个面的 Triangle/Quad 类型；
- `mesh.face_tags` 的数量、顺序和内容；
- 所有顶点编号的取值集合。

连续调用两次必须恢复原始 `SurfaceMesh`。

## 5. 文件和构建

新增实现文件：

```text
src/mesh/surface_orientation.cpp
```

该实现加入 `boundary_mesh_core`，因此使用者通过 `BoundaryMesh::Core` 或其上层目标即可获得该函数。

不把实现直接放入 `mesh_surface.hpp`，以保持基础数据结构定义与可执行网格操作分离。

## 6. 错误处理

函数不做拓扑验证，不返回错误：

- Triangle 和 Quad 的顶点数组长度由类型固定；
- 翻转过程不分配内存；
- 即使输入网格为空，调用也合法；
- 输入是否合法仍由 `SurfaceTopologyBuilder` 负责检查。

因此接口声明为 `noexcept`。

## 7. 测试

新增独立单元测试，覆盖：

1. Triangle 从 `[0,1,2]` 变为 `[0,2,1]`；
2. Quad 从 `[0,1,2,3]` 变为 `[0,3,2,1]`；
3. 顶点坐标保持不变；
4. `face_tags` 保持不变；
5. 面的类型和顺序保持不变；
6. 连续翻转两次恢复原始绕向；
7. 空网格调用成功。

实现采用测试驱动流程：先加入无法编译或断言失败的测试并确认 RED，再实现最小函数使其转为 GREEN，最后运行 Debug 和 Release 全量回归。

## 8. 非目标

本任务不包括：

- 判断闭合表面朝内还是朝外；
- 根据体积自动选择绕向；
- 从一个种子面传播并修复局部不一致；
- 只翻转某个边界类别或某些面；
- 修改 CGNS 原文件；
- 在 CLI 中新增自动翻转参数。
