# SurfaceTopology 设计

## 1. 目标

本模块为混合三角形/四边形 `SurfaceMesh` 构建稳定、只读且可验证的表面拓扑。

成功结果必须提供：

- 稳定的 `EdgeId`；
- 点到面的邻接；
- 边到面的邻接；
- 面到边的邻接；
- 面到面的邻接；
- 足以支持后续法向计算、生长停止传播和过渡区域搜索的数据。

拓扑构建只处理离散连接关系，不计算面积、法向、夹角或碰撞。

## 2. 输入约定

`SurfaceMesh` 保存完整计算域表面，包括：

- `SurfaceBoundaryKind::Wall`；
- `SurfaceBoundaryKind::Symmetry`；
- `SurfaceBoundaryKind::Farfield`。

调用者必须提供真实的对称面网格，而不是只提供壁面与对称边界曲线。完整 `SurfaceMesh` 必须是封闭、流形且方向一致的二维表面。

后续边界层模块只选择需要生长的 `Wall` 面形成 `GrowthPatch`。因此应区分：

- 完整表面中的几何边界边：只连接一个表面面片，本阶段视为错误；
- 生长区域边界：边连接两个完整表面面片，但只有一侧属于 `GrowthPatch`，本阶段不是错误。

例如 Wall 面与 Symmetry 面共享的边在完整 `SurfaceTopology` 中连接两个面；它只在后续生长面子集中表现为区域边界。

## 3. 方案选择

采用“构建期哈希查边，完成后稠密数组存储”的方案。

未采用半边结构，因为本阶段不支持表面增删和局部拓扑修改，半边的不变量与维护成本超过当前需要。

未采用长期保存端点哈希映射的方案，因为后续模块需要稳定 `EdgeId`、确定性遍历和低耦合查询接口。

## 4. 数据模型

### 4.1 边

```cpp
struct Edge
{
    std::array<VertexId, 2> vertex_ids{};
};
```

每条边使用规范端点顺序：

```text
vertex_ids[0] < vertex_ids[1]
```

边的方向只在构建期间用于检查相邻面朝向，不保存在最终 `Edge` 中。

### 4.2 面到边

```cpp
using TriangleEdgeIds = std::array<EdgeId, 3>;
using QuadEdgeIds = std::array<EdgeId, 4>;
using FaceEdgeIds = std::variant<TriangleEdgeIds, QuadEdgeIds>;
```

`FaceEdgeIds` 保持源面的局部边顺序。对于顶点序列 `[v0, v1, v2]`，边顺序为：

```text
v0 -> v1
v1 -> v2
v2 -> v0
```

四边形同理增加 `v3 -> v0`。不得为三角形伪造第四条边或使用无效 `EdgeId` 填充。

### 4.3 面到面

```cpp
using TriangleNeighborIds = std::array<SurfaceFaceId, 3>;
using QuadNeighborIds = std::array<SurfaceFaceId, 4>;
using FaceNeighborIds =
    std::variant<TriangleNeighborIds, QuadNeighborIds>;
```

`face_neighbors[face_id]` 与 `face_edges[face_id]` 按局部下标一一对应。成功结果来自封闭表面，因此每个局部边都具有唯一有效的相邻面，不使用 `-1` 或无效 ID。

### 4.4 拓扑快照

`SurfaceTopology` 内部保存：

```text
edges             EdgeId -> 两个端点
edge_faces        EdgeId -> 两个相邻面
face_edges        SurfaceFaceId -> 3或4个EdgeId
face_neighbors    SurfaceFaceId -> 跨每条局部边的相邻面
vertex_faces      VertexId -> 所有关联面
```

`SurfaceTopology` 使用私有数据和只读访问器，不提供添加、删除或修改实体的公开接口。`SurfaceMesh` 发生改变后必须重新构建拓扑。

构建后的拓扑不持有 `SurfaceMesh` 引用，避免生命周期耦合。调用方负责保证配套网格在使用拓扑期间没有改变。

## 5. 稳定 EdgeId

`EdgeId` 按以下规则确定：

1. 按 `SurfaceFaceId` 从小到大扫描面；
2. 按源面的局部边顺序扫描；
3. 规范边第一次出现时分配下一个 `EdgeId`；
4. 后续遇到相同规范边时复用已有 ID。

构建期使用：

```text
(min(vertex_a, vertex_b), max(vertex_a, vertex_b)) -> EdgeId
```

哈希表只用于查重，不能通过遍历哈希表决定最终 ID。因此同一输入总是得到相同的 `EdgeId` 和邻接顺序。

## 6. 构建流程

`SurfaceTopologyBuilder` 是无状态构建器：

```cpp
Result<SurfaceTopology, SurfaceTopologyError>
SurfaceTopologyBuilder::build(const SurfaceMesh& mesh) const;
```

构建顺序固定为：

1. 检查顶点、面和标签数组的基本尺寸关系；
2. 按面 ID 验证顶点引用和面内顶点唯一性；
3. 检测重复面；
4. 构建 `vertex_faces`；
5. 扫描局部有向边并分配或复用 `EdgeId`；
6. 在第二个面使用同一边时检查方向相反；
7. 在第三个面使用同一边时报告非流形错误；
8. 扫描完成后拒绝只连接一个面的边；
9. 从 `edge_faces` 生成 `face_neighbors`；
10. 仅在全部不变量成立时返回完整拓扑。

构建复杂度目标为：

```text
期望时间：O(所有面的边数)
空间：O(顶点数 + 边数 + 面数 + 邻接项数)
```

## 7. 验证边界

本阶段验证离散拓扑：

- 表面非空；
- `faces.size() == face_tags.size()`；
- 所有顶点 ID 均在 `vertices` 范围内；
- 三角形和四边形的面内顶点不重复；
- 不存在重复面；
- 每条边恰好连接两个不同面；
- 每条共享边两侧的局部方向相反。

本阶段不验证：

- 三个点或四个点是否共面；
- 面积是否接近零；
- 自相交；
- 面法向和外法向；
- 二面角或特征边；
- Wall、Symmetry、Farfield 标签组合是否适合某次边界层生长。

这些内容分别属于 geometry、quality 或后续生长输入准备模块。

## 8. 错误模型

新增 C++17 通用结果类型：

```cpp
template<class T, class E>
class Result;
```

它明确保存一个成功值或一个错误值，提供 `hasValue()`、`value()` 和 `error()`，避免用异常表达可预期的输入验证失败。调用错误分支的访问器属于程序错误并抛出 `std::logic_error`；正常控制流必须先检查 `hasValue()`。

拓扑错误使用显式变体：

```cpp
using SurfaceTopologyError = std::variant<
    EmptySurface,
    FaceTagCountMismatch,
    InvalidVertexReference,
    DegenerateFace,
    DuplicateFace,
    BoundaryEdge,
    NonManifoldEdge,
    InconsistentOrientation>;
```

每个错误结构只携带相关实体 ID。例如方向错误携带规范边端点和两个面 ID。构建器按确定顺序遇到第一个错误后立即返回，不输出日志，不返回部分拓扑，也不修改输入面顺序。

## 9. CMake 与实现边界

拓扑构建包含非平凡实现，不放入公开头文件。实现阶段将 `boundary_mesh_core` 从纯 `INTERFACE` 目标改为包含 `.cpp` 的静态库，同时保留公开别名：

```text
BoundaryMesh::Core
```

文件职责：

```text
include/boundary_mesh/core/result.hpp
    通用 Result<T, E>

include/boundary_mesh/mesh/surface_topology.hpp
    Edge、面邻接类型和 SurfaceTopology 只读接口

include/boundary_mesh/mesh/surface_topology_error.hpp
    显式拓扑错误类型

include/boundary_mesh/mesh/surface_topology_builder.hpp
    无状态构建器公开接口

src/mesh/surface_topology_builder.cpp
    构建、验证和临时哈希实现
```

公开头文件不暴露临时 `unordered_map`、哈希函数或构建累加器。

## 10. 测试策略

### 10.1 小型有效案例

主单元测试使用封闭三棱柱外壳：

```text
6个顶点
2个三角形端面
3个四边形侧面
9条边
5个面
每条边连接2个面
```

该案例验证：

- 三角形/四边形混合；
- 稳定 `EdgeId`；
- `edge_faces`；
- `face_edges`；
- `face_neighbors`；
- `vertex_faces`。

### 10.2 失败案例

每类错误使用最小人工网格独立测试：

- 空表面；
- 标签数量不匹配；
- 顶点编号越界；
- 面内重复顶点；
- 重复面；
- 缺少一个面的开放外壳；
- 一条边连接三个面；
- 两个相邻面沿共享边方向相同。

每个功能按 RED、GREEN、REFACTOR 顺序实现，并在独立可验证节点提交。

### 10.3 大型真实案例

本地文件 `2dot5_cf_gmsh_recombine.ply` 已确认包含：

```text
52010个顶点
58599个面
13186个三角形
45413个四边形
110605条边
0条边界边
0条非流形边
0个方向不一致共享边
```

该文件当前位于构建目录，且新工程尚未实现正式 PLY 读取模块。本模块不增加临时读取器，也不依赖该路径。等 IO 阶段完成后，再将它登记为大型集成与性能回归案例。

## 11. 非目标

本模块不实现：

- PLY 读取；
- 面法向或节点法向；
- 特征边识别；
- `GrowthPatch`；
- 对称方向约束；
- 表面拓扑增量修改；
- 半边结构；
- 边界层单元生成。

## 12. 完成标准

本模块完成时必须满足：

- `BoundaryMesh::Core` 可作为编译库独立构建；
- 有效封闭混合表面能得到确定性的完整拓扑；
- 所有列出的无效拓扑返回精确错误；
- 构建器不修改 `SurfaceMesh`；
- 公开接口不泄漏构建期哈希实现；
- 全部已有测试和新增测试在 Debug 配置下通过；
- 工作区通过 `git diff --check`。
