# Internal 与 Non-Internal 分层表面拓扑设计

## 目标

`SurfaceTopologyBuilder` 将同一 `SurfaceMesh` 中的面分为两套独立二维拓扑层：

- `SurfaceBoundaryKind::Internal` 属于 internal 层；
- 其余所有 `SurfaceBoundaryKind` 属于 non-internal 层。

两层共享全局唯一的几何 `EdgeId`，但分别进行边关联数、共享边方向和邻接检查。Non-internal 层必须封闭；internal 层允许开放边。

## 不变范围

以下输入检查保持现有顺序和语义：

- `EmptySurface`；
- `FaceTagCountMismatch`；
- `NonFiniteVertex`；
- `InvalidVertexReference`；
- `DegenerateFace`；
- `DuplicateFace`。

`edges()`、`faceEdges()` 和 `vertexFaces()` 保持统一几何语义。尤其是 `vertexFaces()` 继续记录一个顶点关联的全部面，不按拓扑层拆分。

## 公开数据模型

几何边仍由规范化端点确定，同一对端点只创建一个 `EdgeId`。

`EdgeFaceIds` 改为同时保存两套最多两个面的关联：

```cpp
using OptionalSurfaceFaceId =
    std::optional<SurfaceFaceId>;

struct EdgeFaceIds
{
    std::array<OptionalSurfaceFaceId, 2>
        non_internal_faces{};

    std::array<OptionalSurfaceFaceId, 2>
        internal_faces{};
};
```

三角形和四边形的逐边邻接元素改为 `OptionalSurfaceFaceId`。`nullopt` 表示当前面的同层拓扑在该局部边上没有邻面。

公开访问器名称保持不变：`edgeFaces()` 返回完整的双层 incidence，`faceNeighbors()` 返回逐边可选邻面。

## 构建期数据流

构建期为每个全局几何边维护一个 `EdgeIncidence`，分别保存：

- non-internal 面及其局部边方向；
- internal 面及其局部边方向。

扫描每个面时，根据对应 `mesh.face_tags[face_id].kind` 判定层级，并将所有局部边登记到该层。`appendFace()` 仍无条件登记全部 `vertex_faces`，不会过滤 internal 面。

## 边合法性

每一层独立执行以下规则：

1. 第一个关联面直接登记；
2. 第二个关联面必须沿共享边使用相反方向，否则返回 `InconsistentOrientation`；
3. 第三个同层关联面返回 `NonManifoldEdge`。

错误中的面 ID 只来自发生冲突的同一层。Internal 与 non-internal 面之间不比较方向，总关联面数也不用于判断非流形。

因此每条几何边允许最多两个 non-internal 面和最多两个 internal 面；`2 + 1` 与 `2 + 2` 均合法。

## 闭合性

完成全部面扫描后，只检查 non-internal incidence：

- 数量为 1：返回现有 `BoundaryEdge`；
- 数量为 0：该边仅属于 internal 层，合法；
- 数量为 2：non-internal 层在该边封闭，合法。

Internal incidence 数量为 1 表示开放边，不产生错误。`BoundaryEdge` 的注释调整为“non-internal 表面的开放边”。不新增 internal 专用错误类型。

仅包含 internal 面且至少包含一个面的 `SurfaceMesh` 可以成功构建，只要各 internal 边满足流形性和方向规则。

## 邻接语义

`makeNeighbors()` 根据当前面的层级，仅查询同一层 incidence：

- 同层有两个面时，邻面为另一个面；
- internal 层只有当前面时，邻面为 `nullopt`；
- non-internal 层在邻接生成前已通过闭合检查，因而应有两个面。

Internal 与 non-internal 面即使共享同一几何边，也不会互为 `faceNeighbors()` 邻面。

例如 `wall A + wall B + internal C` 共边时：

```text
wall A     -> wall B
wall B     -> wall A
internal C -> nullopt
```

## 下游适配

所有遍历 `FaceNeighborIds` 的调用方必须显式跳过 `nullopt`。当前仓库需要同步检查并适配：

- `src/growth/termination_propagator.cpp`；
- `src/transition/transition_layer_coordinator.cpp`。

`GrowthPatchBuilder`、`TerminationPropagator` 中读取 `vertexFaces()` 的逻辑继续获得全部关联面，无需因本设计拆分 point-to-face 数据。

## 测试

在现有 topology builder 测试基础上补充或更新以下覆盖：

1. 普通封闭 non-internal 混合标签表面仍成功；
2. non-internal 开放边返回 `BoundaryEdge`；
3. 三个 non-internal 面共边返回 `NonManifoldEdge`；
4. 单个 internal 面及 internal 开放 patch 成功；
5. 三个 internal 面共边返回 `NonManifoldEdge`；
6. `wall-wall-internal` 共边成功，并保留 `2 + 1` incidence；
7. `wall-wall-internal` 的 wall 互为邻面，internal 邻面为 `nullopt`；
8. 两个 internal 面反向共边时互为邻面；
9. 两个同向 internal 面返回 `InconsistentOrientation`；
10. internal 与 non-internal 的方向互不检查；
11. 同一顶点关联两类面时，`vertexFaces()` 仍保留全部面；
12. 下游邻接消费者面对 `nullopt` 不越界、不传播伪面 ID。

原有基础合法性错误测试继续运行，以证明扫描顺序和错误语义未回退。

## 兼容性与风险

这是有意的公开 API 变更：`EdgeFaceIds` 的布局和 `FaceNeighborIds` 的元素类型发生变化。仓库内调用方和测试将同步修改；仓库外调用方需要重新编译并适配 optional 邻接及双层 incidence。

实现不得保留一个含义模糊、有损的旧 `edgeFaces()` 兼容视图，也不得用特殊无效 ID 代替 `std::optional`。

最大实现风险是把跨层共边误判为非流形或错误邻接。测试将以 `wall-wall-internal`、`2 + 2` incidence 和跨层同向边作为主要回归场景。
