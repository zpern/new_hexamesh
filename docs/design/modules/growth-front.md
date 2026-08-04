# GrowthPatch 与动态活动前沿设计

## 1. 目标

本模块为规则边界层生长准备逐层变化的表面状态。

输入是已经通过 `SurfaceTopologyBuilder` 验证的完整 `SurfaceMesh` 和 `SurfaceTopology`。完整表面包含 Wall、Symmetry 和 Farfield，并且必须封闭、流形且方向一致。

本模块负责：

- 从完整表面中提取需要生长的 Wall 面；
- 建立允许开放边界的 `GrowthPatch`；
- 建立第 0 层 `GrowthFront` 和源实体映射；
- 对当前层活动前沿动态计算面积、法向和翘曲；
- 根据当前关联面计算节点方向；
- 应用一个或多个对称面方向约束；
- 在前沿移动后重新计算，而不复用初始表面的旧结果。

本阶段不生成新层节点、Prism、Hexa 或其他体单元。

## 2. 模块边界

### 2.1 `mesh`

`mesh` 继续负责：

- `SurfaceMesh` 数据；
- 完整封闭表面的 `SurfaceTopology`；
- 边界标签；
- 输入索引和离散拓扑验证。

顶点坐标包含 NaN 或无穷值时，任何后续算法都无法运行，因此有限坐标检查加入 `SurfaceTopologyBuilder` 的输入验证。

### 2.2 `surface`

`surface` 提供无状态、可复用的局部表面算法：

- 三角形面积、面积向量、中心和单位法向；
- 四边形面积、面积向量、中心、单位法向和翘曲角；
- 顶点内角；
- 向量归一化前的退化判定。

公开 CMake target 为：

```text
BoundaryMesh::Surface
```

`surface` 不接收完整 `SurfaceMesh`，不保存逐面数组，也不知道层号、源面编号或生长策略。

### 2.3 `growth`

`growth` 负责：

- 选择 Wall 面；
- 建立 Patch 和 Front；
- 决定何时调用 `surface` 算法；
- 将局部算法错误补充为包含层号和源实体 ID 的生长诊断；
- 计算当前前沿节点方向；
- 应用对称约束；
- 管理逐层重新计算的生命周期。

公开 CMake target 为：

```text
BoundaryMesh::BoundaryLayer
```

依赖方向固定为：

```text
BoundaryMesh::Core
        ↑
BoundaryMesh::Surface
        ↑
BoundaryMesh::BoundaryLayer
```

## 3. 输入坐标合法性

`SurfaceTopologyBuilder` 在现有离散检查之外，增加所有顶点坐标的有限性检查：

```cpp
struct NonFiniteVertex
{
    VertexId vertex_id{};
};
```

顶点的三个坐标分量都必须满足：

```cpp
std::isfinite(value)
```

检查全部输入顶点，包括当前没有被面引用的顶点。输入容器中的任何非有限坐标都视为数据损坏，不允许进入后续阶段。

该检查只回答“坐标是否是有限数”，不检查：

- 两个不同 ID 是否具有重合坐标；
- 边长是否接近零；
- 面积是否退化；
- 新一层前沿是否发生折叠。

这些问题可能在生长过程中出现，必须由当前前沿评价逐层检查。

## 4. 无状态表面算法

### 4.1 结果类型

三角形和四边形使用统一结果：

```cpp
struct FaceEvaluation
{
    Point3 centroid{Point3::Zero()};
    Vector3 area_vector{Vector3::Zero()};
    Vector3 unit_normal{Vector3::Zero()};
    Scalar area{};
    Scalar warpage_angle{};
};
```

`area_vector` 保留输入顶点绕序表达的方向。`unit_normal` 是其归一化结果。三角形的 `warpage_angle` 固定为零。

局部算法错误不携带网格实体 ID：

```cpp
enum class FaceEvaluationError
{
    NonFiniteCoordinate,
    DegenerateEdge,
    DegenerateAreaVector
};
```

调用方负责将局部错误包装成带有前沿面编号、源面编号和层号的错误。

### 4.2 三角形

接口概念为：

```cpp
Result<FaceEvaluation, FaceEvaluationError>
evaluateTriangle(
    const Point3& v0,
    const Point3& v1,
    const Point3& v2,
    Scalar length_tolerance);
```

面积向量：

```text
0.5 × (v1 - v0) × (v2 - v0)
```

面积是面积向量的模，中心是三个顶点的平均值。面积向量无法稳定归一化时返回退化错误。

### 4.3 四边形

四边形固定使用对角线 `v0-v2` 拆分：

```text
(v0, v1, v2)
(v0, v2, v3)
```

两个子三角形分别产生面积向量。四边形结果定义为：

- `area`：两个子三角形面积之和；
- `area_vector`：两个子三角形面积向量之和；
- `unit_normal`：统一面积向量归一化；
- `centroid`：四个顶点的平均值；
- `warpage_angle`：两个有效子三角形单位法向的夹角。

任一子三角形退化，或两个面积向量相互抵消导致统一法向失效时，返回退化错误。

固定拆分保证同一输入总是得到相同结果。阶段 03 不根据翘曲角决定停止，只提供后续质量策略需要的量。

### 4.4 顶点内角

节点方向使用当前面在当前顶点处的内角作为权重。`surface` 提供无状态内角计算：

```cpp
Result<Scalar, FaceEvaluationError>
cornerAngle(
    const Point3& previous,
    const Point3& center,
    const Point3& next,
    Scalar length_tolerance);
```

任何一条角边退化时返回错误，不产生 NaN。

## 5. 数值容差

不能使用与模型单位无关的单一绝对常数。

当前前沿根据包围盒对角线计算特征长度：

```text
characteristic_length = bounding_box_diagonal
```

配置：

```cpp
struct SurfaceEvaluationOptions
{
    Scalar relative_length_tolerance{1e-12};
};
```

有效长度容差：

```text
max(characteristic_length × relative_length_tolerance,
    数值安全下限)
```

面积退化判断使用有效长度容差的平方。整个前沿尺度接近零时返回明确错误。

局部 `surface` 函数只接收已经计算好的有效长度容差，不自行猜测全局尺度。

## 6. GrowthPatch

### 6.1 含义

`GrowthPatch` 表示完整表面中本次需要生长的 Wall 子集。

完整 `SurfaceTopology` 必须封闭，但 `GrowthPatch` 可以开放。Wall 与 Symmetry 或 Farfield 的共享边，在完整表面中连接两个面，在 Patch 中则形成合法边界。

### 6.2 数据模型

```cpp
enum class PatchEdgeKind
{
    Interior,
    SymmetryBoundary,
    FarfieldBoundary
};

struct PatchEdge
{
    EdgeId source_edge_id{};
    std::array<SurfaceFaceId, 2> complete_face_ids{};
    PatchEdgeKind kind{PatchEdgeKind::Interior};
};

class GrowthPatch
{
public:
    const std::vector<VertexId>& sourceVertexIds() const noexcept;
    const std::vector<SurfaceFaceId>& sourceFaceIds() const noexcept;
    const std::vector<PatchEdge>& edges() const noexcept;
};
```

`GrowthPatch` 是只读快照，由无状态 `GrowthPatchBuilder` 创建。

### 6.3 确定性顺序

- `source_face_ids` 按 `SurfaceFaceId` 升序；
- `source_vertex_ids` 按 `VertexId` 升序；
- `edges` 按完整表面的稳定 `EdgeId` 升序。

不能通过遍历 `unordered_map` 决定最终顺序。

### 6.4 边界分类

对于 Wall 面使用的每条完整表面边，读取 `SurfaceTopology::edgeFaces()` 和两侧标签：

```text
Wall + Wall      -> Interior
Wall + Symmetry  -> SymmetryBoundary
Wall + Farfield  -> FarfieldBoundary
```

同一边两侧都不是 Wall 时，不属于 GrowthPatch。

阶段 03 选择全部 `SurfaceBoundaryKind::Wall`。按 Wall region 选择不同生长参数属于后续生长配置，不在本阶段裁剪 Patch。

允许 GrowthPatch 包含多个不相连的 Wall 连通分量。没有任何 Wall 面时返回明确错误。

## 7. GrowthFront

### 7.1 含义

`GrowthPatch` 描述源表面中的生长范围，`GrowthFront` 描述该范围在当前层的实际空间位置。

```cpp
struct GrowthFront
{
    std::uint32_t layer{};
    std::vector<Point3> vertices;
    std::vector<SurfaceFace> faces;
    std::vector<VertexId> source_vertex_ids;
    std::vector<SurfaceFaceId> source_face_ids;
};
```

`front.vertices` 使用紧凑局部下标。`front.faces` 中的 `VertexId` 索引当前 `front.vertices`，而不是直接索引原始 `SurfaceMesh::vertices`。

映射不变量：

```text
front.source_vertex_ids[front_vertex_index]
    -> 原始 SurfaceMesh 顶点

front.source_face_ids[front_face_index]
    -> 原始 SurfaceMesh 面
```

禁止使用“源顶点编号 + 层号 × 顶点数量”的隐式公式。

### 7.2 第 0 层构建

`GrowthFrontBuilder::buildInitial()`：

1. 按 GrowthPatch 的稳定源顶点顺序复制坐标；
2. 建立源 `VertexId` 到前沿局部下标的临时映射；
3. 按源面顺序复制 Wall 面并将顶点编号改为前沿局部下标；
4. 保留输入面的顶点绕序；
5. 设置 `layer = 0`。

构建完成后不保存临时哈希映射。

### 7.3 生命周期

阶段 03 只正式构造第 0 层，并使用人工移动后的 Front 验证重新计算。下一层候选节点和正式 Front 更新属于规则生长阶段。

## 8. 当前前沿动态评价

### 8.1 结果

```cpp
struct FrontFaceEvaluation
{
    SurfaceFaceId source_face_id{};
    FaceEvaluation value;
};

struct FrontEvaluation
{
    std::uint32_t layer{};
    Scalar characteristic_length{};
    std::vector<FrontFaceEvaluation> faces;
};
```

`FrontEvaluation` 只对应一次 `GrowthFront` 评价。它不是初始表面的永久缓存，也不能在顶点移动后继续使用。

### 8.2 评价流程

1. 检查当前前沿所有坐标为有限数；
2. 计算当前包围盒和特征长度；
3. 计算有效长度容差；
4. 按前沿面顺序调用三角形或四边形 `surface` 函数；
5. 将局部错误包装成前沿错误；
6. 仅在全部面有效时返回完整结果。

### 8.3 错误信息

```cpp
struct DegenerateFrontFace
{
    std::size_t front_face_index{};
    SurfaceFaceId source_face_id{};
    std::uint32_t layer{};
};

struct NonFiniteFrontVertex
{
    std::size_t front_vertex_index{};
    VertexId source_vertex_id{};
    std::uint32_t layer{};
};
```

错误必须说明问题发生在哪个当前实体、哪个源实体和哪一层。

## 9. 当前节点方向

### 9.1 基础方向

每个前沿顶点收集当前关联面，使用角度加权单位法向：

```text
raw_direction =
    sum(face_unit_normal × corner_angle)

direction = normalize(raw_direction)
```

采用角度加权而不是面积加权，避免混合网格中一个大面完全压制相邻小面，并降低方向对局部三角形/四边形尺寸划分的敏感性。

阶段 03 计算的方向遵循输入面的绕序。沿法向还是逆法向施加层高属于规则生长参数，不在本阶段移动节点。

### 9.2 关联关系

节点关联面必须来自当前 `GrowthFront` 的局部连接关系，不能直接使用完整表面的 `vertexFaces()`。完整表面还包含不参与生长的 Symmetry 和 Farfield 面。

当前方向失去有效模长时返回：

```cpp
struct UndefinedGrowthDirection
{
    std::size_t front_vertex_index{};
    VertexId source_vertex_id{};
    std::uint32_t layer{};
};
```

## 10. 对称约束

### 10.1 约束来源

`PatchEdgeKind::SymmetryBoundary` 保存的完整表面邻接能够确定对应 Symmetry 面。对称面法向通过 `BoundaryMesh::Surface` 的无状态算法计算。

对称面本身不随 Wall 前沿生长，因此其约束平面来自输入表面。阶段 03 只约束方向；候选节点位置投影属于规则生长阶段。

每个 `SurfaceBoundaryKind::Symmetry` region 必须描述一个真实平面。三角形天然共面；四边形以及同一 region 中的多个面必须在相对容差内满足：

- 所有顶点到参考平面的距离接近零；
- 各面单位法向平行或反平行；
- 结合完整表面绕序后用于约束的法向保持确定。

不满足平面一致性时返回 `InvalidSymmetrySurface`，不能用平均法向掩盖弯曲或错误标记的对称边界。

### 10.2 单平面约束

对于单位法向 `n`：

```text
projected = direction - dot(direction, n) × n
```

投影后重新归一化。结果必须位于对称面内。

### 10.3 多平面约束

多个约束不能依赖遍历顺序反复投影。

- 一个独立法向：投影到平面；
- 两个独立法向：允许方向沿两个平面的交线 `cross(n0, n1)`；
- 共线或近共线法向：去重后视为一个约束；
- 三个线性独立法向：不存在非零允许方向，返回过约束错误。

交线方向选择与原始基础方向点积非负的一侧，保证结果确定。

## 11. 错误模型

阶段 03 的错误按职责分层：

```text
SurfaceTopologyError
    输入 SurfaceMesh 的有限坐标错误

FaceEvaluationError
    无实体 ID 的局部表面算法错误

GrowthPatchError
    Wall 选择、标签组合或 Mesh/Topology 不匹配

FrontEvaluationError
    带层号和源实体 ID 的动态前沿错误

GrowthDirectionError
    未定义方向、无效对称平面或对称约束过度
```

所有可预期错误通过 `Result<T, E>` 返回。库代码不打印日志，不返回部分成功对象，也不修改输入。

## 12. 文件职责

```text
include/boundary_mesh/surface/face_evaluation.hpp
    FaceEvaluation、局部错误和无状态表面函数

src/surface/face_evaluation.cpp
    三角形、四边形和顶点内角实现

include/boundary_mesh/growth/growth_patch.hpp
    GrowthPatch、PatchEdge 和边界分类

include/boundary_mesh/growth/growth_patch_builder.hpp
src/growth/growth_patch_builder.cpp
    从完整表面提取 Wall Patch

include/boundary_mesh/growth/growth_front.hpp
    当前层前沿和源实体映射

include/boundary_mesh/growth/growth_front_builder.hpp
src/growth/growth_front_builder.cpp
    第 0 层前沿构建

include/boundary_mesh/growth/front_evaluation.hpp
include/boundary_mesh/growth/front_evaluator.hpp
src/growth/front_evaluator.cpp
    当前层动态表面评价

include/boundary_mesh/growth/growth_direction.hpp
src/growth/growth_direction.cpp
    角度加权方向和对称约束
```

## 13. 测试策略

### 13.1 输入合法性

- NaN 顶点；
- 正无穷和负无穷顶点；
- 错误携带稳定 `VertexId`；
- 现有八项拓扑测试保持通过。

### 13.2 局部表面算法

- 单位直角三角形的面积、中心和法向；
- 平面单位四边形的面积、中心、法向和零翘曲；
- 轻微非共面四边形的确定性翘曲角；
- 共线三角形、重合边和面积向量抵消；
- 不同模型尺度下使用显式容差。

### 13.3 GrowthPatch

使用封闭三棱柱混合表面，同时包含 Wall、Symmetry 和 Farfield：

- 只选择 Wall 面；
- 源实体顺序稳定；
- Wall-Wall 边分类为 Interior；
- Wall-Symmetry 边分类为 SymmetryBoundary；
- Wall-Farfield 边分类为 FarfieldBoundary；
- Patch 开放边界不被当作完整表面错误；
- 无 Wall 面时返回错误。

### 13.4 GrowthFront

- 第 0 层坐标与源顶点一致；
- 前沿顶点编号紧凑；
- 面绕序保持不变；
- 源顶点和源面映射正确；
- 输入 Mesh 和 Patch 不被修改。

### 13.5 动态评价与方向

- 第 0 层平面前沿得到预期法向；
- 人工移动后的第 1 层得到不同面积和法向；
- 第 1 层重合边或退化面返回带层号错误；
- 角度加权方向符合解析结果；
- 单对称面移除法向分量；
- 非平面或同一 region 法向不一致的 Symmetry 输入被拒绝；
- 两个对称面产生确定性交线方向；
- 三个独立约束返回过约束错误。

## 14. 实施任务边界

阶段 03 保持八个独立 TDD Task：

1. `NonFiniteVertex` 输入坐标检查；
2. Triangle/Quad 无状态 `Surface` 算法；
3. GrowthPatch 提取和边界分类；
4. 第 0 层 GrowthFront 与源实体映射；
5. 当前前沿动态面积、法向与退化检查；
6. 角度加权节点方向；
7. 单个及多个对称面约束；
8. 下一层前沿重新计算集成测试。

Task 之间依次提交，严格执行 RED、GREEN、REFACTOR。

## 15. 非目标

阶段 03 不实现：

- 新层候选节点生成；
- 层高序列；
- Prism 或 Hexa；
- `VolumeMesh` 写入；
- 体单元 Jacobian 或质量；
- libigl、AABB、最近点或碰撞；
- 局部停止和停止传播；
- Pyramid 或 Tetra 过渡；
- PLY/VTK IO。

## 16. 完成标准

阶段 03 完成时必须满足：

- `BoundaryMesh::Surface` 可独立链接并且不保存网格状态；
- `BoundaryMesh::BoundaryLayer` 能提取确定性 Wall GrowthPatch；
- 第 0 层 GrowthFront 具有紧凑连接和显式源实体映射；
- 每次 Front 顶点变化后都能重新计算面积和法向；
- 中间层退化返回带源实体和层号的错误；
- 节点方向只使用当前活动前沿的关联面；
- 单个和多个对称约束结果与遍历顺序无关；
- 不生成任何新层节点或体单元；
- 全部既有测试和新增测试通过；
- 工作区通过 `git diff --check`。
