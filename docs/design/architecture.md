# new_boundaryMesh 架构设计

## 1. 项目目标

`new_boundaryMesh` 是一个 C++17 边界层体网格生成库。

输入表面网格允许同时包含三角形和四边形：

- 三角形表面正常生长为三棱柱；
- 四边形表面正常生长为六面体；
- 遇到复杂角点、局部碰撞或层数不一致时，允许局部提前停止；
- 使用金字塔和四面体构造共形过渡区域；
- 支持对称边界约束；
- 输出共形的混合体网格。

本项目不保持旧 `HexaMesh` API 兼容。旧工程仅作为算法和输出行为的参考。

## 2. 设计原则

1. 网格数据、几何计算、生长控制、质量评价和过渡构造相互分离。
2. 三角形和四边形使用同一套边界层生长流程。
3. 三棱柱和六面体属于规则边界层单元。
4. 金字塔和四面体属于局部过渡单元。
5. 单元类型必须显式表达，禁止通过节点数量推测类型。
6. 模块之间通过输入和结果传递数据，避免共享大规模可变状态。
7. Eigen 是公共基础依赖。
8. libigl 是固定依赖，但其类型限制在 `geometry` 模块内部。
9. 算法库不直接打印日志，而是返回错误和诊断信息。
10. 每个迁移阶段都必须具备可运行测试。

## 3. 总体目录

```text
new_boundaryMesh/
├── CMakeLists.txt
├── cmake/
├── include/boundary_mesh/
│   ├── core/
│   ├── mesh/
│   ├── geometry/
│   ├── growth/
│   ├── transition/
│   ├── quality/
│   ├── io/
│   └── boundary_layer.hpp
├── src/
│   ├── core/
│   ├── mesh/
│   ├── geometry/
│   ├── growth/
│   ├── transition/
│   ├── quality/
│   └── io/
├── apps/
├── examples/
├── tests/
│   ├── unit/
│   ├── integration/
│   ├── data/
│   └── baselines/
├── third/
└── docs/
    ├── design/
    └── plans/
```

## 4. 模块职责

### 4.1 `core`

负责：

- 标量、点和向量类型；
- 强语义实体 ID；
- 通用 `Result` 和错误类型。

不包含网格算法。Eigen 可以作为 `core` 的公共依赖。

### 4.2 `mesh`

负责：

- 混合三角形/四边形表面网格；
- 四面体、金字塔、三棱柱和六面体；
- 体网格容器；
- 表面拓扑和邻接关系；
- 边界标签；
- 单元来源与层数元数据。

`mesh` 不负责法向、碰撞或边界层生长。

### 4.3 `geometry`

负责无状态或局部状态的几何能力：

- 面法向和节点法向；
- 精确几何谓词；
- 三角形相交；
- AABB 空间查询；
- 最近点查询；
- 对称面方向约束和位置投影；
- 各类体单元的体积和 Jacobian 计算。

libigl 和 geom 依赖限制在本模块内部，不向 `growth` 暴露第三方类型。

### 4.4 `quality`

负责：

- 四面体质量；
- 金字塔质量；
- 三棱柱质量；
- 六面体质量；
- 正体积和 Jacobian 检查；
- skewness、长宽比等质量指标；
- 根据阈值生成接受或拒绝结果。

`quality` 负责评价，不负责决定停止范围。

### 4.5 `growth`

负责规则边界层生长：

- 计算生长方向；
- 计算每层高度；
- 生成候选层节点；
- 应用对称约束；
- 构造候选三棱柱和六面体；
- 调用质量与碰撞检查；
- 接受有效候选单元；
- 记录每个源表面最终生成层数；
- 传播局部停止和相邻层数差约束；
- 维护当前活动前沿。

`growth` 不生成金字塔或四面体过渡单元。

### 4.6 `transition`

负责规则生长结束后的局部共形过渡：

- 检测不同生长层数之间的开口；
- 将相连开口划分为过渡区域；
- 枚举金字塔和四面体候选模板；
- 评价候选模板的拓扑、质量和碰撞；
- 选择有效候选方案；
- 必要时请求 `growth` 扩大局部停止区域并重试；
- 输出最终封闭外表面。

### 4.7 `io`

负责：

- PLY 表面网格读取；
- VTK 混合体网格输出；
- 文件格式与项目网格类型之间的转换。

`io` 不参与生成算法。

## 5. 核心数据模型

### 5.1 基础类型

```cpp
using Scalar = double;
using Point3 = Eigen::Vector3d;
using Vector3 = Eigen::Vector3d;

using VertexId = std::uint32_t;
using EdgeId = std::uint32_t;
using SurfaceFaceId = std::uint32_t;
using VolumeCellId = std::uint32_t;
```

### 5.2 混合表面

```cpp
struct Triangle
{
    std::array<VertexId, 3> vertices;
};

struct Quadrilateral
{
    std::array<VertexId, 4> vertices;
};

using SurfaceFace = std::variant<Triangle, Quadrilateral>;
```

`SurfaceMesh` 保存：

- 顶点坐标；
- 混合表面单元；
- 与每个面直接关联的边界标签。

`SurfaceTopology` 独立保存：

- 点到面的邻接；
- 边到面的邻接；
- 面到面的邻接；
- 边界边；
- 非流形信息。

### 5.3 混合体网格

```cpp
struct Tetrahedron
{
    std::array<VertexId, 4> vertices;
};

struct Pyramid
{
    std::array<VertexId, 5> vertices;
};

struct Prism
{
    std::array<VertexId, 6> vertices;
};

struct Hexahedron
{
    std::array<VertexId, 8> vertices;
};

using VolumeCell =
    std::variant<Tetrahedron, Pyramid, Prism, Hexahedron>;
```

`VolumeMesh` 保存：

- 所有体网格节点；
- 显式类型的混合体单元；
- 单元元数据；
- 最终外边界面。

单元元数据至少包含：

- 规则层单元或过渡单元；
- 对应的源表面面片；
- 所属层数。

## 6. 层节点映射

不得依赖以下隐式编号公式：

```text
源节点编号 + 层号 × 原始节点数量
```

使用 `LayerVertexTable` 显式记录：

```text
(source vertex, layer) -> volume vertex
```

规则层节点和新增过渡节点统一存入 `VolumeMesh::vertices`。金字塔顶点不能伪装为额外坐标层。

## 7. 生长状态

每个源表面面片保存独立状态：

```cpp
struct FaceGrowthState
{
    SurfaceFaceId source_face;
    std::uint32_t requested_layers;
    std::uint32_t accepted_layers;
    StopReason stop_reason;
};
```

停止原因至少包括：

- 完成目标层数；
- 几何无效；
- 质量不合格；
- 发生碰撞；
- 用户限制；
- 相邻层差约束；
- 过渡构造请求回退。

停止传播逻辑由专门的 `TerminationPropagator` 负责。

## 8. 统一生成流程

```text
输入 SurfaceMesh
        ↓
输入验证与 SurfaceTopology 构建
        ↓
计算法向、生长方向和对称约束
        ↓
生成当前层候选节点
        ↓
三角面构造候选 Prism
四边面构造候选 Hexahedron
        ↓
几何有效性检查
        ↓
质量检查
        ↓
活动前沿碰撞检查
        ↓
接受规则单元或记录局部停止
        ↓
传播相邻区域层数约束
        ↓
重复后续层
        ↓
检测 TransitionRegion
        ↓
生成 Pyramid/Tetrahedron 候选模板
        ↓
验证并选择过渡方案
        ↓
必要时请求局部缩短并重试
        ↓
输出 VolumeMesh
```

## 9. `Growth` 与 `Transition` 的交互

`Growth` 输出：

- 已接受的规则层单元；
- 层节点映射；
- 每个源表面的实际生成层数；
- 停止原因；
- 当前外露前沿。

`Transition` 读取 `Growth` 的结果，不直接修改 `Growth` 内部状态。

如果过渡无法构造，`Transition` 返回显式的重试请求：

```cpp
struct GrowthRetryRequest
{
    std::vector<SurfaceFaceId> faces;
    std::uint32_t reduce_to_layer;
    std::string reason;
};
```

顶层生成器负责执行有限次数的回退与重试，避免两个模块相互修改内部成员。

## 10. 候选单元接受流程

候选单元按固定顺序检查：

1. 拓扑完整性；
2. 正体积和几何有效性；
3. 质量阈值；
4. 与活动前沿的碰撞；
5. 接受后更新活动前沿。

失败结果必须携带具体诊断，不能只返回 `false`。

## 11. 错误与诊断

以下情况使整个调用失败：

- 参数非法；
- 输入索引越界；
- 不支持的输入表面；
- 无法处理的非流形拓扑；
- 内部不变量被破坏。

以下情况属于局部诊断：

- 候选单元 Jacobian 为负；
- skewness 超过阈值；
- 候选层发生碰撞；
- 过渡模板质量不合格；
- 局部提前停止；
- 过渡区域触发回退。

算法库不直接使用 `std::cout`、`std::cerr` 或 spdlog。调用方决定如何展示诊断。

## 12. CMake 目标

第一阶段只建立四个主要库目标：

- `BoundaryMesh::Core`
- `BoundaryMesh::Geometry`
- `BoundaryMesh::BoundaryLayer`
- `BoundaryMesh::IO`

依赖方向：

```text
Core
 ↑  ↑
Geometry  IO
 ↑
BoundaryLayer
```

具体约束：

- `Core` 公开依赖 Eigen；
- `Geometry` 私有依赖 libigl 和 geom；
- `BoundaryLayer` 依赖 `Core` 和 `Geometry`；
- `IO` 依赖 `Core`；
- `IO` 不能成为边界层算法的依赖。

依赖来源采用混合方式：

- 如果上层工程已经提供标准 CMake target，则直接复用；
- 否则使用 `third/` 下的本地依赖。

## 13. 测试结构

### 13.1 单元测试

覆盖：

- 混合表面类型；
- 表面邻接；
- 非流形检测；
- 四类体单元质量；
- 层节点映射；
- 停止传播；
- 单个过渡模板。

### 13.2 集成测试

覆盖：

- 纯三角形表面；
- 纯四边形表面；
- 混合三角形和四边形表面；
- 对称边界；
- 局部碰撞；
- 不同层数相邻区域；
- 复杂角点；
- 金字塔过渡；
- 四面体过渡；
- 过渡失败后的局部回退。

## 14. 分阶段迁移

1. 建立工程骨架和测试框架。
2. 实现基础类型和显式混合网格类型。
3. 实现 `SurfaceTopology`。
4. 迁移 `geometry`。
5. 实现四类单元质量评价。
6. 实现无碰撞、等层数的规则生长。
7. 实现动态碰撞和局部停止。
8. 实现过渡区域检测。
9. 实现金字塔和四面体过渡模板。
10. 迁移 PLY、VTK 和命令行程序。
11. 对比旧工程案例并完成回归测试。

每一阶段必须先通过测试，再进入下一阶段。
