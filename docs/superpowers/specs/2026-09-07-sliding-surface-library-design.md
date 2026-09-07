# Symmetry/Internal 滑移面库分层设计

## 目标

把 Symmetry 和 Internal 共用的区域识别、曲面表达、方向约束与位置投影从边界层生成模块中分离，形成可独立编译和链接的 `BoundaryMesh::SlidingSurface` 静态库。Internal 双层拓扑继续属于核心网格模块；碰撞继续作为重要的独立模块演进；逐层侧面维护继续属于边界层模块。

本次重构保持现有网格生成行为和输入输出格式不变，不实现新的碰撞算法。

## 模块与依赖

最终依赖方向为：

```text
BoundaryMesh::Core（包含纯边界类型分类）
        |
        +--> BoundaryMesh::Spatial
                    |
                    +--> BoundaryMesh::SlidingSurface
                                |
                                +--> BoundaryMesh::BoundaryLayer

BoundaryMesh::Spatial --------------------> BoundaryMesh::BoundaryLayer
```

`SlidingSurface` 使用 `Spatial` 中通用的三角曲面最近点索引，但不依赖 `CollisionIndex` 或任何边界层类型。`BoundaryLayer` 同时使用滑移约束和碰撞服务。该方向避免循环依赖，也允许碰撞模块以后独立扩展。

## SlidingSurface 库职责

新增以下目录：

```text
include/boundary_mesh/sliding/
src/sliding/
```

现有 `growth/sliding_surface*` 和 `growth/sliding_constraints*` 的通用能力迁入该目录。库提供：

- 按 `region_id` 收集并验证 Symmetry/Internal 区域；
- 构造轴对齐解析平面或一般三角曲面索引；
- 表达单个区域及区域集合；
- 为顶点建立一个或多个线性独立的滑移约束；
- 将方向约束到允许的切向子空间；
- 将候选位置投影到单面、曲面或多面交集；
- 返回与边界层流程无关的滑移错误和投影诊断信息。

库不负责：前沿创建、层高计算、质量判定、碰撞判定、停止传播、体单元生成或边界侧面追踪。

`isSlidingBoundary(SurfaceBoundaryKind)` 是不涉及几何算法的纯边界类型分类。为避免 `Spatial → SlidingSurface → Spatial` 循环，它放在 `BoundaryMesh::Core` 的表面边界类型接口中，由 SlidingSurface、Spatial 和 BoundaryLayer 共同使用。

## 与 BoundaryLayer 的接口解耦

当前 `SlidingConstraintBuilder` 直接接收 `GrowthFront`、`FrontEvaluation`，错误类型也使用 `GrowthDirectionError`。这些类型会导致新库反向依赖 `BoundaryLayer`。

新库改用最小输入模型，例如逐顶点输入只包含：

- 调用方使用的顶点索引；
- 源表面 `VertexId`；
- 该点所属的滑移 `region_id` 集合；
- 前沿特征长度、有效长度容差和层号。

`BoundaryLayer` 内保留一个薄适配器，将 `GrowthFront` 和 `FrontEvaluation` 转换为上述输入，并将滑移库错误映射为现有 `GrowthDirectionError`。现有边界层调用者不需要了解新输入模型。

为控制迁移风险，原有 `boundary_mesh/growth/sliding_*.hpp` 可以在一个兼容阶段保留为转发头或边界层适配器；新代码统一包含 `boundary_mesh/sliding/...`。

## Internal 双层拓扑

Internal 面与非 Internal 面可能共享同一条几何边，但不能因此互相成为拓扑邻面。`BoundaryMesh::Core` 继续保存两套边关联：

```text
non_internal_faces
internal_faces
```

重构会把 `surface_topology_builder.cpp` 中的面分类、分层边关联和同层邻面选择提取为聚焦的 Core 内部组件。公共的 `MeshSurfaceTopology` 数据结构及其语义保持不变，因此这不是 SlidingSurface 库的依赖，也不会改变现有调用方式。

## 碰撞模块边界与扩展点

碰撞属于 `BoundaryMesh::Spatial`，不并入滑移库。当前 Symmetry/Internal 在静态与动态障碍构建时被跳过，这只是现阶段策略，不定义为滑移面的永久语义。

本次把直接写死的分类判断收敛到显式碰撞策略，例如 `CollisionBoundaryPolicy`。默认策略保持当前结果：Symmetry/Internal 暂不进入障碍集合。策略接口应能在后续支持：

- Symmetry 可接触但不可穿透；
- Internal 单面或双面碰撞；
- 按区域启用碰撞；
- 区分接触、相交和穿透；
- 对原始表面与动态生成侧面采用不同规则。

滑移库只提供边界类型及几何投影能力，不决定某个面是否参与碰撞。碰撞策略也不调用边界层生成流程。

## 侧面追踪

`ExposedBoundaryTracker` 和侧面创建继续属于 `BoundaryMesh::BoundaryLayer`，因为它们管理逐层生成状态。它们调用 Core 提供的统一滑移边界分类接口，并继续保留每个侧面的原始 `SurfaceBoundaryKind` 与 `region_id`，不会把 Internal 改写成 Symmetry。

## 构建系统与测试

CMake 新增真实目标 `boundary_mesh_sliding_surface` 和别名 `BoundaryMesh::SlidingSurface`。其公开依赖仅为 `BoundaryMesh::Core` 与 `BoundaryMesh::Spatial`。`BoundaryMesh::BoundaryLayer` 公开链接新库。

测试分为四层：

1. 新库单元测试：区域收集、平面/曲面构造、方向约束、位置投影和多面交线；
2. 构建边界测试：只链接 `BoundaryMesh::SlidingSurface` 的小型可执行文件可以编译运行，防止意外依赖 BoundaryLayer；
3. Core 拓扑回归测试：Internal 与非 Internal 共边时仍保持双层邻接；
4. BoundaryLayer/Spatial 回归测试：现有管线结果不变，默认碰撞策略仍暂时跳过 Symmetry/Internal，侧面 kind 与 region 保持不变。

## 迁移顺序

1. 先增加独立库的构建边界测试，使当前耦合以失败测试呈现；
2. 建立滑移库的中立数据类型与错误类型；
3. 迁移曲面构建和约束算法；
4. 在 BoundaryLayer 中增加输入及错误适配器并切换调用；
5. 提取 Core 内部的双层拓扑辅助组件；
6. 引入显式碰撞边界策略，保持默认行为；
7. 更新侧面追踪中的分类调用、文档和完整回归测试。

## 完成标准

- `BoundaryMesh::SlidingSurface` 可以被独立链接，且不链接 `BoundaryMesh::BoundaryLayer`；
- 新库公共头不包含 `boundary_mesh/growth/` 下的头文件；
- Internal 双层拓扑行为及公共接口保持兼容；
- 碰撞模块仍为独立目标，默认过滤行为有专项测试，并为后续策略扩展保留接口；
- Symmetry/Internal 的滑移分类不再以重复布尔表达式散落在滑移、碰撞和侧面代码中；
- 现有单元测试和集成测试全部通过，生成结果不发生非预期变化。
