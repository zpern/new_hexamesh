# 边界层过渡模块收敛设计

## 1. 目标

删除已经被逐层增量算法替代的 reserved-layer 旧路径，消除旧的试生长层数语义，并把当前位于 `transition` 中的完整边界层生成流程迁入独立的 `boundary_layer` 编排模块。

本次修改是明确的破坏性 API 变更，不为 `generateReservedLayerTransition()` 或相关旧类型提供兼容转发层。逐层算法的网格结果、停止规则、碰撞回退和确定性选择行为保持不变。

## 2. 模块边界

依赖方向统一为：

```text
CLI
  ↓
boundary_layer
  ├─ growth
  ├─ transition
  └─ multi_normal
```

各模块职责如下：

- `boundary_layer`：完整边界层用例的公开入口、逐层流程编排和最终网格物化。
- `growth`：规则层候选生成、质量检查、碰撞检查和活动前沿推进。
- `transition`：层差协调、角点压制、模板选择、临时暴露边界检查和回退决策。
- `multi_normal`：多法向起始区生成及其局部过渡。

`growth`、`transition` 和 `multi_normal` 不得依赖 `boundary_layer`。`transition` 不再拥有完整边界层生成入口。

## 3. 目标文件结构

```text
include/boundary_mesh/boundary_layer/
  boundary_layer_generator.hpp
  incremental_boundary_layer_generator.hpp
  incremental_topology_finalizer.hpp

src/boundary_layer/
  boundary_layer_generator.cpp
  incremental_boundary_layer_generator.cpp
  incremental_topology_finalizer.cpp

include/boundary_mesh/transition/
  incremental_transition_types.hpp
  transition_template_types.hpp
  triangle_side_transition.hpp
  incremental_transition_templates.hpp
  provisional_transition_builder.hpp
  layer_transition_resolver.hpp
  corner_suppression.hpp
  accepted_stopped_front_carry.hpp
  transition_boundary_checker.hpp
  quad_high_neighbor_selector.hpp
  quad_diagonal.hpp
  layer_quad_diagonal_table.hpp
```

对应的 transition 实现继续放在 `src/transition/`。

## 4. 边界层编排组件

### 4.1 `boundary_layer_generator`

`generateBoundaryLayers()` 保持为用户和 CLI 的唯一完整生成入口。它负责调用 multi-normal 生成、逐层规则/过渡生成及最终网格合并。

头文件从 `boundary_mesh/transition/boundary_layer_generator.hpp` 迁移到 `boundary_mesh/boundary_layer/boundary_layer_generator.hpp`。旧 include 路径直接删除，不提供转发头文件。

### 4.2 `incremental_boundary_layer_generator`

该组件是内部流程编排器，负责：

1. 为 `RegularLayerGenerator` 安装每层候选拒绝回调；
2. 合并上游拒绝结果和跨层携带的已接受停止面；
3. 建立当前层的停止集合；
4. 调用 `LayerTransitionResolver`；
5. 把固定点结果转换为规则层提交所需的拒绝集合；
6. 规则生长结束后调用最终拓扑物化器。

它不再包含临时过渡几何构造或最终单元物化的具体实现。

### 4.3 `incremental_topology_finalizer`

该组件接收源表面、初始前沿及稳定的 `RegularLayerGrowthResult`，并负责：

- 找到各柱的最终 Hexa/Prism；
- 解析 Quad 规范对角线；
- 将最高层 Hexa 替换为 Pyramid/Tetra 顶盖；
- 生成 Triangle/Quad 一级侧向过渡；
- 形成全三角形顶面；
- 过滤非体网格边界三角形；
- 重建 farfield 边界。

现有 `finalizeIncrementalLayerTopology()` 迁入该组件并保留函数签名，以便模块内部调用和针对性测试。

## 5. Transition 组件收敛

### 5.1 临时过渡构造器

新增 `ProvisionalTransitionBuilder` 或等价自由函数接口。输入为 current front、candidate front、排序后的 retained high faces 和 `LayerFaceSets`；输出为 `ProvisionalLayerTransitionResult`。

它负责：

- 建立当前面、共享边和候选顶点索引；
- 为规则高候选生成碰撞代理面；
- 为 Triangle/Quad 低面选择一级模板；
- 登记规范对角线要求；
- 为顶盖和侧向模板附加准确的回退所有权。

模板错误直接通过返回值传播，不再借助外部 `optional` 副通道。

### 5.2 公共模板类型

新增 `transition_template_types.hpp`，容纳新路径真正共享的类型：

- `TransitionTemplateError`；
- `InvalidTransitionTemplateInput`；
- 新增顶点、体单元、元数据和顶面集合的模板结果类型。

`incremental_transition_templates.hpp` 只声明 Quad 顶盖、单高边和相邻双高边一级模板。

Triangle 模板收敛为 `buildTriangleSideTransition()`。它只表达相邻层差为一的侧向模板，不再暴露 `trial_layers`、`continuing_edge_local_index` 或多阶段 reserved 语义。

### 5.3 协调类型

新路径仍使用的 `TransitionCoordinationError` 及其错误明细迁入 `incremental_transition_types.hpp`，或在实现时若能显著减少依赖则放入独立的 `transition_coordination_types.hpp`。最终选择必须满足：该头文件不依赖任何 reserved 类型，且错误类型只定义一次。

`LayerFaceSets`、`LayerStopState` 和 `StopOrigin` 保留为逐层算法的核心状态。

## 6. 删除范围

删除以下生产文件：

- `include/boundary_mesh/transition/reserved_layer_transition.hpp`
- `src/transition/reserved_layer_transition.cpp`
- `include/boundary_mesh/transition/reserved_layer_growth.hpp`
- `src/transition/reserved_layer_growth.cpp`
- `include/boundary_mesh/transition/transition_coordination.hpp`
- `include/boundary_mesh/transition/transition_templates.hpp`
- `src/transition/quad_transition_template.cpp`

`src/transition/triangle_transition_template.cpp` 由新的一级 Triangle 模板实现替代。删除以下旧符号及语义：

- `generateReservedLayerTransition()`；
- `ReservedLayerTransitionResult`；
- `CombinedReservedLayerTransitionError`；
- `ReservedTransitionLayerCount`；
- `FaceLayerState`；
- `ReservedLayerCountOverflow`；
- `regularLayerCount()`；
- `occupiedLayerCount()`；
- `makeReservedTrialProfiles()`；
- `CoordinatedTransitionFace`；
- `coordinateTransitionFront()`；
- `SourceTransitionResult`；
- `TriangleTransitionInput`；
- `QuadTransitionInput`；
- `buildTriangleTransition()`；
- `buildQuadTransition()`。

同时从 CMake 中移除相应源文件和仅验证旧路径的测试目标。

## 7. 测试迁移

删除只描述 reserved 行为的测试：

- `tests/unit/transition/reserved_layer_growth_test.cpp`
- `tests/integration/reserved_layer_transition_pipeline_test.cpp`
- `tests/cmake/reserved_transition_single_entry_test.cmake`
- 依赖通用旧 Quad 模板的 reserved 模板测试。

保留并迁移对当前生产行为有价值的测试：

- Triangle 单高边模板用例迁到 `triangle_side_transition_test.cpp`；
- Quad 顶盖、单高边和相邻双高边行为由增量模板测试覆盖；
- 新增临时过渡构造器组件测试，覆盖 Triangle、Quad、规范对角线和回退依赖；
- 新增或迁移最终物化器测试，覆盖零层顶盖、最高层 Hexa 剖分、一级台阶和 farfield 重建；
- 保留全部角点压制、碰撞边界、固定点求解及增量管线集成测试。

每个新生产接口先添加一个因接口不存在或行为缺失而失败的测试，再实现最小迁移代码使其通过。纯文件移动通过现有行为测试验证。

## 8. 文档与构建更新

- 当前 CMake 目标 `boundary_mesh_boundary_layer` 实际只包含 `src/growth/`。将其改名为 `boundary_mesh_growth`，公开别名改为 `BoundaryMesh::Growth`，并更新所有直接依赖规则生长组件的目标和测试；
- 新建真正的 `boundary_mesh_boundary_layer` 目标及 `BoundaryMesh::BoundaryLayer` 别名，包含 `src/boundary_layer/`，并依赖 `BoundaryMesh::Growth`、`BoundaryMesh::Transition` 和 `BoundaryMesh::MultiNormal`；
- `BoundaryMesh::Transition` 依赖 `BoundaryMesh::Growth`，不再依赖 `BoundaryMesh::MultiNormal` 或新的顶层 `BoundaryMesh::BoundaryLayer`，从而保持依赖图无环；
- CMake 新增 `src/boundary_layer/` 源文件并删除 reserved 源文件；
- CLI include 改为新的 `boundary_layer` 路径；
- README 只描述 `generateBoundaryLayers()` 当前生产路径；
- `docs/reserved_layer_transition.md` 改名为当前算法名称，删除旧兼容章节；
- 历史 `docs/superpowers/specs/` 和 `docs/superpowers/plans/` 保持不变，作为决策记录；源码清理检查不把历史文档中的旧名称视为残留。

## 9. 错误处理与行为约束

- 所有现有逐层错误继续通过 `IncrementalLayerGrowthError` 返回；
- 模板构造失败不得被静默记录后继续执行；
- 对角线冲突、碰撞回退和停止集合更新规则不变；
- 结果必须保持确定性，不引入依赖哈希遍历顺序的选择；
- 最终外表面继续全部由 Triangle 构成；
- 任意直接相邻源面最终层差继续限制为 0 或 1。

## 10. 完成标准

完成后必须满足：

1. 非历史文档和源码中不存在 reserved API、旧层数类型或旧 include 路径；
2. `transition` 目录不再包含完整边界层生成器；
3. `incremental_boundary_layer_generator.cpp` 只保留编排职责；
4. CLI 通过 `boundary_layer/boundary_layer_generator.hpp` 调用唯一公开入口；
5. Transition 单元测试、增量管线集成测试、完整构建和全量 CTest 全部通过；
6. 构建输出不包含由本次迁移引入的警告。
