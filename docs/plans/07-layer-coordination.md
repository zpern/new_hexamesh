# Layer Coordination and Stop Propagation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 通过逐源面单调层数上限和确定性共享边传播，使最终相邻层数差不超过外部配置，并输出可供远场体网格生成的完整边界表面。

**Architecture:** `FaceLayerConstraintTable` 保存逐面请求上限、当前允许上限和直接停止原因；`TerminationPropagator` 构造 Patch 共享边图并执行初始化及运行时松弛。Generator 在质量、固定障碍碰撞和同层碰撞三个决策点之间传播并过滤候选，最终通过阶段 06 的外露面构建器生成 `farfield_boundary`。

**Tech Stack:** C++17、CMake 3.20+、Eigen、BoundaryMesh::Spatial、tiger_geom、CTest、MSVC Debug/Release。

## Global Constraints

- 执行时只允许主代理内联完成，不使用子代理。
- 阶段 06、07、10 的计划全部确认以前，不执行本计划。
- `max_neighbor_layer_difference` 默认 1，允许外部传入 0 或任意 `uint32_t`。
- 只沿 GrowthPatch 内共享完整源边传播，不沿仅共享顶点传播。
- 用户请求、质量失败、碰撞和邻接约束全部参与传播。
- 直接停止原因优先，邻接原因不得覆盖直接质量或碰撞原因。
- 允许层数只减不增，不回滚已经提交的历史单元。
- 不生成 Pyramid/Tetra，不识别过渡区域，不处理 Symmetry。
- 最终结果必须包含 `VolumeMesh mesh` 和 `SurfaceMesh farfield_boundary`。
- 新公共字段和枚举值必须带中文 `//` 注释。
- `.superpowers/` 不加入任何提交。
- 每项任务执行 RED、GREEN、全量回归和独立提交。

---

## File Structure

```text
include/boundary_mesh/growth/face_layer_constraint.hpp   逐面上限、种类和查询表
include/boundary_mesh/growth/termination_propagator.hpp  邻接构建和传播接口
src/growth/face_layer_constraint.cpp                     请求上限表构建
src/growth/termination_propagator.cpp                    初始化及动态队列松弛

include/boundary_mesh/growth/regular_layer_growth.hpp       配置与 Neighbor 原因
include/boundary_mesh/growth/regular_layer_growth_error.hpp 约束程序错误
include/boundary_mesh/growth/regular_layer_stepper.hpp      Stepper 接收约束表
src/growth/regular_layer_stepper.cpp                        预推出前上限筛选
src/growth/regular_layer_generator.cpp                      三阶段传播和提交编排

tests/unit/growth/face_layer_constraint_test.cpp          面请求上限
tests/unit/growth/termination_propagator_test.cpp         邻接和松弛
tests/unit/growth/regular_layer_stepper_test.cpp          上限筛选语义
tests/integration/layer_coordination_pipeline_test.cpp    质量/碰撞传播流水线
tests/integration/layer_coordination_failure_test.cpp     事务和错误传播
tests/integration/collision_growth_pipeline_test.cpp      两段碰撞接口衔接
tests/unit/growth/farfield_boundary_builder_test.cpp      协调后最终边界
tests/CMakeLists.txt                                      注册阶段 07 测试

docs/design/roadmap.md                                   完成状态
docs/design/modules/layer-coordination.md                最终接口同步
docs/design/modules/collision-local-stop.md              两段碰撞接口同步
docs/plans/06-collision-local-stop.md                     阶段 06 执行顺序同步
docs/plans/07-layer-coordination.md                       实际提交和验证记录
```

---

### Task 1: 逐面请求上限和公共配置

**Files:**
- Create: `include/boundary_mesh/growth/face_layer_constraint.hpp`
- Create: `src/growth/face_layer_constraint.cpp`
- Modify: `include/boundary_mesh/growth/regular_layer_growth.hpp`
- Modify: `include/boundary_mesh/growth/regular_layer_growth_error.hpp`
- Modify: `CMakeLists.txt`
- Test: `tests/unit/growth/face_layer_constraint_test.cpp`
- Modify: `tests/unit/growth/regular_layer_growth_types_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `GrowthPatch`、初始 `GrowthFront`、`GrowthProfileTable`。
- Produces: `FaceLayerConstraintTable`、`FaceLayerLimitKind`、`max_neighbor_layer_difference` 和 `NeighborLayerConstraint`。

- [ ] **Step 1: 写公共类型和逐面最小值 RED 测试**

```cpp
RegularLayerGrowthOptions options;
assert(options.max_neighbor_layer_difference == 1);
options.max_neighbor_layer_difference = 0;
assert(options.max_neighbor_layer_difference == 0);

const auto table = buildFaceLayerConstraints(patch, front, profiles);
assert(table.hasValue());
assert(table.value().find(face0)->requested_layer_count == 2);
assert(table.value().find(face0)->allowed_layer_count == 2);
assert(table.value().find(face1)->requested_layer_count == 7);
```

`face0` 三个顶点请求分别为 2、5、8；`face1` 四个顶点均请求 7。

- [ ] **Step 2: 运行 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_face_layer_constraint_test boundary_mesh_regular_layer_growth_types_test
```

Expected: FAIL，缺少 `face_layer_constraint.hpp`、配置字段和枚举值。

- [ ] **Step 3: 定义完整公共/内部类型**

```cpp
enum class FaceLayerLimitKind
{
    Requested,          // 当前上限仍等于源面原始请求
    NeighborConstraint, // 当前上限由共享边邻域传播降低
    DirectStop          // 当前上限由质量或碰撞直接降低
};

struct FaceLayerConstraint
{
    SurfaceFaceId source_face_id{}; // 输入 Wall 面编号
    std::uint32_t requested_layer_count{}; // 源面顶点请求的最小层数
    std::uint32_t allowed_layer_count{}; // 当前最大允许层数
    FaceLayerLimitKind limit_kind{FaceLayerLimitKind::Requested}; // 上限来源
    FaceStopReason direct_reason{FaceStopReason::None}; // 直接停止原因
};

class FaceLayerConstraintTable
{
public:
    const FaceLayerConstraint *find(SurfaceFaceId id) const noexcept;
    FaceLayerConstraint *find(SurfaceFaceId id) noexcept;
    const std::vector<FaceLayerConstraint> &entries() const noexcept;
};
```

在 `FaceStopReason` 增加：

```cpp
NeighborLayerConstraint // 因共享边邻域层数上限传播而提前停止
```

在 options 增加：

```cpp
std::uint32_t max_neighbor_layer_difference{1}; // 共享边两侧最大允许层数差
```

新增：

```cpp
struct InvalidFaceConstraintState
{
    SurfaceFaceId source_face_id{}; // 状态不一致的源 Wall 面
    std::uint32_t layer{};          // 发现错误时的目标层号
};
```

并加入 `RegularLayerGrowthError`。

- [ ] **Step 4: 实现逐面请求上限构建**

```cpp
Result<FaceLayerConstraintTable, InvalidFaceConstraintState>
buildFaceLayerConstraints(
    const GrowthPatch &patch,
    const GrowthFront &initial_front,
    const GrowthProfileTable &profiles);
```

每个源面按初始 Front 的源顶点映射查询 profile，取 `layer_count` 最小值。输出按 `source_face_id` 升序。缺失 profile、重复源面、Patch/Front 不一致均返回错误。

- [ ] **Step 5: 运行目标测试和回归**

```powershell
cmake --build build --config Debug --target boundary_mesh_face_layer_constraint_test boundary_mesh_regular_layer_growth_types_test
ctest --test-dir build -C Debug -R "boundary_mesh_(face_layer_constraint|regular_layer_growth_types)_test" --output-on-failure
ctest --test-dir build -C Debug --output-on-failure
```

- [ ] **Step 6: 提交逐面约束基础**

```powershell
git add CMakeLists.txt include/boundary_mesh/growth/face_layer_constraint.hpp src/growth/face_layer_constraint.cpp include/boundary_mesh/growth/regular_layer_growth.hpp include/boundary_mesh/growth/regular_layer_growth_error.hpp tests/CMakeLists.txt tests/unit/growth/face_layer_constraint_test.cpp tests/unit/growth/regular_layer_growth_types_test.cpp
git diff --cached --check
git commit -m "feat: define face layer constraints"
```

---

### Task 2: Patch 共享边邻接与初始化传播

**Files:**
- Create: `include/boundary_mesh/growth/termination_propagator.hpp`
- Create: `src/growth/termination_propagator.cpp`
- Modify: `CMakeLists.txt`
- Test: `tests/unit/growth/termination_propagator_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `GrowthPatch`、`SurfaceTopology`、约束表和最大差值。
- Produces: `TerminationPropagator::build(...)`、只读邻接和 `propagateInitial(...)`。

- [ ] **Step 1: 写邻接语义 RED 测试**

构造 Triangle–Triangle、Triangle–Quad、Quad–Quad 三条共享边和一对仅共享顶点的面：

```cpp
const auto propagator = TerminationPropagator::build(patch, topology);
assert(propagator.hasValue());
assert((propagator.value().neighbors(face0) ==
        std::vector<SurfaceFaceId>{face1}));
assert(propagator.value().neighbors(vertex_only_face).empty());
```

另建 Patch 外 Wall 邻面，断言不进入图。

- [ ] **Step 2: 写差值 0/1/2 初始化传播 RED 测试**

```cpp
assert(propagateChain({2, 10, 10}, 0) == std::vector<std::uint32_t>({2, 2, 2}));
assert(propagateChain({2, 10, 10}, 1) == std::vector<std::uint32_t>({2, 3, 4}));
assert(propagateChain({2, 10, 10}, 2) == std::vector<std::uint32_t>({2, 4, 6}));
```

- [ ] **Step 3: 运行 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_termination_propagator_test
```

Expected: FAIL，缺少 `termination_propagator.hpp`。

- [ ] **Step 4: 实现确定性邻接图**

```cpp
class TerminationPropagator
{
public:
    static Result<TerminationPropagator, InvalidFaceConstraintState>
    build(const GrowthPatch &patch, const SurfaceTopology &topology);

    const std::vector<SurfaceFaceId> &
    neighbors(SurfaceFaceId source_face_id) const;

    Result<std::vector<SurfaceFaceId>, InvalidFaceConstraintState>
    propagateInitial(
        FaceLayerConstraintTable &constraints,
        std::uint32_t max_difference) const;
};
```

只使用 `topology.faceNeighbors()` 中双方都属于 Patch 的关系。邻接按 `SurfaceFaceId` 排序去重，不通过共享顶点表补边。

- [ ] **Step 5: 实现单调队列松弛**

队列使用按 `SurfaceFaceId` 排序的最小堆。候选上限使用 `uint64_t` 计算：

```cpp
const std::uint64_t candidate =
    std::uint64_t(current.allowed_layer_count) + max_difference;
const std::uint32_t bounded = candidate > UINT32_MAX
    ? UINT32_MAX
    : static_cast<std::uint32_t>(candidate);
```

只有 `neighbor.allowed_layer_count > bounded` 时才降低、标为 `NeighborConstraint` 并重新入队。

- [ ] **Step 6: 添加环、多源和顺序不变量测试**

覆盖环形图、两个低层源、断开分量、最大 `uint32_t` 差值，以及反转 Patch 面序/邻接插入序后输出完全相同。

- [ ] **Step 7: 回归并提交**

```powershell
cmake --build build --config Debug --target boundary_mesh_termination_propagator_test
ctest --test-dir build -C Debug -R boundary_mesh_termination_propagator_test --output-on-failure
ctest --test-dir build -C Debug --output-on-failure
git add CMakeLists.txt include/boundary_mesh/growth/termination_propagator.hpp src/growth/termination_propagator.cpp tests/CMakeLists.txt tests/unit/growth/termination_propagator_test.cpp
git diff --cached --check
git commit -m "feat: propagate initial face layer limits"
```

---

### Task 3: 运行时直接停止传播和候选过滤

**Files:**
- Modify: `include/boundary_mesh/growth/termination_propagator.hpp`
- Modify: `src/growth/termination_propagator.cpp`
- Modify: `tests/unit/growth/termination_propagator_test.cpp`

**Interfaces:**
- Consumes: 当前目标层、直接 `FaceStopEvent`、约束表和 `LayerStepResult`。
- Produces: `applyDirectStops(...)` 与 `filterCandidates(...)`。

- [ ] **Step 1: 写动态质量/碰撞传播 RED 测试**

```cpp
const std::vector<FaceStopEvent> direct{
    {0, face0, 5, FaceStopReason::Collision}};
auto changed = propagator.applyDirectStops(constraints, direct, 1);
assert(changed.hasValue());
assert(constraints.find(face0)->allowed_layer_count == 4);
assert(constraints.find(face0)->direct_reason == FaceStopReason::Collision);
assert(constraints.find(face1)->allowed_layer_count == 5);
assert(constraints.find(face2)->allowed_layer_count == 6);
```

增加质量原因版本和同层两个碰撞源版本。

- [ ] **Step 2: 写直接原因优先和候选过滤 RED 测试**

将 face1 先限制为 Neighbor，再在更低层直接碰撞，预期最终原因为 Collision。对 `target_layer == 5`：allowed 4 的候选删除，allowed 5 的候选保留。

- [ ] **Step 3: 运行 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_termination_propagator_test
```

Expected: FAIL，缺少动态传播接口。

- [ ] **Step 4: 实现动态接口**

```cpp
Result<std::vector<SurfaceFaceId>, InvalidFaceConstraintState>
applyDirectStops(
    FaceLayerConstraintTable &constraints,
    const std::vector<FaceStopEvent> &events,
    std::uint32_t max_difference) const;

Result<LayerStepResult, InvalidFaceConstraintState>
filterCandidates(
    const GrowthFront &current_front,
    const LayerStepResult &step,
    const FaceLayerConstraintTable &constraints) const;
```

`applyDirectStops` 先验证全部事件再修改表。直接事件将本面上限设为 `event.layer - 1`、`limit_kind=DirectStop` 并保存原原因，再调用同一传播核心。

`filterCandidates` 删除 `allowed < step.layer` 的 next-front 面，紧凑顶点和映射，并追加唯一的 `NeighborLayerConstraint` 停止事件。`allowed == step.layer` 的候选保留。

- [ ] **Step 5: 验证无覆盖、无重复和顺序不变量**

同一源面重复事件只保留直接原因；邻接传播不得覆盖 DirectStop；输出停止事件按源面 ID 排序。事件或 Front 引用未知源面时返回 `InvalidFaceConstraintState`。

- [ ] **Step 6: 回归并提交**

```powershell
cmake --build build --config Debug --target boundary_mesh_termination_propagator_test
ctest --test-dir build -C Debug -R boundary_mesh_termination_propagator_test --output-on-failure
ctest --test-dir build -C Debug --output-on-failure
git add include/boundary_mesh/growth/termination_propagator.hpp src/growth/termination_propagator.cpp tests/unit/growth/termination_propagator_test.cpp
git diff --cached --check
git commit -m "feat: propagate runtime face stops"
```

---

### Task 4: Stepper 按逐面上限预筛选

**Files:**
- Modify: `include/boundary_mesh/growth/regular_layer_stepper.hpp`
- Modify: `src/growth/regular_layer_stepper.cpp`
- Modify: `tests/unit/growth/regular_layer_stepper_test.cpp`

**Interfaces:**
- Consumes: 当前 Front、逐点 profile、逐面 constraint 和质量 options。
- Produces: 在几何评价前移除已达允许上限面的 `LayerStepResult`。

- [ ] **Step 1: 写正常完成和邻接停止 RED 测试**

```cpp
const auto step = RegularLayerStepper{}.step(
    current_front,
    profiles,
    constraints,
    options);
assert(step.hasValue());
assert(containsCompleted(step.value(), requested_face, FaceStopReason::VertexLayerLimit));
assert(containsStopped(step.value(), constrained_face, FaceStopReason::NeighborLayerConstraint));
```

两张面当前层都等于各自 allowed，但只有第一张 `allowed == requested`。

- [ ] **Step 2: 运行 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_regular_layer_stepper_test
```

Expected: FAIL，Stepper 入口缺少约束表参数。

- [ ] **Step 3: 修改 Stepper 入口和 eligibility**

```cpp
Result<LayerStepResult, RegularLayerGrowthError>
step(
    const GrowthFront &current_front,
    const GrowthProfileTable &profiles,
    const FaceLayerConstraintTable &constraints,
    const RegularLayerGrowthOptions &options = {}) const;
```

对每个当前面先查 constraint。若 `current_front.layer >= allowed_layer_count`，不进入 `eligible_face_indices`：Requested 生成 completed 事件，NeighborConstraint 生成 stopped 事件，DirectStop 面若仍出现在 Front 则返回 `InvalidFaceConstraintState`。

- [ ] **Step 4: 验证预筛选发生在 FrontEvaluator 以前**

构造一个达到上限且几何退化的面；预期返回正常 terminal 事件，而不是 `FrontEvaluationFailure`，证明该面没有进入动态几何计算。

- [ ] **Step 5: 更新全部 Stepper 测试调用并回归**

```powershell
cmake --build build --config Debug --target boundary_mesh_regular_layer_stepper_test
ctest --test-dir build -C Debug -R boundary_mesh_regular_layer_stepper_test --output-on-failure
ctest --test-dir build -C Debug --output-on-failure
```

- [ ] **Step 6: 提交 Stepper 约束接入**

```powershell
git add include/boundary_mesh/growth/regular_layer_stepper.hpp src/growth/regular_layer_stepper.cpp tests/unit/growth/regular_layer_stepper_test.cpp
git diff --cached --check
git commit -m "feat: prefilter constrained growth faces"
```

---

### Task 5: Generator 三阶段传播编排

**Files:**
- Modify: `src/growth/regular_layer_generator.cpp`
- Modify: `tests/integration/collision_growth_pipeline_test.cpp`
- Create: `tests/integration/layer_coordination_pipeline_test.cpp`
- Create: `tests/integration/layer_coordination_failure_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: 两段式 `LayerCollisionChecker`、constraint table、propagator 和 tracker。
- Produces: 满足层数差的 `RegularLayerGrowthResult`。

- [ ] **Step 1: 写差值 0/1 的集成 RED 测试**

案例 A 在目标第 5 层直接质量失败：

```cpp
assert(faceRecord(diff0, face_a).accepted_layer_count == 4);
assert(faceRecord(diff0, face_b).accepted_layer_count == 4);
assert(faceRecord(diff0, face_b).stop_reason ==
       FaceStopReason::NeighborLayerConstraint);

assert(faceRecord(diff1, face_a).accepted_layer_count == 4);
assert(faceRecord(diff1, face_b).accepted_layer_count == 5);
```

- [ ] **Step 2: 写碰撞传播和幽灵候选 RED 测试**

构造 A 撞固定障碍、B 因差值 0 被传播退出、C 只会与 B 同层相撞。预期 A 为 Collision、B 为 NeighborLayerConstraint、C 继续；证明 B 在 `filterSelfCollisions` 前已移除。

- [ ] **Step 3: 运行 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_layer_coordination_pipeline_test
```

Expected: FAIL，新测试目标或传播编排尚不存在。

- [ ] **Step 4: 按固定顺序接入 Generator**

初始化：构建 profiles、constraints、propagator，执行 `propagateInitial`，再进入层循环。

每层调用顺序必须是：

```text
Stepper
applyDirectStops(仅质量 stopped 事件)
filterCandidates
filterAgainstObstacles
applyDirectStops(固定障碍 Collision)
filterCandidates
filterSelfCollisions
applyDirectStops(同层 Collision)
filterCandidates
prepare exposed delta
原子提交
```

`completed_faces` 的请求上限已经包含在初始化表中，不传入 `applyDirectStops`，也不改变直接原因；它们只用于最终记录。质量和碰撞事件才把面标为 `DirectStop`。

- [ ] **Step 5: 保证提交和约束状态一致**

只为最终候选分配新 `VertexId`。所有约束、碰撞和外露 delta 在修改 `VolumeMesh` 以前验证。过滤后无面引用的顶点不得进入网格或 `LayerVertexTable`。

- [ ] **Step 6: 运行流水线和失败事务测试**

失败测试注入未知 source face 的事件和约束表缺失，预期 `InvalidFaceConstraintState`，输入对象不变且当前层没有半提交。

```powershell
cmake --build build --config Debug --target boundary_mesh_layer_coordination_pipeline_test boundary_mesh_layer_coordination_failure_test boundary_mesh_collision_growth_pipeline_test
ctest --test-dir build -C Debug -R "boundary_mesh_(layer_coordination|collision_growth)_.*test" --output-on-failure
```

- [ ] **Step 7: 全量回归并提交**

```powershell
ctest --test-dir build -C Debug --output-on-failure
git add src/growth/regular_layer_generator.cpp tests/CMakeLists.txt tests/integration/collision_growth_pipeline_test.cpp tests/integration/layer_coordination_pipeline_test.cpp tests/integration/layer_coordination_failure_test.cpp
git diff --cached --check
git commit -m "feat: coordinate neighboring layer stops"
```

---

### Task 6: 协调后的最终远场边界

**Files:**
- Modify: `tests/unit/growth/farfield_boundary_builder_test.cpp`
- Modify: `tests/integration/layer_coordination_pipeline_test.cpp`
- Modify: `src/growth/regular_layer_generator.cpp`

**Interfaces:**
- Consumes: 协调后的最终 `ExposedBoundaryTracker` 和原始 `SurfaceMesh`。
- Produces: `RegularLayerGrowthResult::farfield_boundary`。

- [ ] **Step 1: 写层差远场边界 RED 测试**

构造相邻最终层数 4/5 的 Triangle/Quad 混合案例，断言输出包含：原始 Farfield、两个最终顶面和一层台阶侧面；不包含原始 Wall 或内部共享侧面。

```cpp
assert(countKind(result.farfield_boundary,
                 SurfaceBoundaryKind::Farfield) == original_farfield_count);
assert(countKind(result.farfield_boundary,
                 SurfaceBoundaryKind::BoundaryLayerInterface) == expected_interface_count);
assert(hasNoUnreferencedVertices(result.farfield_boundary));
```

- [ ] **Step 2: 运行 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_layer_coordination_pipeline_test
```

Expected: 如果阶段 06 物化没有保留协调后的台阶 owner/region 或绕序，测试失败并指出具体断言。

- [ ] **Step 3: 完善最终物化时机和元数据**

只在层循环和全部传播结束后调用 `buildFarfieldBoundary`。顶面继承源 Wall region；台阶继承较高一侧单元 region；开放侧面继承所属单元 region。接口面反向绕序，Farfield 保持输入绕序。

- [ ] **Step 4: 验证标签、region、绕序和紧凑编号**

对每类输出面验证 `face_tags.size() == faces.size()`；通过法向点积验证 BoundaryLayerInterface 与 tracker 外露面方向相反；所有输出顶点至少被一个面引用。

- [ ] **Step 5: 回归并提交**

```powershell
cmake --build build --config Debug --target boundary_mesh_farfield_boundary_builder_test boundary_mesh_layer_coordination_pipeline_test
ctest --test-dir build -C Debug -R "boundary_mesh_(farfield_boundary_builder|layer_coordination_pipeline)_test" --output-on-failure
ctest --test-dir build -C Debug --output-on-failure
git add src/growth/regular_layer_generator.cpp tests/unit/growth/farfield_boundary_builder_test.cpp tests/integration/layer_coordination_pipeline_test.cpp
git diff --cached --check
git commit -m "test: verify coordinated farfield boundary"
```

---

### Task 7: 确定性、双配置回归与文档收尾

**Files:**
- Modify: `tests/integration/layer_coordination_pipeline_test.cpp`
- Modify: `docs/design/roadmap.md`
- Modify: `docs/design/modules/layer-coordination.md`
- Modify: `docs/design/modules/collision-local-stop.md`
- Modify: `docs/plans/06-collision-local-stop.md`
- Modify: `docs/plans/07-layer-coordination.md`

**Interfaces:**
- Consumes: 阶段 06、07 完整流水线。
- Produces: 输入顺序不变量、完整回归证据和完成状态。

- [ ] **Step 1: 写整图顺序不变量测试**

同一混合 Patch 以正序和反序构建，按 `source_face_id` 比较：

```cpp
assert(sortedFaceRecords(forward) == sortedFaceRecords(reversed));
assert(sortedCellMetadata(forward) == sortedCellMetadata(reversed));
assert(canonicalSurface(forward.farfield_boundary) ==
       canonicalSurface(reversed.farfield_boundary));
```

案例同时包含用户低层请求、质量失败、固定障碍碰撞、同层碰撞、断开分量和仅共点面。

- [ ] **Step 2: 运行目标测试并修正非确定容器遍历**

```powershell
cmake --build build --config Debug --target boundary_mesh_layer_coordination_pipeline_test
ctest --test-dir build -C Debug -R boundary_mesh_layer_coordination_pipeline_test --output-on-failure
```

Expected: PASS；输出面、事件和记录均使用明确排序，不依赖 `unordered_map` 遍历。

- [ ] **Step 3: Debug 完整验证**

```powershell
cmake -S . -B build
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

Expected: 阶段 01–07 全部测试通过，0 failed。

- [ ] **Step 4: Release 完整验证**

```powershell
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Expected: 全部测试通过，0 failed。

- [ ] **Step 5: 更新设计与路线图**

将阶段 07 标记为“已完成”，同步最终接口名称和实际测试命令。在 06 设计/计划中确认碰撞检查已经拆为两个阶段，且远场边界构建器由最终 tracker 物化结果。

- [ ] **Step 6: 提交文档和最终测试**

```powershell
git add tests/integration/layer_coordination_pipeline_test.cpp docs/design/roadmap.md docs/design/modules/layer-coordination.md docs/design/modules/collision-local-stop.md docs/plans/06-collision-local-stop.md docs/plans/07-layer-coordination.md
git diff --cached --check
git commit -m "test: complete layer coordination stage"
git status --short --branch
git log -12 --oneline --decorate
```

Expected: 仅 `.superpowers/` 未跟踪；阶段 07 其他变更全部已提交。

---

## Final Verification

阶段 07 只有在以下条件全部满足时才可完成：

```text
面请求上限等于其顶点请求最小值
共享边传播，只有共点不传播
默认差值 1，外部差值 0 和更大值均有效
初始化请求预传播，运行时停止动态传播
允许层数只降低，不回滚历史单元
质量、固定碰撞、同层碰撞之间均插入传播过滤
传播退出候选不进入同层碰撞树
直接原因优先，邻接原因只用于间接停止
输出不依赖面、队列或空间 primitive 顺序
farfield_boundary 包含 Farfield、最终顶面和全部外露侧面
BoundaryLayerInterface 标签、region、绕序和紧凑编号正确
不实现仅共点传播、Symmetry 或过渡单元
Debug/Release 全量 CTest 均为 0 failed
```
