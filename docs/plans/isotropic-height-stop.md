# Isotropic Height Stop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在候选 Prism/Hexa 的平均侧边长度与当前底面尺度之比达到外部阈值时，保留当前单元并停止该源面后续生长。

**Architecture:** `RegularLayerStepper` 在质量通过后用当前层动态底面面积计算各向同性比值，并通过 `accepted_stopped_faces` 表达“已接受但本层后终止”。候选单元继续经过邻域传播和碰撞过滤，`RegularLayerGenerator` 提交幸存单元后再从下一层活动前沿中剔除各向同性停止面，同时保留其顶面作为最终外露边界。

**Tech Stack:** C++17、Eigen、CMake、CTest、MSVC，多配置 Debug/Release 构建。

## Global Constraints

- 判据固定为 `average_side_length / sqrt(current_base_area) >= isotropic_height`。
- Prism 使用 3 条对应侧边，Hexa 使用 4 条对应侧边。
- `current_base_area` 必须来自当前层动态前沿评估。
- 当前候选单元通过质量和碰撞检查后必须保留；停止面不得进入下一层活动前沿。
- 默认 `isotropic_height` 为 `1.0`，外部值必须有限且严格大于 `0`。
- 各向同性停止必须参与 `max_neighbor_layer_difference` 传播。
- 不修改 skewness、碰撞、平滑和过渡单元算法。
- 使用 TDD；每个生产行为必须先观察对应测试因功能缺失而失败。
- 不提交 `.superpowers/`。

---

### Task 1: 公共类型、参数验证与单层分类

**Files:**
- Modify: `include/boundary_mesh/growth/regular_layer_growth.hpp`
- Modify: `include/boundary_mesh/growth/regular_layer_growth_error.hpp`
- Modify: `src/growth/regular_layer_stepper.cpp`
- Test: `tests/unit/growth/regular_layer_growth_types_test.cpp`
- Test: `tests/unit/growth/regular_layer_stepper_test.cpp`
- Test: `tests/integration/regular_layer_growth_failure_test.cpp`

**Interfaces:**
- Consumes: `FrontEvaluation::faces[i].value.area`、当前/候选 `GrowthFrontVertex::position`、`RegularLayerGrowthOptions`。
- Produces: `RegularLayerGrowthOptions::isotropic_height`、`FaceStopReason::IsotropicHeightReached`、`LayerStepResult::accepted_stopped_faces`、`InvalidIsotropicHeight`。

- [ ] **Step 1: 写公共类型和默认值失败测试**

在 `regular_layer_growth_types_test.cpp` 断言默认阈值和新事件集合：

```cpp
const RegularLayerGrowthOptions options;
if (options.isotropic_height != Scalar{1}) return 5;

const LayerStepResult step;
if (!step.accepted_stopped_faces.empty()) return 6;

static_assert(static_cast<std::size_t>(
    FaceStopReason::IsotropicHeightReached) == 8);
```

在 `regular_layer_growth_failure_test.cpp` 分别用 `0`、负数、NaN 和无穷值调用 `generateRegularLayers`，要求返回 `InvalidIsotropicHeight`，且输入前沿不被修改。

- [ ] **Step 2: 运行目标测试并确认 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_regular_layer_growth_types_test boundary_mesh_regular_layer_growth_failure_test
ctest --test-dir build -C Debug -R "boundary_mesh_regular_layer_growth_(types|failure)_test" --output-on-failure
```

预期：编译因缺少 `isotropic_height`、`accepted_stopped_faces`、`IsotropicHeightReached` 和 `InvalidIsotropicHeight` 失败。

- [ ] **Step 3: 增加最小公共类型和配置验证**

在 `regular_layer_growth.hpp` 追加停止原因以保持现有枚举序号稳定，并增加参数和事件集合：

```cpp
enum class FaceStopReason
{
    // 保留已有 0..7 项
    IsotropicHeightReached // 当前单元已接受，达到各向同性阈值后停止
};

struct LayerStepResult
{
    // 保留现有字段
    std::vector<FaceStopEvent> accepted_stopped_faces;
};

struct RegularLayerGrowthOptions
{
    VolumeCellQualityOptions cell_quality;
    std::uint32_t max_neighbor_layer_difference{1};
    Scalar isotropic_height{1};
};
```

在错误变体中增加：

```cpp
struct InvalidIsotropicHeight
{
    Scalar value{}; // 非有限或非正的各向同性停止阈值
};
```

`RegularLayerStepper::step` 在任何前沿计算前验证：

```cpp
if (!std::isfinite(options.isotropic_height) ||
    options.isotropic_height <= Scalar{0})
{
    return StepResult::failure(
        InvalidIsotropicHeight{options.isotropic_height});
}
```

- [ ] **Step 4: 运行类型和错误测试并确认 GREEN**

运行 Step 2 的命令，预期两个测试通过。

- [ ] **Step 5: 写 Prism/Hexa 判据失败测试**

在 `regular_layer_stepper_test.cpp` 增加三个独立行为：

```cpp
// 单位直角三角形面积 0.5，侧边 0.25：比值小于 1，继续。
options.isotropic_height = Scalar{1};
assert(step.next_front.faces.size() == 1);
assert(step.accepted_stopped_faces.empty());

// 阈值精确设置成 0.25 / sqrt(0.5)：相等时停止。
options.isotropic_height = Scalar{0.25} / std::sqrt(Scalar{0.5});
assert(step.next_front.faces.size() == 1);
assert(step.accepted_stopped_faces.size() == 1);
assert(step.accepted_stopped_faces[0].layer == 2);
assert(step.accepted_stopped_faces[0].reason ==
       FaceStopReason::IsotropicHeightReached);

// 单位正方形和 0.5 高度 Hexa：四侧边平均值为 0.5，比值为 0.5。
options.isotropic_height = Scalar{0.5};
assert(hexa_step.accepted_stopped_faces.size() == 1);
```

再构造面积缩放后的第二层前沿，证明使用当前底面面积而不是初始 Wall 面面积。

- [ ] **Step 6: 运行 Stepper 测试并确认 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_regular_layer_stepper_test
ctest --test-dir build -C Debug -R "^boundary_mesh_regular_layer_stepper_test$" --output-on-failure
```

预期：测试运行失败，因为尚未生成 `accepted_stopped_faces`。

- [ ] **Step 7: 实现动态面积和平均侧边判据**

在 `regular_layer_stepper.cpp` 增加局部函数：

```cpp
template <class Face>
Scalar averageSideLength(
    const Face &face,
    const GrowthFront &bottom,
    const GrowthFront &top)
{
    Scalar sum = Scalar{0};
    for (VertexId id : face.vertex_ids)
    {
        const std::size_t index = static_cast<std::size_t>(id);
        sum += (top.vertices[index].position -
                bottom.vertices[index].position).norm();
    }
    return sum / static_cast<Scalar>(face.vertex_ids.size());
}
```

只在 `quality.value().acceptable` 后计算：

```cpp
const Scalar area =
    front_evaluation.value().faces[eligible_face_index].value.area;
const Scalar ratio = std::visit(
    [&](const auto &face)
    {
        return averageSideLength(face, eligible.front, candidate_front) /
               std::sqrt(area);
    },
    eligible.front.faces[eligible_face_index]);

accepted_eligible_faces.push_back(eligible_face_index);
if (ratio >= options.isotropic_height)
{
    output.accepted_stopped_faces.push_back({
        previous_face_index,
        current_front.source_face_ids[previous_face_index],
        target_layer + 1,
        FaceStopReason::IsotropicHeightReached});
}
```

事件的 `layer` 是首个不接受层，因此第 `L` 层候选对应 `L + 1`。

- [ ] **Step 8: 运行 Stepper 测试并确认 GREEN**

运行 Step 6 的命令，预期通过。

- [ ] **Step 9: 提交 Task 1**

```powershell
git add include/boundary_mesh/growth/regular_layer_growth.hpp include/boundary_mesh/growth/regular_layer_growth_error.hpp src/growth/regular_layer_stepper.cpp tests/unit/growth/regular_layer_growth_types_test.cpp tests/unit/growth/regular_layer_stepper_test.cpp tests/integration/regular_layer_growth_failure_test.cpp
git commit -m "feat: classify isotropic layer candidates"
```

---

### Task 2: 保留当前单元、停止下一层并传播层差

**Files:**
- Modify: `src/growth/termination_propagator.cpp`
- Modify: `src/growth/layer_collision_checker.cpp`
- Modify: `src/growth/regular_layer_generator.cpp`
- Test: `tests/unit/growth/termination_propagator_test.cpp`
- Test: `tests/unit/growth/layer_collision_checker_test.cpp`
- Test: `tests/integration/regular_layer_growth_pipeline_test.cpp`
- Test: `tests/integration/layer_coordination_pipeline_test.cpp`

**Interfaces:**
- Consumes: `LayerStepResult::accepted_stopped_faces`，事件中的 `layer=L+1`。
- Produces: 本层被提交、下一层被剔除的确定性结果，以及邻域 `allowed_layer_count` 传播。

- [ ] **Step 1: 写传播和过滤失败测试**

在 `termination_propagator_test.cpp` 建立第 2 层已接受停止事件：

```cpp
const FaceStopEvent isotropic{
    0, SurfaceFaceId{2}, 3,
    FaceStopReason::IsotropicHeightReached};
const auto changed = propagator.applyDirectStops(
    constraints, {isotropic}, 1);
assert(changed.hasValue());
assert(constraints.find(2)->allowed_layer_count == 2);
assert(constraints.find(1)->allowed_layer_count == 3);
```

在 collision checker 测试中令带 `accepted_stopped_faces` 的候选发生碰撞，要求碰撞剔除该候选时同步删除对应“已接受停止”事件，使最终直接原因成为 `Collision`。

- [ ] **Step 2: 运行传播与碰撞测试并确认 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_termination_propagator_test boundary_mesh_layer_collision_checker_test
ctest --test-dir build -C Debug -R "boundary_mesh_(termination_propagator|layer_collision_checker)_test" --output-on-failure
```

预期：事件没有被正确保留或过滤，测试失败。

- [ ] **Step 3: 让所有候选压缩操作同步处理终止事件**

在 `TerminationPropagator::filterCandidates` 和 `layer_collision_checker.cpp::compactStep` 中复制 `accepted_stopped_faces`，但仅保留输出 `next_front.source_face_ids` 中仍存在的事件：

```cpp
const auto candidate_survives = [&](const FaceStopEvent &event)
{
    return std::find(
        output.next_front.source_face_ids.begin(),
        output.next_front.source_face_ids.end(),
        event.source_face_id) != output.next_front.source_face_ids.end();
};
std::copy_if(
    input.accepted_stopped_faces.begin(),
    input.accepted_stopped_faces.end(),
    std::back_inserter(output.accepted_stopped_faces),
    candidate_survives);
```

这样各向同性候选仍接受完整碰撞检查；若被碰撞剔除，不会错误登记为已提交的各向同性停止。

- [ ] **Step 4: 运行传播与碰撞测试并确认 GREEN**

运行 Step 2 的命令，预期通过。

- [ ] **Step 5: 写 Generator 端到端失败测试**

在 `regular_layer_growth_pipeline_test.cpp` 用单个 Wall 三角面生成多层，并设置首层比值恰好等于阈值，断言：

```cpp
assert(growth.mesh.cells.size() == 1);
assert(growth.faces[0].accepted_layer_count == 1);
assert(growth.faces[0].status == FaceGrowthStatus::Stopped);
assert(growth.faces[0].stop_reason ==
       FaceStopReason::IsotropicHeightReached);
assert(growth.faces[0].stop_layer == 2);
// 顶面存在于 farfield_boundary，且没有第二层进度输出。
```

在 `layer_coordination_pipeline_test.cpp` 构造相邻源面，验证直接各向同性停止使邻面层数满足 `max_neighbor_layer_difference`。

- [ ] **Step 6: 运行流水线测试并确认 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_regular_layer_growth_pipeline_test boundary_mesh_layer_coordination_pipeline_test
ctest --test-dir build -C Debug -R "boundary_mesh_(regular_layer_growth|layer_coordination)_pipeline_test" --output-on-failure
```

预期：当前实现继续生成下一层，测试失败。

- [ ] **Step 7: 在 Generator 提交后构建继续生长前沿**

在 Stepper 返回后，先传播“已接受但终止”的事件：

```cpp
const auto isotropic_propagation = propagator.applyDirectStops(
    constraints,
    step_result.value().accepted_stopped_faces,
    options.max_neighbor_layer_difference);
```

由于事件层号是 `L+1`，约束上限为 `L`，本层候选在后续 `filterCandidates` 中仍保留并继续经过障碍和同层碰撞检查。

最终 `step.next_front` 中全部幸存面照常创建单元、更新外露边界、增加 `accepted_layer_count`。提交完成后，用约束表压缩得到下一轮前沿：

```cpp
const bool continues =
    constraint->allowed_layer_count > step.layer;
```

同步压缩 `current_global_ids`，避免终止面独占顶点进入下一轮。共享顶点只要仍被继续生长面使用就保留。

对最终幸存的 `accepted_stopped_faces` 写入：

```cpp
record->status = FaceGrowthStatus::Stopped;
record->stop_reason = FaceStopReason::IsotropicHeightReached;
record->stop_layer = event.layer;
```

`current_front` 更新为压缩后的继续生长前沿；外露边界仍使用提交前的完整 `step`，从而保留终止单元顶面。

- [ ] **Step 8: 运行流水线测试并确认 GREEN**

运行 Step 6 的命令，预期通过。

- [ ] **Step 9: 提交 Task 2**

```powershell
git add src/growth/termination_propagator.cpp src/growth/layer_collision_checker.cpp src/growth/regular_layer_generator.cpp tests/unit/growth/termination_propagator_test.cpp tests/unit/growth/layer_collision_checker_test.cpp tests/integration/regular_layer_growth_pipeline_test.cpp tests/integration/layer_coordination_pipeline_test.cpp
git commit -m "feat: stop growth at isotropic height"
```

---

### Task 3: CLI 参数、摘要与停止统计

**Files:**
- Modify: `src/cli/boundary_mesh_command.hpp`
- Modify: `src/cli/boundary_mesh_command.cpp`
- Modify: `tests/integration/cgns_cli_pipeline_test.cpp`

**Interfaces:**
- Consumes: `RegularLayerGrowthOptions::isotropic_height`、`FaceStopReason::IsotropicHeightReached`。
- Produces: `--isotropic-height VALUE`、`isotropic_height=<value>`、`stop_isotropic_height=<count>`。

- [ ] **Step 1: 写 CLI 失败测试**

在 `cgns_cli_pipeline_test.cpp` 增加：

```cpp
expect_argument_error(base_args_with("--isotropic-height", "0"));
expect_argument_error(base_args_with("--isotropic-height", "-1"));
expect_argument_error(base_args_with("--isotropic-height", "nan"));

// 默认运行
assert(output.str().find("isotropic_height=1") != std::string::npos);
assert(output.str().find("stop_isotropic_height=") != std::string::npos);

// 显式覆盖
arguments.insert(arguments.end(), {"--isotropic-height", "0.75"});
assert(output.str().find("isotropic_height=0.75") != std::string::npos);
```

- [ ] **Step 2: 运行 CLI 测试并确认 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_cgns_cli_pipeline_test
ctest --test-dir build -C Debug -R "^boundary_mesh_cgns_cli_pipeline_test$" --output-on-failure
```

预期：新参数被报告为 unknown argument，测试失败。

- [ ] **Step 3: 实现 CLI 参数和 9 项统计**

在命令配置中增加：

```cpp
Scalar isotropic_height{1}; // 各向同性停止阈值
```

解析器增加：

```cpp
else if (name == "--isotropic-height")
{
    if (!parseScalar(value, options.isotropic_height) ||
        options.isotropic_height <= Scalar{0})
    {
        message = "invalid isotropic height";
        return ParseStatus::Failure;
    }
}
```

将值传入 `growth_options.isotropic_height`，usage 增加参数。停止统计数组从 8 扩为 9，并在末尾输出：

```cpp
<< "stop_isotropic_height=" << counts[8] << '\n';
```

配置摘要增加：

```cpp
<< "isotropic_height=" << command_options.isotropic_height << '\n';
```

- [ ] **Step 4: 运行 CLI 测试并确认 GREEN**

运行 Step 2 的命令，预期通过。

- [ ] **Step 5: 提交 Task 3**

```powershell
git add src/cli/boundary_mesh_command.hpp src/cli/boundary_mesh_command.cpp tests/integration/cgns_cli_pipeline_test.cpp
git commit -m "feat: expose isotropic height threshold"
```

---

### Task 4: 全量回归与最终审查

**Files:**
- Modify only if a regression exposes an implementation defect.

**Interfaces:**
- Consumes: Tasks 1–3 的完整实现。
- Produces: Debug/Release 全量验证证据和干净的功能分支。

- [ ] **Step 1: Debug 全量构建和测试**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

预期：全部测试通过，失败数为 0。

- [ ] **Step 2: Release 全量构建和测试**

```powershell
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

预期：全部测试通过，失败数为 0。

- [ ] **Step 3: 静态差异检查**

```powershell
git diff --check
git status --short --branch
git diff --stat HEAD~3..HEAD
```

预期：无 whitespace 错误；只保留未跟踪且不提交的 `.superpowers/`；变更范围与本计划一致。

- [ ] **Step 4: 逐项核对设计要求**

确认：`>=`、默认 `1.0`、3/4 侧边平均、当前动态面积、当前单元保留、顶面外露、下一层剔除、邻域传播、CLI 校验和 9 项统计均有自动测试证据。
