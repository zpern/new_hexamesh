# 步长平滑状态分离实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 分离理论基准步长和继承修正的临时预测步长，消除步长修正的跨层累乘。

**Architecture:** `RegularLayerStepper` 为每个活动点同时构造理论步长和临时步长；`GrowthFieldSmoother` 仅用临时步长预测邻点位置，并用理论步长计算相对误差和最终步长。前沿仍只保存已接受层的 `actual_height`，不增加长期修正比例状态。

**Tech Stack:** C++17、Eigen、CMake、CTest、CGNS、ASCII Legacy VTK。

## Global Constraints

- 最终公式保持 `actual_height = reference_height * (1 + correction)`。
- Logistic 响应系数保持 `0.5`。
- 实际步长限制在理论基准步长的 `50%～150%`。
- 不实现 `FixHightRatio`，不改变法向、质量、碰撞和停止算法。
- 不增加外部参数，不使用子代理。

---

### Task 1: 分离平滑器的两种输入步长

**Files:**
- Modify: `tests/unit/growth/growth_field_smoother_test.cpp`
- Modify: `include/boundary_mesh/growth/growth_field_smoother.hpp`
- Modify: `include/boundary_mesh/growth/growth_field_smoothing_error.hpp`
- Modify: `src/growth/growth_field_smoother.cpp`

**Interfaces:**
- Consumes: `reference_heights` 和 `provisional_heights`，长度均等于活动点数。
- Produces: `SmoothedGrowthFields::actual_heights`，只以 `reference_heights` 为最终缩放基准。

- [ ] **Step 1: 写出失败测试**

将测试中的预期函数改为分别接收两组步长：

```cpp
Scalar expectedHeight(
    std::size_t vertex_index,
    const GrowthFront &front,
    const FrontAdjacency &adjacency,
    const std::vector<Vector3> &directions,
    const std::vector<Scalar> &reference_heights,
    const std::vector<Scalar> &provisional_heights)
```

预测位置使用：

```cpp
front.vertices[neighbor].position +
    provisional_heights[neighbor] * directions[neighbor]
```

相对误差和最终步长使用：

```cpp
const Scalar relative =
    (predicted - reference_heights[vertex_index]) /
    reference_heights[vertex_index];

return std::clamp(
    reference_heights[vertex_index] *
        (Scalar{1} + correction),
    Scalar{0.5} * reference_heights[vertex_index],
    Scalar{1.5} * reference_heights[vertex_index]);
```

测试数据令 `provisional_heights` 与 `reference_heights` 不同，并把两组数组传给 `smooth`。另增加临时步长数量不匹配和非正临时步长的错误断言。

- [ ] **Step 2: 验证 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_growth_field_smoother_test
```

预期：编译因 `GrowthFieldSmoother::smooth` 尚未接收第六个参数而失败，证明测试要求的新接口尚不存在。

- [ ] **Step 3: 实现最小接口和算法**

新接口：

```cpp
Result<SmoothedGrowthFields, GrowthFieldSmoothingError>
smooth(
    const GrowthFront &front,
    const FrontEvaluation &evaluation,
    const FrontAdjacency &adjacency,
    const GrowthDirections &raw_directions,
    const std::vector<Scalar> &reference_heights,
    const std::vector<Scalar> &provisional_heights) const;
```

更新 `GrowthFieldInputMismatch`，分别记录：

```cpp
std::size_t reference_height_count{};
std::size_t provisional_height_count{};
```

校验两组输入都有限且大于零。构造预计位置时使用 `provisional_heights`，计算 `relative`、`actual` 和上下限时使用 `reference_heights`。

- [ ] **Step 4: 验证平滑器 GREEN**

```powershell
cmake --build build --config Debug --target boundary_mesh_growth_field_smoother_test
ctest --test-dir build -C Debug -R "^boundary_mesh_growth_field_smoother_test$" --output-on-failure
```

预期：目标构建成功，测试 1/1 通过。

---

### Task 2: 在规则层步进器中提供两种步长

**Files:**
- Modify: `tests/unit/growth/regular_layer_stepper_test.cpp`
- Modify: `src/growth/regular_layer_stepper.cpp`

**Interfaces:**
- Consumes: `GrowthProfileTable::height(source_vertex_id, target_layer)` 和上一层 `GrowthFrontVertex::actual_height`。
- Produces: 传给平滑器的 `reference_heights` 与 `provisional_heights`。

- [ ] **Step 1: 写出跨层不累乘测试**

修改现有 `ratio_step` 用例。第二层理论步长为 `0.25 * 2.0 = 0.5`，临时步长为 `0.08 * 2.0 = 0.16`。对于平面单位法向：

```cpp
const Scalar reference = Scalar{0.5};
const Scalar provisional = Scalar{0.16};
const Scalar relative =
    (provisional - reference) / reference;
const Scalar correction =
    Scalar{1} /
        (Scalar{1} + std::exp(Scalar{-0.5} * relative)) -
    Scalar{0.5};
const Scalar expected_height =
    reference * (Scalar{1} + correction);
```

断言新位置为旧位置加 `expected_height * UnitZ`，而不是旧实现的 `0.16 * UnitZ`。

- [ ] **Step 2: 验证 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_regular_layer_stepper_test
ctest --test-dir build -C Debug -R "^boundary_mesh_regular_layer_stepper_test$" --output-on-failure
```

预期：测试失败，因为步进器尚未构造和传递理论步长。

- [ ] **Step 3: 实现步长数据流**

每个活动点先调用：

```cpp
const auto reference_result =
    profiles.height(vertex.source_vertex_id, target_layer);
```

然后构造：

```cpp
const Scalar reference_height = reference_result.value();
const Scalar provisional_height = target_layer == 1
    ? reference_height
    : vertex.actual_height * profile->growth_ratio;
```

分别写入两个数组并传给 `GrowthFieldSmoother::smooth`。

- [ ] **Step 4: 验证步进器和完整回归**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

预期：完整构建成功，全部测试通过。

---

### Task 3: Release 真实算例和质量对比

**Files:**
- Generated: `2dot5_cf_height_state_separated_boundary_layer.vtk`
- Generated: `2dot5_cf_height_state_separated_farfield_boundary.vtk`

**Interfaces:**
- Consumes: `2dot5_cf.cgns` 与同目录 `2dot5_cf.bc.txt`。
- Produces: 20 层边界层体网格、远场边界和控制台统计。

- [ ] **Step 1: 构建并运行**

```powershell
cmake --build build --config Release --target boundary_mesh_cli
.\build\Release\boundary_mesh_cli.exe --input "C:\Users\zpern\Desktop\todo\jiuyuan-quailty\test_case\2dot5_cf\2dot5_cf.cgns" --first-height 0.1 --growth-ratio 1.2 --layer-count 20 --maximum-skewness 0.9 --isotropic-height 1.0 --output-prefix "C:\Users\zpern\Desktop\todo\jiuyuan-quailty\test_case\2dot5_cf\2dot5_cf_height_state_separated"
```

预期：返回码 0，输出 20 层逐层统计和两个 VTK 文件。

- [ ] **Step 2: 分析质量**

使用 `C:\Users\zpern\Desktop\program\tools\quailty\vtk_mesh_quality.py` 的 `read_legacy_ascii_vtk` 与 `analyze_mesh`，报告总单元数、skewness 直方图、最大 skewness、最小有向体积和负体积计数。

- [ ] **Step 3: 最终检查和提交**

```powershell
git diff --check
git status --short
git add include/boundary_mesh/growth/growth_field_smoother.hpp include/boundary_mesh/growth/growth_field_smoothing_error.hpp src/growth/growth_field_smoother.cpp src/growth/regular_layer_stepper.cpp tests/unit/growth/growth_field_smoother_test.cpp tests/unit/growth/regular_layer_stepper_test.cpp
git commit -m "fix: separate reference and provisional heights"
```
