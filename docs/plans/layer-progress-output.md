# Boundary Layer Progress Output Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在每一层边界层生成开始和正式提交完成后，通过 `std::cout` 输出层号及本层新增体单元数。

**Architecture:** 输出由 `RegularLayerGenerator::generate()` 统一负责，因为它掌握逐层事务的开始、失败和提交边界。开始信息位于 `RegularLayerStepper::step()` 之前；结束信息位于本层顶点、体单元、元数据及外露边界全部提交之后，统计值使用 `new_cells.size()`。

**Tech Stack:** C++17、CMake、CTest、MSVC、标准库 `std::cout` / `std::endl`

## Global Constraints

- Debug 与 Release 均输出。
- 不引入回调或日志抽象，直接使用 `std::cout`。
- 每层开始输出 `generate N boundarylayer`。
- 每层提交完成输出 `finish N boundarylayer. add X cell`，其中 `X` 是本层新增体单元数。
- 即使本层新增 0 个单元，也输出结束信息。
- 本层提交前若返回错误，不输出虚假的结束信息。
- 使用 `std::endl` 立即刷新。
- 不提交 `.superpowers/`。

---

### Task 1: 用集成测试锁定逐层输出

**Files:**
- Modify: `tests/integration/regular_layer_growth_pipeline_test.cpp`

**Interfaces:**
- Consumes: `generateRegularLayers(...)`
- Produces: 对两层、每层两个混合面单元的精确标准输出断言

- [ ] **Step 1: 写失败测试**

在测试中引入 `<iostream>`、`<sstream>`、`<string>`，把第一次 `generateRegularLayers(...)` 调用包在 `std::cout.rdbuf()` 重定向中，并在调用后恢复原缓冲区。断言捕获文本等于：

```text
generate 1 boundarylayer
finish 1 boundarylayer. add 2 cell
generate 2 boundarylayer
finish 2 boundarylayer. add 2 cell
```

使用 RAII 辅助对象恢复 `std::cout`，避免测试提前返回时污染进程输出。

- [ ] **Step 2: 运行目标测试，确认 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_regular_layer_growth_pipeline_test
ctest --test-dir build -C Debug -R "^boundary_mesh_regular_layer_growth_pipeline_test$" --output-on-failure
```

预期：构建成功，CTest 因捕获结果为空而失败。

### Task 2: 在逐层事务边界输出进度

**Files:**
- Modify: `src/growth/regular_layer_generator.cpp`
- Test: `tests/integration/regular_layer_growth_pipeline_test.cpp`

**Interfaces:**
- Consumes: `current_front.layer`、`step.layer`、`new_cells.size()`
- Produces: 两条逐层标准输出信息

- [ ] **Step 1: 写最小实现**

引入 `<iostream>`。在 `while (!current_front.faces.empty())` 循环中、调用 stepper 前输出：

```cpp
const std::uint32_t target_layer = current_front.layer + 1;
std::cout << "generate " << target_layer
          << " boundarylayer" << std::endl;
```

在 `result.mesh.cells`、`result.mesh.metadata` 和 `exposed_boundary` 全部提交后，且更新下一层前沿前输出：

```cpp
std::cout << "finish " << step.layer
          << " boundarylayer. add " << new_cells.size()
          << " cell" << std::endl;
```

- [ ] **Step 2: 运行目标测试，确认 GREEN**

```powershell
cmake --build build --config Debug --target boundary_mesh_regular_layer_growth_pipeline_test
ctest --test-dir build -C Debug -R "^boundary_mesh_regular_layer_growth_pipeline_test$" --output-on-failure
```

预期：目标测试通过。

- [ ] **Step 3: 运行 Debug 与 Release 全量回归**

```powershell
cmake --build build --config Debug -- /m:1
ctest --test-dir build -C Debug --output-on-failure
cmake --build build --config Release -- /m:1
ctest --test-dir build -C Release --output-on-failure
```

预期：两个配置均为 47/47 通过。

- [ ] **Step 4: 提交实现**

```powershell
git add docs/plans/layer-progress-output.md tests/integration/regular_layer_growth_pipeline_test.cpp src/growth/regular_layer_generator.cpp
git diff --cached --check
git commit -m "feat: report boundary layer progress"
```

### Task 3: 运行 2dot5 十层 Release 案例

**Files:**
- Input: `C:/Users/zpern/Desktop/todo/九院项目质量对标/test_case/2dot5_cf/2dot5_cf.cgns`
- Input: `C:/Users/zpern/Desktop/todo/九院项目质量对标/test_case/2dot5_cf/2dot5_cf.bc.txt`
- Output: `build/real_case/2dot5_cf_10_layers_boundary_layer.vtk`
- Output: `build/real_case/2dot5_cf_10_layers_farfield_boundary.vtk`

**Interfaces:**
- Consumes: Release CLI 与 CGNS/边界条件文件
- Produces: 十层逐层新增量、最终体单元数、远场面数和停止原因统计

- [ ] **Step 1: 运行十层案例**

```powershell
.\build\Release\boundary_mesh_cli.exe `
  --input "C:\Users\zpern\Desktop\todo\九院项目质量对标\test_case\2dot5_cf\2dot5_cf.cgns" `
  --first-height 0.1 `
  --growth-ratio 1.0 `
  --layer-count 10 `
  --maximum-skewness 0.95 `
  --max-neighbor-layer-difference 1 `
  --output-prefix ".\build\real_case\2dot5_cf_10_layers"
```

预期：退出码为 0，输出十组 generate/finish 信息并生成两个 VTK 文件。

- [ ] **Step 2: 检查生成物**

记录每层 `add X cell`、最终 `volume_cells`、`farfield_faces` 和各停止原因；检查两个 VTK 的 `POINTS`、`CELLS`、`CELL_TYPES`，并确认文本中不存在 `nan` 或 `inf` 数值。

