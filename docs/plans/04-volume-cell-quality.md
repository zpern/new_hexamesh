# 固定子四面体有向体积质量评价实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将阶段 04 的 Prism/Hexa evaluator 从 Jacobian 采样完整替换为固定子四面体有向体积分类，同时保留面 equiangular skewness。

**Architecture:** `BoundaryMesh::Quality` 对 Prism 固定检查 3 个子四面体、对 Hexa 固定检查 6 个子四面体，以严格正、负、零分类几何有效性，并汇总总体积与最差子体积诊断。模块继续依赖 `BoundaryMesh::Surface` 计算面 skewness，但不再包含 Jacobian、形函数、参考采样或高斯积分代码。

**Tech Stack:** C++17、Eigen3、CMake 3.20、CTest、MSVC 17.14。

## Global Constraints

- 仅支持 Prism 和 Hexa；Tetra/Pyramid 延后到阶段 09。
- Prism 固定分解为 `(0,1,2,3)`、`(1,2,3,4)`、`(2,3,4,5)`。
- Hexa 固定沿 `0–6` 分解为 `(0,1,2,6)`、`(0,2,3,6)`、`(0,3,7,6)`、`(0,7,4,6)`、`(0,4,5,6)`、`(0,5,1,6)`。
- 子体积只与严格的 `0` 比较，不使用体积容差、长度容差或相对容差。
- 最终代码不得保留 Jacobian、归一化 Jacobian、形函数采样、参考点或高斯积分接口、字段、枚举、帮助函数和测试。
- 保留面 equiangular skewness，唯一配置项为 `maximum_skewness{0.95}`。
- 所有新公共字段、枚举值和分解常量添加中文行尾注释。
- evaluator 无状态、可重入、线程安全、无动态分配、不输出日志、不创建线程。
- 每个行为修改遵循 RED→GREEN；每个任务完成后运行覆盖测试并独立提交。

---

### Task 1：用有向体积完整替换 Prism/Hexa Jacobian 核心

**Files:**
- Modify: `include/boundary_mesh/quality/volume_cell_evaluation.hpp`
- Modify: `include/boundary_mesh/quality/volume_cell_evaluation_error.hpp`
- Modify: `src/quality/volume_cell_evaluator_internal.hpp`
- Modify: `src/quality/volume_cell_evaluator_common.cpp`
- Modify: `src/quality/prism_evaluator.cpp`
- Modify: `src/quality/hexa_evaluator.cpp`
- Modify: `tests/unit/quality/volume_cell_types_test.cpp`
- Modify: `tests/unit/quality/prism_evaluator_test.cpp`
- Modify: `tests/unit/quality/hexa_evaluator_test.cpp`

**Produces:**

```cpp
struct VolumeCellQualityOptions
{
    Scalar maximum_skewness{0.95};
};

struct VolumeCellEvaluation
{
    VolumeCellValidity validity{VolumeCellValidity::Degenerate};
    Scalar signed_volume{};
    Scalar minimum_subtet_signed_volume{};
    Scalar maximum_subtet_signed_volume{};
    std::size_t worst_subtet_index{};
    Scalar skewness{};
    bool acceptable{};
};
```

- [ ] **Step 1: 写新的公共契约和标准单元 RED 测试**

修改类型测试，要求旧 Jacobian 字段和 options 不再存在，新子体积字段类型正确；修改 Prism/Hexa 测试，分别断言标准总体积 `0.5`/`1`、最小和最大子体积、最早最差下标、`Valid` 与 skewness `0`。

- [ ] **Step 2: 验证 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_volume_cell_types_test boundary_mesh_prism_evaluator_test boundary_mesh_hexa_evaluator_test
```

预期：编译失败，提示新子体积字段不存在或旧配置仍存在。

- [ ] **Step 3: 实现固定分解和公共累加器**

内部只保留固定大小数组与如下计算：

```cpp
const Scalar signed_volume_6 =
    (b - a).dot((c - a).cross(d - a));
const Scalar signed_volume =
    signed_volume_6 / Scalar{6};
```

累加 `has_positive/has_negative/has_zero`、总体积、最小/最大子体积，并用严格 `<` 保留最早最差下标。任何中间结果非有限都返回带 cell kind 和可选子四面体下标的 failure。

- [ ] **Step 4: 重写 Prism 与 Hexa evaluator**

两个入口验证 `maximum_skewness` 和坐标，按设计固定数组计算子体积，再计算所有组成面的 skewness；长度容差传 `0`。正常退化、反转、局部翻转和 skewness 超限均返回 success evaluation。

- [ ] **Step 5: 验证 GREEN 和全量回归**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
git diff --check
```

- [ ] **Step 6: 提交**

```powershell
git add include/boundary_mesh/quality src/quality tests/unit/quality
git diff --cached --check
git commit -m "refactor: evaluate cells with signed subtet volumes"
```

### Task 2：严格零分类、错误语义与无 Jacobian 残留

**Files:**
- Modify: `tests/unit/quality/prism_evaluator_test.cpp`
- Modify: `tests/unit/quality/hexa_evaluator_test.cpp`
- Modify: `tests/unit/quality/volume_cell_error_test.cpp`
- Modify: `src/quality/volume_cell_evaluator_common.cpp`
- Modify: `src/quality/prism_evaluator.cpp`
- Modify: `src/quality/hexa_evaluator.cpp`
- Modify or delete: `src/quality/volume_cell_evaluator_internal.hpp`

- [ ] **Step 1: 写分类 RED 测试**

分别为 Prism/Hexa 构造：全部正、全部负、正负混合、仅正和零、正负零同时存在。断言优先级为 mixed→`LocallyInverted`、zero→`Degenerate`、all negative→`Reversed`、all positive→`Valid`。

- [ ] **Step 2: 写确定性和极薄 RED 测试**

断言相同最小子体积保留最早 `worst_subtet_index`；高度使用可表示的极小正数时仍为 `Valid`；整体平移不改变诊断；整体缩放时体积按三次方缩放、分类与 skewness 不变。

- [ ] **Step 3: 写错误 RED 测试**

覆盖非法 `maximum_skewness`、NaN/Infinity 顶点、坐标差溢出、叉积/点积/总体积累加溢出。错误精确携带 cell kind、可选顶点或子四面体下标；正常无效候选不得返回 failure。

- [ ] **Step 4: 实现最小修复并删除全部 Jacobian 残留**

```powershell
Get-ChildItem include,src,tests -Recurse -File |
    Select-String -Pattern "Jacobian|jacobian|IntegrationPoint|normalized_jacobian"
```

预期最终无输出。若内部头文件只服务旧 Jacobian accumulator，则删除它并把少量固定体积帮助函数放入 common 或各 evaluator 的匿名 namespace。

- [ ] **Step 5: 回归和提交**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
git diff --check
git add include src tests
git diff --cached --check
git commit -m "test: cover signed volume cell validity"
```

### Task 3：阶段 05 调用模拟与 Release 性能基准

**Files:**
- Create or modify: `tests/integration/volume_cell_quality_pipeline_test.cpp`
- Create: `benchmarks/volume_cell_quality_benchmark.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

- [ ] **Step 1: 写预推出流水线 RED 测试**

模拟阶段 05：标准 Prism/Hexa 的 `acceptable=true` 允许提交；反转、局部翻转、严格退化和 skewness 超限得到 `acceptable=false` 并停止面片；本测试不写入正式 `VolumeMesh`。

- [ ] **Step 2: 注册测试并验证 GREEN**

```powershell
cmake --build build --config Debug --target boundary_mesh_volume_cell_quality_pipeline_test
ctest --test-dir build -C Debug -R volume_cell_quality_pipeline --output-on-failure
```

- [ ] **Step 3: 添加 Release benchmark**

对固定 Prism/Hexa 各重复评价至少一百万次，累加返回量以避免优化删除，输出总 candidates/s；benchmark 不设置机器相关的 CTest 时间阈值。

- [ ] **Step 4: Debug/Release 全量验证并提交**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
./build/Release/boundary_mesh_volume_cell_quality_benchmark.exe
git diff --check
git add CMakeLists.txt tests/CMakeLists.txt tests/integration benchmarks
git diff --cached --check
git commit -m "test: integrate signed volume quality evaluation"
```

### Task 4：文档与阶段完成验证

**Files:**
- Modify: `docs/design/roadmap.md`
- Modify: `docs/plans/04-volume-cell-quality.md`

- [ ] **Step 1: 核对实现与设计一致**

确认公共 API、固定分解、严格零分类、错误模型、测试和 benchmark 均与设计文档一致，不保留旧 Jacobian 名称或行为。

- [ ] **Step 2: 标记阶段 04 完成并运行最终验证**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
git diff --check
git status --short --branch
```

- [ ] **Step 3: 提交**

```powershell
git add docs/design/roadmap.md docs/plans/04-volume-cell-quality.md
git diff --cached --check
git commit -m "docs: complete signed volume quality stage"
```

## 完成判据

- 最终代码和测试中不存在 Jacobian、归一化 Jacobian、形函数采样或高斯积分实现。
- Prism/Hexa 仅按批准的固定子四面体分解和严格零比较分类。
- skewness 保留，options 只有 `maximum_skewness`。
- Debug/Release 全量测试均 100% 通过。
- Release benchmark 成功报告百万级候选评价吞吐量。
- `git diff --check` 无输出。
