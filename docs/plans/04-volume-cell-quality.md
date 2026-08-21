# Prism/Hexa 候选单元质量评价实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为阶段 05 提供无状态、可重入的 Prism/Hexa 候选单元 Jacobian、体积和 equiangular skewness 评价接口。

**Architecture:** `BoundaryMesh::Surface` 提供三角形/四边形面 skewness；新的 `BoundaryMesh::Quality` 只依赖 Core 与 Surface，并按固定形函数采样候选单元。几何无效和质量超限作为成功评价返回，只有输入、配置或数值错误返回 `Result::failure`。

**Tech Stack:** C++17、Eigen3、CMake 3.20、CTest、MSVC 17.14。

## Global Constraints

- 仅支持 `Prism` 和 `Hexa`；`Tetra`、`Pyramid` 延后到阶段 09。
- 每个生产接口必须先有会因功能缺失而失败的测试，再实现最小代码。
- 所有公共 struct 字段、枚举值和顶点顺序常量都添加中文行尾注释。
- 单次评价不分配动态内存、不写日志、不创建线程，也不依赖 GrowthFront、GrowthPatch 或 VolumeMesh。
- Prism 固定使用 13 个 Jacobian 检查点；Hexa 固定使用 17 个检查点。
- `acceptable = validity == Valid && skewness <= maximum_skewness`，不缩短层高重试。

---

## 文件结构

- `include/boundary_mesh/surface/face_skewness.hpp`：面 equiangular skewness 公共接口。
- `src/surface/face_skewness.cpp`：无动态分配的三角形/四边形角度极值算法。
- `include/boundary_mesh/quality/volume_cell_evaluation.hpp`：配置、顶点数组、有效性和评价结果。
- `include/boundary_mesh/quality/volume_cell_evaluation_error.hpp`：配置、输入和数值错误载荷。
- `include/boundary_mesh/quality/volume_cell_evaluator.hpp`：Prism/Hexa 两个公开入口。
- `src/quality/volume_cell_evaluator_internal.hpp`：两个 evaluator 共用的内部固定大小计算类型。
- `src/quality/volume_cell_evaluator_common.cpp`：配置验证、归一化 Jacobian、分类和结果组装。
- `src/quality/prism_evaluator.cpp`：Prism 形函数、13 点采样和 6 点积分。
- `src/quality/hexa_evaluator.cpp`：Hexa 形函数、17 点采样和 8 点积分。
- `tests/unit/surface/face_skewness_test.cpp`：面 skewness 单元测试。
- `tests/unit/quality/volume_cell_types_test.cpp`：公共类型和顺序约定测试。
- `tests/unit/quality/prism_evaluator_test.cpp`：Prism 有效性、体积和质量测试。
- `tests/unit/quality/hexa_evaluator_test.cpp`：Hexa 有效性、体积和质量测试。
- `tests/unit/quality/volume_cell_error_test.cpp`：错误、尺度不变性和诊断确定性测试。
- `tests/integration/volume_cell_quality_pipeline_test.cpp`：模拟阶段 05 接受/停止决策。
- `benchmarks/volume_cell_quality_benchmark.cpp`：Release 固定候选批量吞吐量程序。

### Task 1：面 equiangular skewness

**Files:** Create `face_skewness.hpp/.cpp` and `face_skewness_test.cpp`; modify root/test CMake.

**Produces:** `triangleEquiangularSkewness(const std::array<Point3,3>&, Scalar)` and `quadEquiangularSkewness(const std::array<Point3,4>&, Scalar)`, returning `Result<Scalar, FaceEvaluationError>`.

- [ ] Write tests for ideal triangle/square, a known distorted angle, `[0,1]`, and a zero-length edge.
- [ ] Register the target and run it; expected RED is missing `boundary_mesh/surface/face_skewness.hpp`.
- [ ] Implement fixed-array cosine extrema, at most two `acos` calls per face kind, and clamp the formula.
- [ ] Run the new target plus existing Surface tests.
- [ ] Commit as `feat: add surface equiangular skewness`.

### Task 2：Quality 公共类型与顺序契约

**Files:** Create three public quality headers and `volume_cell_types_test.cpp`; modify `mesh_volume.hpp` and CMake.

**Produces:** `PrismPoints`, `HexaPoints`, `VolumeCellQualityOptions`, `VolumeCellValidity`, `JacobianSampleKind`, `JacobianSampleLocation`, `VolumeCellEvaluation`, `VolumeCellEvaluationError`, `evaluatePrism`, `evaluateHexa`.

- [ ] Write static/runtime tests for fixed sizes, defaults, field types, and documented vertex order.
- [ ] Run target; expected RED is missing quality headers.
- [ ] Add commented public types and `BoundaryMesh::Quality` linked to Core and Surface.
- [ ] Run type target and CTest filter `volume_cell_types`.
- [ ] Commit as `feat: define volume cell quality API`.

### Task 3：Prism evaluator

**Files:** Create `prism_evaluator_test.cpp`, internal helper header, common implementation, and `prism_evaluator.cpp`; modify CMake.

- [ ] Write standard Prism test: volume `0.5`, positive Jacobians, skewness `0`, `Valid`, acceptable.
- [ ] Run target; expected RED is unresolved `evaluatePrism`.
- [ ] Implement linear-triangle × linear-thickness shape functions, fixed 13-point sampling, normalized Jacobian and six-point volume integration.
- [ ] Add one RED/GREEN case at a time for thin valid, reversed, locally inverted, degenerate and skewness-rejected candidates.
- [ ] Run Prism tests and commit as `feat: evaluate prism candidate quality`.

### Task 4：Hexa evaluator

**Files:** Create `hexa_evaluator_test.cpp` and `hexa_evaluator.cpp`; modify CMake.

- [ ] Write unit cube test: volume `1`, positive Jacobians, skewness `0`, `Valid`, acceptable.
- [ ] Run target; expected RED is unresolved `evaluateHexa`.
- [ ] Implement trilinear eight-node shape functions, fixed 17-point sampling and eight-point Gauss volume integration.
- [ ] Add one RED/GREEN case at a time for thin valid, reversed, locally inverted, degenerate and skewness-rejected candidates.
- [ ] Run Hexa tests and commit as `feat: evaluate hexa candidate quality`.

### Task 5：错误模型、尺度稳定性和确定性诊断

**Files:** Create `volume_cell_error_test.cpp`; modify common/Prism/Hexa implementations and CMake.

- [ ] Write RED tests for nonpositive/nonfinite tolerances, skewness outside `[0,1]`, NaN/Infinity vertices, and numeric overflow.
- [ ] Return exact error category plus cell kind and vertex/sample payload before any invalid computation escapes.
- [ ] Write RED tests scaling candidates by `1e-9`/`1e9` and for deterministic earliest-sample tie breaking.
- [ ] Implement scale-stable length checks and strict `<` replacement for worst-location selection; run all Quality tests.
- [ ] Commit as `test: harden volume quality diagnostics`.

### Task 6：阶段 05 调用模拟、性能程序与收尾

**Files:** Create integration test and benchmark; modify CMake and `docs/design/roadmap.md`.

- [ ] Write an integration test simulating stage-05 commit/stop decisions from `evaluation.acceptable`.
- [ ] Verify standard candidates commit and invalid/skewness-rejected candidates stop without touching `VolumeMesh`.
- [ ] Add a Release benchmark evaluating at least one million fixed Prism/Hexa candidates and printing cells/s; do not impose a machine-specific CTest threshold.
- [ ] Run full Debug and Release builds/tests, benchmark, and `git diff --check`.
- [ ] Mark stage 04 complete in the roadmap and commit as `feat: complete volume cell quality stage`.

## 每个任务的固定命令

```powershell
cmake --build build --config Debug --target <target>
ctest --test-dir build -C Debug -R <filter> --output-on-failure
git diff --check
git add <task-files>
git diff --cached --check
git commit -m "<task-message>"
```

## 完成判据

- Debug 与 Release 全量 CTest 均 100% 通过。
- 标准、薄层、反转、局部翻转、退化、质量超限、非法输入、尺度变换均有自动化覆盖。
- Quality evaluator 的公开路径不分配动态内存，不输出日志，不创建线程。
- `git diff --check` 无输出，工作树干净。
