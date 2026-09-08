# 逐层过渡区滑移面联合检测 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在逐层生长的常规候选和层差过渡联合检测中阻止 `Symmetry`/`Internal` 非法相交，同时让多法向生成阶段保持完全不检测。

**Architecture:** 复用已存在的静态 `SlidingIntersectionIndex`。常规路径保持现有检查；增量生成器把同一索引和滑移拓扑传入 `TransitionBoundaryChecker`，由过渡边界三角形携带显式 region/真实边/完整侧面权限并沿既有 `rollback_high_faces` 回退。`generateMultiNormalTransition()` 的接口和实现均不接触该索引。

**Tech Stack:** C++17、Eigen、现有 `SlidingIntersectionIndex`、`LayerTransitionResolver`、CTest、legacy VTK。

## Global Constraints

- 多法向预处理及 `generateMultiNormalTransition()` 不执行滑移面相交检测。
- 所有属于逐层生长流程的常规和层差过渡候选都执行滑移面检测。
- `Symmetry` 与 `Internal` 使用相同算法并按 `region_id` 隔离权限。
- 人工三角剖分对角线不得获得真实边权限。
- 顶盖不得获得完整面豁免；完整侧四边形只有四个节点同属 region 时才豁免。
- 非有限、退化、权限映射缺失和局部方位不可判定时保守失败或回退。
- 真实回归继续使用原始 `BoundaryMeshing.txt`、20 层、首层高度 `0.10000000100000001`、增长率 `1.19999999999999996`、最大偏斜度 1、多法向关闭。

---

## 文件结构

- 修改 `include/boundary_mesh/transition/transition_boundary_checker.hpp`：为过渡三角形增加滑移接触描述，为联合输入增加静态索引指针。
- 修改 `src/transition/transition_boundary_checker.cpp`：复用 `buildSlidingContactPermissions()` 和 `SlidingIntersectionIndex::query()` 执行滑移检查。
- 修改 `src/transition/provisional_transition_builder.cpp`：在模板边界生成时保存逐点 region、真实边掩码和完整侧四边形 region。
- 修改 `include/boundary_mesh/transition/transition_template_types.hpp`：如边界三角形在此定义，则在该公共类型中承载权限数据，避免几何反推。
- 修改 `src/boundary_layer/incremental_boundary_layer_generator.cpp`：把逐层生成的滑移索引/集合传给过渡协调回调。
- 修改 `include/boundary_mesh/growth/regular_layer_growth.hpp`：扩展候选回调上下文，使增量层协调复用生成器已构建的滑移索引，避免重复构树。
- 修改 `tests/unit/transition/transition_boundary_checker_test.cpp`：覆盖合法接触、穿越、Internal 等价和跨 region。
- 修改 `tests/integration/incremental_layer_transition_pipeline_test.cpp`：覆盖过渡回退和最终过渡单元保留。
- 修改 `tests/integration/sliding_surface_intersection_regression_test.cpp`：走完整增量入口并审计过渡单元。
- 修改 `README.md`：说明多法向阶段跳过、逐层常规与过渡阶段检测。

---

### Task 1: 过渡边界三角形的滑移权限数据

**Files:**
- Modify: `include/boundary_mesh/transition/transition_boundary_checker.hpp`
- Modify: `include/boundary_mesh/transition/transition_template_types.hpp`
- Modify: `src/transition/provisional_transition_builder.cpp`
- Test: `tests/unit/transition/transition_boundary_checker_test.cpp`

**Interfaces:**
- Consumes: `SlidingContactPermission`、`PatchVertex::sliding_region_ids` 和过渡模板显式边界拓扑。
- Produces: `OwnedBoundaryTriangle::vertex_sliding_region_ids`、`physical_edge_mask`、`complete_face_exemption_regions`。

- [ ] **Step 1: 写失败测试，要求过渡候选保留权限元数据**

在 `transition_boundary_checker_test.cpp` 新增 helper，并对一个侧四边形的两个剖分三角形断言：

```cpp
OwnedBoundaryTriangle value = triangle(...);
value.vertex_sliding_region_ids = {{{7}, {7}, {7}}};
value.physical_edge_mask = 0b101;
value.complete_face_exemption_regions = {7};
if (value.physical_edge_mask != 0b101 ||
    value.complete_face_exemption_regions != std::vector<std::uint32_t>{7})
    return 20;
```

扩展 provisional transition 测试，断言侧四边形人工对角线对应 bit 未设置，真实外边 bit 已设置。

- [ ] **Step 2: 构建测试并验证红灯**

Run:

```powershell
cmake --build build --config Release --target boundary_mesh_transition_boundary_checker_test -- /m:1
```

Expected: 编译失败，`OwnedBoundaryTriangle` 尚无滑移权限字段。

- [ ] **Step 3: 增加最小公共数据结构**

在 `OwnedBoundaryTriangle` 中加入：

```cpp
std::array<std::vector<std::uint32_t>, 3>
    vertex_sliding_region_ids;
std::uint8_t physical_edge_mask{};
std::vector<std::uint32_t> complete_face_exemption_regions;
```

所有 region vector 在构造边界三角形时排序去重。物理边 bit 顺序固定为 `(0,1)=0b001`、`(1,2)=0b010`、`(2,0)=0b100`。

- [ ] **Step 4: 在 provisional builder 中从模板拓扑填充数据**

为边界三角形构造 helper：

```cpp
OwnedBoundaryTriangle makeOwnedTriangle(
    const std::array<TransitionVertex,3> &vertices,
    std::uint8_t physical_edges,
    const std::vector<std::uint32_t> &complete_regions,
    LayerBoundaryOwner owner);
```

其中 `TransitionVertex` 使用当前前沿顶点已有的 `sliding_region_ids`。侧四边形拆分时只标记三条真实边中的对应 bit，不标记人工对角线；只有完整四节点公共 region 写入两个三角形的 `complete_face_exemption_regions`。顶盖始终传空完整豁免。

- [ ] **Step 5: 运行相关测试**

```powershell
cmake --build build --config Release --target boundary_mesh_transition_boundary_checker_test boundary_mesh_incremental_transition_templates_test -- /m:1
ctest --test-dir build -C Release -R "transition_boundary_checker|incremental_transition_templates" --output-on-failure
```

Expected: 两个测试通过。

- [ ] **Step 6: 提交**

```powershell
git add -- include/boundary_mesh/transition/transition_boundary_checker.hpp include/boundary_mesh/transition/transition_template_types.hpp src/transition/provisional_transition_builder.cpp tests/unit/transition/transition_boundary_checker_test.cpp
git commit -m "feat: preserve sliding permissions on transition boundaries"
```

---

### Task 2: 过渡区滑移面联合检测与回退

**Files:**
- Modify: `include/boundary_mesh/transition/transition_boundary_checker.hpp`
- Modify: `src/transition/transition_boundary_checker.cpp`
- Test: `tests/unit/transition/transition_boundary_checker_test.cpp`

**Interfaces:**
- Consumes: Task 1 的 `OwnedBoundaryTriangle` 权限字段和 `SlidingIntersectionIndex::query()`。
- Produces: `TransitionBoundaryInput::sliding_surface` 及非法命中到 `rollback_high_faces` 的确定性映射。

- [ ] **Step 1: 写 Symmetry/Internal 失败测试**

构造位于 `z=0` 的滑移三角形和穿越它的过渡候选：

```cpp
TransitionBoundaryInput input;
input.candidate_triangles.push_back(triangle(
    {{{0.2,0.2,-0.1},{0.8,0.2,0.1},{0.2,0.8,0.1}}},
    candidate_keys, 70, {70}));
input.sliding_surface = &sliding_index.value();
const auto rollback = checker.findRollbackFaces(input);
if (!rollback.hasValue() ||
    rollback.value() != std::vector<SurfaceFaceId>{70})
    return 21;
```

同一测试把表面 tag 改为 `Internal`，预期仍回退 `{70}`。另加授权边贴合预期空回退，以及授权自身 region 后仍命中第二 region 预期回退。

- [ ] **Step 2: 运行并确认红灯**

```powershell
cmake --build build --config Release --target boundary_mesh_transition_boundary_checker_test -- /m:1
```

Expected: 编译失败，`TransitionBoundaryInput` 尚无 `sliding_surface`。

- [ ] **Step 3: 增加输入并执行查询**

在 `TransitionBoundaryInput` 加入：

```cpp
const SlidingIntersectionIndex *sliding_surface{};
```

在 `findRollbackFaces()` 的每个 exposed candidate 上执行：

```cpp
const auto permissions = buildSlidingContactPermissions(
    owned[index].vertex_sliding_region_ids,
    owned[index].physical_edge_mask,
    owned[index].complete_face_exemption_regions);
const auto hit = input.sliding_surface->query(
    owned[index].points, permissions);
if (!hit.hasValue())
    return RollbackResult::failure(
        TransitionBoundaryError{hit.error()});
if (hit.value().intersected)
    appendOwner(rollback, owned[index].owner);
```

若候选需要自身 region 同侧豁免，复用常规检查器已有逻辑抽成 Spatial/Growth 内部纯 helper；忽略自身 region 后必须再次查询其他 region。禁止把整个滑移索引塞入普通 `CollisionIndex`。

- [ ] **Step 4: 运行 focused tests**

```powershell
cmake --build build --config Release --target boundary_mesh_transition_boundary_checker_test boundary_mesh_layer_collision_checker_test -- /m:1
ctest --test-dir build -C Release -R "transition_boundary_checker|layer_collision_checker|sliding_intersection" --output-on-failure
```

Expected: 合法接触不回退，Symmetry/Internal 穿越均回退，其他 region 不被自身豁免隐藏。

- [ ] **Step 5: 提交**

```powershell
git add -- include/boundary_mesh/transition/transition_boundary_checker.hpp src/transition/transition_boundary_checker.cpp tests/unit/transition/transition_boundary_checker_test.cpp
git commit -m "feat: check transition boundaries against sliding surfaces"
```

---

### Task 3: 将静态索引接入逐层过渡协调，但隔离多法向阶段

**Files:**
- Modify: `include/boundary_mesh/growth/regular_layer_growth.hpp`
- Modify: `src/growth/regular_layer_generator.cpp`
- Modify: `src/boundary_layer/incremental_boundary_layer_generator.cpp`
- Modify: `include/boundary_mesh/transition/layer_transition_resolver.hpp`
- Modify: `src/transition/layer_transition_resolver.cpp`
- Test: `tests/integration/incremental_layer_transition_pipeline_test.cpp`
- Test: `tests/integration/transition/boundary_layer_generation_pipeline_test.cpp`

**Interfaces:**
- Consumes: Task 2 的 `TransitionBoundaryInput::sliding_surface`。
- Produces: 逐层 candidate callback 可访问生成器已构建的 `SlidingIntersectionIndex` 和 `SlidingSurfaceSet`；多法向 API 不变。

- [ ] **Step 1: 写增量流水线红灯测试**

在 `incremental_layer_transition_pipeline_test.cpp` 增加一个两相邻 Wall 面不同停止层、过渡模板穿越 `Symmetry` 的案例。断言接入前穿越候选仍被保留；期望新行为为：

```cpp
if (!result.hasValue()) return 40;
const auto &growth = result.value();
if (growth.faces[high_face].stop_reason !=
        FaceStopReason::NeighborLayerConstraint &&
    growth.faces[high_face].stop_reason != FaceStopReason::Collision)
    return 41;
if (std::none_of(growth.mesh.metadata.begin(), growth.mesh.metadata.end(),
    [](const CellMetadata &m)
    { return m.role == CellRole::LayerTransition; }))
    return 42;
```

测试还需独立审计接受的逐层常规/过渡单元没有跨越滑移面。

- [ ] **Step 2: 运行测试确认因未接线失败**

```powershell
cmake --build build --config Release --target boundary_mesh_incremental_layer_transition_pipeline_test -- /m:1
ctest --test-dir build -C Release -R incremental_layer_transition_pipeline --output-on-failure
```

Expected: 新断言失败，过渡候选尚未查询滑移索引。

- [ ] **Step 3: 扩展逐层候选回调上下文**

将 `RegularLayerGrowthOptions::candidate_rejections` 的签名扩展为：

```cpp
std::function<std::vector<SurfaceFaceId>(
    const GrowthFront &, const LayerStepResult &,
    const std::vector<VertexId> &,
    const CollisionIndex &, const SlidingIntersectionIndex &,
    const SlidingSurfaceSet &,
    const ExposedBoundaryTracker &)>
    candidate_rejections;
```

`RegularLayerGenerator` 使用其已构建一次的 `sliding_collision` 和 `sliding_surfaces` 调用回调，避免增量层每层重复构树。

- [ ] **Step 4: 传入过渡 resolver**

增量生成器 lambda 接收新增参数，并设置：

```cpp
input.sliding_surface = &sliding_surface;
input.sliding_surfaces = &sliding_surfaces;
```

`LayerTransitionResolver` 每轮生成 `TransitionBoundaryInput` 时原样转发这些只读指针。不得修改 `generateMultiNormalTransition()`、`MultiNormalOptions` 或 `MultiNormalTransitionGenerator`。

- [ ] **Step 5: 加模块边界断言**

在完整生成流水线测试中断言多法向结果与接线前夹具数量一致；在 CMake 模块边界脚本中确认 `src/multi_normal` 和 `include/boundary_mesh/multi_normal` 未新增 `sliding_intersection` include。

- [ ] **Step 6: 运行逐层和完整入口测试**

```powershell
cmake --build build --config Release --target boundary_mesh_incremental_layer_transition_pipeline_test boundary_mesh_boundary_layer_generation_pipeline_test boundary_mesh_multi_normal_module_boundary_test -- /m:1
ctest --test-dir build -C Release -R "incremental_layer_transition_pipeline|boundary_layer_generation_pipeline|multi_normal_module_boundary" --output-on-failure
```

Expected: 过渡候选发生确定性回退，稳定结果保留过渡单元，多法向模块边界不变。

- [ ] **Step 7: 提交**

```powershell
git add -- include/boundary_mesh/growth/regular_layer_growth.hpp src/growth/regular_layer_generator.cpp src/boundary_layer/incremental_boundary_layer_generator.cpp include/boundary_mesh/transition/layer_transition_resolver.hpp src/transition/layer_transition_resolver.cpp tests/integration/incremental_layer_transition_pipeline_test.cpp tests/integration/transition/boundary_layer_generation_pipeline_test.cpp
git commit -m "feat: enforce sliding checks during layer transitions"
```

---

### Task 4: 真实完整生成回归和 VTK

**Files:**
- Modify: `tests/integration/sliding_surface_intersection_regression_test.cpp`
- Modify: `README.md`

**Interfaces:**
- Consumes: Task 3 完整逐层过渡检测。
- Produces: Symmetry/Internal 完整体网格 VTK 和存在过渡层、无非法相交的回归证据。

- [ ] **Step 1: 将真实测试切换到完整增量入口并观察失败**

把 `generateCase()` 从 `generateRegularLayers()` 改为 `generateIncrementalBoundaryLayers()`；保留多法向关闭。增加：

```cpp
const auto transition_count = std::count_if(
    result.mesh.metadata.begin(), result.mesh.metadata.end(),
    [](const CellMetadata &metadata)
    { return metadata.role == CellRole::LayerTransition; });
if (transition_count == 0) return 9;
```

Run:

```powershell
cmake --build build --config Release --target boundary_mesh_sliding_surface_intersection_regression_test -- /m:1
ctest --test-dir build -C Release -R boundary_mesh_sliding_surface_intersection_regression_test --output-on-failure
```

Expected: 当前仅规则层入口或未接入的过渡滑移检测使新增断言失败。

- [ ] **Step 2: 增加独立最终网格审计**

对所有 `CellRole::RegularLayer` 和 `CellRole::LayerTransition` 单元按 Prism/Hexa/Pyramid/Tetra 的边界三角形模板遍历，调用 `SlidingIntersectionIndex::query()`。从 `LayerVertexTable` 和源表面邻接重建逐点 region；真实边和完整侧面权限必须来自单元模板，不能授权人工对角线。断言 Symmetry/Internal 两份输出的非法计数均为 0。

- [ ] **Step 3: 写完整 VTK 并核对两种边界类型**

继续写：

```text
build/tests/symm_intersection_symmetry_boundary_layer.vtk
build/tests/symm_intersection_internal_boundary_layer.vtk
```

断言文件非空、两种类型逐面停止决策一致，且两份输出均包含 `CellRole::LayerTransition`。

- [ ] **Step 4: 更新 README**

明确说明：多法向生成阶段跳过滑移相交；逐层规则单元和逐层层差过渡联合检测均使用静态滑移索引。

- [ ] **Step 5: 运行真实回归及 focused suite**

```powershell
cmake --build build --config Release --target boundary_mesh_sliding_surface_intersection_regression_test -- /m:1
ctest --test-dir build -C Release -R "sliding_surface_intersection_regression|transition_boundary_checker|incremental_layer_transition_pipeline" --output-on-failure
```

Expected: 两份完整 VTK 含过渡单元，所有接受的逐层单元无非法滑移相交。

- [ ] **Step 6: 提交**

```powershell
git add -- tests/integration/sliding_surface_intersection_regression_test.cpp README.md
git commit -m "test: preserve transitions in sliding intersection regression"
```

---

### Task 5: 全量验证与范围审计

**Files:**
- Verify only; only fix files directly implicated by failures.

**Interfaces:**
- Consumes: Tasks 1-4。
- Produces: 经验证的逐层常规及过渡滑移相交检测。

- [ ] **Step 1: 检查差异和多法向范围**

```powershell
git diff --check HEAD~4..HEAD
git diff --name-only HEAD~4..HEAD | Select-String 'src/multi_normal|include/boundary_mesh/multi_normal'
```

Expected: 除原样测试夹具既有尾随空格外无格式错误；第二条无输出。

- [ ] **Step 2: 完整 Release 构建**

```powershell
cmake -S . -B build
cmake --build build --config Release -- /m:1
```

Expected: exit code 0。

- [ ] **Step 3: focused tests**

```powershell
ctest --test-dir build -C Release -R "sliding_intersection|layer_collision_checker|transition_boundary_checker|incremental_layer_transition_pipeline|boundary_layer_generation_pipeline|multi_normal_module_boundary" --output-on-failure
```

Expected: 0 failed。

- [ ] **Step 4: 完整 CTest**

```powershell
ctest --test-dir build -C Release --output-on-failure
```

Expected: 0 failed。

- [ ] **Step 5: 最终状态**

```powershell
git -c submodule.recurse=false status --short --ignore-submodules=all
git log --oneline -5
```

Expected: 工作区干净，提交仅包含逐层过渡滑移检测、测试和文档。
