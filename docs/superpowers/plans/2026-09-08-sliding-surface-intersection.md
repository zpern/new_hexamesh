# 常规边界层滑移面相交检测实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为常规逐层生成增加 `Symmetry` 和 `Internal` 静态相交检测，放行拓扑允许的贴面接触，阻止接受单元穿过滑移面，并用旧 `blmesh` 的 `symm_intersection/BoundaryMeshing.txt` 原参数完成回归。

**Architecture:** 保留现有普通 `CollisionIndex`，新增坐标驱动的 `SlidingIntersectionIndex`。`LayerCollisionChecker` 为候选棱柱/六面体生成带区域权限和真实物理边掩码的三角形查询；自身曲面命中可通过单元级局部同侧判断仅忽略当前区域，然后继续查询其他区域。

**Tech Stack:** C++17、Eigen、现有 `BinaryAabbTree`/`Result`/CTest、legacy ASCII `BoundaryMeshing.txt` 测试夹具。

## Global Constraints

- 仅接入常规逐层生成；不得修改多法向过渡相交路径。
- `Symmetry` 与 `Internal` 使用同一几何算法，以 `region_id` 独立授权。
- 普通输入障碍、历史暴露边界和同层自碰撞继续使用现有 `CollisionIndex`。
- 人工三角剖分对角线不得获得物理边接触权限。
- 完整面豁免仅适用于四个上下节点共同属于同一区域的侧四边形；顶面不得完整豁免。
- 非有限、退化和局部方位不可判定时保守失败或保留相交。
- 真实回归使用 `nLN=20`、`dLen=0.10000000100000001`、`dRto=1.19999999999999996`、`max_prism_skewness=1.0`、关闭多法向。
- `BoundaryMeshing.txt` 是唯一原始输入；`input_surface.vtk` 只是同一网格的导出结果。

---

## 文件结构

- 新建 `include/boundary_mesh/spatial/sliding_intersection.hpp`：交集类型、区域权限、纯几何与纯权限 helper。
- 新建 `src/spatial/sliding_intersection.cpp`：非共面裁切、共面二维裁剪、允许点/边判断、动态容差。
- 新建 `include/boundary_mesh/spatial/sliding_intersection_index.hpp`：静态滑移三角形、命中结果、按区域查询和局部方位接口。
- 新建 `src/spatial/sliding_intersection_index.cpp`：从 `SurfaceMesh` 构树、AABB 查询、最近三角形法向与有符号侧值。
- 修改 `include/boundary_mesh/growth/exposed_boundary.hpp`：候选顶/底点携带逐点滑移区域。
- 修改 `include/boundary_mesh/growth/layer_collision_checker.hpp` 与 `src/growth/layer_collision_checker.cpp`：构造真实边权限并执行滑移面检查。
- 修改 `src/growth/regular_layer_generator.cpp`：初始化静态索引并传给常规层碰撞过滤。
- 修改 `include/boundary_mesh/growth/regular_layer_growth_error.hpp`：若需要，复用或扩展碰撞初始化错误包装。
- 修改根 `CMakeLists.txt` 与 `tests/CMakeLists.txt`：加入实现和测试目标。
- 新建 `tests/unit/spatial/sliding_intersection_test.cpp`、`tests/unit/spatial/sliding_intersection_index_test.cpp`。
- 修改 `tests/unit/growth/layer_collision_checker_test.cpp` 和 `tests/integration/collision_growth_pipeline_test.cpp`。
- 新建 `tests/integration/sliding_surface_intersection_regression_test.cpp`。
- 新建 `tests/support/boundary_meshing_fixture.hpp/.cpp`：只供测试使用的严格旧格式读取器。
- 新建 `tests/data/symm_intersection/BoundaryMeshing.txt`：真实案例的原样副本。

---

### Task 1: 坐标交集分类与区域接触权限

**Files:**
- Create: `include/boundary_mesh/spatial/sliding_intersection.hpp`
- Create: `src/spatial/sliding_intersection.cpp`
- Create: `tests/unit/spatial/sliding_intersection_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Produces: `SlidingContactPermission { std::uint8_t vertex_mask; std::uint8_t edge_mask; bool complete_face_exemption; }`.
- Produces: `SlidingIntersectionGeometry classifySlidingIntersection(const TrianglePoints &, const TrianglePoints &)`.
- Produces: `Result<bool, SpatialError> hasInvalidSlidingIntersection(const TrianglePoints &, const TrianglePoints &, const SlidingContactPermission &)`.
- Produces: `std::map<std::uint32_t, SlidingContactPermission> buildSlidingContactPermissions(const std::array<std::vector<std::uint32_t>,3> &, std::uint8_t, const std::vector<std::uint32_t> &)`.
- Produces: `bool slidingSideValuesStayOnOneSide(const std::vector<Scalar> &, Scalar)`.

- [ ] **Step 1: 写失败测试，覆盖交集分类与权限语义**

测试使用平面三角形 `surface={{0,0,0},{2,0,0},{0,2,0}}`，至少断言：无权限穿透为非法；授权顶点的纯点接触合法；从授权顶点延伸到内部非法；授权物理边上的线接触合法；未授权人工对角线非法；无完整豁免的共面面积重叠非法；有完整豁免合法；NaN 和重叠退化三角形返回 `SpatialError` 或非法；`{-0.1,0,0.2}` 判跨侧，`{0,0.1,0.2}` 判同侧。

核心断言形式：

```cpp
SlidingContactPermission edge;
edge.vertex_mask = 0b011;
edge.edge_mask = 0b001;
assert(!hasInvalidSlidingIntersection(edge_touch, surface, edge).value());
assert(hasInvalidSlidingIntersection(interior_crossing, surface, edge).value());

SlidingContactPermission complete;
complete.complete_face_exemption = true;
assert(!hasInvalidSlidingIntersection(coplanar_area, surface, complete).value());
```

- [ ] **Step 2: 加入空声明和 CMake 目标并验证失败**

Run:

```powershell
cmake --build build --config Debug --target boundary_mesh_sliding_intersection_test
```

Expected: 编译或链接失败，因为新接口尚未实现。

- [ ] **Step 3: 实现动态容差、非共面裁切和共面二维裁剪**

在 `.cpp` 内实现：

```cpp
Scalar slidingTolerance(const TrianglePoints &a, const TrianglePoints &b)
{
    Scalar scale = Scalar{1};
    for (const auto &tri : {a, b})
        for (const Point3 &p : tri)
            scale = std::max(scale, p.cwiseAbs().maxCoeff());
    for (const auto &tri : {a, b})
        for (std::size_t i = 0; i < 3; ++i)
            scale = std::max(scale, (tri[(i + 1) % 3] - tri[i]).norm());
    return std::max(Scalar{1e-10},
        Scalar{64} * std::numeric_limits<Scalar>::epsilon() * scale);
}
```

分类必须产生 `Empty/Points/Segments/CoplanarArea`，点按容差去重；二维裁剪的叉积阈值使用 `tolerance * clip_edge_length`。

- [ ] **Step 4: 实现按允许顶点和真实边验证完整交集集合**

`CoplanarArea` 仅由 `complete_face_exemption` 放行。`Points` 中每个点必须位于允许顶点或允许边；`Segments` 的两个端点必须同时位于同一条允许边。先校验所有坐标有限，再做 AABB 和退化判断。

- [ ] **Step 5: 实现区域权限构造和同侧纯函数**

物理边 bit 顺序固定为 `(0,1)=0b001`、`(1,2)=0b010`、`(2,0)=0b100`。只有物理边 bit 已设置且两端都包含同一区域时才设置 `edge_mask`。完整豁免列表仅设置对应区域的 `complete_face_exemption`。

- [ ] **Step 6: 构建并运行测试**

```powershell
cmake --build build --config Debug --target boundary_mesh_sliding_intersection_test
ctest --test-dir build -C Debug -R "boundary_mesh_sliding_intersection_test" --output-on-failure
```

Expected: 目标构建成功，测试通过。

- [ ] **Step 7: 提交纯几何实现**

```powershell
git add -- CMakeLists.txt tests/CMakeLists.txt include/boundary_mesh/spatial/sliding_intersection.hpp src/spatial/sliding_intersection.cpp tests/unit/spatial/sliding_intersection_test.cpp
git commit -m "feat: classify sliding surface contacts"
```

---

### Task 2: 静态滑移面 AABB 索引与局部方位

**Files:**
- Create: `include/boundary_mesh/spatial/sliding_intersection_index.hpp`
- Create: `src/spatial/sliding_intersection_index.cpp`
- Create: `tests/unit/spatial/sliding_intersection_index_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `SlidingContactPermission` 和 `hasInvalidSlidingIntersection` from Task 1.
- Produces: `SlidingIntersectionIndex::build(const SurfaceMesh &)`.
- Produces: `Result<SlidingIntersectionHit, SpatialError> query(const TrianglePoints &, const std::map<std::uint32_t, SlidingContactPermission> &, const std::set<std::uint32_t> &ignored = {}) const`.
- Produces: `Result<Vector3, SpatialError> faceNormalAtPoint(std::uint32_t, const Point3 &) const`.
- Produces: `Result<Scalar, SpatialError> signedSideToRegion(std::uint32_t, const Point3 &, const Vector3 &) const`.

- [ ] **Step 1: 写失败测试**

构造两个相交平面区域：区域 10 为 `Symmetry` 的 `z=0`，区域 11 为 `Internal` 的 `x=0`；另加 Wall 面。断言索引只含两个滑移区域；查询同时穿过两面先命中一个；忽略 10 后仍命中 11；区域 10 权限不放行区域 11；未知区域的法向查询失败；正负 z 点对区域 10 返回相反符号。

- [ ] **Step 2: 声明接口、注册构建文件并验证失败**

```powershell
cmake --build build --config Debug --target boundary_mesh_sliding_intersection_index_test
```

Expected: 新索引接口未实现导致链接失败。

- [ ] **Step 3: 实现 `build`**

遍历 `SurfaceMesh::faces/face_tags`，只收集 `isSlidingBoundary(tag.kind)`。三角形直接加入；四边形使用 `{0,1,2}`、`{0,2,3}`。为每个图元保存 `TrianglePoints`、kind、region、source face、local triangle，并通过 `makeAabb` 构建 `BinaryAabbTree`。标签数量不匹配、索引越界、非有限或退化时返回对应 `SpatialError`。

- [ ] **Step 4: 实现按坐标和区域权限查询**

查询先验证坐标并构造 AABB，遍历树返回的候选；跳过 `ignored` 区域，为当前图元取同 `region_id` 权限（不存在则空权限），调用 Task 1 窄阶段。返回首个非法命中的区域和图元编号；空树或无命中返回 `intersected=false`。

- [ ] **Step 5: 实现指定区域最近三角形法向和有符号侧值**

只搜索目标 `region_id`。复用或在文件内实现三角形最近点算法；选择平方距离最小的非退化三角形。法向单位化；若与参考法向点积为负则翻转：

```cpp
if (local_normal.dot(reference_normal) < Scalar{0})
    local_normal = -local_normal;
return (query - closest_point).dot(local_normal);
```

- [ ] **Step 6: 运行索引及既有空间测试**

```powershell
cmake --build build --config Debug --target boundary_mesh_sliding_intersection_index_test boundary_mesh_collision_index_test
ctest --test-dir build -C Debug -R "sliding_intersection|collision_index" --output-on-failure
```

Expected: 全部通过。

- [ ] **Step 7: 提交静态索引**

```powershell
git add -- CMakeLists.txt tests/CMakeLists.txt include/boundary_mesh/spatial/sliding_intersection_index.hpp src/spatial/sliding_intersection_index.cpp tests/unit/spatial/sliding_intersection_index_test.cpp
git commit -m "feat: index sliding boundary surfaces"
```

---

### Task 3: 候选单元权限和自身区域同侧判断

**Files:**
- Modify: `include/boundary_mesh/growth/exposed_boundary.hpp`
- Modify: `include/boundary_mesh/growth/layer_collision_checker.hpp`
- Modify: `src/growth/layer_collision_checker.cpp`
- Modify: `tests/unit/growth/layer_collision_checker_test.cpp`

**Interfaces:**
- Consumes: `SlidingIntersectionIndex::query/faceNormalAtPoint/signedSideToRegion` from Task 2.
- Produces: `buildLayerBoundaryCandidates(..., const SlidingSurfaceSet &)`，其中 `bottom/top.vertex_sliding_region_ids` 与 points 一一对应。
- Produces: `LayerCollisionChecker::filterAgainstObstacles(const CollisionIndex &, const SlidingIntersectionIndex &, const SlidingSurfaceSet &, const ExposedBoundaryTracker &, const GrowthFront &, const LayerStepResult &) const`.

- [ ] **Step 1: 写候选拓扑失败测试**

为三角前沿和四边形前沿分别构造一步结果，断言顶/底每个点保留对应 `sliding_region_ids`，侧面拆分的物理边掩码不包含内部对角线，四节点公共区域只产生于完整侧四边形。

- [ ] **Step 2: 写碰撞行为失败测试**

构造：合法贴合自身 `Symmetry`；穿过 `Symmetry`；穿过 `Internal`；同侧自身区域同时撞到另一区域。断言前三者依次为保留、停止、停止，最后一个仍停止。

- [ ] **Step 3: 扩展 `BoundaryFace` 的逐点区域数据**

加入：

```cpp
std::vector<std::vector<std::uint32_t>> vertex_sliding_region_ids;
```

`buildLayerBoundaryCandidates` 从 `current_front` 和 `step.next_front` 复制排序去重后的区域；校验长度与 `points` 一致。

- [ ] **Step 4: 实现候选顶面/侧面滑移查询描述**

在 `layer_collision_checker.cpp` 私有区定义查询记录，包含三角坐标、三点区域、物理边 mask、完整豁免区域。侧四边形拆分采用设计固定顺序；三角顶面 mask 为 `0b111`，四边形顶面两个三角形只标记真实外边。

- [ ] **Step 5: 实现单元级同侧判断**

对首次命中区域 F，列关联条件为上下点均包含 F。关联列数量必须 `>0 && < column_count`。在一个关联上点取参考法向；对所有非关联列的上下点取有符号侧值；全部调用成功且 `slidingSideValuesStayOnOneSide(values, 1e-10)` 时，重新查询当前三角形并忽略 F。重新查询的其他区域命中仍为碰撞。

- [ ] **Step 6: 组合普通障碍、历史边界和滑移面结果**

每个候选的停止条件为：

```cpp
stopped[index] =
    hitsIndex(triangles, original_surface) ||
    hitsIndex(triangles, history) ||
    hitsSlidingSurface(candidate, sliding_index);
```

保留旧签名为测试/兼容转发重载，使用空滑移索引；或者一次性更新所有调用点，二选一时优先显式更新调用点，避免隐藏未检测路径。

- [ ] **Step 7: 运行 focused tests**

```powershell
cmake --build build --config Debug --target boundary_mesh_layer_collision_checker_test
ctest --test-dir build -C Debug -R "layer_collision_checker|sliding_intersection" --output-on-failure
```

Expected: 合法贴面保留，Symmetry/Internal 穿透停止，既有断言不回归。

- [ ] **Step 8: 提交候选级检测**

```powershell
git add -- include/boundary_mesh/growth/exposed_boundary.hpp include/boundary_mesh/growth/layer_collision_checker.hpp src/growth/layer_collision_checker.cpp tests/unit/growth/layer_collision_checker_test.cpp
git commit -m "feat: check regular cells against sliding surfaces"
```

---

### Task 4: 接入常规逐层生成器

**Files:**
- Modify: `src/growth/regular_layer_generator.cpp`
- Modify: `include/boundary_mesh/growth/regular_layer_growth_error.hpp`
- Modify: `tests/integration/collision_growth_pipeline_test.cpp`

**Interfaces:**
- Consumes: `SlidingIntersectionIndex::build` and expanded `filterAgainstObstacles`.
- Produces: regular generation initialization failure mapped to `CollisionInitializationFailure` and accepted cells filtered by sliding intersection.

- [ ] **Step 1: 扩展集成测试使其先失败**

保留已有 Farfield 障碍案例，增加相同几何分别标记 `Symmetry`/`Internal` 的参数化运行；候选穿面时断言相关 `FaceGrowthRecord.stop_reason == FaceStopReason::Collision`，合法贴面时断言 cell 被接受。

- [ ] **Step 2: 在生成器初始化静态索引**

在现有 `original_collision` 后构建：

```cpp
const auto sliding_collision = SlidingIntersectionIndex::build(surface_mesh);
if (!sliding_collision.hasValue())
    return GrowthResult::failure(
        CollisionInitializationFailure{sliding_collision.error()});
```

把已构建的 `sliding_surfaces.value()` 和 `sliding_collision.value()` 传给每层 `filterAgainstObstacles`。索引只构建一次，不随层更新。

- [ ] **Step 3: 运行常规层集成测试**

```powershell
cmake --build build --config Debug --target boundary_mesh_collision_growth_pipeline_test boundary_mesh_regular_layer_growth_pipeline_test
ctest --test-dir build -C Debug -R "collision_growth_pipeline|regular_layer_growth_pipeline" --output-on-failure
```

Expected: 新增 Symmetry/Internal 穿透测试通过，已有规则层行为通过。

- [ ] **Step 4: 提交生成器接入**

```powershell
git add -- src/growth/regular_layer_generator.cpp include/boundary_mesh/growth/regular_layer_growth_error.hpp tests/integration/collision_growth_pipeline_test.cpp
git commit -m "feat: enforce sliding collisions during regular growth"
```

---

### Task 5: 真实 `BoundaryMeshing.txt` 夹具与修改前基线

**Files:**
- Create: `tests/data/symm_intersection/BoundaryMeshing.txt`
- Create: `tests/support/boundary_meshing_fixture.hpp`
- Create: `tests/support/boundary_meshing_fixture.cpp`
- Create: `tests/integration/sliding_surface_intersection_regression_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Produces: `BoundaryMeshingFixture readBoundaryMeshingFixture(const std::filesystem::path &)` containing `SurfaceMesh mesh`, face parameter records and global controls.
- Produces: CTest `boundary_mesh_sliding_surface_intersection_regression_test`.

- [ ] **Step 1: 原样复制夹具并验证哈希一致**

```powershell
New-Item -ItemType Directory -Force 'tests/data/symm_intersection'
Copy-Item -LiteralPath 'C:/Users/zpern/Desktop/program/BoundaryLayer/blmesh/build/input/symm_intersection/BoundaryMeshing.txt' -Destination 'tests/data/symm_intersection/BoundaryMeshing.txt'
$sourceHash = (Get-FileHash 'C:/Users/zpern/Desktop/program/BoundaryLayer/blmesh/build/input/symm_intersection/BoundaryMeshing.txt' -Algorithm SHA256).Hash
$fixtureHash = (Get-FileHash 'tests/data/symm_intersection/BoundaryMeshing.txt' -Algorithm SHA256).Hash
if ($sourceHash -ne $fixtureHash) { throw 'fixture hash mismatch' }
```

Expected: hashes equal。执行时若必须遵守补丁式文件写入约束，使用 `apply_patch` 添加同字节内容，并仍执行哈希校验。

- [ ] **Step 2: 写解析器失败测试**

断言读取结果为 4298 点、8592 三角形；全局参数精确等于文件值；面 3 为旧类型 2；面 0/1/2/4/5 为旧类型 1；六条 `variable_para` 均被读取，face 3 为零层，其余为 20 层。

- [ ] **Step 3: 实现严格测试读取器**

按旧 `vmesh/src/main.cpp::readBoundaryMeshing` 顺序读取全部字段。点 ID 和三角形 ID 校验为 1 基连续范围；三角形节点按文件规则转换到当前 `VertexId`；`boundary_info` 类型映射为当前 enum；`variable_para` 以 face ID 存储。未知 key、重复/越界 ID、缺失数据立即抛出测试读取异常。

- [ ] **Step 4: 写禁用滑移检测的缺陷证明**

用当前默认/旧签名障碍过滤路径生成候选或完整常规层，然后用 `SlidingIntersectionIndex` 对接受单元做只读审计。断言至少存在一个未授权的 region 3 命中。该断言放在专门 helper `countInvalidAcceptedSlidingContacts` 的基线分支中，明确说明它证明夹具覆盖旧缺陷；CTest 产品路径不把“产生错误网格”当成成功结果。

- [ ] **Step 5: 运行解析和基线测试**

```powershell
cmake --build build --config Release --target boundary_mesh_sliding_surface_intersection_regression_test
ctest --test-dir build -C Release -R boundary_mesh_sliding_surface_intersection_regression_test --output-on-failure
```

Expected: 解析断言通过；禁用新检测时非法命中计数大于零；启用路径若尚未写断言则测试在下一步前保持明确失败。

- [ ] **Step 6: 提交夹具、解析器和基线**

```powershell
git add -- tests/data/symm_intersection/BoundaryMeshing.txt tests/support/boundary_meshing_fixture.hpp tests/support/boundary_meshing_fixture.cpp tests/integration/sliding_surface_intersection_regression_test.cpp tests/CMakeLists.txt
git commit -m "test: reproduce sliding surface intersection case"
```

---

### Task 6: 同参数 Symmetry/Internal 端到端验收与 VTK 产物

**Files:**
- Modify: `tests/integration/sliding_surface_intersection_regression_test.cpp`
- Modify: `tests/CMakeLists.txt`
- Modify: `README.md`

**Interfaces:**
- Consumes: Task 5 fixture and production regular generator.
- Produces: two end-to-end assertions and optional VTK outputs under the test binary directory.

- [ ] **Step 1: 为同一案例建立两个变体**

从夹具复制 mesh：原始 face 3 映射为 `Symmetry`；第二份只把 face 3 改为 `Internal`。两份均只从 face 0/1/2/4/5 的 Wall 顶点创建 profiles：`layer_count=20`、`first_height=0.10000000100000001`、`growth_ratio=1.19999999999999996`。设置：

```cpp
RegularLayerGrowthOptions options;
options.cell_quality.maximum_skewness = Scalar{1};
options.isotropic_height = Scalar{1};
options.max_layer_diff = 6; // old vmesh debug entry default
```

未在当前 API 中存在的 pyramid/ratio-diff 参数只做解析值断言，不错误映射。

- [ ] **Step 2: 写启用检测后的独立结果审计**

对两个生成结果的所有最终接受 Prism/Hexa 边界三角形，用测试侧独立遍历滑移面并调用 `hasInvalidSlidingIntersection`，权限由源点区域和真实边重建。断言非法计数为 0；同时断言两变体的每个源 Wall 面 `accepted_layer_count/status/stop_reason/stop_layer` 一致。

- [ ] **Step 3: 写两个可视化输出**

使用现有 `writeLegacyVtk` 输出：

```text
std::filesystem::current_path()/symm_intersection_symmetry_boundary_layer.vtk
std::filesystem::current_path()/symm_intersection_internal_boundary_layer.vtk
```

CTest 断言输出成功且文件非空；不得写入或覆盖旧 `blmesh/build/input/symm_intersection`。

- [ ] **Step 4: 运行真实回归**

```powershell
cmake --build build --config Release --target boundary_mesh_sliding_surface_intersection_regression_test
ctest --test-dir build -C Release -R boundary_mesh_sliding_surface_intersection_regression_test --output-on-failure
```

Expected: 禁用检测基线命中数 `>0`；Symmetry 和 Internal 启用检测后的非法命中数均为 `0`；两个 VTK 均生成；两变体决策一致。

- [ ] **Step 5: 更新 README**

在规则层碰撞阶段说明：原始与动态滑移面仍不进入普通 `CollisionIndex`，但现在由独立静态索引执行带区域权限的非法相交检测；多法向过渡暂未接入。

- [ ] **Step 6: 提交真实回归完成项**

```powershell
git add -- tests/integration/sliding_surface_intersection_regression_test.cpp tests/CMakeLists.txt README.md
git commit -m "test: verify symmetry and internal collision regression"
```

---

### Task 7: 全量验证与范围审计

**Files:**
- Verify only; fix only files implicated by failures.

**Interfaces:**
- Consumes: all previous tasks.
- Produces: verified regular-layer-only sliding collision feature.

- [ ] **Step 1: 运行格式和差异检查**

```powershell
git -c diff.ignoreSubmodules=all diff --check
git -c submodule.recurse=false status --short --ignore-submodules=all
```

Expected: 无 whitespace error；仅存在本功能相关修改。

- [ ] **Step 2: 构建全部目标**

```powershell
cmake --build build --config Release
```

Expected: 构建成功，无新 warning-as-error。

- [ ] **Step 3: 运行 focused suite**

```powershell
ctest --test-dir build -C Release -R "sliding_intersection|layer_collision_checker|collision_growth_pipeline|regular_layer_growth_pipeline" --output-on-failure
```

Expected: 全部通过。

- [ ] **Step 4: 运行完整 CTest**

```powershell
ctest --test-dir build -C Release --output-on-failure
```

Expected: 0 failed。

- [ ] **Step 5: 审计非目标路径**

```powershell
git diff --name-only HEAD~6..HEAD | rg "multi_normal"
```

Expected: 无输出，证明多法向过渡路径未被修改。
