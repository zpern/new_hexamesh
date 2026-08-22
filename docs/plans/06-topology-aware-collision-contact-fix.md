# 拓扑感知合法碰撞接触修复实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修复连续 Wall 前沿中共享顶点、共享边和共享侧面的候选 Prism/Hexa 被误判为碰撞的问题，同时保留公共拓扑区域以外的真实穿插检测。

**Architecture:** 每个碰撞三角形除自身三角形 key 外，还保存所属完整 Triangle/Quad 边界面的点和 key。Spatial 从两个完整边界面的共享 key 构造允许公共点、公共边或公共面，再判断三角形实际交集是否完全包含在该公共特征内；Growth 继续整层批量收集非法候选对，保持顺序不变量。

**Tech Stack:** C++17、Eigen、`tiger_geom`、CMake、CTest、MSVC Debug/Release。

## Global Constraints

- 不使用子代理；在现有 `feature/collision-local-stop` 分支内执行。
- 先 RED、再 GREEN；每个生产行为必须先看到对应测试按预期失败。
- Growth、Quality、Surface 不得包含 `geom_func.h`，第三方精确谓词仍由 Spatial 私有实现使用。
- 合法共享关系只使用 `CollisionVertexKey`，不得使用坐标相等猜测拓扑。
- 不增加碰撞容差，所有几何判断继续严格与零比较。
- 同层检测必须整层收集后同时停止双方，结果不得依赖遍历顺序。
- 公共区域之外的穿插、重叠或非拓扑零距离接触仍必须停止。
- 新公共字段和枚举值添加中文 `//` 注释。
- `.superpowers/` 永不暂存或提交。

---

## File Structure

```text
include/boundary_mesh/spatial/collision_index.hpp
    为碰撞三角形保存所属完整边界面的点和拓扑 key

include/boundary_mesh/spatial/triangle_contact.hpp
src/spatial/triangle_contact.cpp
    提取三角形实际交集证据，并判断是否超出允许公共特征

src/spatial/collision_index.cpp
src/growth/layer_collision_checker.cpp
src/growth/exposed_boundary.cpp
    原始面、候选顶/侧面和历史外露面统一填充完整边界面元数据

tests/unit/spatial/triangle_contact_test.cpp
tests/unit/spatial/collision_index_test.cpp
tests/unit/growth/layer_collision_checker_test.cpp
    公共点、公共边、公共面的合法接触和越界穿插回归

src/cli/boundary_mesh_command.cpp
tests/integration/cgns_cli_pipeline_test.cpp
    输出并验证逐 `FaceStopReason` 汇总
```

---

### Task 1: 完整边界面拓扑元数据

**Files:**
- Modify: `include/boundary_mesh/spatial/collision_index.hpp`
- Modify: `src/spatial/collision_index.cpp`
- Modify: `src/growth/layer_collision_checker.cpp`
- Modify: `src/growth/exposed_boundary.cpp`
- Test: `tests/unit/spatial/collision_index_test.cpp`

**Interfaces:**
- Consumes: Triangle/Quad 边界面的完整点序、完整 `CollisionVertexKey` 序列及其固定 `v0-v2` 三角化。
- Produces: 每个 `CollisionTriangle` 同时具有三角形局部数据和所属完整边界面的数据。

- [ ] **Step 1: 写缺少完整边界面来源的 RED 测试**

在 `collision_index_test.cpp` 构造一个属于 Quad 的碰撞三角形，并通过索引稳定图元接口验证完整边界面元数据被保留：

```cpp
CollisionTriangle triangle = makeCollisionTriangle(
    {point0, point1, point2},
    {key0, key1, key2},
    {point0, point1, point2, point3},
    {key0, key1, key2, key3});

const auto index = CollisionIndex::build({triangle});
assert(index.hasValue());
assert(index.value().primitive(0).boundary_vertex_count == 4);
assert(index.value().primitive(0).boundary_vertex_keys[3] == key3);
```

- [ ] **Step 2: 运行 RED**

```powershell
cmake --build build --config Debug `
  --target boundary_mesh_spatial_collision_index_test
```

Expected: 编译失败，因为 `CollisionTriangle` 尚不能表达所属完整边界面。

- [ ] **Step 3: 增加完整边界面字段**

在 `CollisionTriangle` 末尾增加固定容量元数据：

```cpp
std::array<Point3, 4> boundary_points{}; // 所属完整 Triangle/Quad 边界面的点序
std::array<CollisionVertexKey, 4> boundary_vertex_keys{}; // 完整边界面的分层拓扑 key
std::uint8_t boundary_vertex_count{}; // 完整边界面有效顶点数，只允许 3 或 4
```

保留原字段顺序，使没有指定新字段的旧测试初始化仍能编译；新判定在计数为零时使用现有三角形局部语义。

- [ ] **Step 4: 让三类生产者填充元数据**

- `appendFaceTriangles(...)`：原始 Triangle/Quad 的每个拆分三角形携带原完整面；
- `candidateTriangles(...)`：候选顶面和每个侧面的拆分三角形携带对应完整 `BoundaryFace`；
- `ExposedBoundaryTracker::collisionTriangles()`：历史外露面的拆分三角形携带完整外露面。

统一使用小型内部辅助函数复制 3/4 个点和 key，并验证数量匹配。

- [ ] **Step 5: 运行目标测试 GREEN**

```powershell
cmake --build build --config Debug `
  --target boundary_mesh_spatial_collision_index_test
ctest --test-dir build -C Debug `
  -R boundary_mesh_spatial_collision_index_test `
  --output-on-failure
```

- [ ] **Step 6: 提交元数据基础**

```powershell
git add include/boundary_mesh/spatial/collision_index.hpp `
  src/spatial/collision_index.cpp `
  src/growth/layer_collision_checker.cpp `
  src/growth/exposed_boundary.cpp `
  tests/unit/spatial/collision_index_test.cpp
git diff --cached --check
git commit -m "fix: preserve collision boundary provenance"
```

---

### Task 2: 公共拓扑特征内的合法交集

**Files:**
- Modify: `include/boundary_mesh/spatial/triangle_contact.hpp`
- Modify: `src/spatial/triangle_contact.cpp`
- Modify: `src/spatial/collision_index.cpp`
- Test: `tests/unit/spatial/triangle_contact_test.cpp`
- Test: `tests/unit/spatial/collision_index_test.cpp`

**Interfaces:**
- Consumes: 两个 `CollisionTriangle` 及其完整边界面元数据。
- Produces: `hasIllegalTriangleContact(const CollisionTriangle &, const CollisionTriangle &)`，只在实际交集超出公共拓扑特征时返回 `true`。

- [ ] **Step 1: 写公共点、公共边和公共面的 RED 测试**

分别覆盖：

```cpp
// 只共享一个完整边界点：仅该点接触合法，沿其他方向延伸的交线非法。
assert(!hasIllegalTriangleContact(vertex_touch_a, vertex_touch_b).value());
assert(hasIllegalTriangleContact(vertex_cross_a, vertex_cross_b).value());

// 完整边界面共享一条相邻边：交集完全落在该边上合法，越过边进入面内非法。
assert(!hasIllegalTriangleContact(edge_touch_a, edge_touch_b).value());
assert(hasIllegalTriangleContact(edge_cross_a, edge_cross_b).value());

// 完整 Triangle/Quad 面相同：不同对角线三角化产生的共面重叠合法。
assert(!hasIllegalTriangleContact(shared_face_a, shared_face_b).value());
```

再构造坐标相同但 key 不同的共点/共边，断言仍为非法。

- [ ] **Step 2: 运行 RED**

```powershell
cmake --build build --config Debug `
  --target boundary_mesh_spatial_triangle_contact_test `
           boundary_mesh_spatial_collision_index_test
```

Expected: 编译失败，缺少 `CollisionTriangle` 重载；或断言失败，当前实现仍只按局部三角形 key 数量判断。

- [ ] **Step 3: 提取三角形交集证据**

将 `classifyTriangleContact(...)` 的内部过程扩展为私有详细结果：

```cpp
struct TriangleContactEvidence
{
    TriangleContactKind kind{TriangleContactKind::Disjoint};
    std::vector<Point3> points;
};
```

- 非共面相交复用 `appendPlaneIntersections(...)` 得到交点或交线端点；
- 共面情况保留裁剪多边形，并用被丢弃主轴对应的平面方程恢复三维点；
- 现有公共 `classifyTriangleContact(...)` 只返回详细结果中的 `kind`，保持 API 兼容。

- [ ] **Step 4: 从完整边界面建立允许特征**

比较完整 `boundary_vertex_keys`：

- 无共享 key：`None`；
- 一个共享 key：`Point`；
- 两个 key 且在两边界面循环中均相邻：`Segment`；
- 三个 key 且两边界面都是同一 Triangle，或四个 key 且两边界面是同一 Quad：`Face`；
- 两个非相邻共享 key：两个独立允许点，不构造对角线。

允许面使用规范 key 顺序三角化，不能依赖两输入面的起点或绕序。

- [ ] **Step 5: 判断交集是否完全包含**

严格零判断：

- 点：交集证据只能等于允许点；
- 线段：每个证据点必须与线段共线且位于闭区间；
- 面：每个证据点必须位于规范允许 Triangle 或 Quad 的两个三角形之一；
- `Disjoint` 总是合法；无允许特征的任何非分离接触均非法。

共面裁剪多边形除顶点外还检查每条边中点，防止多边形跨出非平面或分片允许区域。

- [ ] **Step 6: 切换 CollisionIndex 到新重载并运行 GREEN**

```powershell
cmake --build build --config Debug `
  --target boundary_mesh_spatial_triangle_contact_test `
           boundary_mesh_spatial_collision_index_test
ctest --test-dir build -C Debug `
  -R "boundary_mesh_spatial_(triangle_contact|collision_index)_test" `
  --output-on-failure
```

- [ ] **Step 7: 提交公共特征判定**

```powershell
git add include/boundary_mesh/spatial/triangle_contact.hpp `
  src/spatial/triangle_contact.cpp `
  src/spatial/collision_index.cpp `
  tests/unit/spatial/triangle_contact_test.cpp `
  tests/unit/spatial/collision_index_test.cpp
git diff --cached --check
git commit -m "fix: allow contacts within shared topology"
```

---

### Task 3: LayerCollisionChecker 行为回归

**Files:**
- Modify: `tests/unit/growth/layer_collision_checker_test.cpp`
- Modify: `tests/unit/growth/collision_growth_pipeline_test.cpp`
- Modify: `src/growth/layer_collision_checker.cpp` only if candidate-pair filtering needs correction

**Interfaces:**
- Consumes: Task 1–2 的完整边界来源与拓扑感知 `CollisionIndex`。
- Produces: 连续前沿合法邻接不停止，公共区域外真实碰撞仍停止双方。

- [ ] **Step 1: 写混合相邻候选 RED 测试**

在 `layer_collision_checker_test.cpp` 构造同一连续前沿上的：

- 90 度折角 Triangle/Triangle；
- Triangle/Quad 公共源边；
- 两个只共享源顶点的候选；
- 相反面起点和绕序形成不同 Quad 对角线的公共侧面。

为共享源顶点复用相同 `source_vertex_id`，下一层复用相同顶点，断言：

```cpp
const auto result = LayerCollisionChecker{}.filterSelfCollisions(
    current_front,
    quality_step);
assert(result.hasValue());
assert(result.value().next_front.faces.size() == expected_face_count);
assert(result.value().stopped_faces.empty());
```

- [ ] **Step 2: 运行 RED**

```powershell
cmake --build build --config Debug `
  --target boundary_mesh_layer_collision_checker_test
ctest --test-dir build -C Debug `
  -R boundary_mesh_layer_collision_checker_test `
  --output-on-failure --timeout 10
```

Expected: 当前实现至少一个合法邻接场景被压缩为 Collision。

- [ ] **Step 3: 写越界真实碰撞 RED 测试**

保持相同源拓扑关系，但移动一个非共享顶点，使顶面或外侧面穿过公共侧面区域。
断言两个源面均产生唯一 `FaceStopReason::Collision`。再交换两个源面在 Front 中的
顺序，断言停止的 `source_face_id` 集合相同。

- [ ] **Step 4: 最小修正候选对过滤**

`filterSelfCollisions(...)` 不再使用只接受“两三角形全部属于共享侧面”的
`legalSharedSideContact(...)` 作为最终判断。由 `CollisionIndex` 的完整边界面
公共特征判定决定合法性；候选 owner 相同仍直接跳过，owner 不同的非法命中仍
同时停止双方。

- [ ] **Step 5: 验证候选、原始障碍和历史边界**

```powershell
cmake --build build --config Debug `
  --target boundary_mesh_layer_collision_checker_test `
           boundary_mesh_collision_growth_pipeline_test `
           boundary_mesh_collision_growth_failure_test
ctest --test-dir build -C Debug `
  -R "boundary_mesh_(layer_collision_checker|collision_growth)_" `
  --output-on-failure
```

- [ ] **Step 6: 提交生长接入**

```powershell
git add src/growth/layer_collision_checker.cpp `
  tests/unit/growth/layer_collision_checker_test.cpp `
  tests/unit/growth/collision_growth_pipeline_test.cpp
git diff --cached --check
git commit -m "fix: preserve adjacent layer candidates"
```

---

### Task 4: CLI 停止原因和真实案例验收

**Files:**
- Modify: `src/cli/boundary_mesh_command.cpp`
- Modify: `tests/integration/cgns_cli_pipeline_test.cpp`
- Modify: `benchmarks/cgns_pipeline_benchmark.cpp`
- Modify: `docs/design/modules/collision-local-stop.md`
- Modify: `docs/plans/06-topology-aware-collision-contact-fix.md`

**Interfaces:**
- Consumes: `RegularLayerGrowthResult::faces`。
- Produces: 稳定的逐 `FaceStopReason` 汇总和修复后的 `2dot5_cf` 真实一层结果。

- [ ] **Step 1: 写 CLI 统计 RED 测试**

在小型 CGNS CLI 流水线断言 stdout 包含固定字段：

```cpp
assert(output.str().find("stop_vertex_layer_limit=") != std::string::npos);
assert(output.str().find("stop_degenerate_candidate=") != std::string::npos);
assert(output.str().find("stop_reversed_candidate=") != std::string::npos);
assert(output.str().find("stop_locally_inverted_candidate=") != std::string::npos);
assert(output.str().find("stop_skewness_exceeded=") != std::string::npos);
assert(output.str().find("stop_collision=") != std::string::npos);
assert(output.str().find("stop_neighbor_layer_constraint=") != std::string::npos);
```

- [ ] **Step 2: 运行 RED**

```powershell
cmake --build build --config Debug `
  --target boundary_mesh_cgns_cli_pipeline_test
ctest --test-dir build -C Debug `
  -R boundary_mesh_cgns_cli_pipeline_test `
  --output-on-failure
```

Expected: 断言失败，因为 CLI 当前只输出总单元数和两个配置值。

- [ ] **Step 3: 实现确定性停止原因汇总**

在 CLI 内按枚举值计数 `growth.value().faces`，按上述固定字段顺序输出。`None`
单独输出为 `stop_none`，便于发现仍处于 Active 的异常结果；不逐面刷屏。

benchmark 使用相同字段名输出逐原因计数，避免再次添加临时诊断代码。

- [ ] **Step 4: 运行小型 GREEN 和 Debug 全回归**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

- [ ] **Step 5: 运行 `2dot5_cf` Release 一层验收**

```powershell
cmake --build build --config Release `
  --target boundary_mesh_cgns_pipeline_benchmark
& .\build\Release\boundary_mesh_cgns_pipeline_benchmark.exe `
  "C:\Users\zpern\Desktop\todo\九院项目质量对标\test_case\2dot5_cf\2dot5_cf.cgns" `
  ".\build\real_case\topology_contact_fix"
```

验收要求：

- 退出码 0；
- 两个 VTK 非空；
- 峰值工作集不超过 1 GiB；
- `stop_collision` 相比修复前 54,885 明显下降；
- `volume_cells` 相比修复前 3,252 明显增加；
- 若仍有大量 Collision，使用正式原因统计继续区分障碍和同层来源，不放宽真实穿插规则。

- [ ] **Step 6: Release 和 IO-OFF 回归**

```powershell
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
cmake -S . -B build-no-io -DBOUNDARY_MESH_ENABLE_CGNS_IO=OFF
cmake --build build-no-io --config Debug `
  --target boundary_mesh_boundary_layer
```

- [ ] **Step 7: 更新结果、勾选计划并提交**

将真实案例修复后计数、耗时和峰值工作集写入碰撞设计第 15 节，将本计划全部
检查项改为 `[x]`，然后：

```powershell
git add src/cli/boundary_mesh_command.cpp `
  tests/integration/cgns_cli_pipeline_test.cpp `
  benchmarks/cgns_pipeline_benchmark.cpp `
  docs/design/modules/collision-local-stop.md `
  docs/plans/06-topology-aware-collision-contact-fix.md
git diff --cached --check
git commit -m "test: verify topology-aware collision contacts"
git status --short --branch
```

Expected: 仅 `.superpowers/` 保持未跟踪。

---

## Final Verification

```text
完整边界面元数据覆盖原始面、候选顶/侧面和历史外露面
没有共享拓扑 key 的任何接触仍非法
公共点、公共边和公共侧面内的接触合法
交集超出公共特征时仍停止
坐标相同但拓扑 key 不同仍停止
Triangle/Triangle、Triangle/Quad、Quad/Quad 均有覆盖
同层非法碰撞双方同时停止且顺序不变
CLI 和 benchmark 输出逐停止原因统计
2dot5_cf 一层单元数明显高于 3,252
2dot5_cf Collision 明显低于 54,885
Debug/Release 全量 CTest 为 0 failed
IO-OFF 核心构建通过
阶段 08、09 仍未实施
```
