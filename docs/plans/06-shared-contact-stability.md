# Shared Collision Contact Stability Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 消除合法共享点、共享边和共享面的浮点误碰撞，同时继续拒绝公共拓扑特征之外的真实穿插。

**Architecture:** 保留 AABB、TiGER 三角形初筛及无拓扑关联对象的现有严格检测。仅在两个碰撞三角形具有一致 `CollisionVertexKey` 时，使用原始三角形边而不是重建交点判断非共享部分是否穿插；一个局部共享 key 时检查双方的非共享对边，两个共享 key 和完整共享面继续使用现有拓扑规则。

**Tech Stack:** C++17、Eigen、TiGER `lin_tri_intersect3d`、CMake、CTest、MSVC

## Global Constraints

- 不增加用户可配置碰撞容差。
- 不对拓扑相邻候选无条件跳过。
- 坐标相同但 `CollisionVertexKey` 不同的接触仍然非法。
- 没有共享拓扑 key 的检测行为不变。
- 非有限坐标和退化三角形仍按现有 `SpatialError` 处理。
- 不修改 Growth、质量、层数协调和输出公共接口。
- 不提交 `.superpowers/`。

---

### Task 1: 用小步长非轴对齐接触复现误判

**Files:**
- Modify: `tests/unit/spatial/triangle_contact_test.cpp`

**Interfaces:**
- Consumes: `hasIllegalTriangleContact(const CollisionTriangle &, const CollisionTriangle &)`
- Produces: 合法局部单 key 接触和非共享对边穿插的回归断言

- [x] **Step 1: 增加合法邻接 RED 用例**

构造两个完整边界面共享 `CollisionVertexKey{50, 1}` 与 `{51, 1}`，但当前两个拆分三角形只共同包含 `{50, 1}`。使用非轴对齐大坐标和 `0.001` 层高，使两个三角形只在允许公共特征内接触：

```cpp
const Point3 shared_a{1000000.1, 2000000.2, 3000000.3};
const Point3 shared_b{1000001.1, 2000000.4, 3000000.7};
const Point3 top_other{1000000.4, 2000001.3, 3000000.9};
const Point3 side_lower{1000000.0999, 2000000.1998, 2999999.9991};

const CollisionTriangle stable_top = makeCollisionTriangle(
    {{shared_a, shared_b, top_other}},
    {{{50, 1}, {51, 1}, {52, 1}}},
    {{shared_a, shared_b, top_other, Point3{}}},
    {{{50, 1}, {51, 1}, {52, 1}, {}}},
    3);

const CollisionTriangle stable_side_split = makeCollisionTriangle(
    {{shared_a, side_lower, top_other}},
    {{{50, 1}, {50, 0}, {53, 0}}},
    {{shared_a, shared_b, side_lower, top_other}},
    {{{50, 1}, {51, 1}, {50, 0}, {53, 0}}},
    4);

assert(!hasIllegalTriangleContact(stable_top, stable_side_split).value());
```

若该几何被 TiGER 判为分离而不能触发旧错误，只允许调整 `side_lower` 和 `top_other` 的非共享坐标，保持完整边界共享 key、局部单共享 key 和 `0.001` 尺度不变，直到当前实现稳定返回非法。

实际执行时，初始合成坐标没有触发误判，因此从 `2dot5_cf` 捕获完整 double 精度的合法候选侧面—源面共享顶点夹具（源 key 470/471/472），当前实现稳定返回非法。

- [x] **Step 2: 增加真实越界保护用例**

复制合法用例，移动 `side_lower` 使其非共享对边穿过 `stable_top` 内部：

```cpp
CollisionTriangle stable_side_cross = stable_side_split;
stable_side_cross.points[1] =
    Scalar{0.5} * (shared_b + top_other) - Vector3{0.0, 0.0, 1.0};
stable_side_cross.points[2] =
    Scalar{0.5} * (shared_b + top_other) + Vector3{0.0, 0.0, 1.0};
assert(hasIllegalTriangleContact(stable_top, stable_side_cross).value());
```

实际执行复用了同文件既有 `shared_vertex_cross` 非共享对边穿插用例；它在修复前后均返回非法。

- [x] **Step 3: 运行 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_spatial_triangle_contact_test
ctest --test-dir build -C Debug -R "^boundary_mesh_spatial_triangle_contact_test$" --output-on-failure
```

预期：构建成功；合法邻接断言失败，而真实越界断言保持通过。

### Task 2: 使用非共享对边判断局部单 key 穿插

**Files:**
- Modify: `src/spatial/triangle_contact.cpp`
- Test: `tests/unit/spatial/triangle_contact_test.cpp`

**Interfaces:**
- Consumes: 两个 `CollisionTriangle`、局部共享 key 的双方顶点索引
- Produces: `oppositeEdgeIntersectsTriangle(...)` 私有辅助函数和稳定的合法接触判断

- [x] **Step 1: 扩展局部共享 key 结果**

将私有 `LocalSharedKeys` 扩展为：

```cpp
struct LocalSharedKeys
{
    std::size_t count{};
    std::array<std::size_t, 3> first_indices{};
    std::array<std::size_t, 3> second_indices{};
};
```

`localSharedKeys(...)` 每发现一个相同 key，就保存双方局部索引后增加 `count`。

- [x] **Step 2: 增加非共享对边检测**

在 `triangle_contact.cpp` 私有命名空间内增加：

```cpp
bool oppositeEdgeIntersectsTriangle(
    const CollisionTriangle &edge_source,
    std::size_t shared_vertex,
    const CollisionTriangle &target)
{
    double line[2][3];
    double face[3][3];
    const std::size_t first = (shared_vertex + 1) % 3;
    const std::size_t second = (shared_vertex + 2) % 3;
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        line[0][axis] = edge_source.points[first][axis];
        line[1][axis] = edge_source.points[second][axis];
        for (std::size_t corner = 0; corner < 3; ++corner)
        {
            face[corner][axis] = target.points[corner][axis];
        }
    }
    int intersection_type{};
    int intersection_code{};
    double intersection_point[3]{};
    bool epsilon = false;
    return TiGER_GEOM_FUNC::lin_tri_intersect3d(
        line,
        face,
        &intersection_type,
        &intersection_code,
        intersection_point,
        epsilon) != 0;
}
```

- [x] **Step 3: 替换局部单 key 的重建交点判断**

在 `hasIllegalTriangleContact(const CollisionTriangle &, ...)` 中保留 `Disjoint`、完整共享面和双 key 公共边分支。对 `shared.count == 1 && feature.kind != SharedFeatureKind::None` 改为：

```cpp
return Result<bool, SpatialError>::success(
    oppositeEdgeIntersectsTriangle(
        first, shared.first_indices[0], second) ||
    oppositeEdgeIntersectsTriangle(
        second, shared.second_indices[0], first));
```

只有非共享对边进入另一个三角形才判为非法；公共点或公共边内部的接触不依赖重建交点的严格共线判断。

- [x] **Step 4: 运行目标 GREEN**

```powershell
cmake --build build --config Debug --target boundary_mesh_spatial_triangle_contact_test boundary_mesh_spatial_collision_index_test boundary_mesh_layer_collision_checker_test
ctest --test-dir build -C Debug -R "boundary_mesh_(spatial_(triangle_contact|collision_index)|layer_collision_checker)_test" --output-on-failure
```

预期：三项专项测试全部通过。

- [x] **Step 5: 提交稳定接触实现**

```powershell
git add src/spatial/triangle_contact.cpp tests/unit/spatial/triangle_contact_test.cpp docs/plans/06-shared-contact-stability.md
git diff --cached --check
git commit -m "fix: stabilize shared collision contacts"
```

### Task 3: 全量回归和 2dot5 真实案例验收

**Files:**
- Input: `C:/Users/zpern/Desktop/todo/九院项目质量对标/test_case/2dot5_cf/2dot5_cf.cgns`
- Input: `C:/Users/zpern/Desktop/todo/九院项目质量对标/test_case/2dot5_cf/2dot5_cf.bc.txt`
- Output: `build/real_case/2dot5_cf_shared_contact_*`

**Interfaces:**
- Consumes: 修正后的 Release CLI
- Produces: 一层双高度对照、十层单元统计和 VTK 完整性证据

- [x] **Step 1: 运行 Debug 和 Release 全量回归**

```powershell
cmake --build build --config Debug -- /m:1
ctest --test-dir build -C Debug --output-on-failure
cmake --build build --config Release -- /m:1
ctest --test-dir build -C Release --output-on-failure
```

预期：两个配置均为 47/47 通过。

- [x] **Step 2: 运行首层双高度对照**

分别以 `--first-height 0.1` 和 `--first-height 0.001`、`--layer-count 1` 运行 CLI。记录 `volume_cells`、`stop_collision` 及其他停止原因。

验收：高度缩小 100 倍后，`stop_collision` 不得从 17,705 上升到 39,705；两个高度下的合法邻接误停均应显著低于修复前。

- [x] **Step 3: 运行十层案例**

使用 `first_height=0.1`、`growth_ratio=1.0`、`layer_count=10`、`maximum_skewness=0.95`、`max_neighbor_layer_difference=1`。记录每层新增体单元、总 `volume_cells`、`farfield_faces` 和停止原因。

- [x] **Step 4: 检查 VTK**

确认边界层与远场 VTK 均包含 `POINTS`、`CELLS`、`CELL_TYPES`，且数值中没有 NaN/Inf。

- [x] **Step 5: 更新设计实测结果并提交**

将修复后双高度和十层实测结果追加到 `docs/design/modules/collision-shared-contact-stability.md`，然后：

```powershell
git add docs/design/modules/collision-shared-contact-stability.md
git diff --cached --check
git commit -m "docs: record shared contact validation"
```
