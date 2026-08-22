# Collision-Aware Local Stopping Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在规则层候选提交前检测原始表面、历史外露边界和同层候选碰撞，并以确定性的逐源面停止替代非法 Prism/Hexa 提交。

**Architecture:** 新增独立 `BoundaryMesh::Spatial`，整理 HexaMesh 的 AABB 树思路并通过适配层复用 `tiger_geom`。`RegularLayerStepper` 保持无空间状态；`RegularLayerGenerator` 在质量检查后调用 `LayerCollisionChecker`，再原子提交存活候选和增量外露边界。

**Tech Stack:** C++17、CMake 3.20+、Eigen、`tiger_geom`、CTest、MSVC Debug/Release。

## Global Constraints

- 执行时只允许主代理内联完成，不使用子代理。
- 三个阶段 06、07、10 的计划全部确认以前，不执行本计划。
- 不兼容旧 `generateRegularLayers` 入口，直接改为显式接收 `SurfaceMesh` 和 `SurfaceTopology`。
- Growth、Surface 和 Quality 不得包含 `geom_func.h`；只有 Spatial 实现可以调用 `tiger_geom`。
- 不引入 libigl、最近点、最大安全步长、自动缩短重试或体包含检测。
- Symmetry 不进入阶段 06 的原始碰撞树。
- 碰撞不使用用户容差；AABB 为闭区间，非合法零距离接触算碰撞。
- 新公共字段和枚举值必须带中文 `//` 注释。
- `.superpowers/` 永远不加入提交。
- 每个任务必须先观察 RED，再完成 GREEN、目标测试、全量回归和独立提交。

---

## File Structure

计划新增或修改的文件职责如下：

```text
third/geom/                                      固定版本 geom 子模块
cmake/Dependencies.cmake                         独立构建或 TiGER 复用 tiger_geom
CMakeLists.txt                                   BoundaryMesh::Spatial 及 Growth 依赖

include/boundary_mesh/spatial/aabb.hpp           AABB 和闭区间重叠
include/boundary_mesh/spatial/spatial_error.hpp  空间模块程序级错误
include/boundary_mesh/spatial/binary_aabb_tree.hpp 只读共享 AABB 树
include/boundary_mesh/spatial/triangle_contact.hpp tiger_geom 接触分类接口
include/boundary_mesh/spatial/collision_index.hpp 碰撞图元、顶点键和查询索引
src/spatial/binary_aabb_tree.cpp                  批量建树和查询
src/spatial/triangle_contact.cpp                  geom 适配及合法共享特征判断
src/spatial/collision_index.cpp                   原始表面三角形化和空间查询

include/boundary_mesh/growth/exposed_boundary.hpp 增量外露面集合
src/growth/exposed_boundary.cpp                   面抵消和更新事务
include/boundary_mesh/growth/farfield_boundary_builder.hpp 远场边界物化接口
src/growth/farfield_boundary_builder.cpp           合并 Farfield 与边界层接口
include/boundary_mesh/growth/layer_collision_checker.hpp 整层碰撞过滤接口
src/growth/layer_collision_checker.cpp            三阶段确定性过滤

include/boundary_mesh/growth/regular_layer_growth.hpp       Collision 停止原因
include/boundary_mesh/growth/regular_layer_growth_error.hpp 空间程序错误包装
include/boundary_mesh/growth/regular_layer_generator.hpp    新生成入口
src/growth/regular_layer_generator.cpp                      碰撞前置和原子提交

tests/unit/spatial/aabb_test.cpp                 AABB 语义
tests/unit/spatial/binary_aabb_tree_test.cpp     树查询与确定性
tests/unit/spatial/triangle_contact_test.cpp     接触分类和拓扑放行
tests/unit/spatial/collision_index_test.cpp      Wall/Farfield/Symmetry 索引
tests/unit/growth/exposed_boundary_test.cpp      外露面增量维护
tests/unit/growth/farfield_boundary_builder_test.cpp 远场边界标签、绕序和紧凑编号
tests/unit/growth/layer_collision_checker_test.cpp 三类碰撞过滤
tests/integration/collision_growth_pipeline_test.cpp 生成器事务和混合单元
tests/integration/collision_growth_failure_test.cpp 程序错误零提交
tests/CMakeLists.txt                             注册阶段 06 测试
docs/design/roadmap.md                           实施完成状态
docs/design/modules/collision-local-stop.md      与最终接口同步
```

---

### Task 1: tiger_geom 依赖、Spatial Target 与 AABB

**Files:**
- Modify: `.gitmodules`
- Add submodule: `third/geom`
- Modify: `cmake/Dependencies.cmake`
- Modify: `CMakeLists.txt`
- Create: `include/boundary_mesh/spatial/aabb.hpp`
- Create: `include/boundary_mesh/spatial/spatial_error.hpp`
- Test: `tests/unit/spatial/aabb_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `boundary_mesh::Point3`、父工程可选目标 `tiger_geom`。
- Produces: `Aabb makeAabb(...)`、`bool overlaps(const Aabb &, const Aabb &)`、`BoundaryMesh::Spatial`。

- [ ] **Step 1: 添加失败的 AABB 测试和测试目标**

测试必须覆盖分离、体重叠、面相切、边相切和点相切：

```cpp
#include <cassert>
#include <boundary_mesh/spatial/aabb.hpp>

using namespace boundary_mesh;

int main()
{
    const Aabb unit{Point3{0, 0, 0}, Point3{1, 1, 1}};
    assert(!overlaps(unit, Aabb{Point3{2, 0, 0}, Point3{3, 1, 1}}));
    assert(overlaps(unit, Aabb{Point3{0.5, 0.5, 0.5}, Point3{2, 2, 2}}));
    assert(overlaps(unit, Aabb{Point3{1, 0, 0}, Point3{2, 1, 1}}));
    assert(overlaps(unit, Aabb{Point3{1, 1, 0}, Point3{2, 2, 1}}));
    assert(overlaps(unit, Aabb{Point3{1, 1, 1}, Point3{2, 2, 2}}));
}
```

在 `tests/CMakeLists.txt` 注册 `boundary_mesh_spatial_aabb_test` 并链接 `BoundaryMesh::Spatial`。

- [ ] **Step 2: 配置并验证 RED**

Run:

```powershell
cmake -S . -B build
cmake --build build --config Debug --target boundary_mesh_spatial_aabb_test
```

Expected: 配置或编译失败，指出 `BoundaryMesh::Spatial` 或 `boundary_mesh/spatial/aabb.hpp` 不存在。

- [ ] **Step 3: 固定 geom 子模块并配置依赖复用**

Run:

```powershell
git submodule add -b main https://github.com/mySharedsource/geom.git third/geom
```

在 `cmake/Dependencies.cmake` 增加：

```cmake
if(NOT TARGET tiger_geom)
    if(EXISTS "${TIGER_DEPENDENCIES_DIR}/geom/CMakeLists.txt")
        add_subdirectory(
            "${TIGER_DEPENDENCIES_DIR}/geom"
            "${CMAKE_BINARY_DIR}/boundary_mesh_geom"
        )
    else()
        message(FATAL_ERROR
            "tiger_geom is unavailable and third/geom is missing")
    endif()
endif()
```

在根 `CMakeLists.txt` 创建 `boundary_mesh_spatial`，公开链接 `BoundaryMesh::Core`，私有链接 `tiger_geom`，并建立别名 `BoundaryMesh::Spatial`。随后让 `boundary_mesh_boundary_layer` 公开链接 `BoundaryMesh::Spatial`。

由于当前 `geom/CMakeLists.txt` 没有发布 include 目录，`boundary_mesh_spatial` 还要私有加入 `${TIGER_DEPENDENCIES_DIR}/geom`；该路径不得通过 `PUBLIC` 或 `INTERFACE` 泄漏给消费者。

- [ ] **Step 4: 实现最小 AABB 类型**

`aabb.hpp` 定义：

```cpp
struct Aabb
{
    Point3 minimum{}; // 三个坐标分量的下界
    Point3 maximum{}; // 三个坐标分量的上界
};

inline bool overlaps(const Aabb &left, const Aabb &right) noexcept
{
    return left.minimum.x() <= right.maximum.x() &&
           left.maximum.x() >= right.minimum.x() &&
           left.minimum.y() <= right.maximum.y() &&
           left.maximum.y() >= right.minimum.y() &&
           left.minimum.z() <= right.maximum.z() &&
           left.maximum.z() >= right.minimum.z();
}
```

同时提供从三个或四个有限坐标构造包围盒的重载。非有限坐标返回 `SpatialError::NonFiniteCoordinate`，最小值大于最大值返回 `SpatialError::InvalidAabb`。

`spatial_error.hpp` 的完整初始定义为：

```cpp
enum class SpatialError
{
    NonFiniteCoordinate,       // 碰撞图元包含 NaN 或无穷坐标
    DegenerateTriangle,        // 碰撞三角形面积严格等于零
    InvalidAabb,               // 包围盒下界大于上界
    InvalidTopologyReference,  // 碰撞图元引用的顶点或面不存在
    PrimitiveIdOverflow        // 空间图元数量无法由内部编号表达
};
```

- [ ] **Step 5: 运行目标测试和全量回归**

Run:

```powershell
cmake -S . -B build
cmake --build build --config Debug --target boundary_mesh_spatial_aabb_test
ctest --test-dir build -C Debug --output-on-failure
```

Expected: 新测试通过，全部既有测试通过。

- [ ] **Step 6: 提交依赖和 AABB 基础**

```powershell
git add .gitmodules third/geom cmake/Dependencies.cmake CMakeLists.txt include/boundary_mesh/spatial tests/CMakeLists.txt tests/unit/spatial/aabb_test.cpp
git diff --cached --check
git commit -m "build: add spatial geometry foundation"
```

---

### Task 2: 共享只读 BinaryAabbTree

**Files:**
- Create: `include/boundary_mesh/spatial/binary_aabb_tree.hpp`
- Create: `src/spatial/binary_aabb_tree.cpp`
- Modify: `CMakeLists.txt`
- Test: `tests/unit/spatial/binary_aabb_tree_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `std::vector<Aabb>`。
- Produces: `Result<BinaryAabbTree, SpatialError> BinaryAabbTree::build(...)` 和排序去重后的 `query(...)`。

- [ ] **Step 1: 写批量建树和查询的失败测试**

```cpp
const std::vector<Aabb> boxes{
    {{0, 0, 0}, {1, 1, 1}},
    {{3, 0, 0}, {4, 1, 1}},
    {{1, 1, 1}, {2, 2, 2}}};

const auto tree = BinaryAabbTree::build(boxes);
assert(tree.hasValue());
assert((tree.value().query({{0.5, 0.5, 0.5}, {1, 1, 1}}) ==
        std::vector<std::size_t>{0, 2}));
assert(tree.value().query({{10, 10, 10}, {11, 11, 11}}).empty());
```

增加输入顺序反转测试，查询结果仍按 primitive ID 升序返回。

- [ ] **Step 2: 运行 RED**

Run:

```powershell
cmake --build build --config Debug --target boundary_mesh_spatial_binary_aabb_tree_test
```

Expected: FAIL，`binary_aabb_tree.hpp` 不存在。

- [ ] **Step 3: 从 HexaMesh 思路整理紧凑树实现**

公共类只暴露：

```cpp
class BinaryAabbTree
{
public:
    static Result<BinaryAabbTree, SpatialError>
    build(const std::vector<Aabb> &primitive_bounds);

    std::vector<std::size_t>
    query(const Aabb &bounds) const;

private:
    struct Node
    {
        Aabb bounds{}; // 当前节点覆盖的空间范围
        std::size_t first{}; // 叶节点 primitive 区间起点
        std::size_t count{}; // 叶节点 primitive 数量
        std::size_t left{}; // 左子节点编号
        std::size_t right{}; // 右子节点编号
        bool leaf{}; // 当前节点是否为叶节点
    };
};
```

批量构建时沿包围盒中心跨度最大的轴稳定二分；叶节点容量固定为 16。查询使用显式栈，不递归分配；输出最后排序并去重。空输入产生合法空树。

- [ ] **Step 4: 运行目标测试**

Run:

```powershell
cmake --build build --config Debug --target boundary_mesh_spatial_binary_aabb_tree_test
ctest --test-dir build -C Debug -R boundary_mesh_spatial_binary_aabb_tree_test --output-on-failure
```

Expected: PASS。

- [ ] **Step 5: 加入边界和规模正确性用例**

追加 4096 个规则小盒，逐个查询并与线性扫描结果比较；测试树根边界、空树、全部盒重合和 primitive 数超过 `uint32_t` 可表达范围时的错误路径（通过内部上限注入测试，不实际分配超大数组）。

- [ ] **Step 6: 完整回归并提交**

```powershell
ctest --test-dir build -C Debug --output-on-failure
git add CMakeLists.txt include/boundary_mesh/spatial/binary_aabb_tree.hpp src/spatial/binary_aabb_tree.cpp tests/CMakeLists.txt tests/unit/spatial/binary_aabb_tree_test.cpp
git diff --cached --check
git commit -m "feat: add shared binary AABB tree"
```

---

### Task 3: tiger_geom 三角形接触适配

**Files:**
- Create: `include/boundary_mesh/spatial/triangle_contact.hpp`
- Create: `src/spatial/triangle_contact.cpp`
- Modify: `CMakeLists.txt`
- Test: `tests/unit/spatial/triangle_contact_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: 两组三角形坐标和对应 `CollisionVertexKey`。
- Produces: `classifyTriangleContact(...)` 与 `hasIllegalTriangleContact(...)`。

- [ ] **Step 1: 写完整接触矩阵的失败测试**

测试数据必须明确覆盖：分离、非共面穿透、非拓扑共点、非拓扑共边、共面部分重叠、完全重合、合法共享点、合法共享边、合法共享三角形，以及共享边以外额外交叉。

核心断言形式：

```cpp
assert(classifyTriangleContact(disjoint_a, disjoint_b).value() ==
       TriangleContactKind::Disjoint);
assert(classifyTriangleContact(crossing_a, crossing_b).value() ==
       TriangleContactKind::ProperIntersect);
const auto touching = hasIllegalTriangleContact(
    touching_a, touching_a_keys, touching_b, touching_b_keys);
const auto legal_edge = hasIllegalTriangleContact(
    legal_edge_a, legal_edge_a_keys, legal_edge_b, legal_edge_b_keys);
const auto extra_cross = hasIllegalTriangleContact(
    extra_cross_a, extra_cross_a_keys, extra_cross_b, extra_cross_b_keys);
assert(touching.hasValue() && touching.value());
assert(legal_edge.hasValue() && !legal_edge.value());
assert(extra_cross.hasValue() && extra_cross.value());
```

- [ ] **Step 2: 运行 RED**

Run:

```powershell
cmake --build build --config Debug --target boundary_mesh_spatial_triangle_contact_test
```

Expected: FAIL，缺少 `triangle_contact.hpp`。

- [ ] **Step 3: 定义带中文注释的接触类型**

```cpp
enum class TriangleContactKind
{
    Disjoint,         // 两个三角形完全分离
    VertexTouch,      // 仅在一个点发生零距离接触
    EdgeTouch,        // 沿边或边的一部分发生接触
    CoplanarOverlap,  // 共面区域存在正面积重叠
    ProperIntersect   // 非共面穿透或交叉
};
```

接触接口使用以下完整类型：

```cpp
using TrianglePoints = std::array<Point3, 3>; // 三角形的三个空间坐标

struct CollisionVertexKey
{
    VertexId source_vertex_id{}; // 对应的输入表面顶点编号
    std::uint32_t layer{};        // 该顶点所属的边界层层号
};

Result<TriangleContactKind, SpatialError>
classifyTriangleContact(
    const TrianglePoints &first,
    const TrianglePoints &second);

Result<bool, SpatialError>
hasIllegalTriangleContact(
    const TrianglePoints &first,
    const std::array<CollisionVertexKey, 3> &first_keys,
    const TrianglePoints &second,
    const std::array<CollisionVertexKey, 3> &second_keys);
```

只有键的 `source_vertex_id` 和 `layer` 都相同，才表示拓扑上同一个分层顶点。为保持 C++17，键比较必须显式比较两个字段，不使用 C++20 默认比较。

- [ ] **Step 4: 实现 geom 适配和额外交叉判断**

`triangle_contact.cpp` 是唯一包含 `geom_func.h` 的文件。先调用 `tri_tri_overlap_test_3d` 排除分离情况，再使用 `orient2d/orient3d`、主平面投影和共享键集合分类接触。

规则必须写成显式分支：

```text
无共享键：任何非 Disjoint 都非法
共享一个键：只允许交集恰好等于该顶点
共享两个键：只允许交集恰好等于该共享边
共享三个键：只允许同一拓扑三角形的完全重合
```

不能沿用旧 `same_count == 1` 的不完整实现。函数不打印、不使用 epsilon、不抛出业务异常。

- [ ] **Step 5: 运行目标测试和顺序置换测试**

对每组用例交换三角形顺序、反转顶点绕序并循环置换顶点；接触类型和非法判定必须保持一致。

Run:

```powershell
cmake --build build --config Debug --target boundary_mesh_spatial_triangle_contact_test
ctest --test-dir build -C Debug -R boundary_mesh_spatial_triangle_contact_test --output-on-failure
```

Expected: PASS。

- [ ] **Step 6: 回归并提交**

```powershell
ctest --test-dir build -C Debug --output-on-failure
git add CMakeLists.txt include/boundary_mesh/spatial/triangle_contact.hpp src/spatial/triangle_contact.cpp tests/CMakeLists.txt tests/unit/spatial/triangle_contact_test.cpp
git diff --cached --check
git commit -m "feat: classify triangle collision contacts"
```

---

### Task 4: CollisionIndex 与原始表面过滤

**Files:**
- Create: `include/boundary_mesh/spatial/collision_index.hpp`
- Create: `src/spatial/collision_index.cpp`
- Modify: `CMakeLists.txt`
- Test: `tests/unit/spatial/collision_index_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `SurfaceMesh`、`SurfaceTopology` 或任意 `std::vector<CollisionTriangle>`。
- Produces: `CollisionIndex::build(...)`、`queryIllegalContacts(...)`、`buildOriginalSurfaceCollisionIndex(...)`。

- [ ] **Step 1: 写原始边界选择和 Quad 拆分失败测试**

构造含一个 Wall Quad、一个 Farfield Quad 和一个 Symmetry Quad 的网格，断言：

```cpp
const auto index = buildOriginalSurfaceCollisionIndex(mesh, topology);
assert(index.hasValue());
assert(index.value().primitiveCount() == 4); // Wall 1 个三角形 + Farfield 1 个 + Symmetry 0
assert(index.value().queryIllegalContacts(wall_hit).size() == 1);
assert(index.value().queryIllegalContacts(symmetry_only_hit).empty());
```

另建 Quad 用例，断言固定产生 `(v0,v1,v2)` 和 `(v0,v2,v3)`。

- [ ] **Step 2: 运行 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_spatial_collision_index_test
```

Expected: FAIL，缺少 `collision_index.hpp`。

- [ ] **Step 3: 实现图元批量索引**

`collision_index.hpp` 先定义完整图元归属：

```cpp
enum class CollisionOwnerKind
{
    OriginalSurface,  // 输入 Wall 或 Farfield 面
    ExposedBoundary,  // 已提交边界层的外露面
    LayerCandidate    // 当前目标层的候选单元边界面
};

struct CollisionTriangle
{
    TrianglePoints points{}; // 三角形空间坐标
    std::array<CollisionVertexKey, 3> vertex_keys{}; // 分层拓扑顶点键
    CollisionOwnerKind owner_kind{CollisionOwnerKind::OriginalSurface}; // 图元归属类别
    std::uint32_t owner_id{}; // 所属源面、外露面或候选单元编号
};
```

```cpp
class CollisionIndex
{
public:
    static Result<CollisionIndex, SpatialError>
    build(std::vector<CollisionTriangle> triangles);

    std::vector<std::size_t>
    queryIllegalContacts(const CollisionTriangle &query) const;

    std::size_t primitiveCount() const noexcept;
};
```

查询先使用 `BinaryAabbTree` 粗筛，再调用 `hasIllegalTriangleContact`。输出 primitive 下标排序去重。

- [ ] **Step 4: 实现原始表面索引构建**

验证 `faces.size() == face_tags.size()` 和拓扑数组尺寸。只插入 `Wall`、`Farfield`；跳过 `Symmetry`。原始顶点键固定为 `{source_vertex_id, 0}`，owner 保存原始 `SurfaceFaceId` 供内部过滤。

- [ ] **Step 5: 运行测试并覆盖非法输入**

增加缺失标签、越界顶点、非有限坐标和退化原始面测试，预期返回 `SpatialError`，而不是空索引。

```powershell
cmake --build build --config Debug --target boundary_mesh_spatial_collision_index_test
ctest --test-dir build -C Debug -R boundary_mesh_spatial_collision_index_test --output-on-failure
```

- [ ] **Step 6: 全量回归并提交**

```powershell
ctest --test-dir build -C Debug --output-on-failure
git add CMakeLists.txt include/boundary_mesh/spatial/collision_index.hpp src/spatial/collision_index.cpp tests/CMakeLists.txt tests/unit/spatial/collision_index_test.cpp
git diff --cached --check
git commit -m "feat: index collision surface triangles"
```

---

### Task 5: 增量 ExposedBoundaryTracker 与远场边界物化

**Files:**
- Create: `include/boundary_mesh/growth/exposed_boundary.hpp`
- Create: `src/growth/exposed_boundary.cpp`
- Create: `include/boundary_mesh/growth/farfield_boundary_builder.hpp`
- Create: `src/growth/farfield_boundary_builder.cpp`
- Modify: `include/boundary_mesh/mesh/mesh_surface.hpp`
- Modify: `CMakeLists.txt`
- Test: `tests/unit/growth/exposed_boundary_test.cpp`
- Test: `tests/unit/growth/farfield_boundary_builder_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: 最终接受的 Prism/Hexa 候选、源面 ID、层号和分层顶点键。
- Produces: `ExposedBoundaryUpdate`、`ExposedBoundaryTracker::prepare/apply`、碰撞三角形快照和最终 `SurfaceMesh` 远场边界。

- [ ] **Step 1: 写面抵消和台阶保留失败测试**

测试依次提交两个相邻 Prism：第一个提交后存在顶面和三个侧面；第二个提交后共享侧面被抵消。再让其中一个 Prism 生长下一层，断言旧顶面删除、新顶面加入且层差侧面保留。

核心断言：

```cpp
auto tracker = ExposedBoundaryTracker{};
tracker.apply(tracker.prepare({first_prism}).value());
assert(tracker.faceCount() == 4);
tracker.apply(tracker.prepare({second_prism}).value());
assert(!tracker.contains(shared_side_key));
assert(tracker.contains(first_top_key));
```

- [ ] **Step 2: 运行 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_exposed_boundary_test
```

Expected: FAIL，缺少 `exposed_boundary.hpp`。

- [ ] **Step 3: 实现规范面键和两阶段更新**

`BoundaryFaceKey` 保存 3 或 4 个 `CollisionVertexKey`，规范化时排序，但记录原绕序几何用于三角形化。

```cpp
struct ExposedBoundaryUpdate
{
    std::vector<BoundaryFaceKey> erase_faces; // 提交后从外露集合删除的面
    std::vector<BoundaryFace> insert_faces;   // 提交后加入外露集合的面
};
```

`prepare` 完成全部引用和重复检查且不修改 tracker；`apply` 只应用已经验证的 delta。初始 tracker 为空。

- [ ] **Step 4: 实现 Prism/Hexa 面枚举**

Prism 枚举顶面和三个 Quad 侧面；Hexa 枚举顶面和四个 Quad 侧面。提交下一层时，如果底面存在则加入 `erase_faces`。同一批候选的相同侧面出现两次时相互抵消，不写入最终集合。

- [ ] **Step 5: 运行目标测试并覆盖混合邻接**

加入 Triangle/Quad Patch 边界、停止顶面永久保留、不同接受层数形成台阶，以及交换候选输入顺序后外露键集合相同的测试。

```powershell
cmake --build build --config Debug --target boundary_mesh_exposed_boundary_test
ctest --test-dir build -C Debug -R boundary_mesh_exposed_boundary_test --output-on-failure
```

- [ ] **Step 6: 回归并提交**

在提交前增加 `FarfieldBoundaryBuilder` 的失败测试和实现：

```cpp
Result<SurfaceMesh, SpatialError>
buildFarfieldBoundary(
    const SurfaceMesh &original_surface,
    const ExposedBoundaryTracker &exposed_boundary);
```

同时为 `SurfaceBoundaryKind` 增加：

```cpp
BoundaryLayerInterface // 边界层与后续远场体网格之间的界面
```

构建器复制原始 Farfield 面及标签，跳过 Wall/Symmetry；追加最终顶面、台阶侧面和 Patch 开放侧面，将其标记为 `BoundaryLayerInterface` 并继承源 Wall `region_id`。边界层接口绕序相对 tracker 中的边界层外露面反转，输出顶点重新紧凑编号。

测试必须断言：原始 Farfield 标签不变、Wall 底面不出现、接口标签和 region 正确、内部共享面不出现、接口绕序反转、没有未引用顶点。

Run:

```powershell
cmake --build build --config Debug --target boundary_mesh_exposed_boundary_test boundary_mesh_farfield_boundary_builder_test
ctest --test-dir build -C Debug -R "boundary_mesh_(exposed_boundary|farfield_boundary_builder)_test" --output-on-failure
```

- [ ] **Step 7: 回归并提交**

```powershell
ctest --test-dir build -C Debug --output-on-failure
git add CMakeLists.txt include/boundary_mesh/mesh/mesh_surface.hpp include/boundary_mesh/growth/exposed_boundary.hpp src/growth/exposed_boundary.cpp include/boundary_mesh/growth/farfield_boundary_builder.hpp src/growth/farfield_boundary_builder.cpp tests/CMakeLists.txt tests/unit/growth/exposed_boundary_test.cpp tests/unit/growth/farfield_boundary_builder_test.cpp
git diff --cached --check
git commit -m "feat: build exposed farfield boundary"
```

---

### Task 6: LayerCollisionChecker 三阶段确定性过滤

**Files:**
- Create: `include/boundary_mesh/growth/layer_collision_checker.hpp`
- Create: `src/growth/layer_collision_checker.cpp`
- Modify: `CMakeLists.txt`
- Test: `tests/unit/growth/layer_collision_checker_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: 原始 `CollisionIndex`、当前 Front、质量合格 `LayerStepResult` 和外露边界快照。
- Produces: 分别经过固定障碍和同层候选过滤的 `LayerStepResult`，碰撞面追加 `FaceStopEvent`。

- [ ] **Step 1: 写三类碰撞失败测试**

分别构造：撞另一 Wall、撞 Farfield、撞历史台阶、两个非相邻候选相撞、合法相邻候选共享侧面。断言：

```cpp
assert(stoppedSourceIds(result) == expected_stopped_ids);
assert(allReasonsAre(result, FaceStopReason::Collision));
assert(result.next_front.source_face_ids == expected_survivors);
```

另建幽灵候选用例：A 撞原始 Wall，B 只与 A 相交；预期只停止 A，因为 A 不进入同层树。

- [ ] **Step 2: 运行 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_layer_collision_checker_test
```

Expected: FAIL，缺少 `layer_collision_checker.hpp`。

- [ ] **Step 3: 定义无状态单层过滤接口**

```cpp
class LayerCollisionChecker
{
public:
    Result<LayerStepResult, SpatialError>
    filterAgainstObstacles(
        const CollisionIndex &original_surface,
        const ExposedBoundaryTracker &exposed_boundary,
        const GrowthFront &current_front,
        const LayerStepResult &quality_step) const;

    Result<LayerStepResult, SpatialError>
    filterSelfCollisions(
        const GrowthFront &current_front,
        const LayerStepResult &obstacle_step) const;
};
```

两个返回值都保留已有停止和完成事件，并追加去重后的碰撞事件。阶段 06 的 Generator 连续调用两步；阶段 07 会在两步之间插入停止传播。

- [ ] **Step 4: 实现静态、历史、同层三阶段过滤**

两个操作合起来的算法顺序固定：

```text
构造全部质量合格候选
→ 查询 OriginalSurfaceTree
→ 查询 ExposedBoundaryTree
→ 删除上述碰撞候选
→ 为剩余候选建立 CandidateLayerTree
→ 收集全部非法候选对
→ 双方同时停止
→ 按原 Front 顺序压缩面和顶点
```

同层候选对使用规范 `(min_id,max_id)` 去重。停止集合先按 `source_face_id` 排序再生成事件，不能依赖树返回顺序。

- [ ] **Step 5: 添加全侧面、无底面和顺序不变量测试**

为 Prism 三个侧面、Hexa 四个侧面和顶面分别制造碰撞；候选底面与当前 Front 重合不得停止。将源面、候选和空间 primitive 顺序反转，最终停止源面集合必须相同。

- [ ] **Step 6: 运行目标测试和全量回归**

```powershell
cmake --build build --config Debug --target boundary_mesh_layer_collision_checker_test
ctest --test-dir build -C Debug -R boundary_mesh_layer_collision_checker_test --output-on-failure
ctest --test-dir build -C Debug --output-on-failure
```

- [ ] **Step 7: 提交整层碰撞过滤**

```powershell
git add CMakeLists.txt include/boundary_mesh/growth/layer_collision_checker.hpp src/growth/layer_collision_checker.cpp tests/CMakeLists.txt tests/unit/growth/layer_collision_checker_test.cpp
git diff --cached --check
git commit -m "feat: filter colliding layer candidates"
```

---

### Task 7: RegularLayerGenerator 事务接入

**Files:**
- Modify: `include/boundary_mesh/growth/regular_layer_growth.hpp`
- Modify: `include/boundary_mesh/growth/regular_layer_growth_error.hpp`
- Modify: `include/boundary_mesh/growth/regular_layer_generator.hpp`
- Modify: `src/growth/regular_layer_generator.cpp`
- Modify: `tests/integration/regular_layer_growth_pipeline_test.cpp`
- Modify: `tests/integration/regular_layer_growth_failure_test.cpp`
- Test: `tests/integration/collision_growth_pipeline_test.cpp`
- Test: `tests/integration/collision_growth_failure_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: 完整 `SurfaceMesh`、`SurfaceTopology`、Patch、初始 Front 和逐点参数。
- Produces: 带 `FaceStopReason::Collision` 的 `RegularLayerGrowthResult`，不保留旧入口。

- [ ] **Step 1: 写新入口和碰撞事务失败测试**

将现有调用改为：

```cpp
const auto result = generateRegularLayers(
    surface_mesh,
    topology,
    patch,
    initial_front,
    profiles,
    options);
```

集成案例包含一个碰撞 Triangle、一个安全 Quad。断言碰撞源面零单元、零孤立新顶点，安全源面生成 Hexa，记录分别为 Stopped/Active 或 Completed。

- [ ] **Step 2: 运行 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_collision_growth_pipeline_test
```

Expected: FAIL，旧生成入口参数数量不匹配或缺少 `FaceStopReason::Collision`。

- [ ] **Step 3: 扩展停止原因和程序错误**

```cpp
enum class FaceStopReason
{
    // 已有值保持不变
    Collision // 候选单元发生非法几何接触
};

struct CollisionInitializationFailure
{
    SpatialError cause; // 原始表面碰撞索引建立失败的具体原因
};

struct CollisionStateFailure
{
    std::uint32_t layer{}; // 外露边界或本层碰撞状态无效的目标层号
    SpatialError cause;    // 空间模块返回的具体程序错误
};
```

将两个错误加入 `RegularLayerGrowthError`。

同时为 `RegularLayerGrowthResult` 增加：

```cpp
SurfaceMesh farfield_boundary; // 原始 Farfield 与边界层最终外露接口组成的远场边界
```

- [ ] **Step 4: 修改 Generator 数据流**

初始化时建立原始索引和空 `ExposedBoundaryTracker`。每层 Stepper 返回后先调用 `filterAgainstObstacles`、再调用 `filterSelfCollisions`；只对两步过滤后 Front 分配最终 `VertexId`、构造体单元和更新 `LayerVertexTable`。生成循环结束后调用 `buildFarfieldBoundary(surface_mesh, exposed_boundary)` 写入 `result.farfield_boundary`。

在修改 `VolumeMesh` 之前完成：碰撞过滤、Front 映射检查、顶点 ID 上限检查及 `ExposedBoundaryTracker::prepare`。全部成功后按固定顺序提交网格、记录、外露 delta 和当前 Front。

- [ ] **Step 5: 修正全部旧调用并验证无兼容重载**

搜索：

```powershell
Get-ChildItem include,src,tests -Recurse -File |
    Select-String "generateRegularLayers|RegularLayerGenerator.*generate"
```

所有调用必须传入 `surface_mesh` 和 `topology`。不得保留转发旧重载。

- [ ] **Step 6: 运行事务和失败测试**

程序错误案例应验证原始输入对象不变、返回 failure，并且没有可观察的半层输出。普通碰撞案例应返回 success。

```powershell
cmake --build build --config Debug --target boundary_mesh_collision_growth_pipeline_test boundary_mesh_collision_growth_failure_test
ctest --test-dir build -C Debug -R "boundary_mesh_collision_growth_(pipeline|failure)_test" --output-on-failure
```

- [ ] **Step 7: 全量回归并提交**

```powershell
ctest --test-dir build -C Debug --output-on-failure
git add include/boundary_mesh/growth/regular_layer_growth.hpp include/boundary_mesh/growth/regular_layer_growth_error.hpp include/boundary_mesh/growth/regular_layer_generator.hpp src/growth/regular_layer_generator.cpp tests/CMakeLists.txt tests/integration/regular_layer_growth_pipeline_test.cpp tests/integration/regular_layer_growth_failure_test.cpp tests/integration/collision_growth_pipeline_test.cpp tests/integration/collision_growth_failure_test.cpp
git diff --cached --check
git commit -m "feat: stop colliding regular layer faces"
```

---

### Task 8: 确定性回归、双配置验证与文档收尾

**Files:**
- Modify: `tests/integration/collision_growth_pipeline_test.cpp`
- Modify: `docs/design/roadmap.md`
- Modify: `docs/design/modules/collision-local-stop.md`
- Modify: `docs/plans/06-collision-local-stop.md`

**Interfaces:**
- Consumes: 阶段 06 全部公共接口。
- Produces: 顺序不变量证明、Debug/Release 验证证据和完成状态。

- [ ] **Step 1: 增加整层顺序不变量集成测试**

同一几何分别使用原始源面顺序和反转顺序运行，按 `source_face_id` 排序比较：

```cpp
assert(sortedFaceRecords(forward) == sortedFaceRecords(reversed));
assert(sortedCellMetadata(forward) == sortedCellMetadata(reversed));
assert(countOrphanVertices(forward.mesh) == 0);
assert(countOrphanVertices(reversed.mesh) == 0);
```

案例必须同时包含：静态碰撞、历史台阶碰撞、同层双方停止和安全混合 Triangle/Quad。

- [ ] **Step 2: 运行目标测试 RED/GREEN**

先在未完善排序逻辑的版本上观察失败，再修正所有依赖插入顺序的容器遍历，直至：

```powershell
cmake --build build --config Debug --target boundary_mesh_collision_growth_pipeline_test
ctest --test-dir build -C Debug -R boundary_mesh_collision_growth_pipeline_test --output-on-failure
```

Expected: PASS。

- [ ] **Step 3: Debug 全量验证**

```powershell
cmake -S . -B build
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

Expected: 旧 27 项和阶段 06 新增测试全部通过，0 failed。

- [ ] **Step 4: Release 全量验证**

```powershell
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Expected: 全部测试通过，0 failed。

- [ ] **Step 5: 静态检查依赖边界和未实现功能**

```powershell
Get-ChildItem include,src -Recurse -File |
    Select-String "geom_func.h|TiGER_GEOM_FUNC"
Get-ChildItem include,src -Recurse -File |
    Select-String "libigl|safe_step|nearest|Symmetry.*Collision"
```

Expected: `geom_func.h`/`TiGER_GEOM_FUNC` 只出现在 Spatial 实现；其他禁用功能没有生产实现。

- [ ] **Step 6: 更新完成状态**

将路线图阶段 06 改为“已完成”。在设计文档记录最终公共文件和验证命令；在本计划末尾增加实际提交编号与 Debug/Release 测试数量，不改动阶段 07、10 的未实施状态。

- [ ] **Step 7: 提交测试和文档收尾**

```powershell
git add tests/integration/collision_growth_pipeline_test.cpp docs/design/roadmap.md docs/design/modules/collision-local-stop.md docs/plans/06-collision-local-stop.md
git diff --cached --check
git commit -m "test: complete collision local stopping stage"
git status --short --branch
git log -10 --oneline --decorate
```

Expected: 仅 `.superpowers/` 保持未跟踪；阶段 06 的所有其他文件已提交。

---

## Final Verification

阶段 06 只有在以下条件同时满足时才可以标记完成：

```text
OriginalSurfaceTree 只包含 Wall/Farfield
ExposedBoundaryTree 按层重建且数据来自增量 tracker
CandidateLayerTree 不包含已被静态/历史障碍停止的候选
候选检查顶面和全部侧面，不检查底面
合法拓扑连接放行，其他零距离接触停止
同层碰撞双方停止且与输入顺序无关
碰撞不缩短、不重试、不记录障碍详情
Symmetry、包含检测、最大安全步长没有进入实现
无碰撞孤立顶点、半层单元或未提交映射
Debug 和 Release 全量 CTest 均为 0 failed
```
