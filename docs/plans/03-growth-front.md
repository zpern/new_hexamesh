# GrowthPatch 与动态活动前沿实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 从已验证的封闭混合表面提取 Wall `GrowthPatch`，建立第 0 层 `GrowthFront`，并对任意当前层前沿重新计算逐面几何、角度加权节点方向和对称约束。

**Architecture:** `BoundaryMesh::Surface` 只处理单个 Triangle/Quad 的无状态算法；`BoundaryMesh::BoundaryLayer` 遍历整个 `GrowthFront`，逐面调用 Surface 算法，并补充层号和源实体 ID。输入有限性由 `SurfaceTopologyBuilder` 检查，生长后的有限性与退化由 `FrontEvaluator` 每层检查。

**Tech Stack:** C++17、Eigen、CMake 3.20+、CTest、MSVC Debug

## Global Constraints

- C++ namespace 固定为 `boundary_mesh`，公共 include 前缀固定为 `<boundary_mesh/...>`。
- 公共 CMake target 使用 `BoundaryMesh::Core`、`BoundaryMesh::Surface`、`BoundaryMesh::BoundaryLayer`。
- `Surface` 逐个计算 Triangle/Quad，不接收完整 `SurfaceMesh`，不保存网格状态。
- `BoundaryLayer` 遍历整个当前 `GrowthFront`，逐面调用 `Surface` 并汇总结果。
- 完整 `SurfaceTopology` 必须封闭；提取后的 Wall `GrowthPatch` 允许开放边界和多个连通分量。
- `GrowthFront` 使用紧凑局部顶点编号，显式保存到源顶点、源面的映射。
- 前沿坐标变化后必须重新评价，禁止复用上一层面积、法向或方向。
- 容差由当前前沿包围盒尺度计算，局部 Surface 算法只接收有效长度容差。
- 可预期错误使用 `Result<T, E>`；库代码不打印日志、不返回部分结果、不让异常跨公共 API。
- 新增源码写中文注释，说明用途、约束、不变量和返回意义。
- 每个 Task 严格执行 RED、GREEN、REFACTOR；用户提供实际编译测试输出后再提交。
- 阶段 03 不生成新层节点、Prism、Hexa、碰撞结构或过渡单元。

---

## File Map

```text
include/boundary_mesh/mesh/surface_topology_error.hpp
src/mesh/surface_topology_builder.cpp
    输入 SurfaceMesh 的有限坐标检查

include/boundary_mesh/surface/face_evaluation.hpp
src/surface/face_evaluation.cpp
    单个 Triangle/Quad 的无状态面积、法向、翘曲和顶点内角算法

include/boundary_mesh/growth/growth_patch.hpp
include/boundary_mesh/growth/growth_patch_error.hpp
include/boundary_mesh/growth/growth_patch_builder.hpp
src/growth/growth_patch_builder.cpp
    从完整表面提取确定性 Wall GrowthPatch

include/boundary_mesh/growth/growth_front.hpp
include/boundary_mesh/growth/growth_front_error.hpp
include/boundary_mesh/growth/growth_front_builder.hpp
src/growth/growth_front_builder.cpp
    第 0 层紧凑前沿及源实体映射

include/boundary_mesh/growth/front_evaluation.hpp
include/boundary_mesh/growth/front_evaluation_error.hpp
include/boundary_mesh/growth/front_evaluator.hpp
src/growth/front_evaluator.cpp
    遍历整个当前前沿，逐面调用 Surface 算法并汇总

include/boundary_mesh/growth/growth_direction.hpp
include/boundary_mesh/growth/growth_direction_error.hpp
src/growth/growth_direction.cpp
    基于当前前沿局部关联面的角度加权节点方向

include/boundary_mesh/growth/symmetry_constraints.hpp
include/boundary_mesh/growth/symmetry_constraint_builder.hpp
src/growth/symmetry_constraint_builder.cpp
    Symmetry region 平面验证以及单/多平面方向约束
```

---

### Task 2 前置整理: 测试目录按模块迁移

**Files:**

- Move: `tests/unit/core_types_test.cpp` → `tests/unit/core/core_types_test.cpp`
- Move: `tests/unit/result_test.cpp` → `tests/unit/core/result_test.cpp`
- Move: `tests/unit/surface_mesh_test.cpp` → `tests/unit/mesh/surface_mesh_test.cpp`
- Move: `tests/unit/volume_mesh_test.cpp` → `tests/unit/mesh/volume_mesh_test.cpp`
- Move: `tests/unit/surface_topology_types_test.cpp` → `tests/unit/mesh/surface_topology_types_test.cpp`
- Move: `tests/unit/surface_topology_validation_test.cpp` → `tests/unit/mesh/surface_topology_validation_test.cpp`
- Move: `tests/unit/surface_topology_builder_test.cpp` → `tests/unit/mesh/surface_topology_builder_test.cpp`
- Move: `tests/unit/surface_topology_edge_error_test.cpp` → `tests/unit/mesh/surface_topology_edge_error_test.cpp`
- Move: `tests/unit/surface_face_evaluation_test.cpp` → `tests/unit/surface/face_evaluation_test.cpp`
- Modify: `tests/CMakeLists.txt`

- [ ] 保持所有 CMake target 和 CTest name 不变，只修改 `.cpp` 相对路径。
- [ ] 重新运行 `cmake -S . -B build`，避免旧生成系统继续引用原路径。
- [ ] Debug 构建并运行全部 CTest；迁移提交不得包含生产代码变化。

---

### Task 1: 输入顶点有限坐标检查

**Files:**

- Modify: `include/boundary_mesh/mesh/surface_topology_error.hpp`
- Modify: `src/mesh/surface_topology_builder.cpp`
- Modify: `tests/unit/mesh/surface_topology_validation_test.cpp`

**Interfaces:**

- Consumes: `SurfaceMesh::vertices`
- Produces: `NonFiniteVertex{VertexId vertex_id}`，加入 `SurfaceTopologyError`

- [ ] **Step 1: 写 RED 测试**

在现有 validation test 中分别构造包含 NaN 和负无穷的顶点；第二个非有限顶点故意不被任何面引用。断言 `build()` 失败且 `vertex_id` 分别准确返回 1 和 3。

```cpp
#include <limits>

SurfaceMesh invalid = validTriangleMesh();
invalid.vertices[1].x() =
    std::numeric_limits<Scalar>::quiet_NaN();
const auto nan_result = builder.build(invalid);
const auto *nan_error = nan_result.hasValue()
    ? nullptr
    : std::get_if<NonFiniteVertex>(&nan_result.error());
if (nan_error == nullptr || nan_error->vertex_id != VertexId{1})
{
    return 7;
}

invalid = validTriangleMesh();
invalid.vertices.push_back(Point3{
    0.0,
    -std::numeric_limits<Scalar>::infinity(),
    0.0});
const auto infinity_result = builder.build(invalid);
const auto *infinity_error = infinity_result.hasValue()
    ? nullptr
    : std::get_if<NonFiniteVertex>(&infinity_result.error());
if (infinity_error == nullptr ||
    infinity_error->vertex_id != VertexId{3})
{
    return 8;
}
```

- [ ] **Step 2: 运行 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_surface_topology_validation_test
```

预期：编译失败，`NonFiniteVertex` 未声明。

- [ ] **Step 3: 添加错误并实现完整扫描**

```cpp
struct NonFiniteVertex
{
    VertexId vertex_id{};
};
```

将其加入 `SurfaceTopologyError`。在标签数量检查之后、任何面索引检查之前扫描全部输入顶点：

```cpp
for (std::size_t vertex_index = 0;
     vertex_index < mesh.vertices.size();
     ++vertex_index)
{
    const Point3 &point = mesh.vertices[vertex_index];
    if (!std::isfinite(point.x()) ||
        !std::isfinite(point.y()) ||
        !std::isfinite(point.z()))
    {
        return BuildResult::failure(
            SurfaceTopologyError{NonFiniteVertex{
                static_cast<VertexId>(vertex_index)}});
    }
}
```

- [ ] **Step 4: 验证并提交**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
git add include/boundary_mesh/mesh/surface_topology_error.hpp src/mesh/surface_topology_builder.cpp tests/unit/mesh/surface_topology_validation_test.cpp
git diff --cached --check
git commit -m "feat: reject non-finite surface vertices"
```

---

### Task 2: Triangle/Quad 无状态 Surface 算法

**Files:**

- Create: `include/boundary_mesh/surface/face_evaluation.hpp`
- Create: `src/surface/face_evaluation.cpp`
- Create: `tests/unit/surface/face_evaluation_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**

- Produces: `FaceEvaluation`、`FaceEvaluationError`
- Produces: `evaluateTriangle(...)`、`evaluateQuad(...)`、`cornerAngle(...)`
- Produces target: `BoundaryMesh::Surface`

- [ ] **Step 1: 写 RED 测试并注册 target**

测试单位直角三角形、单位平面四边形、直角顶点和共线三角形：

```cpp
const Scalar tolerance = 1e-12;
const auto triangle = evaluateTriangle(
    Point3{0, 0, 0}, Point3{1, 0, 0}, Point3{0, 1, 0}, tolerance);
if (!triangle.hasValue() ||
    std::abs(triangle.value().area - 0.5) > tolerance ||
    (triangle.value().unit_normal - Vector3{0, 0, 1}).norm() > tolerance)
{
    return 1;
}

const auto quad = evaluateQuad(
    Point3{0, 0, 0}, Point3{1, 0, 0},
    Point3{1, 1, 0}, Point3{0, 1, 0}, tolerance);
if (!quad.hasValue() ||
    std::abs(quad.value().area - 1.0) > tolerance ||
    std::abs(quad.value().warpage_angle) > tolerance)
{
    return 2;
}

const auto angle = cornerAngle(
    Point3{1, 0, 0}, Point3{0, 0, 0}, Point3{0, 1, 0}, tolerance);
if (!angle.hasValue() ||
    std::abs(angle.value() - std::acos(-1.0) / 2.0) > tolerance)
{
    return 3;
}

const auto degenerate = evaluateTriangle(
    Point3{0, 0, 0}, Point3{1, 0, 0}, Point3{2, 0, 0}, tolerance);
if (degenerate.hasValue() ||
    degenerate.error() != FaceEvaluationError::DegenerateAreaVector)
{
    return 4;
}
```

```cmake
add_executable(boundary_mesh_surface_face_evaluation_test unit/surface/face_evaluation_test.cpp)
target_link_libraries(boundary_mesh_surface_face_evaluation_test PRIVATE BoundaryMesh::Surface)
add_test(NAME boundary_mesh_surface_face_evaluation_test COMMAND boundary_mesh_surface_face_evaluation_test)
```

- [ ] **Step 2: 运行 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_surface_face_evaluation_test
```

预期：`BoundaryMesh::Surface` 或头文件不存在。

- [ ] **Step 3: 定义接口**

```cpp
struct FaceEvaluation
{
    Point3 centroid{Point3::Zero()};
    Vector3 area_vector{Vector3::Zero()};
    Vector3 unit_normal{Vector3::Zero()};
    Scalar area{};
    Scalar warpage_angle{};
};

enum class FaceEvaluationError
{
    NonFiniteCoordinate,
    DegenerateEdge,
    DegenerateAreaVector
};

Result<FaceEvaluation, FaceEvaluationError> evaluateTriangle(
    const Point3 &, const Point3 &, const Point3 &, Scalar length_tolerance);
Result<FaceEvaluation, FaceEvaluationError> evaluateQuad(
    const Point3 &, const Point3 &, const Point3 &, const Point3 &,
    Scalar length_tolerance);
Result<Scalar, FaceEvaluationError> cornerAngle(
    const Point3 &previous, const Point3 &center, const Point3 &next,
    Scalar length_tolerance);
```

- [ ] **Step 4: 实现固定算法和 target**

三角形使用 `0.5 * (v1-v0).cross(v2-v0)`；四边形固定拆成 `(v0,v1,v2)` 与 `(v0,v2,v3)`；四边形面积为两子三角形面积和，面积向量为两者之和，翘曲角为两子三角形单位法向夹角。所有边先与 `length_tolerance` 比较，面积与其平方比较，`acos` 输入 clamp 到 `[-1,1]`。

```cmake
add_library(boundary_mesh_surface STATIC src/surface/face_evaluation.cpp)
add_library(BoundaryMesh::Surface ALIAS boundary_mesh_surface)
target_compile_features(boundary_mesh_surface PUBLIC cxx_std_17)
target_include_directories(boundary_mesh_surface PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(boundary_mesh_surface PUBLIC BoundaryMesh::Core)
```

- [ ] **Step 5: 验证并提交**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
git add CMakeLists.txt tests/CMakeLists.txt include/boundary_mesh/surface/face_evaluation.hpp src/surface/face_evaluation.cpp tests/unit/surface/face_evaluation_test.cpp
git diff --cached --check
git commit -m "feat: add stateless surface evaluation"
```

---

### Task 3: GrowthPatch 提取与边界分类

**Files:**

- Create: `include/boundary_mesh/growth/growth_patch.hpp`
- Create: `include/boundary_mesh/growth/growth_patch_error.hpp`
- Create: `include/boundary_mesh/growth/growth_patch_builder.hpp`
- Create: `src/growth/growth_patch_builder.cpp`
- Create: `tests/unit/growth/growth_patch_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**

- Consumes: `SurfaceMesh`、`SurfaceTopology`
- Produces: `GrowthPatchBuilder::build(const SurfaceMesh&, const SurfaceTopology&)`
- Produces target: `BoundaryMesh::BoundaryLayer`

- [ ] **Step 1: 用封闭三棱柱写 RED 测试**

为五个面赋 Wall、Farfield、Symmetry 标签。断言只选择 Wall 面；源面、源顶点和源边按 ID 升序；边分别被分类为 Interior、SymmetryBoundary、FarfieldBoundary；全部改成 Farfield 后返回 `EmptyGrowthPatch`。

```cpp
enum class PatchEdgeKind { Interior, SymmetryBoundary, FarfieldBoundary };

struct PatchEdge
{
    EdgeId source_edge_id{};
    std::array<SurfaceFaceId, 2> complete_face_ids{};
    PatchEdgeKind kind{PatchEdgeKind::Interior};
};
```

- [ ] **Step 2: 运行 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_growth_patch_test
```

预期：GrowthPatch 接口或 `BoundaryMesh::BoundaryLayer` 不存在。

- [ ] **Step 3: 定义只读 GrowthPatch**

```cpp
class GrowthPatch
{
public:
    const std::vector<VertexId> &sourceVertexIds() const noexcept;
    const std::vector<SurfaceFaceId> &sourceFaceIds() const noexcept;
    const std::vector<PatchEdge> &edges() const noexcept;
private:
    friend class GrowthPatchBuilder;
    GrowthPatch(std::vector<VertexId>, std::vector<SurfaceFaceId>,
                std::vector<PatchEdge>);
    std::vector<VertexId> source_vertex_ids_;
    std::vector<SurfaceFaceId> source_face_ids_;
    std::vector<PatchEdge> edges_;
};

struct EmptyGrowthPatch {};
struct MeshTopologyMismatch
{
    std::size_t mesh_vertex_count{};
    std::size_t topology_vertex_count{};
    std::size_t mesh_face_count{};
    std::size_t topology_face_count{};
};
using GrowthPatchError = std::variant<EmptyGrowthPatch, MeshTopologyMismatch>;
```

- [ ] **Step 4: 实现确定性提取**

依次扫描面 ID 选择 Wall；使用按输入大小建立的 `vector<bool>` 标记顶点和边；最后按索引升序输出。每条已选边读取 `edgeFaces()` 两侧标签并按 `Wall+Wall`、`Wall+Symmetry`、`Wall+Farfield` 分类。禁止用 `unordered_map` 遍历顺序生成公开数组。

- [ ] **Step 5: 建立 BoundaryLayer target、验证并提交**

```cmake
add_library(boundary_mesh_boundary_layer STATIC src/growth/growth_patch_builder.cpp)
add_library(BoundaryMesh::BoundaryLayer ALIAS boundary_mesh_boundary_layer)
target_compile_features(boundary_mesh_boundary_layer PUBLIC cxx_std_17)
target_include_directories(boundary_mesh_boundary_layer PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(boundary_mesh_boundary_layer PUBLIC BoundaryMesh::Core BoundaryMesh::Surface)
```

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
git add CMakeLists.txt tests/CMakeLists.txt include/boundary_mesh/growth src/growth/growth_patch_builder.cpp tests/unit/growth/growth_patch_test.cpp
git diff --cached --check
git commit -m "feat: extract wall growth patch"
```

---

### Task 4: 第 0 层 GrowthFront 与源实体映射

**Files:**

- Create: `include/boundary_mesh/growth/growth_front.hpp`
- Create: `include/boundary_mesh/growth/growth_front_error.hpp`
- Create: `include/boundary_mesh/growth/growth_front_builder.hpp`
- Create: `src/growth/growth_front_builder.cpp`
- Create: `tests/unit/growth/growth_front_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**

- Consumes: `SurfaceMesh`、`GrowthPatch`
- Produces: `GrowthFrontBuilder::buildInitial(const SurfaceMesh&, const GrowthPatch&)`

- [ ] **Step 1: 写紧凑编号 RED 测试**

使用 Task 3 的三棱柱数据，构造 topology、patch、front 后断言：`layer == 0`；坐标依照 `source_vertex_ids` 复制；面数量与 `source_face_ids` 一致；每个面改用 `[0, front.vertices.size())` 的局部编号；绕序保持不变。

- [ ] **Step 2: 运行 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_growth_front_test
```

预期：`growth_front_builder.hpp` 不存在。

- [ ] **Step 3: 定义数据与错误**

```cpp
struct GrowthFront
{
    std::uint32_t layer{};
    std::vector<Point3> vertices;
    std::vector<SurfaceFace> faces;
    std::vector<VertexId> source_vertex_ids;
    std::vector<SurfaceFaceId> source_face_ids;
};

struct InvalidPatchVertex { VertexId source_vertex_id{}; };
struct InvalidPatchFace { SurfaceFaceId source_face_id{}; };
struct PatchFaceUsesUnknownVertex
{
    SurfaceFaceId source_face_id{};
    VertexId source_vertex_id{};
};
using GrowthFrontError = std::variant<
    InvalidPatchVertex,
    InvalidPatchFace,
    PatchFaceUsesUnknownVertex>;
```

- [ ] **Step 4: 用固定两阶段算法实现 buildInitial**

第一阶段复制源顶点并建立 `source VertexId -> local VertexId` 映射；验证每个源顶点在 mesh 范围内。随后对每个源面执行以下两个明确阶段，不使用异常控制流：

1. 先遍历源面的全部 `vertex_ids`。任一顶点不在映射中，立即返回 `Result<GrowthFront, GrowthFrontError>::failure(PatchFaceUsesUnknownVertex{source_face_id, source_vertex_id})`。
2. 全部验证通过后，再用不可能失败的 `std::visit` lambda 将 Triangle/Quad 的源顶点 ID 重建为局部 ID。

```cpp
for (std::size_t face_index = 0;
     face_index < patch.sourceFaceIds().size();
     ++face_index)
{
    const SurfaceFaceId source_face_id =
        patch.sourceFaceIds()[face_index];
    if (static_cast<std::size_t>(source_face_id) >= mesh.faces.size())
    {
        return FrontResult::failure(
            GrowthFrontError{InvalidPatchFace{source_face_id}});
    }

    const SurfaceFace &source_face =
        mesh.faces[static_cast<std::size_t>(source_face_id)];
    bool missing = false;
    VertexId missing_id{};
    std::visit([&](const auto &face)
    {
        for (const VertexId source_vertex_id : face.vertex_ids)
        {
            if (local_ids.find(source_vertex_id) == local_ids.end())
            {
                missing = true;
                missing_id = source_vertex_id;
                break;
            }
        }
    }, source_face);
    if (missing)
    {
        return FrontResult::failure(GrowthFrontError{
            PatchFaceUsesUnknownVertex{source_face_id, missing_id}});
    }

    front.faces.push_back(std::visit(
        [&](const auto &face) -> SurfaceFace
        {
            auto local_face = face;
            for (VertexId &vertex_id : local_face.vertex_ids)
            {
                vertex_id = local_ids.at(vertex_id);
            }
            return local_face;
        }, source_face));
}
```

- [ ] **Step 5: 验证并提交**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
git add CMakeLists.txt tests/CMakeLists.txt include/boundary_mesh/growth/growth_front.hpp include/boundary_mesh/growth/growth_front_error.hpp include/boundary_mesh/growth/growth_front_builder.hpp src/growth/growth_front_builder.cpp tests/unit/growth/growth_front_test.cpp
git diff --cached --check
git commit -m "feat: build initial growth front"
```

---

### Task 5: 当前前沿动态面积、法向与退化检查

**Files:**

- Create: `include/boundary_mesh/growth/front_evaluation.hpp`
- Create: `include/boundary_mesh/growth/front_evaluation_error.hpp`
- Create: `include/boundary_mesh/growth/front_evaluator.hpp`
- Create: `src/growth/front_evaluator.cpp`
- Create: `tests/unit/growth/front_evaluator_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**

- Consumes: `GrowthFront`、Task 2 的 `evaluateTriangle/evaluateQuad`
- Produces: `FrontEvaluator::evaluate(const GrowthFront&, const SurfaceEvaluationOptions&)`
- Produces: 当前层尺度、有效长度容差、逐面结果，以及带 layer/source ID 的错误

- [ ] **Step 1: 注册并编写 RED 测试**

测试包含一个 Triangle 和一个 Quad 的同一平面前沿。断言 evaluator 不是“针对一个面”的接口，而是一次接收整个 `GrowthFront` 并返回两个按前沿面顺序排列的结果：

```cpp
GrowthFront front;
front.layer = 4;
front.vertices = {
    Point3{0, 0, 0}, Point3{1, 0, 0},
    Point3{1, 1, 0}, Point3{0, 1, 0},
    Point3{2, 0, 0}, Point3{2, 1, 0}};
front.faces = {
    Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}},
    Quad{{VertexId{1}, VertexId{4}, VertexId{5}, VertexId{2}}}};
front.source_vertex_ids = {
    VertexId{10}, VertexId{11}, VertexId{12},
    VertexId{13}, VertexId{14}, VertexId{15}};
front.source_face_ids = {SurfaceFaceId{20}, SurfaceFaceId{21}};

const auto result = FrontEvaluator{}.evaluate(front);
if (!result.hasValue() || result.value().layer != 4 ||
    result.value().faces.size() != 2)
{
    return 1;
}
if (result.value().faces[0].source_face_id != SurfaceFaceId{20} ||
    std::abs(result.value().faces[0].value.area - 0.5) > 1e-12 ||
    result.value().faces[1].source_face_id != SurfaceFaceId{21} ||
    std::abs(result.value().faces[1].value.area - 1.0) > 1e-12)
{
    return 2;
}
```

继续加入三个失败案例：

```cpp
GrowthFront non_finite = front;
non_finite.vertices[3].z() =
    std::numeric_limits<Scalar>::quiet_NaN();
const auto non_finite_result = FrontEvaluator{}.evaluate(non_finite);
const auto *vertex_error = non_finite_result.hasValue()
    ? nullptr
    : std::get_if<NonFiniteFrontVertex>(&non_finite_result.error());
if (vertex_error == nullptr ||
    vertex_error->front_vertex_index != 3 ||
    vertex_error->source_vertex_id != VertexId{13} ||
    vertex_error->layer != 4)
{
    return 3;
}

GrowthFront invalid_reference = front;
std::get<Triangle>(invalid_reference.faces[0]).vertex_ids[2] = VertexId{99};
const auto reference_result = FrontEvaluator{}.evaluate(invalid_reference);
if (reference_result.hasValue() ||
    !std::holds_alternative<InvalidFrontVertexReference>(
        reference_result.error()))
{
    return 4;
}

GrowthFront degenerate = front;
degenerate.vertices[2] = degenerate.vertices[1];
const auto degenerate_result = FrontEvaluator{}.evaluate(degenerate);
const auto *face_error = degenerate_result.hasValue()
    ? nullptr
    : std::get_if<DegenerateFrontFace>(&degenerate_result.error());
if (face_error == nullptr ||
    face_error->front_face_index != 0 ||
    face_error->source_face_id != SurfaceFaceId{20} ||
    face_error->layer != 4)
{
    return 5;
}
```

注册：

```cmake
add_executable(boundary_mesh_front_evaluator_test unit/growth/front_evaluator_test.cpp)
target_link_libraries(boundary_mesh_front_evaluator_test PRIVATE BoundaryMesh::BoundaryLayer)
add_test(NAME boundary_mesh_front_evaluator_test COMMAND boundary_mesh_front_evaluator_test)
```

- [ ] **Step 2: 运行 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_front_evaluator_test
```

预期：`boundary_mesh/growth/front_evaluator.hpp` 不存在。

- [ ] **Step 3: 定义结果、配置和错误**

创建 `front_evaluation.hpp`：

```cpp
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/surface/face_evaluation.hpp>

namespace boundary_mesh
{
    struct SurfaceEvaluationOptions
    {
        Scalar relative_length_tolerance{1e-12};
    };

    struct FrontFaceEvaluation
    {
        std::size_t front_face_index{};
        SurfaceFaceId source_face_id{};
        FaceEvaluation value;
    };

    struct FrontEvaluation
    {
        std::uint32_t layer{};
        Scalar characteristic_length{};
        Scalar effective_length_tolerance{};
        std::vector<FrontFaceEvaluation> faces;
    };
}
```

`effective_length_tolerance` 保存本次评价实际使用的容差，Task 6 必须复用它，避免同一层的面评价和角度计算采用不同阈值。

创建 `front_evaluation_error.hpp`：

```cpp
#pragma once

#include <cstddef>
#include <cstdint>
#include <variant>

#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/surface/face_evaluation.hpp>

namespace boundary_mesh
{
    struct InvalidSurfaceEvaluationOptions
    {
        Scalar relative_length_tolerance{};
    };

    struct EmptyGrowthFront { std::uint32_t layer{}; };

    struct FrontMappingMismatch
    {
        std::size_t vertex_count{};
        std::size_t source_vertex_count{};
        std::size_t face_count{};
        std::size_t source_face_count{};
        std::uint32_t layer{};
    };

    struct NonFiniteFrontVertex
    {
        std::size_t front_vertex_index{};
        VertexId source_vertex_id{};
        std::uint32_t layer{};
    };

    struct InvalidFrontVertexReference
    {
        std::size_t front_face_index{};
        SurfaceFaceId source_face_id{};
        VertexId front_vertex_id{};
        std::uint32_t layer{};
    };

    struct DegenerateFrontScale { std::uint32_t layer{}; };

    struct DegenerateFrontFace
    {
        std::size_t front_face_index{};
        SurfaceFaceId source_face_id{};
        std::uint32_t layer{};
        FaceEvaluationError cause{FaceEvaluationError::DegenerateAreaVector};
    };

    using FrontEvaluationError = std::variant<
        InvalidSurfaceEvaluationOptions,
        EmptyGrowthFront,
        FrontMappingMismatch,
        NonFiniteFrontVertex,
        InvalidFrontVertexReference,
        DegenerateFrontScale,
        DegenerateFrontFace>;
}
```

创建 `front_evaluator.hpp`：

```cpp
#pragma once

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/front_evaluation.hpp>
#include <boundary_mesh/growth/front_evaluation_error.hpp>
#include <boundary_mesh/growth/growth_front.hpp>

namespace boundary_mesh
{
    class FrontEvaluator
    {
    public:
        Result<FrontEvaluation, FrontEvaluationError>
        evaluate(
            const GrowthFront &front,
            const SurfaceEvaluationOptions &options = {}) const;
    };
}
```

- [ ] **Step 4: 实现整个前沿的评价流程**

在 `src/growth/front_evaluator.cpp` 按固定顺序实现：

1. `relative_length_tolerance` 必须有限且大于零。
2. `vertices/faces` 非空，两个 source 映射长度分别与实体数组一致。
3. 扫描全部当前顶点有限性；错误包含当前下标、源 ID、layer。
4. 计算 AABB 对角线 `characteristic_length`；非有限或不大于零返回 `DegenerateFrontScale`。
5. `effective_length_tolerance = max(characteristic_length * relative, sqrt(numeric_limits<Scalar>::min()))`。
6. 按 `front.faces` 顺序验证局部顶点 ID 后，使用 `std::visit` 对 Triangle 调用 `evaluateTriangle`，对 Quad 调用 `evaluateQuad`。
7. 任一局部调用失败，包装为 `DegenerateFrontFace`；全部成功才返回完整结果。

逐面调用的核心必须是：

```cpp
const auto local_result = std::visit(
    [&](const auto &face)
        -> Result<FaceEvaluation, FaceEvaluationError>
    {
        using Face = std::decay_t<decltype(face)>;
        if constexpr (std::is_same_v<Face, Triangle>)
        {
            return evaluateTriangle(
                front.vertices[face.vertex_ids[0]],
                front.vertices[face.vertex_ids[1]],
                front.vertices[face.vertex_ids[2]],
                effective_tolerance);
        }
        else
        {
            return evaluateQuad(
                front.vertices[face.vertex_ids[0]],
                front.vertices[face.vertex_ids[1]],
                front.vertices[face.vertex_ids[2]],
                front.vertices[face.vertex_ids[3]],
                effective_tolerance);
        }
    },
    front.faces[face_index]);
```

在调用前已保证所有 `vertex_ids` 在范围内，因此该 lambda 不承担索引错误处理。

- [ ] **Step 5: GREEN、全回归并提交**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
git add CMakeLists.txt tests/CMakeLists.txt include/boundary_mesh/growth/front_evaluation.hpp include/boundary_mesh/growth/front_evaluation_error.hpp include/boundary_mesh/growth/front_evaluator.hpp src/growth/front_evaluator.cpp tests/unit/growth/front_evaluator_test.cpp
git diff --cached --check
git commit -m "feat: evaluate dynamic growth front"
```

---

### Task 6: 角度加权节点方向

**Files:**

- Create: `include/boundary_mesh/growth/growth_direction.hpp`
- Create: `include/boundary_mesh/growth/growth_direction_error.hpp`
- Create: `src/growth/growth_direction.cpp`
- Create: `tests/unit/growth/growth_direction_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**

- Consumes: 同一次评价对应的 `GrowthFront`、`FrontEvaluation`
- Produces: `computeGrowthDirections(const GrowthFront&, const FrontEvaluation&)`
- 不使用完整 `SurfaceTopology::vertexFaces()`；关联关系只从当前 `front.faces` 构建

- [ ] **Step 1: 写 RED 测试**

构造两个共享顶点的当前前沿面：一个法向 `+Z` 的直角三角形，一个法向 `+Y` 的直角三角形。共享顶点两个内角都为 π/2，预期方向为 `(0,1,1).normalized()`。同时断言非共享顶点只使用自己的当前关联面。

```cpp
GrowthFront front;
front.layer = 2;
front.vertices = {
    Point3{0, 0, 0}, Point3{1, 0, 0},
    Point3{0, 1, 0}, Point3{0, 0, 1}};
front.faces = {
    Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}},
    Triangle{{VertexId{0}, VertexId{3}, VertexId{1}}}};
front.source_vertex_ids = {
    VertexId{30}, VertexId{31}, VertexId{32}, VertexId{33}};
front.source_face_ids = {SurfaceFaceId{40}, SurfaceFaceId{41}};

const auto evaluation = FrontEvaluator{}.evaluate(front);
const auto directions = computeGrowthDirections(front, evaluation.value());
if (!directions.hasValue() || directions.value().values.size() != 4)
{
    return 1;
}
const Vector3 expected = Vector3{0, 1, 1}.normalized();
if ((directions.value().values[0] - expected).norm() > 1e-12 ||
    (directions.value().values[2] - Vector3{0, 0, 1}).norm() > 1e-12)
{
    return 2;
}
```

再创建一个 `FrontEvaluation`，故意修改 layer 或 face 数量，断言返回 `DirectionInputMismatch`；构造法向加权后相互抵消的前沿，断言返回带当前顶点、源顶点和 layer 的 `UndefinedGrowthDirection`。

- [ ] **Step 2: 运行 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_growth_direction_test
```

预期：`growth_direction.hpp` 不存在。

- [ ] **Step 3: 定义结果和错误**

```cpp
// growth_direction.hpp
struct GrowthDirections
{
    std::uint32_t layer{};
    std::vector<VertexId> source_vertex_ids;
    std::vector<Vector3> values;
};

Result<GrowthDirections, GrowthDirectionError>
computeGrowthDirections(
    const GrowthFront &front,
    const FrontEvaluation &evaluation);
```

```cpp
// growth_direction_error.hpp
struct DirectionInputMismatch
{
    std::uint32_t front_layer{};
    std::uint32_t evaluation_layer{};
};

struct DirectionCornerFailure
{
    std::size_t front_vertex_index{};
    VertexId source_vertex_id{};
    std::size_t front_face_index{};
    SurfaceFaceId source_face_id{};
    std::uint32_t layer{};
    FaceEvaluationError cause{FaceEvaluationError::DegenerateEdge};
};

struct UndefinedGrowthDirection
{
    std::size_t front_vertex_index{};
    VertexId source_vertex_id{};
    std::uint32_t layer{};
};

using GrowthDirectionError = std::variant<
    DirectionInputMismatch,
    DirectionCornerFailure,
    UndefinedGrowthDirection>;
```

- [ ] **Step 4: 实现当前前沿局部关联和角度加权**

先严格检查：layer 相同、`evaluation.faces.size() == front.faces.size()`、每个 `front_face_index/source_face_id` 与当前位置一致、顶点映射长度一致。

创建与 `front.vertices` 等长的累加向量。遍历每个当前面和每个局部角，令 `previous/center/next` 按该面绕序循环取值，调用：

```cpp
const auto angle = cornerAngle(
    front.vertices[previous_id],
    front.vertices[center_id],
    front.vertices[next_id],
    evaluation.effective_length_tolerance);
```

成功时执行：

```cpp
accumulated[center_id] +=
    evaluation.faces[face_index].value.unit_normal * angle.value();
```

遍历完毕后，以 `evaluation.effective_length_tolerance` 为模长下限逐顶点归一化；不足时返回 `UndefinedGrowthDirection`。不查询完整表面的 `vertexFaces()`，因为其中包含不生长的 Symmetry/Farfield 面。

- [ ] **Step 5: GREEN、全回归并提交**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
git add CMakeLists.txt tests/CMakeLists.txt include/boundary_mesh/growth/growth_direction.hpp include/boundary_mesh/growth/growth_direction_error.hpp src/growth/growth_direction.cpp tests/unit/growth/growth_direction_test.cpp
git diff --cached --check
git commit -m "feat: compute angle weighted growth directions"
```

---

### Task 7: 单个及多个对称面约束

**Files:**

- Create: `include/boundary_mesh/growth/symmetry_constraints.hpp`
- Create: `include/boundary_mesh/growth/symmetry_constraint_builder.hpp`
- Create: `src/growth/symmetry_constraint_builder.cpp`
- Create: `tests/unit/growth/symmetry_constraints_test.cpp`
- Modify: `include/boundary_mesh/growth/growth_direction_error.hpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**

- Consumes: 完整 `SurfaceMesh/SurfaceTopology`、`GrowthPatch`、当前 `GrowthFront/FrontEvaluation`
- Produces: `SymmetryConstraintBuilder::build(...)`
- Produces: `SymmetryConstraints::apply(front_vertex_index, raw_direction)`
- 单平面投影、两平面交线、共线法向去重、三个独立约束报错均与遍历顺序无关

- [ ] **Step 1: 写单对称面和 region 平面验证 RED 测试**

在封闭三棱柱数据中把一个侧面标记为 `Symmetry, region_id=7`，提取 patch、front 和 evaluation 后构建约束。对位于该边界上的前沿顶点应用含对称面法向分量的方向，断言结果单位化且点乘平面法向为零；无约束顶点保持原方向单位化。

```cpp
const auto constraints = SymmetryConstraintBuilder{}.build(
    mesh, topology, patch, front, evaluation);
if (!constraints.hasValue())
{
    return 1;
}

const Vector3 raw = Vector3{1.0, 2.0, 3.0}.normalized();
const auto constrained = constraints.value().apply(0, raw);
if (!constrained.hasValue() ||
    std::abs(constrained.value().dot(Vector3{0, 1, 0})) > 1e-12 ||
    std::abs(constrained.value().norm() - 1.0) > 1e-12)
{
    return 2;
}
```

然后把同一 `region_id` 的另一个 Symmetry 面移出参考平面，断言构建失败且错误是 `InvalidSymmetrySurface{region_id, source_face_id}`。这一步禁止用平均法向掩盖弯曲的“对称面”。

- [ ] **Step 2: 写双平面、法向去重和三平面 RED 测试**

分别构造：

- 同一前沿顶点属于两个正交对称 region，约束方向必须沿两平面交线，且与 raw direction 点积非负；
- 两个 region 法向平行或反平行，去重后等价于一个平面；
- 三个 region 法向线性独立，返回 `OverConstrainedGrowthVertex`；
- 将输入面或 patch edge 顺序打乱，约束结果不变。

```cpp
const Vector3 raw_line_hint{1.0, 1.0, 1.0};
const auto line_result = two_planes.value().apply(
    constrained_vertex, raw_line_hint);
if (!line_result.hasValue() ||
    std::abs(line_result.value().dot(n0)) > 1e-12 ||
    std::abs(line_result.value().dot(n1)) > 1e-12 ||
    line_result.value().dot(raw_line_hint) < 0.0)
{
    return 3;
}
```

- [ ] **Step 3: 运行 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_symmetry_constraints_test
```

预期：`symmetry_constraint_builder.hpp` 不存在。

- [ ] **Step 4: 定义平面、顶点约束和错误**

创建 `symmetry_constraints.hpp`：

```cpp
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/growth/growth_direction_error.hpp>

namespace boundary_mesh
{
    struct SymmetryPlane
    {
        std::uint32_t region_id{};
        Point3 point{Point3::Zero()};
        Vector3 unit_normal{Vector3::Zero()};
    };

    struct VertexSymmetryConstraint
    {
        std::size_t front_vertex_index{};
        VertexId source_vertex_id{};
        std::vector<std::size_t> plane_indices;
    };

    class SymmetryConstraintBuilder;

    class SymmetryConstraints
    {
    public:
        const std::vector<SymmetryPlane> &planes() const noexcept;
        const std::vector<VertexSymmetryConstraint> &vertices() const noexcept;

        Result<Vector3, GrowthDirectionError>
        apply(std::size_t front_vertex_index,
              const Vector3 &raw_direction) const;

    private:
        friend class SymmetryConstraintBuilder;
        Scalar angular_tolerance_{};
        Scalar direction_tolerance_{};
        std::uint32_t layer_{};
        std::vector<SymmetryPlane> planes_;
        std::vector<VertexSymmetryConstraint> vertices_;
    };
}
```

在 Task 6 的 `growth_direction_error.hpp` 中加入对称约束错误：

```cpp
struct SymmetryInputMismatch {};
struct InvalidSymmetrySurface
{
    std::uint32_t region_id{};
    SurfaceFaceId source_face_id{};
};
struct OverConstrainedGrowthVertex
{
    std::size_t front_vertex_index{};
    VertexId source_vertex_id{};
    std::uint32_t layer{};
};
struct UndefinedConstrainedDirection
{
    std::size_t front_vertex_index{};
    VertexId source_vertex_id{};
    std::uint32_t layer{};
};
using GrowthDirectionError = std::variant<
    DirectionInputMismatch,
    DirectionCornerFailure,
    UndefinedGrowthDirection,
    SymmetryInputMismatch,
    InvalidSymmetrySurface,
    OverConstrainedGrowthVertex,
    UndefinedConstrainedDirection>;
```

创建 builder 接口：

```cpp
class SymmetryConstraintBuilder
{
public:
    Result<SymmetryConstraints, GrowthDirectionError>
    build(
        const SurfaceMesh &mesh,
        const SurfaceTopology &topology,
        const GrowthPatch &patch,
        const GrowthFront &front,
        const FrontEvaluation &evaluation) const;
};
```

- [ ] **Step 5: 验证 Symmetry region 是真实平面**

按 `region_id` 升序建立 region。对于每个被 Patch 对称边引用的 region：

1. 取该 region 最小 `SurfaceFaceId` 作为参考面；逐面调用 Task 2 的 Triangle/Quad 算法。
2. 参考点取参考面中心，参考法向取其单位法向。
3. 同一 region 所有面法向必须满足 `abs(dot(reference, current)) >= 1-angular_tolerance`。
4. 同一 region 所有面顶点到参考平面的距离必须不超过 `evaluation.effective_length_tolerance`。
5. 任一条件失败返回 `InvalidSymmetrySurface`，错误携带 region 和第一个失败面 ID。

`angular_tolerance` 固定为 `max(1e-12, evaluation.effective_length_tolerance / evaluation.characteristic_length)`。所有 region、面、顶点均按 ID 升序处理。

- [ ] **Step 6: 建立顶点到独立平面的约束**

从每条 `PatchEdgeKind::SymmetryBoundary` 的 `source_edge_id` 读取完整 topology 边端点，并找到其非 Wall 相邻面的 Symmetry `region_id`。通过 `front.source_vertex_ids` 映射到当前局部顶点。

每个顶点的平面按 `region_id` 排序。若两个单位法向满足：

```text
abs(dot(n0, n1)) >= 1 - angular_tolerance
```

则视为同一独立约束，只保留 region_id 较小者。保留三个线性独立法向时立即返回 `OverConstrainedGrowthVertex`。禁止依赖 `unordered_map` 的遍历顺序。

独立性使用确定性的增量 Gram-Schmidt 判断：先按 region_id 排序；对候选法向减去已保留正交基上的全部投影；残量模长不大于 `angular_tolerance` 时，该约束已包含在现有法向张成空间中，只记录为冗余而不增加秩；残量足够大时归一化并加入基。秩从 2 增至 3 的那个候选立即触发过约束错误。这样“三个互不平行但线性相关”的法向仍正确视为秩 2。

- [ ] **Step 7: 实现确定性 apply**

先检查 raw direction 有限且模长大于 `direction_tolerance_`。然后：

```cpp
// 零个独立平面
return normalize(raw_direction);

// 一个独立平面
Vector3 value = raw_direction - raw_direction.dot(n0) * n0;
return normalize_or_error(value);

// 两个独立平面
Vector3 line = n0.cross(n1);
line.normalize();
if (line.dot(raw_direction) < 0.0)
{
    line = -line;
}
return line;
```

两个平面不采用“先投影 A 再投影 B”，因此交换输入顺序不改变允许子空间。投影或交线无法稳定归一化时返回 `UndefinedConstrainedDirection`。

- [ ] **Step 8: GREEN、全回归并提交**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
git add CMakeLists.txt tests/CMakeLists.txt include/boundary_mesh/growth/growth_direction_error.hpp include/boundary_mesh/growth/symmetry_constraints.hpp include/boundary_mesh/growth/symmetry_constraint_builder.hpp src/growth/symmetry_constraint_builder.cpp tests/unit/growth/symmetry_constraints_test.cpp
git diff --cached --check
git commit -m "feat: constrain growth directions on symmetry planes"
```

---

### Task 8: 下一层前沿重新计算集成测试

**Files:**

- Create: `tests/integration/growth_front_pipeline_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**

- Consumes: Task 1–7 的公开接口
- Verifies: 人工移动后的 layer 1 会重新计算，源映射保持，中间层退化错误准确
- 不新增生产代码，不正式生成下一层节点

- [ ] **Step 1: 注册集成测试并写 layer 0 基线**

```cmake
add_executable(
    boundary_mesh_growth_front_pipeline_test
    integration/growth_front_pipeline_test.cpp)
target_link_libraries(
    boundary_mesh_growth_front_pipeline_test
    PRIVATE BoundaryMesh::BoundaryLayer)
add_test(
    NAME boundary_mesh_growth_front_pipeline_test
    COMMAND boundary_mesh_growth_front_pipeline_test)
```

测试从封闭、方向一致、同时含 Wall/Symmetry/Farfield 的三棱柱开始，只通过公开接口依次完成：

```cpp
const auto topology = SurfaceTopologyBuilder{}.build(mesh);
const auto patch = GrowthPatchBuilder{}.build(mesh, topology.value());
const auto layer0 = GrowthFrontBuilder{}.buildInitial(mesh, patch.value());
const auto evaluation0 = FrontEvaluator{}.evaluate(layer0.value());
const auto directions0 = computeGrowthDirections(
    layer0.value(), evaluation0.value());
const auto constraints0 = SymmetryConstraintBuilder{}.build(
    mesh, topology.value(), patch.value(), layer0.value(), evaluation0.value());
```

每一步先检查 `hasValue()` 再读取 `value()`。保存第一个前沿面的 layer 0 面积、法向以及两份 source 映射。

- [ ] **Step 2: 人工移动出 layer 1 并验证重算**

复制 layer 0 front，只在测试中人工修改当前坐标并设置 layer：

```cpp
GrowthFront layer1 = layer0.value();
layer1.layer = 1;
layer1.vertices[1].z() += 0.20;
layer1.vertices[2].z() += 0.10;

const auto evaluation1 = FrontEvaluator{}.evaluate(layer1);
const auto directions1 = computeGrowthDirections(layer1, evaluation1.value());
```

断言：

```cpp
if (!evaluation1.hasValue() || !directions1.hasValue())
{
    return 10;
}
if (evaluation1.value().layer != 1 ||
    evaluation1.value().faces[0].value.area == layer0_area ||
    (evaluation1.value().faces[0].value.unit_normal - layer0_normal).norm()
        <= 1e-12)
{
    return 11;
}
if (layer1.source_vertex_ids != layer0.value().source_vertex_ids ||
    layer1.source_face_ids != layer0.value().source_face_ids)
{
    return 12;
}
```

这证明调用对象是整个当前 `GrowthFront`，并且面积、法向、方向都来自 layer 1 当前坐标，不是 layer 0 缓存。

- [ ] **Step 3: 验证 layer 1 中间退化诊断**

```cpp
GrowthFront collapsed = layer1;
const Triangle &first = std::get<Triangle>(collapsed.faces[0]);
collapsed.vertices[first.vertex_ids[1]] =
    collapsed.vertices[first.vertex_ids[0]];

const auto failure = FrontEvaluator{}.evaluate(collapsed);
const auto *error = failure.hasValue()
    ? nullptr
    : std::get_if<DegenerateFrontFace>(&failure.error());
if (error == nullptr ||
    error->layer != 1 ||
    error->front_face_index != 0 ||
    error->source_face_id != collapsed.source_face_ids[0])
{
    return 13;
}
```

错误不得退回 layer 0，不得丢失 source face ID，也不得返回部分 `FrontEvaluation`。

- [ ] **Step 4: 运行 RED/GREEN 和完整 clean build**

该 Task 只增加集成测试。首次注册但未写 `main()` 时应得到链接器缺少 `main` 的 RED；完成 Step 1–3 后执行：

```powershell
cmake --build build --config Debug --target boundary_mesh_growth_front_pipeline_test
ctest --test-dir build -C Debug -R boundary_mesh_growth_front_pipeline_test --output-on-failure
cmake --build build --config Debug --clean-first
ctest --test-dir build -C Debug --output-on-failure
git diff --check
```

预期：集成测试与全部既有测试通过，`git diff --check` 无输出。

- [ ] **Step 5: 提交 Task 8**

```powershell
git add tests/CMakeLists.txt tests/integration/growth_front_pipeline_test.cpp
git diff --cached --check
git commit -m "test: verify dynamic growth front pipeline"
git status --short --branch
```

预期：分支干净。阶段 03 到此结束；后续规则生长阶段才生成候选节点和 Prism/Hexa。

---

## Final Verification Checklist

- [ ] `SurfaceTopologyBuilder` 拒绝全部输入顶点中的 NaN/Infinity。
- [ ] `Surface` 函数只计算单个 Triangle/Quad，不接收完整网格。
- [ ] `FrontEvaluator` 遍历整个当前前沿并逐面调用 `Surface`。
- [ ] `GrowthPatch` 允许开放边界，输出顺序确定。
- [ ] `GrowthFront` 使用紧凑编号并保存显式源实体映射。
- [ ] layer 1 坐标变化后面积、法向和节点方向确实重算。
- [ ] 退化错误包含当前实体、源实体和 layer。
- [ ] 节点方向只使用当前前沿局部关联面。
- [ ] 单/双对称平面约束与输入顺序无关，共线法向会去重。
- [ ] 非平面 Symmetry region 和三个独立约束被明确拒绝。
- [ ] 阶段 03 没有创建新层节点或体单元。
- [ ] 全部新增源文件含中文用途、约束、不变量和返回含义注释。
- [ ] clean build、全部 CTest、`git diff --check` 通过。
