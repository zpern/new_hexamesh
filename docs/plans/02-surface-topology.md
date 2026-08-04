# SurfaceTopology Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 为封闭、流形、方向一致的混合三角形/四边形 `SurfaceMesh` 构建确定性的只读 `SurfaceTopology`。

**Architecture:** 构建阶段按面和局部边顺序扫描，使用规范端点哈希表查找共享边并分配稳定 `EdgeId`；成功后只保留稠密邻接数组。构建器通过 `Result<SurfaceTopology, SurfaceTopologyError>` 返回完整拓扑或首个确定性错误，不修改输入，也不暴露内部哈希结构。

**Tech Stack:** C++17、Eigen、CMake 3.20+、CTest、MSVC Debug 配置。

## Global Constraints

- 公共 C++ 命名空间固定为 `boundary_mesh`。
- 公共头文件前缀固定为 `<boundary_mesh/...>`。
- CMake 公共别名保持 `BoundaryMesh::Core`。
- `SurfaceMesh` 必须包含 Wall、Symmetry、Farfield 的完整封闭表面。
- `SurfaceTopology` 是不可变快照，不支持增删表面实体。
- 拓扑构建只验证离散连接关系，不计算面积、法向、夹角、自相交或碰撞。
- `EdgeId` 必须由面 ID 和局部边扫描顺序确定，不能由哈希表遍历顺序确定。
- 可预期的输入错误通过 `Result` 返回；库代码不直接输出日志。
- PLY 读取和 `2dot5_cf_gmsh_recombine.ply` 大型案例不属于本计划。
- 每个任务严格执行 RED、GREEN、REFACTOR，并在用户提供实际构建与测试输出后进入下一任务。

---

## File Map

```text
include/boundary_mesh/core/result.hpp
    C++17 通用 Result<T, E>

include/boundary_mesh/mesh/mesh_surface_topology.hpp
    Edge、面邻接类型和 SurfaceTopology 只读快照

include/boundary_mesh/mesh/mesh_surface_topology_error.hpp
    显式拓扑错误结构和 SurfaceTopologyError 变体

include/boundary_mesh/mesh/mesh_surface_topology_builder.hpp
    无状态 SurfaceTopologyBuilder 公开接口

src/mesh/surface_topology_builder.cpp
    输入验证、稳定 EdgeId、邻接构建和封闭性检查

tests/unit/result_test.cpp
    Result 成功、失败和错误访问测试

tests/unit/surface_topology_types_test.cpp
    公开拓扑类型和只读接口编译测试

tests/unit/surface_topology_validation_test.cpp
    基础输入验证错误测试

tests/unit/surface_topology_builder_test.cpp
    封闭三棱柱混合拓扑成功测试

tests/unit/surface_topology_edge_error_test.cpp
    非流形、方向错误和开放边测试
```

---

### Task 1: C++17 `Result<T, E>`

**Files:**

- Create: `include/boundary_mesh/core/result.hpp`
- Create: `tests/unit/result_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**

- Consumes: C++17 `std::variant`、`std::logic_error`。
- Produces: `Result<T, E>::success(T)`、`Result<T, E>::failure(E)`、`hasValue()`、`value()`、`error()`。

- [ ] **Step 1: 编写失败测试**

创建 `tests/unit/result_test.cpp`：

```cpp
#include <stdexcept>
#include <string>

#include <boundary_mesh/core/result.hpp>

int main()
{
    using boundary_mesh::Result;

    auto success = Result<int, std::string>::success(42);
    if (!success.hasValue())
    {
        return 1;
    }
    if (success.value() != 42)
    {
        return 2;
    }

    bool success_error_threw = false;
    try
    {
        (void)success.error();
    }
    catch (const std::logic_error&)
    {
        success_error_threw = true;
    }
    if (!success_error_threw)
    {
        return 3;
    }

    auto failure =
        Result<int, std::string>::failure("invalid input");
    if (failure.hasValue())
    {
        return 4;
    }
    if (failure.error() != "invalid input")
    {
        return 5;
    }

    bool failure_value_threw = false;
    try
    {
        (void)failure.value();
    }
    catch (const std::logic_error&)
    {
        failure_value_threw = true;
    }
    if (!failure_value_threw)
    {
        return 6;
    }

    const auto const_success =
        Result<int, std::string>::success(7);
    if (const_success.value() != 7)
    {
        return 7;
    }

    return 0;
}
```

在 `tests/CMakeLists.txt` 末尾添加：

```cmake
# 4
add_executable(boundary_mesh_result_test unit/result_test.cpp)
target_link_libraries(boundary_mesh_result_test PRIVATE BoundaryMesh::Core)
add_test(NAME boundary_mesh_result_test COMMAND boundary_mesh_result_test)
```

- [ ] **Step 2: 运行 RED 测试**

```powershell
cmake --build build --config Debug
```

预期：编译失败，提示无法找到 `boundary_mesh/core/result.hpp`。

- [ ] **Step 3: 编写最小实现**

创建 `include/boundary_mesh/core/result.hpp`：

```cpp
#pragma once

#include <cstddef>
#include <stdexcept>
#include <utility>
#include <variant>

namespace boundary_mesh
{
    template<class T, class E>
    class Result
    {
    public:
        static Result success(T value)
        {
            return Result{
                std::in_place_index<0>,
                std::move(value)};
        }

        static Result failure(E error)
        {
            return Result{
                std::in_place_index<1>,
                std::move(error)};
        }

        bool hasValue() const noexcept
        {
            return storage_.index() == 0;
        }

        T& value()
        {
            if (!hasValue())
            {
                throw std::logic_error(
                    "Result does not contain a value");
            }
            return std::get<0>(storage_);
        }

        const T& value() const
        {
            if (!hasValue())
            {
                throw std::logic_error(
                    "Result does not contain a value");
            }
            return std::get<0>(storage_);
        }

        E& error()
        {
            if (hasValue())
            {
                throw std::logic_error(
                    "Result does not contain an error");
            }
            return std::get<1>(storage_);
        }

        const E& error() const
        {
            if (hasValue())
            {
                throw std::logic_error(
                    "Result does not contain an error");
            }
            return std::get<1>(storage_);
        }

    private:
        template<std::size_t Index, class U>
        Result(std::in_place_index_t<Index> index, U&& value)
            : storage_(index, std::forward<U>(value))
        {
        }

        std::variant<T, E> storage_;
    };
}
```

- [ ] **Step 4: 验证 GREEN**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

预期：4/4 测试通过。

- [ ] **Step 5: 提交**

```powershell
git add include/boundary_mesh/core/result.hpp `
        tests/CMakeLists.txt `
        tests/unit/result_test.cpp
git diff --cached --check
git commit -m "feat: add result value type"
```

---

### Task 2: 只读拓扑与显式错误类型

**Files:**

- Create: `include/boundary_mesh/mesh/mesh_surface_topology.hpp`
- Create: `include/boundary_mesh/mesh/mesh_surface_topology_error.hpp`
- Create: `tests/unit/surface_topology_types_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**

- Consumes: `VertexId`、`EdgeId`、`SurfaceFaceId`。
- Produces: `Edge`、`FaceEdgeIds`、`FaceNeighborIds`、`SurfaceTopology`、`SurfaceTopologyError`。

- [ ] **Step 1: 编写失败测试**

创建 `tests/unit/surface_topology_types_test.cpp`：

```cpp
#include <array>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <boundary_mesh/mesh/mesh_surface_topology.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_error.hpp>

int main()
{
    using namespace boundary_mesh;

    static_assert(
        !std::is_default_constructible_v<SurfaceTopology>);
    static_assert(std::is_same_v<
        decltype(std::declval<const SurfaceTopology&>().edges()),
        const std::vector<Edge>&>);
    static_assert(std::is_same_v<
        decltype(
            std::declval<const SurfaceTopology&>().edgeFaces()),
        const std::vector<EdgeFaceIds>&>);
    static_assert(std::is_same_v<
        decltype(
            std::declval<const SurfaceTopology&>().faceEdges()),
        const std::vector<FaceEdgeIds>&>);
    static_assert(std::is_same_v<
        decltype(
            std::declval<const SurfaceTopology&>().faceNeighbors()),
        const std::vector<FaceNeighborIds>&>);
    static_assert(std::is_same_v<
        decltype(
            std::declval<const SurfaceTopology&>().vertexFaces()),
        const std::vector<std::vector<SurfaceFaceId>>&>);

    const Edge edge{{VertexId{2}, VertexId{5}}};
    if (edge.vertex_ids !=
        std::array<VertexId, 2>{VertexId{2}, VertexId{5}})
    {
        return 1;
    }

    const FaceEdgeIds face_edges =
        TriangleEdgeIds{EdgeId{0}, EdgeId{1}, EdgeId{2}};
    if (!std::holds_alternative<TriangleEdgeIds>(face_edges))
    {
        return 2;
    }

    const SurfaceTopologyError error = BoundaryEdge{
        {VertexId{1}, VertexId{4}}, SurfaceFaceId{7}};
    if (!std::holds_alternative<BoundaryEdge>(error))
    {
        return 3;
    }

    return 0;
}
```

在 `tests/CMakeLists.txt` 末尾添加：

```cmake
# 5
add_executable(
    boundary_mesh_surface_topology_types_test
    unit/surface_topology_types_test.cpp
)
target_link_libraries(
    boundary_mesh_surface_topology_types_test
    PRIVATE BoundaryMesh::Core
)
add_test(
    NAME boundary_mesh_surface_topology_types_test
    COMMAND boundary_mesh_surface_topology_types_test
)
```

- [ ] **Step 2: 运行 RED 测试**

```powershell
cmake --build build --config Debug
```

预期：编译失败，提示找不到 `surface_topology.hpp` 或 `surface_topology_error.hpp`。

- [ ] **Step 3: 定义错误类型**

创建 `include/boundary_mesh/mesh/mesh_surface_topology_error.hpp`：

```cpp
#pragma once

#include <array>
#include <cstddef>
#include <variant>

#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh
{
    struct EmptySurface
    {
    };

    struct FaceTagCountMismatch
    {
        std::size_t face_count{};
        std::size_t face_tag_count{};
    };

    struct InvalidVertexReference
    {
        SurfaceFaceId face_id{};
        VertexId vertex_id{};
    };

    struct DegenerateFace
    {
        SurfaceFaceId face_id{};
    };

    struct DuplicateFace
    {
        SurfaceFaceId first_face_id{};
        SurfaceFaceId duplicate_face_id{};
    };

    struct BoundaryEdge
    {
        std::array<VertexId, 2> edge_vertices{};
        SurfaceFaceId face_id{};
    };

    struct NonManifoldEdge
    {
        std::array<VertexId, 2> edge_vertices{};
        std::array<SurfaceFaceId, 3> face_ids{};
    };

    struct InconsistentOrientation
    {
        std::array<VertexId, 2> edge_vertices{};
        SurfaceFaceId first_face_id{};
        SurfaceFaceId second_face_id{};
    };

    using SurfaceTopologyError = std::variant<
        EmptySurface,
        FaceTagCountMismatch,
        InvalidVertexReference,
        DegenerateFace,
        DuplicateFace,
        BoundaryEdge,
        NonManifoldEdge,
        InconsistentOrientation>;
}
```

- [ ] **Step 4: 定义只读拓扑**

创建 `include/boundary_mesh/mesh/mesh_surface_topology.hpp`：

```cpp
#pragma once

#include <array>
#include <utility>
#include <variant>
#include <vector>

#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh
{
    class SurfaceTopologyBuilder;

    struct Edge
    {
        std::array<VertexId, 2> vertex_ids{};
    };

    using EdgeFaceIds = std::array<SurfaceFaceId, 2>;

    using TriangleEdgeIds = std::array<EdgeId, 3>;
    using QuadEdgeIds = std::array<EdgeId, 4>;
    using FaceEdgeIds =
        std::variant<TriangleEdgeIds, QuadEdgeIds>;

    using TriangleNeighborIds =
        std::array<SurfaceFaceId, 3>;
    using QuadNeighborIds =
        std::array<SurfaceFaceId, 4>;
    using FaceNeighborIds =
        std::variant<TriangleNeighborIds, QuadNeighborIds>;

    class SurfaceTopology
    {
    public:
        const std::vector<Edge>& edges() const noexcept
        {
            return edges_;
        }

        const std::vector<EdgeFaceIds>&
        edgeFaces() const noexcept
        {
            return edge_faces_;
        }

        const std::vector<FaceEdgeIds>&
        faceEdges() const noexcept
        {
            return face_edges_;
        }

        const std::vector<FaceNeighborIds>&
        faceNeighbors() const noexcept
        {
            return face_neighbors_;
        }

        const std::vector<std::vector<SurfaceFaceId>>&
        vertexFaces() const noexcept
        {
            return vertex_faces_;
        }

    private:
        friend class SurfaceTopologyBuilder;

        SurfaceTopology(
            std::vector<Edge> edges,
            std::vector<EdgeFaceIds> edge_faces,
            std::vector<FaceEdgeIds> face_edges,
            std::vector<FaceNeighborIds> face_neighbors,
            std::vector<std::vector<SurfaceFaceId>> vertex_faces)
            : edges_(std::move(edges)),
              edge_faces_(std::move(edge_faces)),
              face_edges_(std::move(face_edges)),
              face_neighbors_(std::move(face_neighbors)),
              vertex_faces_(std::move(vertex_faces))
        {
        }

        std::vector<Edge> edges_;
        std::vector<EdgeFaceIds> edge_faces_;
        std::vector<FaceEdgeIds> face_edges_;
        std::vector<FaceNeighborIds> face_neighbors_;
        std::vector<std::vector<SurfaceFaceId>> vertex_faces_;
    };
}
```

- [ ] **Step 5: 验证 GREEN**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

预期：5/5 测试通过。

- [ ] **Step 6: 提交**

```powershell
git add include/boundary_mesh/mesh/mesh_surface_topology.hpp `
        include/boundary_mesh/mesh/mesh_surface_topology_error.hpp `
        tests/CMakeLists.txt `
        tests/unit/surface_topology_types_test.cpp
git diff --cached --check
git commit -m "feat: define surface topology types"
```

---

### Task 3: 编译库与基础输入验证

**Files:**

- Create: `include/boundary_mesh/mesh/mesh_surface_topology_builder.hpp`
- Create: `src/mesh/surface_topology_builder.cpp`
- Create: `tests/unit/surface_topology_validation_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**

- Consumes: `SurfaceMesh`、`SurfaceTopology`、`SurfaceTopologyError`、`Result<T, E>`。
- Produces: `SurfaceTopologyBuilder::build(const SurfaceMesh&) const`。

- [ ] **Step 1: 编写失败测试**

创建 `tests/unit/surface_topology_validation_test.cpp`：

```cpp
#include <variant>

#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

namespace
{
    using namespace boundary_mesh;

    SurfaceBoundaryTag wallTag()
    {
        return SurfaceBoundaryTag{
            SurfaceBoundaryKind::Wall,
            1};
    }
}

int main()
{
    using namespace boundary_mesh;

    const SurfaceTopologyBuilder builder;

    {
        const SurfaceMesh mesh;
        const auto result = builder.build(mesh);
        if (result.hasValue() ||
            !std::holds_alternative<EmptySurface>(result.error()))
        {
            return 1;
        }
    }

    {
        SurfaceMesh mesh;
        mesh.vertices.resize(3, Point3::Zero());
        mesh.faces.emplace_back(
            Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}});

        const auto result = builder.build(mesh);
        if (result.hasValue())
        {
            return 2;
        }
        const auto* error =
            std::get_if<FaceTagCountMismatch>(&result.error());
        if (error == nullptr || error->face_count != 1 ||
            error->face_tag_count != 0)
        {
            return 3;
        }
    }

    {
        SurfaceMesh mesh;
        mesh.vertices.resize(3, Point3::Zero());
        mesh.faces.emplace_back(
            Triangle{{VertexId{0}, VertexId{1}, VertexId{3}}});
        mesh.face_tags.push_back(wallTag());

        const auto result = builder.build(mesh);
        const auto* error =
            result.hasValue()
                ? nullptr
                : std::get_if<InvalidVertexReference>(
                      &result.error());
        if (error == nullptr ||
            error->face_id != SurfaceFaceId{0} ||
            error->vertex_id != VertexId{3})
        {
            return 4;
        }
    }

    {
        SurfaceMesh mesh;
        mesh.vertices.resize(3, Point3::Zero());
        mesh.faces.emplace_back(
            Triangle{{VertexId{0}, VertexId{1}, VertexId{1}}});
        mesh.face_tags.push_back(wallTag());

        const auto result = builder.build(mesh);
        const auto* error =
            result.hasValue()
                ? nullptr
                : std::get_if<DegenerateFace>(&result.error());
        if (error == nullptr ||
            error->face_id != SurfaceFaceId{0})
        {
            return 5;
        }
    }

    {
        SurfaceMesh mesh;
        mesh.vertices.resize(3, Point3::Zero());
        mesh.faces.emplace_back(
            Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}});
        mesh.faces.emplace_back(
            Triangle{{VertexId{2}, VertexId{1}, VertexId{0}}});
        mesh.face_tags = {wallTag(), wallTag()};

        const auto result = builder.build(mesh);
        const auto* error =
            result.hasValue()
                ? nullptr
                : std::get_if<DuplicateFace>(&result.error());
        if (error == nullptr ||
            error->first_face_id != SurfaceFaceId{0} ||
            error->duplicate_face_id != SurfaceFaceId{1})
        {
            return 6;
        }
    }

    return 0;
}
```

在 `tests/CMakeLists.txt` 末尾添加：

```cmake
# 6
add_executable(
    boundary_mesh_surface_topology_validation_test
    unit/surface_topology_validation_test.cpp
)
target_link_libraries(
    boundary_mesh_surface_topology_validation_test
    PRIVATE BoundaryMesh::Core
)
add_test(
    NAME boundary_mesh_surface_topology_validation_test
    COMMAND boundary_mesh_surface_topology_validation_test
)
```

- [ ] **Step 2: 运行 RED 测试**

```powershell
cmake --build build --config Debug
```

预期：编译失败，提示找不到 `surface_topology_builder.hpp`。

- [ ] **Step 3: 声明构建器**

创建 `include/boundary_mesh/mesh/mesh_surface_topology_builder.hpp`：

```cpp
#pragma once

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/mesh/mesh_surface.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology.hpp>
#include <boundary_mesh/mesh/mesh_surface_topology_error.hpp>

namespace boundary_mesh
{
    class SurfaceTopologyBuilder
    {
    public:
        Result<SurfaceTopology, SurfaceTopologyError>
        build(const SurfaceMesh& mesh) const;
    };
}
```

- [ ] **Step 4: 编写基础验证实现**

创建 `src/mesh/surface_topology_builder.cpp`：

```cpp
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

namespace boundary_mesh
{
    namespace
    {
        struct FaceKey
        {
            std::array<VertexId, 4> sorted_vertex_ids{};
            std::uint8_t vertex_count{};

            bool operator==(const FaceKey& other) const noexcept
            {
                return vertex_count == other.vertex_count &&
                       sorted_vertex_ids ==
                           other.sorted_vertex_ids;
            }
        };

        struct FaceKeyHash
        {
            std::size_t operator()(const FaceKey& key) const noexcept
            {
                std::size_t seed = key.vertex_count;
                for (std::size_t i = 0;
                     i < key.vertex_count;
                     ++i)
                {
                    seed ^= std::hash<VertexId>{}(
                                key.sorted_vertex_ids[i]) +
                            0x9e3779b9U + (seed << 6U) +
                            (seed >> 2U);
                }
                return seed;
            }
        };

        std::vector<VertexId> faceVertexIds(
            const SurfaceFace& face)
        {
            return std::visit(
                [](const auto& value)
                {
                    return std::vector<VertexId>{
                        value.vertex_ids.begin(),
                        value.vertex_ids.end()};
                },
                face);
        }

        FaceKey makeFaceKey(
            const std::vector<VertexId>& vertex_ids)
        {
            FaceKey key;
            key.vertex_count = static_cast<std::uint8_t>(
                vertex_ids.size());
            std::copy(
                vertex_ids.begin(),
                vertex_ids.end(),
                key.sorted_vertex_ids.begin());
            std::sort(
                key.sorted_vertex_ids.begin(),
                key.sorted_vertex_ids.begin() +
                    key.vertex_count);
            return key;
        }
    }

    Result<SurfaceTopology, SurfaceTopologyError>
    SurfaceTopologyBuilder::build(const SurfaceMesh& mesh) const
    {
        using BuildResult =
            Result<SurfaceTopology, SurfaceTopologyError>;

        if (mesh.faces.empty())
        {
            return BuildResult::failure(
                SurfaceTopologyError{EmptySurface{}});
        }

        if (mesh.faces.size() != mesh.face_tags.size())
        {
            return BuildResult::failure(
                SurfaceTopologyError{FaceTagCountMismatch{
                    mesh.faces.size(),
                    mesh.face_tags.size()}});
        }

        std::unordered_map<FaceKey, SurfaceFaceId, FaceKeyHash>
            first_face_by_key;

        for (std::size_t face_index = 0;
             face_index < mesh.faces.size();
             ++face_index)
        {
            const auto face_id =
                static_cast<SurfaceFaceId>(face_index);
            const auto vertex_ids =
                faceVertexIds(mesh.faces[face_index]);

            std::unordered_set<VertexId> unique_vertex_ids;
            for (const VertexId vertex_id : vertex_ids)
            {
                if (static_cast<std::size_t>(vertex_id) >=
                    mesh.vertices.size())
                {
                    return BuildResult::failure(
                        SurfaceTopologyError{
                            InvalidVertexReference{
                                face_id,
                                vertex_id}});
                }

                if (!unique_vertex_ids.insert(vertex_id).second)
                {
                    return BuildResult::failure(
                        SurfaceTopologyError{
                            DegenerateFace{face_id}});
                }
            }

            const FaceKey key = makeFaceKey(vertex_ids);
            const auto insertion =
                first_face_by_key.emplace(key, face_id);
            if (!insertion.second)
            {
                return BuildResult::failure(
                    SurfaceTopologyError{DuplicateFace{
                        insertion.first->second,
                        face_id}});
            }
        }

        return BuildResult::success(SurfaceTopology{
            {},
            {},
            {},
            {},
            std::vector<std::vector<SurfaceFaceId>>(
                mesh.vertices.size())});
    }
}
```

- [ ] **Step 5: 将 Core 转换为编译库**

在根 `CMakeLists.txt` 中用以下内容替换从 `add_library(boundary_mesh_core INTERFACE)` 开始，到现有 `target_link_libraries(...)` 结束的 Core 目标定义：

```cmake
add_library(
    boundary_mesh_core STATIC
    src/mesh/surface_topology_builder.cpp
)
add_library(BoundaryMesh::Core ALIAS boundary_mesh_core)

target_compile_features(boundary_mesh_core PUBLIC cxx_std_17)

target_include_directories(
    boundary_mesh_core
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(
    boundary_mesh_core
    PRIVATE
        BoundaryMesh::CompileOptions
    PUBLIC
        Eigen3::Eigen
)
```

- [ ] **Step 6: 验证 GREEN**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

预期：6/6 测试通过，构建产物中出现 Debug 版本的 `boundary_mesh_core` 静态库。

- [ ] **Step 7: 提交**

```powershell
git add CMakeLists.txt `
        include/boundary_mesh/mesh/mesh_surface_topology_builder.hpp `
        src/mesh/surface_topology_builder.cpp `
        tests/CMakeLists.txt `
        tests/unit/surface_topology_validation_test.cpp
git diff --cached --check
git commit -m "feat: validate surface topology input"
```

---

### Task 4: 构建有效混合表面拓扑

**Files:**

- Create: `tests/unit/surface_topology_builder_test.cpp`
- Modify: `src/mesh/surface_topology_builder.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**

- Consumes: 已通过基础验证的封闭混合 `SurfaceMesh`。
- Produces: 稳定 `edges`、`edgeFaces()`、`faceEdges()`、`faceNeighbors()`、`vertexFaces()`。

- [ ] **Step 1: 编写封闭三棱柱失败测试**

创建 `tests/unit/surface_topology_builder_test.cpp`：

```cpp
#include <cstddef>
#include <array>
#include <variant>
#include <vector>

#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

namespace
{
    using namespace boundary_mesh;

    SurfaceMesh makePrismSurface()
    {
        SurfaceMesh mesh;
        mesh.vertices = {
            Point3{0.0, 0.0, 0.0},
            Point3{1.0, 0.0, 0.0},
            Point3{0.0, 1.0, 0.0},
            Point3{0.0, 0.0, 1.0},
            Point3{1.0, 0.0, 1.0},
            Point3{0.0, 1.0, 1.0}};

        mesh.faces = {
            Triangle{{VertexId{0}, VertexId{2}, VertexId{1}}},
            Triangle{{VertexId{3}, VertexId{4}, VertexId{5}}},
            Quad{{VertexId{0}, VertexId{1}, VertexId{4}, VertexId{3}}},
            Quad{{VertexId{1}, VertexId{2}, VertexId{5}, VertexId{4}}},
            Quad{{VertexId{2}, VertexId{0}, VertexId{3}, VertexId{5}}}};

        mesh.face_tags = {
            {SurfaceBoundaryKind::Wall, 1},
            {SurfaceBoundaryKind::Farfield, 2},
            {SurfaceBoundaryKind::Symmetry, 3},
            {SurfaceBoundaryKind::Wall, 1},
            {SurfaceBoundaryKind::Wall, 1}};
        return mesh;
    }
}

int main()
{
    using namespace boundary_mesh;

    const SurfaceMesh mesh = makePrismSurface();
    const auto result = SurfaceTopologyBuilder{}.build(mesh);
    if (!result.hasValue())
    {
        return 1;
    }

    const SurfaceTopology& topology = result.value();
    const std::vector<Edge> expected_edges = {
        Edge{{VertexId{0}, VertexId{2}}},
        Edge{{VertexId{1}, VertexId{2}}},
        Edge{{VertexId{0}, VertexId{1}}},
        Edge{{VertexId{3}, VertexId{4}}},
        Edge{{VertexId{4}, VertexId{5}}},
        Edge{{VertexId{3}, VertexId{5}}},
        Edge{{VertexId{1}, VertexId{4}}},
        Edge{{VertexId{0}, VertexId{3}}},
        Edge{{VertexId{2}, VertexId{5}}}};

    if (topology.edges().size() != expected_edges.size())
    {
        return 2;
    }
    for (std::size_t i = 0; i < expected_edges.size(); ++i)
    {
        if (topology.edges()[i].vertex_ids !=
            expected_edges[i].vertex_ids)
        {
            return 3;
        }
    }

    const std::vector<EdgeFaceIds> expected_edge_faces = {
        {SurfaceFaceId{0}, SurfaceFaceId{4}},
        {SurfaceFaceId{0}, SurfaceFaceId{3}},
        {SurfaceFaceId{0}, SurfaceFaceId{2}},
        {SurfaceFaceId{1}, SurfaceFaceId{2}},
        {SurfaceFaceId{1}, SurfaceFaceId{3}},
        {SurfaceFaceId{1}, SurfaceFaceId{4}},
        {SurfaceFaceId{2}, SurfaceFaceId{3}},
        {SurfaceFaceId{2}, SurfaceFaceId{4}},
        {SurfaceFaceId{3}, SurfaceFaceId{4}}};
    if (topology.edgeFaces() != expected_edge_faces)
    {
        return 4;
    }

    const auto* face0_edges =
        std::get_if<TriangleEdgeIds>(&topology.faceEdges()[0]);
    const auto* face2_edges =
        std::get_if<QuadEdgeIds>(&topology.faceEdges()[2]);
    if (face0_edges == nullptr ||
        *face0_edges != TriangleEdgeIds{
            EdgeId{0}, EdgeId{1}, EdgeId{2}})
    {
        return 5;
    }
    if (face2_edges == nullptr ||
        *face2_edges != QuadEdgeIds{
            EdgeId{2}, EdgeId{6}, EdgeId{3}, EdgeId{7}})
    {
        return 6;
    }

    const auto* face0_neighbors =
        std::get_if<TriangleNeighborIds>(
            &topology.faceNeighbors()[0]);
    const auto* face2_neighbors =
        std::get_if<QuadNeighborIds>(
            &topology.faceNeighbors()[2]);
    if (face0_neighbors == nullptr ||
        *face0_neighbors != TriangleNeighborIds{
            SurfaceFaceId{4},
            SurfaceFaceId{3},
            SurfaceFaceId{2}})
    {
        return 7;
    }
    if (face2_neighbors == nullptr ||
        *face2_neighbors != QuadNeighborIds{
            SurfaceFaceId{0},
            SurfaceFaceId{3},
            SurfaceFaceId{1},
            SurfaceFaceId{4}})
    {
        return 8;
    }

    const std::vector<std::vector<SurfaceFaceId>>
        expected_vertex_faces = {
            {SurfaceFaceId{0}, SurfaceFaceId{2}, SurfaceFaceId{4}},
            {SurfaceFaceId{0}, SurfaceFaceId{2}, SurfaceFaceId{3}},
            {SurfaceFaceId{0}, SurfaceFaceId{3}, SurfaceFaceId{4}},
            {SurfaceFaceId{1}, SurfaceFaceId{2}, SurfaceFaceId{4}},
            {SurfaceFaceId{1}, SurfaceFaceId{2}, SurfaceFaceId{3}},
            {SurfaceFaceId{1}, SurfaceFaceId{3}, SurfaceFaceId{4}}};
    if (topology.vertexFaces() != expected_vertex_faces)
    {
        return 9;
    }

    if (mesh.faces.size() != 5 || mesh.face_tags.size() != 5)
    {
        return 10;
    }

    return 0;
}
```

在 `tests/CMakeLists.txt` 末尾添加：

```cmake
# 7
add_executable(
    boundary_mesh_surface_topology_builder_test
    unit/surface_topology_builder_test.cpp
)
target_link_libraries(
    boundary_mesh_surface_topology_builder_test
    PRIVATE BoundaryMesh::Core
)
add_test(
    NAME boundary_mesh_surface_topology_builder_test
    COMMAND boundary_mesh_surface_topology_builder_test
)
```

- [ ] **Step 2: 运行 RED 测试**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

预期：新增测试失败并返回 `2`，因为当前成功结果的 `edges()` 为空。

- [ ] **Step 3: 用完整邻接实现替换构建器源码**

用以下内容完整替换 `src/mesh/surface_topology_builder.cpp`：

```cpp
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

namespace boundary_mesh
{
    namespace
    {
        struct FaceKey
        {
            std::array<VertexId, 4> sorted_vertex_ids{};
            std::uint8_t vertex_count{};

            bool operator==(const FaceKey& other) const noexcept
            {
                return vertex_count == other.vertex_count &&
                       sorted_vertex_ids == other.sorted_vertex_ids;
            }
        };

        struct FaceKeyHash
        {
            std::size_t operator()(const FaceKey& key) const noexcept
            {
                std::size_t seed = key.vertex_count;
                for (std::size_t i = 0; i < key.vertex_count; ++i)
                {
                    seed ^= std::hash<VertexId>{}(
                                key.sorted_vertex_ids[i]) +
                            0x9e3779b9U + (seed << 6U) +
                            (seed >> 2U);
                }
                return seed;
            }
        };

        struct EdgeKey
        {
            VertexId first{};
            VertexId second{};

            bool operator==(const EdgeKey& other) const noexcept
            {
                return first == other.first &&
                       second == other.second;
            }
        };

        struct EdgeKeyHash
        {
            std::size_t operator()(const EdgeKey& key) const noexcept
            {
                std::size_t seed = std::hash<VertexId>{}(key.first);
                seed ^= std::hash<VertexId>{}(key.second) +
                        0x9e3779b9U + (seed << 6U) +
                        (seed >> 2U);
                return seed;
            }
        };

        EdgeKey makeEdgeKey(VertexId first, VertexId second)
        {
            if (first < second)
            {
                return EdgeKey{first, second};
            }
            return EdgeKey{second, first};
        }

        int edgeDirection(VertexId first, VertexId second)
        {
            return first < second ? 1 : -1;
        }

        std::vector<VertexId> faceVertexIds(
            const SurfaceFace& face)
        {
            return std::visit(
                [](const auto& value)
                {
                    return std::vector<VertexId>{
                        value.vertex_ids.begin(),
                        value.vertex_ids.end()};
                },
                face);
        }

        FaceKey makeFaceKey(
            const std::vector<VertexId>& vertex_ids)
        {
            FaceKey key;
            key.vertex_count = static_cast<std::uint8_t>(
                vertex_ids.size());
            std::copy(
                vertex_ids.begin(),
                vertex_ids.end(),
                key.sorted_vertex_ids.begin());
            std::sort(
                key.sorted_vertex_ids.begin(),
                key.sorted_vertex_ids.begin() + key.vertex_count);
            return key;
        }

        EdgeId appendEdge(
            VertexId first,
            VertexId second,
            SurfaceFaceId face_id,
            std::unordered_map<EdgeKey, EdgeId, EdgeKeyHash>&
                edge_ids,
            std::vector<Edge>& edges,
            std::vector<std::vector<SurfaceFaceId>>&
                incident_faces,
            std::vector<std::vector<int>>& incident_directions)
        {
            const EdgeKey key = makeEdgeKey(first, second);
            const auto found = edge_ids.find(key);
            if (found == edge_ids.end())
            {
                const auto edge_id =
                    static_cast<EdgeId>(edges.size());
                edge_ids.emplace(key, edge_id);
                edges.push_back(Edge{{key.first, key.second}});
                incident_faces.push_back({face_id});
                incident_directions.push_back(
                    {edgeDirection(first, second)});
                return edge_id;
            }

            const EdgeId edge_id = found->second;
            const auto edge_index =
                static_cast<std::size_t>(edge_id);
            incident_faces[edge_index].push_back(face_id);
            incident_directions[edge_index].push_back(
                edgeDirection(first, second));
            return edge_id;
        }

        template<std::size_t Count>
        std::array<EdgeId, Count> appendFace(
            const std::array<VertexId, Count>& vertex_ids,
            SurfaceFaceId face_id,
            std::unordered_map<EdgeKey, EdgeId, EdgeKeyHash>&
                edge_ids,
            std::vector<Edge>& edges,
            std::vector<std::vector<SurfaceFaceId>>&
                incident_faces,
            std::vector<std::vector<int>>& incident_directions,
            std::vector<std::vector<SurfaceFaceId>>& vertex_faces)
        {
            std::array<EdgeId, Count> face_edge_ids{};
            for (std::size_t i = 0; i < Count; ++i)
            {
                vertex_faces[static_cast<std::size_t>(
                    vertex_ids[i])]
                    .push_back(face_id);
                face_edge_ids[i] = appendEdge(
                    vertex_ids[i],
                    vertex_ids[(i + 1) % Count],
                    face_id,
                    edge_ids,
                    edges,
                    incident_faces,
                    incident_directions);
            }
            return face_edge_ids;
        }

        template<std::size_t Count>
        std::array<SurfaceFaceId, Count> makeNeighbors(
            SurfaceFaceId face_id,
            const std::array<EdgeId, Count>& face_edge_ids,
            const std::vector<EdgeFaceIds>& edge_faces)
        {
            std::array<SurfaceFaceId, Count> neighbors{};
            for (std::size_t i = 0; i < Count; ++i)
            {
                const auto& faces = edge_faces[
                    static_cast<std::size_t>(face_edge_ids[i])];
                neighbors[i] =
                    faces[0] == face_id ? faces[1] : faces[0];
            }
            return neighbors;
        }
    }

    Result<SurfaceTopology, SurfaceTopologyError>
    SurfaceTopologyBuilder::build(const SurfaceMesh& mesh) const
    {
        using BuildResult =
            Result<SurfaceTopology, SurfaceTopologyError>;

        if (mesh.faces.empty())
        {
            return BuildResult::failure(
                SurfaceTopologyError{EmptySurface{}});
        }
        if (mesh.faces.size() != mesh.face_tags.size())
        {
            return BuildResult::failure(
                SurfaceTopologyError{FaceTagCountMismatch{
                    mesh.faces.size(), mesh.face_tags.size()}});
        }

        std::unordered_map<FaceKey, SurfaceFaceId, FaceKeyHash>
            first_face_by_key;
        for (std::size_t face_index = 0;
             face_index < mesh.faces.size();
             ++face_index)
        {
            const auto face_id =
                static_cast<SurfaceFaceId>(face_index);
            const auto vertex_ids =
                faceVertexIds(mesh.faces[face_index]);
            std::unordered_set<VertexId> unique_vertex_ids;
            for (const VertexId vertex_id : vertex_ids)
            {
                if (static_cast<std::size_t>(vertex_id) >=
                    mesh.vertices.size())
                {
                    return BuildResult::failure(
                        SurfaceTopologyError{
                            InvalidVertexReference{
                                face_id, vertex_id}});
                }
                if (!unique_vertex_ids.insert(vertex_id).second)
                {
                    return BuildResult::failure(
                        SurfaceTopologyError{
                            DegenerateFace{face_id}});
                }
            }

            const auto insertion = first_face_by_key.emplace(
                makeFaceKey(vertex_ids), face_id);
            if (!insertion.second)
            {
                return BuildResult::failure(
                    SurfaceTopologyError{DuplicateFace{
                        insertion.first->second, face_id}});
            }
        }

        std::unordered_map<EdgeKey, EdgeId, EdgeKeyHash> edge_ids;
        std::vector<Edge> edges;
        std::vector<std::vector<SurfaceFaceId>> incident_faces;
        std::vector<std::vector<int>> incident_directions;
        std::vector<FaceEdgeIds> face_edges;
        std::vector<std::vector<SurfaceFaceId>> vertex_faces(
            mesh.vertices.size());

        face_edges.reserve(mesh.faces.size());
        for (std::size_t face_index = 0;
             face_index < mesh.faces.size();
             ++face_index)
        {
            const auto face_id =
                static_cast<SurfaceFaceId>(face_index);
            face_edges.push_back(std::visit(
                [&](const auto& face) -> FaceEdgeIds
                {
                    return appendFace(
                        face.vertex_ids,
                        face_id,
                        edge_ids,
                        edges,
                        incident_faces,
                        incident_directions,
                        vertex_faces);
                },
                mesh.faces[face_index]));
        }

        std::vector<EdgeFaceIds> edge_faces(edges.size());
        for (std::size_t edge_index = 0;
             edge_index < edges.size();
             ++edge_index)
        {
            if (!incident_faces[edge_index].empty())
            {
                edge_faces[edge_index][0] =
                    incident_faces[edge_index][0];
            }
            if (incident_faces[edge_index].size() >= 2)
            {
                edge_faces[edge_index][1] =
                    incident_faces[edge_index][1];
            }
        }

        std::vector<FaceNeighborIds> face_neighbors;
        face_neighbors.reserve(face_edges.size());
        for (std::size_t face_index = 0;
             face_index < face_edges.size();
             ++face_index)
        {
            const auto face_id =
                static_cast<SurfaceFaceId>(face_index);
            face_neighbors.push_back(std::visit(
                [&](const auto& ids) -> FaceNeighborIds
                {
                    return makeNeighbors(
                        face_id, ids, edge_faces);
                },
                face_edges[face_index]));
        }

        return BuildResult::success(SurfaceTopology{
            std::move(edges),
            std::move(edge_faces),
            std::move(face_edges),
            std::move(face_neighbors),
            std::move(vertex_faces)});
    }
}
```

- [ ] **Step 4: 验证 GREEN**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

预期：7/7 测试通过。

- [ ] **Step 5: 提交**

```powershell
git add src/mesh/surface_topology_builder.cpp `
        tests/CMakeLists.txt `
        tests/unit/surface_topology_builder_test.cpp
git diff --cached --check
git commit -m "feat: build mixed surface topology"
```

---

### Task 5: 非流形边与方向错误

**Files:**

- Create: `tests/unit/surface_topology_edge_error_test.cpp`
- Modify: `src/mesh/surface_topology_builder.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**

- Consumes: 构建期规范边、相邻面和局部边方向。
- Produces: `NonManifoldEdge`、`InconsistentOrientation`。

- [ ] **Step 1: 编写失败测试**

创建 `tests/unit/surface_topology_edge_error_test.cpp`：

```cpp
#include <array>
#include <variant>

#include <boundary_mesh/mesh/mesh_surface_topology_builder.hpp>

namespace
{
    using namespace boundary_mesh;

    SurfaceBoundaryTag wallTag()
    {
        return {SurfaceBoundaryKind::Wall, 1};
    }
}

int main()
{
    using namespace boundary_mesh;

    const SurfaceTopologyBuilder builder;

    {
        SurfaceMesh mesh;
        mesh.vertices.resize(5, Point3::Zero());
        mesh.faces = {
            Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}},
            Triangle{{VertexId{1}, VertexId{0}, VertexId{3}}},
            Triangle{{VertexId{0}, VertexId{1}, VertexId{4}}}};
        mesh.face_tags = {wallTag(), wallTag(), wallTag()};

        const auto result = builder.build(mesh);
        const auto* error =
            result.hasValue()
                ? nullptr
                : std::get_if<NonManifoldEdge>(&result.error());
        if (error == nullptr)
        {
            return 1;
        }
        if (error->edge_vertices !=
                std::array<VertexId, 2>{VertexId{0}, VertexId{1}} ||
            error->face_ids !=
                std::array<SurfaceFaceId, 3>{
                    SurfaceFaceId{0},
                    SurfaceFaceId{1},
                    SurfaceFaceId{2}})
        {
            return 2;
        }
    }

    {
        SurfaceMesh mesh;
        mesh.vertices.resize(4, Point3::Zero());
        mesh.faces = {
            Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}},
            Triangle{{VertexId{0}, VertexId{1}, VertexId{3}}}};
        mesh.face_tags = {wallTag(), wallTag()};

        const auto result = builder.build(mesh);
        const auto* error =
            result.hasValue()
                ? nullptr
                : std::get_if<InconsistentOrientation>(
                      &result.error());
        if (error == nullptr)
        {
            return 3;
        }
        if (error->edge_vertices !=
                std::array<VertexId, 2>{VertexId{0}, VertexId{1}} ||
            error->first_face_id != SurfaceFaceId{0} ||
            error->second_face_id != SurfaceFaceId{1})
        {
            return 4;
        }
    }

    return 0;
}
```

在 `tests/CMakeLists.txt` 末尾添加：

```cmake
# 8
add_executable(
    boundary_mesh_surface_topology_edge_error_test
    unit/surface_topology_edge_error_test.cpp
)
target_link_libraries(
    boundary_mesh_surface_topology_edge_error_test
    PRIVATE BoundaryMesh::Core
)
add_test(
    NAME boundary_mesh_surface_topology_edge_error_test
    COMMAND boundary_mesh_surface_topology_edge_error_test
)
```

- [ ] **Step 2: 运行 RED 测试**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

预期：新增测试失败，因为构建器尚未拒绝非流形边和同向共享边。

- [ ] **Step 3: 在边登记阶段返回精确错误**

在 `src/mesh/surface_topology_builder.cpp` 的 include 区增加：

```cpp
#include <optional>
```

用以下实现完整替换现有 `appendEdge`：

```cpp
        EdgeId appendEdge(
            VertexId first,
            VertexId second,
            SurfaceFaceId face_id,
            std::unordered_map<EdgeKey, EdgeId, EdgeKeyHash>&
                edge_ids,
            std::vector<Edge>& edges,
            std::vector<std::vector<SurfaceFaceId>>&
                incident_faces,
            std::vector<std::vector<int>>& incident_directions,
            std::optional<SurfaceTopologyError>& error)
        {
            if (error.has_value())
            {
                return EdgeId{};
            }

            const EdgeKey key = makeEdgeKey(first, second);
            const int direction = edgeDirection(first, second);
            const auto found = edge_ids.find(key);
            if (found == edge_ids.end())
            {
                const auto edge_id =
                    static_cast<EdgeId>(edges.size());
                edge_ids.emplace(key, edge_id);
                edges.push_back(Edge{{key.first, key.second}});
                incident_faces.push_back({face_id});
                incident_directions.push_back({direction});
                return edge_id;
            }

            const EdgeId edge_id = found->second;
            const auto edge_index =
                static_cast<std::size_t>(edge_id);
            auto& faces = incident_faces[edge_index];
            auto& directions = incident_directions[edge_index];

            if (faces.size() >= 2)
            {
                error = SurfaceTopologyError{NonManifoldEdge{
                    {key.first, key.second},
                    {faces[0], faces[1], face_id}}};
                return edge_id;
            }

            if (directions[0] == direction)
            {
                error = SurfaceTopologyError{
                    InconsistentOrientation{
                        {key.first, key.second},
                        faces[0],
                        face_id}};
                return edge_id;
            }

            faces.push_back(face_id);
            directions.push_back(direction);
            return edge_id;
        }
```

用以下实现完整替换现有 `appendFace`：

```cpp
        template<std::size_t Count>
        std::array<EdgeId, Count> appendFace(
            const std::array<VertexId, Count>& vertex_ids,
            SurfaceFaceId face_id,
            std::unordered_map<EdgeKey, EdgeId, EdgeKeyHash>&
                edge_ids,
            std::vector<Edge>& edges,
            std::vector<std::vector<SurfaceFaceId>>&
                incident_faces,
            std::vector<std::vector<int>>& incident_directions,
            std::vector<std::vector<SurfaceFaceId>>& vertex_faces,
            std::optional<SurfaceTopologyError>& error)
        {
            std::array<EdgeId, Count> face_edge_ids{};
            for (std::size_t i = 0; i < Count; ++i)
            {
                vertex_faces[static_cast<std::size_t>(
                    vertex_ids[i])]
                    .push_back(face_id);
                face_edge_ids[i] = appendEdge(
                    vertex_ids[i],
                    vertex_ids[(i + 1) % Count],
                    face_id,
                    edge_ids,
                    edges,
                    incident_faces,
                    incident_directions,
                    error);
            }
            return face_edge_ids;
        }
```

在 `build()` 中、`face_edges.reserve(...)` 前增加：

```cpp
        std::optional<SurfaceTopologyError> edge_error;
```

在 `appendFace(...)` 调用的最后一个实参 `vertex_faces` 后增加：

```cpp
                        vertex_faces,
                        edge_error);
```

在每个面的 `std::visit(...)` 和 `face_edges.push_back(...)` 完成后增加：

```cpp
            if (edge_error.has_value())
            {
                return BuildResult::failure(
                    std::move(*edge_error));
            }
```

- [ ] **Step 4: 验证 GREEN**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

预期：8/8 测试通过。

- [ ] **Step 5: 提交**

```powershell
git add src/mesh/surface_topology_builder.cpp `
        tests/CMakeLists.txt `
        tests/unit/surface_topology_edge_error_test.cpp
git diff --cached --check
git commit -m "feat: reject invalid shared edges"
```

---

### Task 6: 拒绝开放表面并完成收尾验证

**Files:**

- Modify: `tests/unit/surface_topology_edge_error_test.cpp`
- Modify: `src/mesh/surface_topology_builder.cpp`

**Interfaces:**

- Consumes: 已完成共享边扫描的 `edges` 和 `incident_faces`。
- Produces: 第一个确定性 `BoundaryEdge`，并保证成功拓扑的每条边恰好有两个相邻面。

- [ ] **Step 1: 在边错误测试中增加开放表面案例**

在 `tests/unit/surface_topology_edge_error_test.cpp` 的第二个测试块之后、`return 0;` 之前增加：

```cpp
    {
        SurfaceMesh mesh;
        mesh.vertices.resize(3, Point3::Zero());
        mesh.faces = {
            Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}}};
        mesh.face_tags = {wallTag()};

        const auto result = builder.build(mesh);
        const auto* error =
            result.hasValue()
                ? nullptr
                : std::get_if<BoundaryEdge>(&result.error());
        if (error == nullptr)
        {
            return 5;
        }
        if (error->edge_vertices !=
                std::array<VertexId, 2>{VertexId{0}, VertexId{1}} ||
            error->face_id != SurfaceFaceId{0})
        {
            return 6;
        }
    }
```

- [ ] **Step 2: 运行 RED 测试**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

预期：`boundary_mesh_surface_topology_edge_error_test` 失败并返回 `5`，因为单个三角形暂时被当作成功结果。

- [ ] **Step 3: 增加封闭性检查**

在 `src/mesh/surface_topology_builder.cpp` 中，完成所有面的扫描后、创建 `edge_faces` 之前增加：

```cpp
        for (std::size_t edge_index = 0;
             edge_index < edges.size();
             ++edge_index)
        {
            if (incident_faces[edge_index].size() == 1)
            {
                return BuildResult::failure(
                    SurfaceTopologyError{BoundaryEdge{
                        edges[edge_index].vertex_ids,
                        incident_faces[edge_index][0]}});
            }
        }
```

成功路径此前已经拒绝第三个相邻面；加上此检查后，每条边恰好有两个相邻面，后续 `edge_faces` 和 `face_neighbors` 不再依赖默认填充值。

- [ ] **Step 4: 验证 GREEN**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

预期：8/8 测试通过。

- [ ] **Step 5: 执行完整收尾检查**

```powershell
git diff --check
git status --short
git diff --stat
```

预期：只有本任务的两个文件发生修改，且 `git diff --check` 没有输出。

- [ ] **Step 6: 提交**

```powershell
git add src/mesh/surface_topology_builder.cpp `
        tests/unit/surface_topology_edge_error_test.cpp
git diff --cached --check
git commit -m "feat: require closed surface topology"
```

- [ ] **Step 7: 提交后重新验证**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
git diff --check
git status --short --branch
```

预期：Debug 构建成功，8/8 测试通过，工作区只显示当前分支且无未提交文件。

---

## Completion Checklist

- [ ] `Result<T, E>` 对成功值和错误值提供明确访问。
- [ ] `SurfaceTopology` 无公开修改接口。
- [ ] `BoundaryMesh::Core` 是包含构建器 `.cpp` 的静态库。
- [ ] 封闭三棱柱产生 9 条稳定编号的边。
- [ ] 点到面、边到面、面到边、面到面邻接均通过精确断言。
- [ ] 空表面、标签数量错误、越界顶点、退化面和重复面返回精确错误。
- [ ] 非流形边、方向不一致和开放边返回精确错误。
- [ ] 构建器不修改输入 `SurfaceMesh`。
- [ ] 现有 foundation 测试继续通过。
- [ ] `2dot5_cf_gmsh_recombine.ply` 未被复制或接入本计划。
