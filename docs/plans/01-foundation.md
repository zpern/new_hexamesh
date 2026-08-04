# Foundation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:test-driven-development and superpowers:verification-before-completion while implementing each task.

**Goal:** 建立可独立编译测试的 C++17 工程，并实现基础类型、混合表面网格和混合体网格数据模型。

**Architecture:** `boundary_mesh_core` 是 header-only 基础目标，公开依赖 Eigen。表面和体单元使用 `std::variant` 显式表达类型，不通过节点数量推断类型。本计划不包含拓扑、几何算法或边界层生成。

**Tech Stack:** C++17、CMake、CTest、Eigen

## Global Constraints

- 使用 C++17。
- Eigen 是 `BoundaryMesh::Core` 的公共依赖。
- 上层已提供 `Eigen3::Eigen` 时直接复用，否则使用 `third/eigen`。
- 所有公共头文件位于 `include/boundary_mesh/`。
- 项目代码位于 `boundary_mesh` 命名空间。
- 测试不依赖网络下载的测试框架。
- 每项任务遵循：失败测试、最小实现、测试通过、提交。

---

## File Map

```text
CMakeLists.txt
cmake/
├── CompileOptions.cmake
└── Dependencies.cmake
include/boundary_mesh/
├── core/
│   └── types.hpp
└── mesh/
    ├── surface_mesh.hpp
    └── volume_mesh.hpp
tests/
├── CMakeLists.txt
└── unit/
    ├── core_types_test.cpp
    ├── surface_mesh_test.cpp
    └── volume_mesh_test.cpp
third/
└── eigen/
```

职责：

- `types.hpp`：标量、三维点/向量和实体 ID。
- `cmake/CompileOptions.cmake`：集中管理本项目的编译器警告、UTF-8 和 PIC 选项。
- `cmake/Dependencies.cmake`：集中发现或创建第三方依赖 target。
- `surface_mesh.hpp`：三角形、四边形、边界标签和混合表面网格。
- `volume_mesh.hpp`：四面体、金字塔、三棱柱、六面体及混合体网格。
- `tests/CMakeLists.txt`：为每个单元测试建立独立可执行程序。

---

### Task 1: CMake 骨架与基础类型

**Files:**

- Modify: `CMakeLists.txt`
- Create: `cmake/CompileOptions.cmake`
- Create: `cmake/Dependencies.cmake`
- Create: `include/boundary_mesh/core/types.hpp`
- Create: `tests/CMakeLists.txt`
- Create: `tests/unit/core_types_test.cpp`
- Create: `.gitmodules`
- Create submodule: `third/eigen`

**Interfaces:**

- Produces: `boundary_mesh::Scalar`
- Produces: `boundary_mesh::Point3`
- Produces: `boundary_mesh::Vector3`
- Produces: `VertexId`, `EdgeId`, `SurfaceFaceId`, `VolumeCellId`
- Produces CMake target: `BoundaryMesh::Core`

- [ ] **Step 1: 添加 Eigen 子模块**

```powershell
git submodule add https://github.com/mySharedsource/eigen.git third/eigen
git submodule status
```

预期输出包含 `third/eigen`。

- [ ] **Step 2: 编写顶层 CMake**

创建 `cmake/CompileOptions.cmake`：

```cmake
include_guard(GLOBAL)

add_library(boundary_mesh_compile_options INTERFACE)
add_library(
    BoundaryMesh::CompileOptions
    ALIAS boundary_mesh_compile_options
)

target_compile_features(
    boundary_mesh_compile_options
    INTERFACE cxx_std_17
)

target_compile_options(
    boundary_mesh_compile_options
    INTERFACE
        $<$<CXX_COMPILER_ID:MSVC>:/utf-8>
        $<$<CXX_COMPILER_ID:MSVC>:/W4>
        $<$<CXX_COMPILER_ID:MSVC>:/permissive->
        $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall>
        $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wextra>
        $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wpedantic>
)

set_target_properties(
    boundary_mesh_compile_options
    PROPERTIES INTERFACE_POSITION_INDEPENDENT_CODE ON
)
```

创建 `cmake/Dependencies.cmake`：

```cmake
include_guard(GLOBAL)

if(TARGET Eigen3::Eigen)
    message(STATUS "BoundaryMesh reuses existing target Eigen3::Eigen")
elseif(TARGET tiger_eigen)
    add_library(boundary_mesh_eigen INTERFACE)
    target_link_libraries(boundary_mesh_eigen INTERFACE tiger_eigen)
    add_library(Eigen3::Eigen ALIAS boundary_mesh_eigen)
    message(STATUS "BoundaryMesh reuses existing target tiger_eigen")
else()
    set(
        BOUNDARY_MESH_DEPENDENCIES_DIR
        ""
        CACHE PATH
        "Optional directory containing BoundaryMesh dependencies"
    )

    set(_boundary_mesh_dependency_candidates)

    if(BOUNDARY_MESH_DEPENDENCIES_DIR)
        list(APPEND _boundary_mesh_dependency_candidates
            "${BOUNDARY_MESH_DEPENDENCIES_DIR}"
        )
    endif()

    if(DEFINED TIGER_ROOT_DIR)
        list(APPEND _boundary_mesh_dependency_candidates
            "${TIGER_ROOT_DIR}/extern"
            "${TIGER_ROOT_DIR}/third"
        )
    endif()

    list(APPEND _boundary_mesh_dependency_candidates
        "${PROJECT_SOURCE_DIR}/third"
        "${PROJECT_SOURCE_DIR}/extern"
    )

    unset(BOUNDARY_MESH_RESOLVED_DEPENDENCIES_DIR)
    foreach(_candidate IN LISTS _boundary_mesh_dependency_candidates)
        if(EXISTS "${_candidate}/eigen/Eigen/Core")
            set(BOUNDARY_MESH_RESOLVED_DEPENDENCIES_DIR "${_candidate}")
            break()
        endif()
    endforeach()

    if(NOT BOUNDARY_MESH_RESOLVED_DEPENDENCIES_DIR)
        message(FATAL_ERROR
            "BoundaryMesh could not find Eigen in: "
            "${_boundary_mesh_dependency_candidates}"
        )
    endif()

    set(BOUNDARY_MESH_EIGEN_DIR
        "${BOUNDARY_MESH_RESOLVED_DEPENDENCIES_DIR}/eigen"
    )

    add_library(boundary_mesh_eigen INTERFACE)

    target_include_directories(
        boundary_mesh_eigen
        SYSTEM INTERFACE
            "${BOUNDARY_MESH_EIGEN_DIR}"
    )

    add_library(Eigen3::Eigen ALIAS boundary_mesh_eigen)

    message(STATUS
        "BoundaryMesh Eigen directory: ${BOUNDARY_MESH_EIGEN_DIR}"
    )
endif()
```

将根目录 `CMakeLists.txt` 替换为：

```cmake
cmake_minimum_required(VERSION 3.20)

project(BoundaryMesh VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include("${PROJECT_SOURCE_DIR}/cmake/CompileOptions.cmake")
include("${PROJECT_SOURCE_DIR}/cmake/Dependencies.cmake")

add_library(boundary_mesh_core INTERFACE)
add_library(BoundaryMesh::Core ALIAS boundary_mesh_core)

target_compile_features(boundary_mesh_core INTERFACE cxx_std_17)
target_include_directories(boundary_mesh_core INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(
    boundary_mesh_core
    INTERFACE
        BoundaryMesh::CompileOptions
        Eigen3::Eigen
)

include(CTest)
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
```

- [ ] **Step 3: 编写失败测试**

创建 `tests/unit/core_types_test.cpp`：

```cpp
#include <cstdint>
#include <type_traits>

#include <boundary_mesh/core/types.hpp>

int main()
{
    using namespace boundary_mesh;

    static_assert(std::is_same_v<Scalar, double>);
    static_assert(std::is_same_v<VertexId, std::uint32_t>);
    static_assert(std::is_same_v<EdgeId, std::uint32_t>);
    static_assert(std::is_same_v<SurfaceFaceId, std::uint32_t>);
    static_assert(std::is_same_v<VolumeCellId, std::uint32_t>);

    const Point3 point{1.0, 2.0, 3.0};
    const Vector3 direction{0.0, 1.0, 0.0};

    if (point.x() != 1.0 || point.y() != 2.0 || point.z() != 3.0) {
        return 1;
    }
    if (direction.norm() != 1.0) {
        return 2;
    }
    return 0;
}
```

创建 `tests/CMakeLists.txt`：

```cmake
add_executable(boundary_mesh_core_types_test unit/core_types_test.cpp)
target_link_libraries(boundary_mesh_core_types_test PRIVATE BoundaryMesh::Core)
add_test(NAME boundary_mesh_core_types_test COMMAND boundary_mesh_core_types_test)
```

- [ ] **Step 4: 验证测试先失败**

```powershell
cmake -S . -B build
cmake --build build --config Debug
```

预期因缺少 `boundary_mesh/core/types.hpp` 而失败。

- [ ] **Step 5: 实现基础类型**

创建 `include/boundary_mesh/core/types.hpp`：

```cpp
#pragma once

#include <cstdint>
#include <Eigen/Core>

namespace boundary_mesh {

using Scalar = double;
using Point3 = Eigen::Vector3d;
using Vector3 = Eigen::Vector3d;

using VertexId = std::uint32_t;
using EdgeId = std::uint32_t;
using SurfaceFaceId = std::uint32_t;
using VolumeCellId = std::uint32_t;

} // namespace boundary_mesh
```

- [ ] **Step 6: 验证测试通过**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

预期 `1/1` 测试通过。

- [ ] **Step 7: 提交**

```powershell
git add CMakeLists.txt cmake/CompileOptions.cmake cmake/Dependencies.cmake include tests
git commit -m "build: establish core library foundation"
```

---

### Task 2: 混合表面网格类型

**Files:**

- Create: `include/boundary_mesh/mesh/surface_mesh.hpp`
- Create: `tests/unit/surface_mesh_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**

- Consumes: `Point3`, `VertexId`
- Produces: `Triangle`, `Quadrilateral`, `SurfaceFace`
- Produces: `SurfaceBoundaryKind`, `SurfaceBoundaryTag`, `SurfaceMesh`

- [ ] **Step 1: 编写失败测试**

创建 `tests/unit/surface_mesh_test.cpp`：

```cpp
#include <variant>
#include <boundary_mesh/mesh/surface_mesh.hpp>

int main()
{
    using namespace boundary_mesh;

    SurfaceMesh mesh;
    mesh.vertices = {
        Point3{0.0, 0.0, 0.0}, Point3{1.0, 0.0, 0.0},
        Point3{1.0, 1.0, 0.0}, Point3{0.0, 1.0, 0.0}
    };
    mesh.faces.emplace_back(
        Triangle{{VertexId{0}, VertexId{1}, VertexId{2}}});
    mesh.faces.emplace_back(Quadrilateral{{
        VertexId{0}, VertexId{1}, VertexId{2}, VertexId{3}}});
    mesh.face_tags = {
        SurfaceBoundaryTag{SurfaceBoundaryKind::Growth, 4},
        SurfaceBoundaryTag{SurfaceBoundaryKind::Symmetry, 9}
    };

    if (mesh.vertices.size() != 4) return 1;
    if (!std::holds_alternative<Triangle>(mesh.faces[0])) return 2;
    if (!std::holds_alternative<Quadrilateral>(mesh.faces[1])) return 3;
    if (mesh.face_tags[1].kind != SurfaceBoundaryKind::Symmetry) return 4;
    if (mesh.face_tags[1].region_id != 9) return 5;
    return 0;
}
```

在 `tests/CMakeLists.txt` 末尾添加：

```cmake
add_executable(boundary_mesh_surface_mesh_test unit/surface_mesh_test.cpp)
target_link_libraries(boundary_mesh_surface_mesh_test PRIVATE BoundaryMesh::Core)
add_test(NAME boundary_mesh_surface_mesh_test COMMAND boundary_mesh_surface_mesh_test)
```

- [ ] **Step 2: 验证测试先失败**

```powershell
cmake --build build --config Debug
```

预期因缺少 `boundary_mesh/mesh/surface_mesh.hpp` 而失败。

- [ ] **Step 3: 实现混合表面类型**

创建 `include/boundary_mesh/mesh/surface_mesh.hpp`：

```cpp
#pragma once

#include <array>
#include <cstdint>
#include <variant>
#include <vector>

#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh {

struct Triangle { std::array<VertexId, 3> vertices{}; };
struct Quadrilateral { std::array<VertexId, 4> vertices{}; };
using SurfaceFace = std::variant<Triangle, Quadrilateral>;

enum class SurfaceBoundaryKind : std::uint8_t
{
    Growth,
    Symmetry,
    Wall,
    Farfield,
    UserDefined
};

struct SurfaceBoundaryTag
{
    SurfaceBoundaryKind kind{SurfaceBoundaryKind::Growth};
    std::int32_t region_id{0};
};

struct SurfaceMesh
{
    std::vector<Point3> vertices;
    std::vector<SurfaceFace> faces;
    std::vector<SurfaceBoundaryTag> face_tags;
};

} // namespace boundary_mesh
```

- [ ] **Step 4: 验证测试通过**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

预期 `2/2` 测试通过。

- [ ] **Step 5: 提交**

```powershell
git add include/boundary_mesh/mesh/surface_mesh.hpp tests
git commit -m "feat: add mixed surface mesh types"
```

---

### Task 3: 混合体网格类型

**Files:**

- Create: `include/boundary_mesh/mesh/volume_mesh.hpp`
- Create: `tests/unit/volume_mesh_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**

- Consumes: `Point3`, `VertexId`, `SurfaceFaceId`
- Produces: `Tetrahedron`, `Pyramid`, `Prism`, `Hexahedron`
- Produces: `VolumeCell`, `VolumeCellType`, `cellType(const VolumeCell&)`
- Produces: `CellRole`, `CellMetadata`, `VolumeMesh`

- [ ] **Step 1: 编写失败测试**

创建 `tests/unit/volume_mesh_test.cpp`：

```cpp
#include <boundary_mesh/mesh/volume_mesh.hpp>

int main()
{
    using namespace boundary_mesh;

    VolumeMesh mesh;
    mesh.vertices.resize(8, Point3::Zero());
    mesh.cells.emplace_back(Tetrahedron{{
        VertexId{0}, VertexId{1}, VertexId{2}, VertexId{3}}});
    mesh.cells.emplace_back(Pyramid{{
        VertexId{0}, VertexId{1}, VertexId{2}, VertexId{3}, VertexId{4}}});
    mesh.cells.emplace_back(Prism{{
        VertexId{0}, VertexId{1}, VertexId{2},
        VertexId{3}, VertexId{4}, VertexId{5}}});
    mesh.cells.emplace_back(Hexahedron{{
        VertexId{0}, VertexId{1}, VertexId{2}, VertexId{3},
        VertexId{4}, VertexId{5}, VertexId{6}, VertexId{7}}});
    mesh.metadata = {
        CellMetadata{CellRole::Transition, SurfaceFaceId{0}, 0},
        CellMetadata{CellRole::Transition, SurfaceFaceId{1}, 0},
        CellMetadata{CellRole::RegularLayer, SurfaceFaceId{2}, 1},
        CellMetadata{CellRole::RegularLayer, SurfaceFaceId{3}, 1}
    };

    if (cellType(mesh.cells[0]) != VolumeCellType::Tetrahedron) return 1;
    if (cellType(mesh.cells[1]) != VolumeCellType::Pyramid) return 2;
    if (cellType(mesh.cells[2]) != VolumeCellType::Prism) return 3;
    if (cellType(mesh.cells[3]) != VolumeCellType::Hexahedron) return 4;
    if (mesh.metadata[2].role != CellRole::RegularLayer) return 5;
    return 0;
}
```

在 `tests/CMakeLists.txt` 末尾添加：

```cmake
add_executable(boundary_mesh_volume_mesh_test unit/volume_mesh_test.cpp)
target_link_libraries(boundary_mesh_volume_mesh_test PRIVATE BoundaryMesh::Core)
add_test(NAME boundary_mesh_volume_mesh_test COMMAND boundary_mesh_volume_mesh_test)
```

- [ ] **Step 2: 验证测试先失败**

```powershell
cmake --build build --config Debug
```

预期因缺少 `boundary_mesh/mesh/volume_mesh.hpp` 而失败。

- [ ] **Step 3: 实现混合体网格**

创建 `include/boundary_mesh/mesh/volume_mesh.hpp`：

```cpp
#pragma once

#include <array>
#include <cstdint>
#include <type_traits>
#include <variant>
#include <vector>

#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh {

struct Tetrahedron { std::array<VertexId, 4> vertices{}; };
struct Pyramid { std::array<VertexId, 5> vertices{}; };
struct Prism { std::array<VertexId, 6> vertices{}; };
struct Hexahedron { std::array<VertexId, 8> vertices{}; };

using VolumeCell = std::variant<Tetrahedron, Pyramid, Prism, Hexahedron>;

enum class VolumeCellType : std::uint8_t
{
    Tetrahedron,
    Pyramid,
    Prism,
    Hexahedron
};

inline VolumeCellType cellType(const VolumeCell& cell)
{
    return std::visit([](const auto& value) {
        using Cell = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<Cell, Tetrahedron>) {
            return VolumeCellType::Tetrahedron;
        } else if constexpr (std::is_same_v<Cell, Pyramid>) {
            return VolumeCellType::Pyramid;
        } else if constexpr (std::is_same_v<Cell, Prism>) {
            return VolumeCellType::Prism;
        } else {
            return VolumeCellType::Hexahedron;
        }
    }, cell);
}

enum class CellRole : std::uint8_t { RegularLayer, Transition };

struct CellMetadata
{
    CellRole role{CellRole::RegularLayer};
    SurfaceFaceId source_face{};
    std::uint32_t layer{0};
};

struct VolumeMesh
{
    std::vector<Point3> vertices;
    std::vector<VolumeCell> cells;
    std::vector<CellMetadata> metadata;
};

} // namespace boundary_mesh
```

- [ ] **Step 4: 验证测试通过**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

预期 `3/3` 测试通过。

- [ ] **Step 5: 提交**

```powershell
git add include/boundary_mesh/mesh/volume_mesh.hpp tests
git commit -m "feat: add mixed volume mesh types"
```

---

### Task 4: 第一阶段整体验证

- [ ] **Step 1: 从干净构建目录重新配置**

先确认删除目标严格等于当前项目下的 `build`，然后删除并重新配置：

```powershell
$buildPath = Join-Path $PWD "build"
$resolvedProject = (Resolve-Path .).Path
if ($buildPath -ne (Join-Path $resolvedProject "build")) {
    throw "Unexpected build path: $buildPath"
}
if (Test-Path -LiteralPath $buildPath) {
    Remove-Item -LiteralPath $buildPath -Recurse -Force
}
cmake -S . -B build
```

- [ ] **Step 2: 完整编译并测试**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

预期三个测试目标全部构建成功，且 `3/3` 测试通过。

- [ ] **Step 3: 检查工作区**

```powershell
git status --short
git log --oneline --decorate -4
```

预期工作区没有未提交的源文件，历史中包含本计划对应的三个实现提交。

---

## Completion Criteria

1. `BoundaryMesh::Core` 可以被其他 CMake 目标链接。
2. Eigen 通过标准 target `Eigen3::Eigen` 使用。
3. 三角形和四边形通过 `SurfaceFace` 显式表达。
4. 四面体、金字塔、三棱柱和六面体通过 `VolumeCell` 显式表达。
5. 不存在通过 `vector.size()` 推测单元类型的代码。
6. 三个测试从干净构建目录全部通过。
7. 尚未引入拓扑、几何、IO 或生成算法。
