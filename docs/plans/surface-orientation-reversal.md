# SurfaceMesh Global Orientation Reversal Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 提供一个原地翻转 `SurfaceMesh` 中全部 Triangle/Quad 绕向的公共辅助函数。

**Architecture:** 将数据结构继续保留在 `mesh_surface.hpp`，把操作接口放入独立公共头文件，并在 `boundary_mesh_core` 中提供非模板实现。函数遍历混合面 variant，保留每个面的首顶点，反转其余顶点；不修改坐标、面顺序或边界标签。

**Tech Stack:** C++17、`std::variant`、`std::array`、CMake、CTest、Visual Studio/MSBuild。

## Global Constraints

- 公共接口固定为 `void reverseSurfaceOrientation(SurfaceMesh &mesh) noexcept;`。
- Triangle 变换固定为 `[v0,v1,v2] -> [v0,v2,v1]`。
- Quad 变换固定为 `[v0,v1,v2,v3] -> [v0,v3,v2,v1]`。
- 只修改 `faces[*].vertex_ids`；顶点、面顺序、面类型和标签保持不变。
- 不自动检测朝向，不修复局部不一致，不新增 CLI 参数。
- 不使用新的第三方依赖。
- `.superpowers/` 不得加入提交。

---

### Task 1: SurfaceMesh 全局绕向翻转辅助函数

**Files:**
- Create: `include/boundary_mesh/mesh/mesh_surface_orientation.hpp`
- Create: `src/mesh/surface_orientation.cpp`
- Create: `tests/unit/mesh/surface_orientation_test.cpp`
- Modify: `CMakeLists.txt:20`
- Modify: `tests/CMakeLists.txt:6-10`

**Interfaces:**
- Consumes: `boundary_mesh::SurfaceMesh`、`boundary_mesh::Triangle`、`boundary_mesh::Quad`。
- Produces: `void boundary_mesh::reverseSurfaceOrientation(SurfaceMesh &mesh) noexcept;`。

- [ ] **Step 1: 注册并编写失败测试**

在 `tests/CMakeLists.txt` 的 surface mesh 测试后加入：

```cmake
add_executable(
    boundary_mesh_surface_orientation_test
    unit/mesh/surface_orientation_test.cpp
)
target_link_libraries(
    boundary_mesh_surface_orientation_test
    PRIVATE BoundaryMesh::Core
)
add_test(
    NAME boundary_mesh_surface_orientation_test
    COMMAND boundary_mesh_surface_orientation_test
)
```

创建 `tests/unit/mesh/surface_orientation_test.cpp`：

```cpp
#include <array>
#include <cstddef>
#include <variant>

#include <boundary_mesh/mesh/mesh_surface_orientation.hpp>

int main()
{
    using namespace boundary_mesh;

    SurfaceMesh mesh;
    mesh.vertices = {
        Point3{0.0, 0.0, 0.0},
        Point3{1.0, 0.0, 0.0},
        Point3{1.0, 1.0, 0.0},
        Point3{0.0, 1.0, 0.0}};
    mesh.faces = {
        Triangle{{0, 1, 2}},
        Quad{{0, 1, 2, 3}}};
    mesh.face_tags = {
        {SurfaceBoundaryKind::Wall, 7},
        {SurfaceBoundaryKind::Farfield, 9}};

    const auto original_vertices = mesh.vertices;
    const auto original_tags = mesh.face_tags;

    reverseSurfaceOrientation(mesh);

    const auto *triangle = std::get_if<Triangle>(&mesh.faces[0]);
    const auto *quad = std::get_if<Quad>(&mesh.faces[1]);
    if (triangle == nullptr ||
        triangle->vertex_ids != std::array<VertexId, 3>{0, 2, 1})
    {
        return 1;
    }
    if (quad == nullptr ||
        quad->vertex_ids != std::array<VertexId, 4>{0, 3, 2, 1})
    {
        return 2;
    }
    if (mesh.vertices.size() != original_vertices.size())
    {
        return 3;
    }
    for (std::size_t index = 0; index < mesh.vertices.size(); ++index)
    {
        if (!mesh.vertices[index].isApprox(original_vertices[index]))
        {
            return 4;
        }
    }
    if (mesh.face_tags.size() != original_tags.size())
    {
        return 5;
    }
    for (std::size_t index = 0; index < mesh.face_tags.size(); ++index)
    {
        if (mesh.face_tags[index].kind != original_tags[index].kind ||
            mesh.face_tags[index].region_id != original_tags[index].region_id)
        {
            return 6;
        }
    }

    reverseSurfaceOrientation(mesh);
    triangle = std::get_if<Triangle>(&mesh.faces[0]);
    quad = std::get_if<Quad>(&mesh.faces[1]);
    if (triangle == nullptr ||
        triangle->vertex_ids != std::array<VertexId, 3>{0, 1, 2} ||
        quad == nullptr ||
        quad->vertex_ids != std::array<VertexId, 4>{0, 1, 2, 3})
    {
        return 7;
    }

    SurfaceMesh empty;
    reverseSurfaceOrientation(empty);
    if (!empty.vertices.empty() ||
        !empty.faces.empty() ||
        !empty.face_tags.empty())
    {
        return 8;
    }

    return 0;
}
```

- [ ] **Step 2: 构建目标并确认 RED**

Run:

```powershell
cmake --build build --config Debug `
    --target boundary_mesh_surface_orientation_test
```

Expected: 构建失败，错误明确指出找不到 `boundary_mesh/mesh/mesh_surface_orientation.hpp`。这是预期 RED，证明新测试依赖尚未实现的公共接口。

- [ ] **Step 3: 添加最小公共接口和实现**

创建 `include/boundary_mesh/mesh/mesh_surface_orientation.hpp`：

```cpp
#pragma once

#include <boundary_mesh/mesh/mesh_surface.hpp>

namespace boundary_mesh
{
    /// 统一反转 SurfaceMesh 中全部 Triangle 和 Quad 的顶点绕序。
    void reverseSurfaceOrientation(
        SurfaceMesh &mesh) noexcept;
}
```

创建 `src/mesh/surface_orientation.cpp`：

```cpp
#include <algorithm>
#include <variant>

#include <boundary_mesh/mesh/mesh_surface_orientation.hpp>

namespace boundary_mesh
{
    void reverseSurfaceOrientation(
        SurfaceMesh &mesh) noexcept
    {
        for (SurfaceFace &face : mesh.faces)
        {
            std::visit(
                [](auto &value)
                {
                    std::reverse(
                        value.vertex_ids.begin() + 1,
                        value.vertex_ids.end());
                },
                face);
        }
    }
}
```

把根 `CMakeLists.txt` 中的 Core 定义改为：

```cmake
add_library(
    boundary_mesh_core
    STATIC
        src/mesh/surface_topology_builder.cpp
        src/mesh/surface_orientation.cpp
)
```

- [ ] **Step 4: 构建并运行目标测试，确认 GREEN**

Run:

```powershell
cmake --build build --config Debug `
    --target boundary_mesh_surface_orientation_test
ctest --test-dir build -C Debug `
    -R "^boundary_mesh_surface_orientation_test$" `
    --output-on-failure
```

Expected: 构建成功，`1/1` 测试通过。

- [ ] **Step 5: 运行完整 Debug 和 Release 回归**

Run:

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
git diff --check
```

Expected: 两种配置全部测试通过，`git diff --check` 无错误。测试总数应在原有 47 项基础上增加为 48 项。

- [ ] **Step 6: 提交实现**

```powershell
git add -- `
    CMakeLists.txt `
    include/boundary_mesh/mesh/mesh_surface_orientation.hpp `
    src/mesh/surface_orientation.cpp `
    tests/CMakeLists.txt `
    tests/unit/mesh/surface_orientation_test.cpp
git diff --cached --check
git commit -m "feat: add surface orientation reversal"
git status --short --branch
```

Expected: 只提交上述五个实现和测试文件；`.superpowers/` 保持未跟踪且不进入提交。
