# Public Header Convention Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 Mesh 公共头文件统一为 `mesh_` 前缀，并为全部公共基础声明补充便于阅读和调试的行尾注释。

**Architecture:** 文件改名只改变 include 路径，不改变 C++ 类型名或模块职责；注释任务只修改声明文本，不修改类型、默认值、成员顺序或算法。当前未提交的 Task 2 Surface 文件必须保留并一起接受回归验证，但不得混入第一个纯改名提交。

**Tech Stack:** C++17、CMake 3.20+、MSVC Debug、CTest、Git

## Global Constraints

- 不保留旧头文件或转发兼容层。
- `SurfaceMesh`、`VolumeMesh`、`SurfaceTopology`、`SurfaceTopologyBuilder`、`SurfaceTopologyError` 类型名不变。
- 旧 include 路径必须在 `include`、`src`、`tests` 和历史 `docs` 中搜索为零；本计划与迁移规范中的旧到新映射除外。
- 公共 `using`、枚举值和 struct 数据成员使用行尾 `//`；函数和类继续使用上方 `///`。
- 不修改可执行语句，不改变 ABI 数据布局。
- 保留工作区中 Task 2 的 `CMakeLists.txt`、Surface 头文件、实现和测试改动。

---

### Task 1: 统一 Mesh 公共头文件名

**Files:**

- Move: `include/boundary_mesh/mesh/surface_mesh.hpp` → `include/boundary_mesh/mesh/mesh_surface.hpp`
- Move: `include/boundary_mesh/mesh/volume_mesh.hpp` → `include/boundary_mesh/mesh/mesh_volume.hpp`
- Move: `include/boundary_mesh/mesh/surface_topology.hpp` → `include/boundary_mesh/mesh/mesh_surface_topology.hpp`
- Move: `include/boundary_mesh/mesh/surface_topology_builder.hpp` → `include/boundary_mesh/mesh/mesh_surface_topology_builder.hpp`
- Move: `include/boundary_mesh/mesh/surface_topology_error.hpp` → `include/boundary_mesh/mesh/mesh_surface_topology_error.hpp`
- Modify: all matching include directives under `include/`, `src/`, `tests/`
- Modify: matching paths under `docs/design/` and `docs/plans/`

- [ ] **Step 1: Capture the current regression baseline**

```powershell
cmake -S . -B build
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

Expected: 9 tests pass before the rename.

- [ ] **Step 2: Move the five tracked headers**

```powershell
git mv include/boundary_mesh/mesh/surface_mesh.hpp include/boundary_mesh/mesh/mesh_surface.hpp
git mv include/boundary_mesh/mesh/volume_mesh.hpp include/boundary_mesh/mesh/mesh_volume.hpp
git mv include/boundary_mesh/mesh/surface_topology.hpp include/boundary_mesh/mesh/mesh_surface_topology.hpp
git mv include/boundary_mesh/mesh/surface_topology_builder.hpp include/boundary_mesh/mesh/mesh_surface_topology_builder.hpp
git mv include/boundary_mesh/mesh/surface_topology_error.hpp include/boundary_mesh/mesh/mesh_surface_topology_error.hpp
```

- [ ] **Step 3: Update all references deterministically**

Replace exactly these include tokens:

```text
boundary_mesh/mesh/surface_mesh.hpp
    -> boundary_mesh/mesh/mesh_surface.hpp
boundary_mesh/mesh/volume_mesh.hpp
    -> boundary_mesh/mesh/mesh_volume.hpp
boundary_mesh/mesh/surface_topology.hpp
    -> boundary_mesh/mesh/mesh_surface_topology.hpp
boundary_mesh/mesh/surface_topology_builder.hpp
    -> boundary_mesh/mesh/mesh_surface_topology_builder.hpp
boundary_mesh/mesh/surface_topology_error.hpp
    -> boundary_mesh/mesh/mesh_surface_topology_error.hpp
```

Do not replace C++ identifiers such as `SurfaceMesh`.

- [ ] **Step 4: Verify old paths are absent and behavior is unchanged**

```powershell
Get-ChildItem include,src,tests -Recurse -File |
    Select-String -Pattern 'boundary_mesh/mesh/(surface_mesh|volume_mesh|surface_topology|surface_topology_builder|surface_topology_error)\.hpp'
Get-ChildItem docs -Recurse -File |
    Where-Object {
        $_.FullName -notlike '*public-header-conventions.md'
    } |
    Select-String -Pattern 'boundary_mesh/mesh/(surface_mesh|volume_mesh|surface_topology|surface_topology_builder|surface_topology_error)\.hpp'
cmake -S . -B build
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
git diff --check
```

Expected: the search prints nothing; all 9 tests pass.

- [ ] **Step 5: Commit only the rename and reference updates**

Stage the five renames and only files whose diff changes the five paths. Do not stage Task 2 production code.

```powershell
git diff --cached --check
git commit -m "refactor: align mesh header filenames"
```

---

### Task 2: 为公共基础声明补充行尾注释

**Files:**

- Modify: `include/boundary_mesh/core/types.hpp`
- Modify: `include/boundary_mesh/core/result.hpp` only if a public declaration lacks a meaningful comment
- Modify: `include/boundary_mesh/mesh/mesh_surface.hpp`
- Modify: `include/boundary_mesh/mesh/mesh_volume.hpp`
- Modify: `include/boundary_mesh/mesh/mesh_surface_topology.hpp`
- Modify: `include/boundary_mesh/mesh/mesh_surface_topology_builder.hpp`
- Modify: `include/boundary_mesh/mesh/mesh_surface_topology_error.hpp`
- Modify: `include/boundary_mesh/surface/face_evaluation.hpp`

- [ ] **Step 1: Annotate aliases**

Use trailing comments on complete declarations:

```cpp
using Scalar = double; // 网格算法统一使用的浮点标量类型
using VertexId = std::uint32_t; // 顶点容器的稳定索引类型
```

For multiline aliases, place the comment after the terminating `;`:

```cpp
using SurfaceFace =
    std::variant<Triangle, Quad>; // 三角形或四边形表面单元
```

- [ ] **Step 2: Annotate every public enum value**

```cpp
enum class SurfaceBoundaryKind
{
    Farfield, // 远场边界
    Wall,     // 需要生成边界层的壁面
    Symmetry  // 对称面
};
```

Apply the same rule to `FaceEvaluationError` and future-facing public enums already present in the headers.

- [ ] **Step 3: Annotate every public struct member**

Each comment states meaning or indexed entity, not merely the member name:

```cpp
struct Triangle
{
    std::array<VertexId, 3> vertex_ids{}; // 按面绕序保存的三个顶点编号
};
```

Keep all member types, order, and initializers byte-for-byte equivalent apart from whitespace/comments.

- [ ] **Step 4: Review coverage and run regression**

Inspect every file returned by:

```powershell
Get-ChildItem include/boundary_mesh -Recurse -Filter *.hpp
```

Then run:

```powershell
cmake -S . -B build
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
git diff --check
```

Expected: all 9 tests pass and no whitespace errors are reported.

- [ ] **Step 5: Commit only comment changes**

```powershell
git add include/boundary_mesh
git diff --cached --check
git commit -m "docs: annotate public declarations"
git status --short --branch
```

The remaining unstaged files after this commit may only be the ongoing Task 2 Surface implementation if it was intentionally excluded from the comment commit.
