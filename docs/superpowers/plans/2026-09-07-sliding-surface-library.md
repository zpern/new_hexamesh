# Sliding Surface Library Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extract Symmetry/Internal geometry constraints into an independently linkable `BoundaryMesh::SlidingSurface` library while retaining Internal dual-layer topology in Core and keeping collision policy independently extensible in Spatial.

**Architecture:** Core owns mesh types, Internal/non-Internal topology, and the pure `isSlidingBoundary()` classification. Spatial owns triangle queries and collision-boundary policy. SlidingSurface depends on Core and Spatial, exposes neutral vertex inputs and sliding-specific errors, and BoundaryLayer adapts its growth types to that API.

**Tech Stack:** C++17, CMake 3.20+, Eigen3, existing `boundary_mesh::Result`, CTest executables using assertions/return codes.

## Global Constraints

- Preserve current mesh-generation behavior and input/output formats.
- Do not implement new collision algorithms in this change.
- `BoundaryMesh::SlidingSurface` must not depend on `BoundaryMesh::BoundaryLayer` or include headers below `boundary_mesh/growth/`.
- Collision remains in `BoundaryMesh::Spatial`; its default policy continues to omit Symmetry/Internal obstacles temporarily.
- Internal and Symmetry retain distinct `SurfaceBoundaryKind` values and original `region_id` values.
- Preserve the public semantics of `MeshSurfaceTopology` and its dual Internal/non-Internal edge incidence.

---

### Task 1: Centralize the pure sliding-boundary classification in Core

**Files:**
- Modify: `include/boundary_mesh/mesh/mesh_surface.hpp`
- Modify: `src/growth/growth_patch_builder.cpp`
- Modify: `src/growth/sliding_surface_builder.cpp`
- Test: `tests/unit/mesh/surface_mesh_test.cpp`

**Interfaces:**
- Consumes: `enum class SurfaceBoundaryKind` from `mesh_surface.hpp`.
- Produces: `constexpr bool isSlidingBoundary(SurfaceBoundaryKind kind) noexcept` in namespace `boundary_mesh`.

- [ ] **Step 1: Add a failing classification test**

Append checks covering every enum value:

```cpp
if (!isSlidingBoundary(SurfaceBoundaryKind::Symmetry) ||
    !isSlidingBoundary(SurfaceBoundaryKind::Internal) ||
    isSlidingBoundary(SurfaceBoundaryKind::Wall) ||
    isSlidingBoundary(SurfaceBoundaryKind::Farfield) ||
    isSlidingBoundary(SurfaceBoundaryKind::BoundaryLayerInterface))
    return 8;
```

- [ ] **Step 2: Build the focused test and verify failure**

Run: `cmake --build build --config Debug --target boundary_mesh_surface_mesh_test`

Expected: compilation fails because `isSlidingBoundary` is not declared.

- [ ] **Step 3: Add the Core helper and replace growth-local boolean expressions**

Add directly after `SurfaceBoundaryKind`:

```cpp
constexpr bool isSlidingBoundary(SurfaceBoundaryKind kind) noexcept
{
    return kind == SurfaceBoundaryKind::Symmetry ||
           kind == SurfaceBoundaryKind::Internal;
}
```

In `growth_patch_builder.cpp` use:

```cpp
if (isSlidingBoundary(tag.kind))
    sliding_region_ids.push_back(tag.region_id);
```

In `sliding_surface_builder.cpp` use:

```cpp
if (!isSlidingBoundary(tag.kind)) continue;
```

- [ ] **Step 4: Run Core and affected growth tests**

Run: `cmake --build build --config Debug --target boundary_mesh_surface_mesh_test boundary_mesh_growth_patch_test boundary_mesh_sliding_surface_builder_test`

Run: `ctest --test-dir build -C Debug --output-on-failure -R "boundary_mesh_(surface_mesh|growth_patch|sliding_surface_builder)_test"`

Expected: all three tests pass.

- [ ] **Step 5: Commit**

```powershell
git add include/boundary_mesh/mesh/mesh_surface.hpp src/growth/growth_patch_builder.cpp src/growth/sliding_surface_builder.cpp tests/unit/mesh/surface_mesh_test.cpp
git commit -m "refactor: centralize sliding boundary classification"
```

### Task 2: Establish a separately linkable SlidingSurface target

**Files:**
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`
- Create: `tests/cmake/sliding_surface_module_boundary_test.cpp`

**Interfaces:**
- Consumes: `BoundaryMesh::Core`, `BoundaryMesh::Spatial`.
- Produces: CMake target `boundary_mesh_sliding_surface` and alias `BoundaryMesh::SlidingSurface`.

- [ ] **Step 1: Add a module-boundary smoke executable before defining the target**

Create:

```cpp
#include <boundary_mesh/growth/sliding_surface.hpp>

int main()
{
    boundary_mesh::SlidingSurfaceSet surfaces;
    return surfaces.surfaces().empty() ? 0 : 1;
}
```

Register it in `tests/CMakeLists.txt`:

```cmake
add_executable(
    boundary_mesh_sliding_surface_module_boundary_test
    cmake/sliding_surface_module_boundary_test.cpp
)
target_link_libraries(
    boundary_mesh_sliding_surface_module_boundary_test
    PRIVATE BoundaryMesh::SlidingSurface
)
add_test(
    NAME boundary_mesh_sliding_surface_module_boundary_test
    COMMAND boundary_mesh_sliding_surface_module_boundary_test
)
```

- [ ] **Step 2: Configure and verify the target is missing**

Run: `cmake -S . -B build -DBOUNDARY_MESH_ENABLE_CGNS_IO=ON`

Expected: configuration fails because `BoundaryMesh::SlidingSurface` does not exist.

- [ ] **Step 3: Add the target and move source ownership without changing headers yet**

Insert before `boundary_mesh_boundary_layer`:

```cmake
add_library(
    boundary_mesh_sliding_surface STATIC
        src/growth/sliding_constraint_builder.cpp
        src/growth/sliding_surface_builder.cpp
)
add_library(BoundaryMesh::SlidingSurface ALIAS boundary_mesh_sliding_surface)
target_compile_features(boundary_mesh_sliding_surface PUBLIC cxx_std_17)
target_include_directories(
    boundary_mesh_sliding_surface
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(
    boundary_mesh_sliding_surface
    PUBLIC BoundaryMesh::Core BoundaryMesh::Spatial
)
```

Remove the two sources from `boundary_mesh_boundary_layer` and add `BoundaryMesh::SlidingSurface` to that target's public links.

- [ ] **Step 4: Build and run the boundary test**

Run: `cmake -S . -B build -DBOUNDARY_MESH_ENABLE_CGNS_IO=ON`

Run: `cmake --build build --config Debug --target boundary_mesh_sliding_surface_module_boundary_test`

Run: `ctest --test-dir build -C Debug --output-on-failure -R boundary_mesh_sliding_surface_module_boundary_test`

Expected: configure, build, and test pass.

- [ ] **Step 5: Commit**

```powershell
git add CMakeLists.txt tests/CMakeLists.txt tests/cmake/sliding_surface_module_boundary_test.cpp
git commit -m "build: add sliding surface library target"
```

### Task 3: Give SlidingSurface neutral public data and error types

**Files:**
- Create: `include/boundary_mesh/sliding/sliding_error.hpp`
- Create: `include/boundary_mesh/sliding/sliding_vertex_input.hpp`
- Move: `include/boundary_mesh/growth/sliding_surface.hpp` → `include/boundary_mesh/sliding/sliding_surface.hpp`
- Move: `include/boundary_mesh/growth/sliding_surface_builder.hpp` → `include/boundary_mesh/sliding/sliding_surface_builder.hpp`
- Move: `include/boundary_mesh/growth/sliding_constraints.hpp` → `include/boundary_mesh/sliding/sliding_constraints.hpp`
- Move: `include/boundary_mesh/growth/sliding_constraint_builder.hpp` → `include/boundary_mesh/sliding/sliding_constraint_builder.hpp`
- Move: `src/growth/sliding_surface_builder.cpp` → `src/sliding/sliding_surface_builder.cpp`
- Move: `src/growth/sliding_constraint_builder.cpp` → `src/sliding/sliding_constraint_builder.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/cmake/sliding_surface_module_boundary_test.cpp`
- Modify: `tests/unit/growth/sliding_constraints_test.cpp`
- Modify: `tests/unit/growth/sliding_surface_builder_test.cpp`

**Interfaces:**
- Produces: `SlidingError`, `SlidingVertexInput`, `SlidingConstraintBuilder::build(const SlidingSurfaceSet&, const std::vector<SlidingVertexInput>&, Scalar, Scalar, std::uint32_t)`.
- Constraint methods return `Result<..., SlidingError>` and use caller-provided compact vertex indices.

- [ ] **Step 1: Strengthen the module-boundary test to include only new public headers**

Replace its includes and body with:

```cpp
#include <boundary_mesh/sliding/sliding_constraint_builder.hpp>
#include <boundary_mesh/sliding/sliding_surface_builder.hpp>

int main()
{
    boundary_mesh::SlidingSurfaceSet surfaces;
    const std::vector<boundary_mesh::SlidingVertexInput> vertices;
    const auto result = boundary_mesh::SlidingConstraintBuilder{}.build(
        surfaces, vertices, 1.0, 1.0e-12, 0);
    return result.hasValue() ? 0 : 1;
}
```

- [ ] **Step 2: Build and verify the new API is absent**

Run: `cmake --build build --config Debug --target boundary_mesh_sliding_surface_module_boundary_test`

Expected: compilation fails because `boundary_mesh/sliding/...` and `SlidingVertexInput` do not exist.

- [ ] **Step 3: Define neutral public types**

Create `sliding_error.hpp`:

```cpp
#pragma once
#include <cstddef>
#include <cstdint>
#include <variant>
#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh {
struct SlidingInputMismatch { std::uint32_t region_id{}; };
struct InvalidSlidingSurface { std::uint32_t region_id{}; SurfaceFaceId source_face_id{}; };
struct OverConstrainedGrowthVertex {
    std::size_t front_vertex_index{}; VertexId source_vertex_id{}; std::uint32_t layer{};
};
struct UndefinedConstrainedDirection {
    std::size_t front_vertex_index{}; VertexId source_vertex_id{}; std::uint32_t layer{};
};
struct SlidingProjectionNotConverged {
    std::size_t front_vertex_index{}; VertexId source_vertex_id{}; std::uint32_t layer{};
    std::vector<std::uint32_t> region_ids; std::uint32_t iterations{};
    Scalar position_change{}; Scalar max_surface_residual{};
};
using SlidingError = std::variant<SlidingInputMismatch, InvalidSlidingSurface,
    OverConstrainedGrowthVertex, UndefinedConstrainedDirection,
    SlidingProjectionNotConverged>;
}
```

Create `sliding_vertex_input.hpp`:

```cpp
#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>
#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh {
struct SlidingVertexInput {
    std::size_t vertex_index{};
    VertexId source_vertex_id{};
    std::vector<std::uint32_t> region_ids;
};
}
```

Change builder declaration to:

```cpp
Result<SlidingConstraints, SlidingError> build(
    const SlidingSurfaceSet &surfaces,
    const std::vector<SlidingVertexInput> &vertices,
    Scalar characteristic_length,
    Scalar effective_length_tolerance,
    std::uint32_t layer) const;
```

Change `apply`, `constrainDirection`, and `projectPosition` return errors from `GrowthDirectionError` to `SlidingError`. Mechanical file moves must use `git mv`; update includes to `boundary_mesh/sliding/...`, and update CMake source paths to `src/sliding/...`.

- [ ] **Step 4: Port the implementation without growth includes**

Replace every read of `GrowthFront`/`FrontEvaluation` with the corresponding `SlidingVertexInput` member:

```cpp
for (const SlidingVertexInput &vertex : vertices) {
    VertexSlidingConstraint constraint;
    constraint.front_vertex_index = vertex.vertex_index;
    constraint.source_vertex_id = vertex.source_vertex_id;
    for (const std::uint32_t region_id : vertex.region_ids) {
        const SlidingSurface *surface = surfaces.find(region_id);
        if (surface == nullptr)
            return BuildResult::failure(SlidingInputMismatch{region_id});
        // Preserve the existing independent-normal selection and plane creation.
    }
    constraints.vertices_.push_back(std::move(constraint));
}
```

Retain the existing numerical formulas and tolerances; only substitute neutral inputs and sliding error variants. Confirm with:

Run: `rg -n "boundary_mesh/growth/|GrowthFront|FrontEvaluation|GrowthDirectionError" include/boundary_mesh/sliding src/sliding`

Expected: no matches.

- [ ] **Step 5: Update focused tests and run them**

Tests construct `SlidingVertexInput` directly, for example:

```cpp
const std::vector<SlidingVertexInput> vertices{{
    0, VertexId{0}, {7}
}};
```

Run: `cmake --build build --config Debug --target boundary_mesh_sliding_constraints_test boundary_mesh_sliding_surface_builder_test boundary_mesh_sliding_surface_module_boundary_test`

Run: `ctest --test-dir build -C Debug --output-on-failure -R "boundary_mesh_(sliding_constraints|sliding_surface_builder|sliding_surface_module_boundary)_test"`

Expected: all tests pass.

- [ ] **Step 6: Commit**

```powershell
git add CMakeLists.txt include/boundary_mesh/sliding src/sliding tests/cmake/sliding_surface_module_boundary_test.cpp tests/unit/growth/sliding_constraints_test.cpp tests/unit/growth/sliding_surface_builder_test.cpp
git commit -m "refactor: decouple sliding surface API from growth"
```

### Task 4: Add the BoundaryLayer adapter and preserve existing pipeline behavior

**Files:**
- Create: `include/boundary_mesh/growth/sliding_constraint_adapter.hpp`
- Create: `src/growth/sliding_constraint_adapter.cpp`
- Create: forwarding headers at the four former `include/boundary_mesh/growth/sliding_*.hpp` paths
- Modify: `src/growth/regular_layer_generator.cpp`
- Modify: `src/growth/regular_layer_stepper.cpp`
- Modify: other files returned by `rg -l "growth/sliding_|SlidingConstraintBuilder" include src tests --glob '!include/boundary_mesh/sliding/**'`
- Modify: `CMakeLists.txt`
- Test: `tests/integration/regular_layer_growth_pipeline_test.cpp`

**Interfaces:**
- Consumes: neutral SlidingSurface API from Task 3, `GrowthFront`, `FrontEvaluation`.
- Produces: `buildGrowthSlidingConstraints(...) -> Result<SlidingConstraints, GrowthDirectionError>`; `GrowthDirectionError` includes the shared sliding error alternatives declared by the sliding library.

- [ ] **Step 1: Add an integration assertion for both boundary kinds**

Keep the existing generated side checks and add an explicit kind-preservation helper:

```cpp
const bool has_symmetry = std::any_of(tags.begin(), tags.end(), [](const auto &tag) {
    return tag.kind == SurfaceBoundaryKind::Symmetry && tag.region_id == 30;
});
const bool has_internal = std::any_of(tags.begin(), tags.end(), [](const auto &tag) {
    return tag.kind == SurfaceBoundaryKind::Internal && tag.region_id == 40;
});
if (!has_symmetry || !has_internal) return 30;
```

- [ ] **Step 2: Build and verify callers fail against the neutral builder**

Run: `cmake --build build --config Debug --target boundary_mesh_regular_layer_growth_pipeline_test`

Expected: compilation fails at old `SlidingConstraintBuilder::build(mesh, front, evaluation)` calls.

- [ ] **Step 3: Implement the adapter**

Declare:

```cpp
Result<SlidingConstraints, GrowthDirectionError> buildGrowthSlidingConstraints(
    const SlidingSurfaceSet &surfaces,
    const GrowthFront &front,
    const FrontEvaluation &evaluation,
    std::uint32_t layer);
```

Build neutral inputs by index:

```cpp
std::vector<SlidingVertexInput> inputs;
inputs.reserve(front.vertices.size());
for (std::size_t i = 0; i < front.vertices.size(); ++i) {
    const auto &vertex = front.vertices[i];
    inputs.push_back(SlidingVertexInput{
        i, vertex.source_vertex_id, vertex.boundary.sliding_region_ids});
}
```

Call the neutral builder with `evaluation.characteristic_length`, `evaluation.effective_length_tolerance`, and `front.layer`. Move the five existing sliding-related error structs from `growth_direction_error.hpp` into `sliding_error.hpp`; include that header and retain those same alternatives in `GrowthDirectionError`. Convert a failed `SlidingError` into `GrowthDirectionError` with `std::visit`, preserving the contained value without field translation.

- [ ] **Step 4: Add forwarding headers and switch production callers**

Each compatibility header contains only:

```cpp
#pragma once
#include <boundary_mesh/sliding/sliding_constraints.hpp>
```

Use the matching new header for each old path. Production growth code calls `buildGrowthSlidingConstraints`; code that only consumes `SlidingConstraints` includes the new public header directly.

- [ ] **Step 5: Run sliding and layer pipeline tests**

Run: `cmake --build build --config Debug --target boundary_mesh_regular_layer_growth_pipeline_test boundary_mesh_regular_layer_stepper_test boundary_mesh_sliding_constraints_test`

Run: `ctest --test-dir build -C Debug --output-on-failure -R "boundary_mesh_(regular_layer_growth_pipeline|regular_layer_stepper|sliding_constraints)_test"`

Expected: all pass, including preserved Symmetry/Internal kinds and regions.

- [ ] **Step 6: Commit**

```powershell
git add CMakeLists.txt include/boundary_mesh/growth src/growth tests/integration/regular_layer_growth_pipeline_test.cpp
git commit -m "refactor: adapt boundary layer to sliding library"
```

### Task 5: Isolate Internal dual-layer topology mechanics inside Core

**Files:**
- Create: `src/mesh/surface_topology_layers.hpp`
- Create: `src/mesh/surface_topology_layers.cpp`
- Modify: `src/mesh/surface_topology_builder.cpp`
- Modify: `CMakeLists.txt`
- Test: `tests/unit/mesh/surface_topology_builder_test.cpp`

**Interfaces:**
- Consumes: `SurfaceBoundaryTag`, `SurfaceFaceId`, `OptionalSurfaceFaceId`.
- Produces: private Core helpers `topologyLayer(tag)`, `EdgeLayerIncidence::append(...)`, and `EdgeLayerIncidence::faces(...)`; no new public API.

- [ ] **Step 1: Add a regression case where Internal and non-Internal share an edge**

Assert the same `EdgeFaceIds` contains both independent layers and that face neighbors stay within kind layer:

```cpp
const EdgeFaceIds &shared = topology.edgeFaces()[shared_edge_id];
if (shared.non_internal_faces[0] != SurfaceFaceId{0} ||
    shared.non_internal_faces[1] != SurfaceFaceId{1} ||
    shared.internal_faces[0] != SurfaceFaceId{2} ||
    shared.internal_faces[1] != SurfaceFaceId{3})
    return 18;
```

- [ ] **Step 2: Run the regression before refactoring**

Run: `cmake --build build --config Debug --target boundary_mesh_surface_topology_builder_test`

Run: `ctest --test-dir build -C Debug --output-on-failure -R boundary_mesh_surface_topology_builder_test`

Expected: pass, documenting preserved behavior.

- [ ] **Step 3: Extract the private layer abstraction**

Create:

```cpp
#pragma once
#include <array>
#include <vector>
#include <boundary_mesh/mesh/mesh_surface.hpp>

namespace boundary_mesh::detail {
enum class SurfaceTopologyLayer { NonInternal, Internal };
constexpr SurfaceTopologyLayer topologyLayer(const SurfaceBoundaryTag &tag) noexcept {
    return tag.kind == SurfaceBoundaryKind::Internal
        ? SurfaceTopologyLayer::Internal
        : SurfaceTopologyLayer::NonInternal;
}
struct EdgeLayerIncidence {
    std::vector<SurfaceFaceId> non_internal;
    std::vector<SurfaceFaceId> internal;
    std::vector<SurfaceFaceId> &faces(SurfaceTopologyLayer layer) noexcept;
    const std::vector<SurfaceFaceId> &faces(SurfaceTopologyLayer layer) const noexcept;
};
}
```

Implement both `faces` overloads with a two-branch return. Replace the builder's local `isInternalFaceTag`, `EdgeIncidence`, and repeated ternaries with `topologyLayer(tag)` and `incidence.faces(layer)`. Keep public `EdgeFaceIds` conversion and validation rules unchanged.

- [ ] **Step 4: Run all Core topology tests**

Run: `cmake --build build --config Debug --target boundary_mesh_surface_topology_types_test boundary_mesh_surface_topology_validation_test boundary_mesh_surface_topology_builder_test boundary_mesh_surface_topology_edge_error_test`

Run: `ctest --test-dir build -C Debug --output-on-failure -R boundary_mesh_surface_topology`

Expected: all topology tests pass.

- [ ] **Step 5: Commit**

```powershell
git add CMakeLists.txt src/mesh/surface_topology_layers.hpp src/mesh/surface_topology_layers.cpp src/mesh/surface_topology_builder.cpp tests/unit/mesh/surface_topology_builder_test.cpp
git commit -m "refactor: isolate internal surface topology layer"
```

### Task 6: Make current collision filtering an explicit Spatial policy

**Files:**
- Create: `include/boundary_mesh/spatial/collision_boundary_policy.hpp`
- Create: `src/spatial/collision_boundary_policy.cpp`
- Modify: `include/boundary_mesh/spatial/collision_index.hpp`
- Modify: `src/spatial/collision_index.cpp`
- Modify: `include/boundary_mesh/growth/exposed_boundary.hpp`
- Modify: `src/growth/exposed_boundary.cpp`
- Modify: `CMakeLists.txt`
- Test: `tests/unit/spatial/collision_index_test.cpp`

**Interfaces:**
- Produces: `CollisionBoundaryPolicy::isObstacle(SurfaceBoundaryKind, CollisionSurfaceOrigin) const noexcept`.
- `CollisionSurfaceOrigin` distinguishes `InputSurface` from `GeneratedBoundary`; default results remain identical for both origins.

- [ ] **Step 1: Add focused default-policy tests**

```cpp
const CollisionBoundaryPolicy policy;
for (const auto origin : {CollisionSurfaceOrigin::InputSurface,
                          CollisionSurfaceOrigin::GeneratedBoundary}) {
    if (policy.isObstacle(SurfaceBoundaryKind::Symmetry, origin)) return 70;
    if (policy.isObstacle(SurfaceBoundaryKind::Internal, origin)) return 71;
    if (!policy.isObstacle(SurfaceBoundaryKind::Wall, origin)) return 72;
    if (!policy.isObstacle(SurfaceBoundaryKind::Farfield, origin)) return 73;
    if (!policy.isObstacle(SurfaceBoundaryKind::BoundaryLayerInterface, origin)) return 74;
}
```

- [ ] **Step 2: Build and verify the policy API is absent**

Run: `cmake --build build --config Debug --target boundary_mesh_collision_index_test`

Expected: compilation fails because `CollisionBoundaryPolicy` is not declared.

- [ ] **Step 3: Implement the explicit default policy**

```cpp
enum class CollisionSurfaceOrigin { InputSurface, GeneratedBoundary };

class CollisionBoundaryPolicy {
public:
    bool isObstacle(SurfaceBoundaryKind kind,
                    CollisionSurfaceOrigin origin) const noexcept;
};
```

Implementation:

```cpp
bool CollisionBoundaryPolicy::isObstacle(
    SurfaceBoundaryKind kind, CollisionSurfaceOrigin) const noexcept
{
    return !isSlidingBoundary(kind);
}
```

Pass a policy into collision-index construction with a default value to preserve source compatibility. Replace the hard-coded skip in `collision_index.cpp` with `!policy.isObstacle(kind, InputSurface)`. Give `ExposedBoundaryTracker` the same defaultable policy and use `GeneratedBoundary` when collecting dynamic collision triangles.

- [ ] **Step 4: Run collision and exposed-boundary tests**

Run: `cmake --build build --config Debug --target boundary_mesh_collision_index_test boundary_mesh_exposed_boundary_test boundary_mesh_collision_growth_pipeline_test`

Run: `ctest --test-dir build -C Debug --output-on-failure -R "boundary_mesh_(collision_index|exposed_boundary|collision_growth_pipeline)_test"`

Expected: all pass and existing Symmetry/Internal hits remain omitted.

- [ ] **Step 5: Commit**

```powershell
git add CMakeLists.txt include/boundary_mesh/spatial/collision_boundary_policy.hpp include/boundary_mesh/spatial/collision_index.hpp include/boundary_mesh/growth/exposed_boundary.hpp src/spatial/collision_boundary_policy.cpp src/spatial/collision_index.cpp src/growth/exposed_boundary.cpp tests/unit/spatial/collision_index_test.cpp
git commit -m "refactor: expose collision boundary policy"
```

### Task 7: Complete migration, documentation, and full verification

**Files:**
- Modify: remaining files reported by classification/include audits below
- Modify: `README.md`
- Modify: `docs/superpowers/specs/2026-09-07-sliding-surface-library-design.md` only if implementation names differ for a justified reason

**Interfaces:**
- Consumes: all preceding targets and APIs.
- Produces: documented library graph and a clean dependency audit.

- [ ] **Step 1: Audit forbidden and duplicated dependencies**

Run:

```powershell
rg -n "boundary_mesh/growth/|GrowthFront|FrontEvaluation|GrowthDirectionError" include/boundary_mesh/sliding src/sliding
rg -n "kind == SurfaceBoundaryKind::Symmetry|kind != SurfaceBoundaryKind::Symmetry" include src --glob '!third/**'
```

Expected: first command has no matches. Second command has no duplicated Symmetry/Internal classification expressions; any Symmetry-only semantic behavior must be individually justified in README comments.

- [ ] **Step 2: Update README target and processing documentation**

Add the target description:

```markdown
- `BoundaryMesh::SlidingSurface`：Symmetry/Internal 区域建模、方向约束与位置投影；仅依赖 Core 和 Spatial，不依赖边界层生成流程。
- `BoundaryMesh::Spatial`：空间索引与碰撞；当前默认碰撞策略暂不把 Symmetry/Internal 作为障碍，后续碰撞语义在该模块独立扩展。
```

Update the pipeline text to state that BoundaryLayer converts front data through an adapter and that Core retains Internal dual-layer topology.

- [ ] **Step 3: Configure and build the full project**

Run: `cmake -S . -B build -DBOUNDARY_MESH_ENABLE_CGNS_IO=ON`

Run: `cmake --build build --config Debug --parallel`

Expected: all targets compile and link.

- [ ] **Step 4: Run the complete test suite**

Run: `ctest --test-dir build -C Debug --output-on-failure`

Expected: 100% tests passed, 0 failed.

- [ ] **Step 5: Verify the no-IO configuration**

Run: `cmake -S . -B build-no-io -DBOUNDARY_MESH_ENABLE_CGNS_IO=OFF`

Run: `cmake --build build-no-io --config Debug --parallel`

Run: `ctest --test-dir build-no-io -C Debug --output-on-failure`

Expected: configure/build pass and all enabled tests pass.

- [ ] **Step 6: Commit**

```powershell
git add README.md include src tests CMakeLists.txt docs/superpowers/specs/2026-09-07-sliding-surface-library-design.md
git commit -m "docs: describe sliding and collision module boundaries"
```
