# Boundary Layer Transition Module Consolidation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove the reserved-layer implementation, isolate incremental transition mechanics, and make `BoundaryMesh::BoundaryLayer` the acyclic top-level orchestration library.

**Architecture:** Rename the existing growth-only CMake target to `BoundaryMesh::Growth`; keep transition decisions in `BoundaryMesh::Transition`; create a true `BoundaryMesh::BoundaryLayer` target that orchestrates Growth, Transition, and MultiNormal. Extract provisional topology construction and final topology materialization from the incremental generator, then remove every non-historical reserved API and test.

**Tech Stack:** C++17, CMake, CTest, existing `boundary_mesh::Result` and mesh types.

## Global Constraints

- This is an intentional breaking API change; do not provide forwarding headers or deprecated wrappers for reserved APIs or old include paths.
- Preserve all current incremental mesh behavior, stop propagation, collision rollback, deterministic tie-breaking, and error propagation.
- Keep the dependency graph acyclic: Growth, Transition, and MultiNormal must not depend on the top-level BoundaryLayer target.
- Every final boundary-layer interface face remains a Triangle.
- Adjacent source faces remain limited to a final layer difference of zero or one.
- Historical files under `docs/superpowers/specs/` and `docs/superpowers/plans/` remain unchanged except for this design and plan.

---

## File Structure

**Create**

- `include/boundary_mesh/boundary_layer/boundary_layer_generator.hpp`: public complete-generation entry point.
- `include/boundary_mesh/boundary_layer/incremental_boundary_layer_generator.hpp`: internal incremental orchestration API and error union.
- `include/boundary_mesh/boundary_layer/incremental_topology_finalizer.hpp`: final topology materialization API.
- `src/boundary_layer/boundary_layer_generator.cpp`: MultiNormal, incremental growth, and merge orchestration.
- `src/boundary_layer/incremental_boundary_layer_generator.cpp`: per-layer transaction wiring only.
- `src/boundary_layer/incremental_topology_finalizer.cpp`: final caps, side transitions, triangular top, and farfield reconstruction.
- `include/boundary_mesh/transition/transition_template_types.hpp`: shared incremental template result and error types.
- `include/boundary_mesh/transition/triangle_side_transition.hpp`: one-layer Triangle side-transition API.
- `src/transition/triangle_side_transition.cpp`: one-layer Triangle side-transition implementation.
- `include/boundary_mesh/transition/provisional_transition_builder.hpp`: provisional transition request and function declaration.
- `src/transition/provisional_transition_builder.cpp`: candidate collision surface and temporary transition construction.
- `tests/unit/transition/triangle_side_transition_test.cpp`: focused Triangle step template tests.
- `tests/unit/transition/provisional_transition_builder_test.cpp`: provisional builder behavior tests.
- `tests/unit/boundary_layer/incremental_topology_finalizer_test.cpp`: finalizer behavior tests.

**Modify**

- `CMakeLists.txt`: Growth rename, Transition dependency cleanup, and real BoundaryLayer target.
- `tests/CMakeLists.txt`: target relinking, new tests, and obsolete-test removal.
- `include/boundary_mesh/transition/incremental_transition_types.hpp`: own incremental coordination errors.
- `include/boundary_mesh/transition/incremental_transition_templates.hpp`: depend only on new template types.
- `include/boundary_mesh/transition/corner_suppression.hpp`: remove old coordination include.
- `include/boundary_mesh/transition/layer_transition_resolver.hpp`: use incremental-owned errors.
- `src/transition/corner_suppression.cpp`: no behavior change; compile against moved types.
- `src/cli/boundary_mesh_command.cpp`: include the new public BoundaryLayer header.
- `README.md`: document only the active architecture and API.
- `docs/reserved_layer_transition.md`: rename and reduce to the active incremental algorithm.

**Delete**

- All production and test files listed in section 6 and section 7 of the approved design.
- `include/boundary_mesh/transition/boundary_layer_generator.hpp`
- `include/boundary_mesh/transition/incremental_boundary_layer_generator.hpp`
- `src/transition/boundary_layer_generator.cpp`
- `src/transition/incremental_boundary_layer_generator.cpp`
- Old Triangle and generic Quad transition-template files after their active behavior is migrated.

---

### Task 1: Establish Acyclic Build Targets

**Files:**

- Modify: `CMakeLists.txt:157-289`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**

- Produces: `BoundaryMesh::Growth`, containing only `src/growth/*.cpp`.
- Preserves temporarily: `BoundaryMesh::BoundaryLayer` as a compatibility target name until Task 6 populates it with orchestration sources.
- Consumes: existing `BoundaryMesh::Core`, `Surface`, `Quality`, `Spatial`, and `SlidingSurface` targets.

- [ ] **Step 1: Add a failing CMake architecture check**

Create a `tests/cmake/boundary_layer_target_boundary_test.cmake` assertion that reads `CMakeLists.txt` and fails unless it finds `add_library(BoundaryMesh::Growth ALIAS boundary_mesh_growth)` and verifies the Transition link block contains `BoundaryMesh::Growth` but not `BoundaryMesh::BoundaryLayer` or `BoundaryMesh::MultiNormal`.

- [ ] **Step 2: Register and run the failing check**

Run:

```powershell
cmake -DPROJECT_ROOT=$PWD -P tests/cmake/boundary_layer_target_boundary_test.cmake
```

Expected: failure reporting that `BoundaryMesh::Growth` is missing.

- [ ] **Step 3: Rename the growth-only target and update direct consumers**

Change the CMake declarations to the following shape:

```cmake
add_library(boundary_mesh_growth STATIC
    src/growth/growth_patch_builder.cpp
    src/growth/growth_front_builder.cpp
    src/growth/front_adjacency.cpp
    src/growth/isotropic_stop_evaluator.cpp
    src/growth/front_evaluator.cpp
    src/growth/growth_direction.cpp
    src/growth/growth_field_smoother.cpp
    src/growth/skewness_direction_refiner.cpp
    src/growth/sliding_constraint_adapter.cpp
    src/growth/growth_profile_builder.cpp
    src/growth/regular_layer_stepper.cpp
    src/growth/regular_layer_generator.cpp
    src/growth/exposed_boundary.cpp
    src/growth/farfield_boundary_builder.cpp
    src/growth/layer_collision_checker.cpp
    src/growth/face_layer_constraint.cpp
    src/growth/termination_propagator.cpp
)
add_library(BoundaryMesh::Growth ALIAS boundary_mesh_growth)
target_link_libraries(boundary_mesh_growth PUBLIC
    BoundaryMesh::Core
    BoundaryMesh::Surface
    BoundaryMesh::Quality
    BoundaryMesh::Spatial
    BoundaryMesh::SlidingSurface
)
```

Update MultiNormal and Transition to link `BoundaryMesh::Growth`. Relink growth-only unit tests to `BoundaryMesh::Growth`. Do not create the final orchestration target in this task.

- [ ] **Step 4: Verify configure and focused build**

Run:

```powershell
cmake -S . -B build
cmake --build build --target boundary_mesh_growth boundary_mesh_multi_normal boundary_mesh_transition -j 4
cmake -DPROJECT_ROOT=$PWD -P tests/cmake/boundary_layer_target_boundary_test.cmake
```

Expected: configure succeeds, all three libraries build, and the architecture script exits zero.

- [ ] **Step 5: Commit**

```powershell
git add CMakeLists.txt tests/CMakeLists.txt tests/cmake/boundary_layer_target_boundary_test.cmake
git commit -m "refactor: separate growth build target"
```

---

### Task 2: Replace Reserved Template Types with Incremental Types

**Files:**

- Create: `include/boundary_mesh/transition/transition_template_types.hpp`
- Create: `include/boundary_mesh/transition/triangle_side_transition.hpp`
- Create: `src/transition/triangle_side_transition.cpp`
- Create: `tests/unit/transition/triangle_side_transition_test.cpp`
- Modify: `include/boundary_mesh/transition/incremental_transition_templates.hpp`
- Modify: `src/transition/incremental_transition_templates.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**

- Produces:

```cpp
struct TransitionTemplateOutput {
    SurfaceFaceId source_face_id{};
    std::vector<Point3> created_vertices;
    std::vector<VolumeCell> volume_cells;
    std::vector<CellMetadata> metadata;
    std::vector<Triangle> top_faces;
};

struct TriangleSideTransitionInput {
    SurfaceFaceId source_face_id{};
    std::uint32_t low_layer{};
    std::array<VertexId, 3> low{};
    std::array<VertexId, 3> high{};
    std::size_t high_edge_local_index{};
};

Result<TransitionTemplateOutput, TransitionTemplateError>
buildTriangleSideTransition(const TriangleSideTransitionInput &input);
```

- Consumes: existing mesh cell types and `InvalidTransitionTemplateInput` semantics.

- [ ] **Step 1: Write the focused Triangle one-layer tests**

Cover all three rotated high edges. For each result assert one Pyramid, matching metadata, no created vertices, and the expected three exposed top triangles. Add invalid-edge coverage with `high_edge_local_index == 3`.

- [ ] **Step 2: Run the test and verify RED**

Run:

```powershell
cmake -S . -B build
cmake --build build --target boundary_mesh_triangle_side_transition_test -j 4
```

Expected: build failure because `triangle_side_transition.hpp` or `buildTriangleSideTransition()` does not exist.

- [ ] **Step 3: Add the shared types and minimal Triangle implementation**

Move only errors and output fields used by the incremental path into `transition_template_types.hpp`. Implement the one-layer Triangle Pyramid construction by translating the existing `trial_layers == 2`/single-high-edge behavior into explicit `low` and `high` arrays. Set metadata layer from `low_layer + 1` inside the function.

- [ ] **Step 4: Switch Quad incremental templates to the shared output type**

Replace `IncrementalTransitionResult` with `TransitionTemplateOutput` in all three Quad function return types. Preserve field order and values; this is a type ownership change only.

- [ ] **Step 5: Verify GREEN and existing Quad templates**

Run:

```powershell
cmake --build build --target boundary_mesh_triangle_side_transition_test boundary_mesh_incremental_transition_templates_test -j 4
ctest --test-dir build -R "triangle_side_transition|incremental_transition_templates" --output-on-failure
```

Expected: both tests pass.

- [ ] **Step 6: Commit**

```powershell
git add include/boundary_mesh/transition/transition_template_types.hpp include/boundary_mesh/transition/triangle_side_transition.hpp src/transition/triangle_side_transition.cpp include/boundary_mesh/transition/incremental_transition_templates.hpp src/transition/incremental_transition_templates.cpp tests/unit/transition/triangle_side_transition_test.cpp CMakeLists.txt tests/CMakeLists.txt
git commit -m "refactor: define incremental transition template API"
```

---

### Task 3: Make Incremental Coordination Types Self-Contained

**Files:**

- Modify: `include/boundary_mesh/transition/incremental_transition_types.hpp`
- Modify: `include/boundary_mesh/transition/corner_suppression.hpp`
- Modify: `include/boundary_mesh/transition/layer_transition_resolver.hpp`
- Modify: `tests/unit/transition/incremental_transition_types_test.cpp`

**Interfaces:**

- Produces: `TransitionCoordinationError` and its four error alternatives without `FaceLayerState` or `CoordinatedTransitionFace`.
- Consumes: `SurfaceFaceId`, `VertexId`, and `LayerFaceSets` only.

- [ ] **Step 1: Add a header-isolation compile test**

Extend `incremental_transition_types_test.cpp` to instantiate every `TransitionCoordinationError` alternative while including only `incremental_transition_types.hpp`. Add compile-time assertions that the variant accepts each alternative.

- [ ] **Step 2: Verify RED**

Run:

```powershell
cmake --build build --target boundary_mesh_incremental_transition_types_test -j 4
```

Expected: compile failure because the coordination types still live in `transition_coordination.hpp`.

- [ ] **Step 3: Move the error types and remove old includes**

Move `MissingTransitionFaceState`, `UncoordinatedTransitionLayerDifference`, `MultipleTransitionHighEdges`, `TransitionCornerLayerViolation`, and `TransitionCoordinationError` into `incremental_transition_types.hpp`. Change `corner_suppression.hpp` and `layer_transition_resolver.hpp` to include that header directly.

- [ ] **Step 4: Verify GREEN and consumers**

Run:

```powershell
cmake --build build --target boundary_mesh_incremental_transition_types_test boundary_mesh_corner_suppression_test boundary_mesh_layer_transition_resolver_test -j 4
ctest --test-dir build -R "incremental_transition_types|corner_suppression|layer_transition_resolver" --output-on-failure
```

Expected: all three tests pass.

- [ ] **Step 5: Commit**

```powershell
git add include/boundary_mesh/transition/incremental_transition_types.hpp include/boundary_mesh/transition/corner_suppression.hpp include/boundary_mesh/transition/layer_transition_resolver.hpp tests/unit/transition/incremental_transition_types_test.cpp
git commit -m "refactor: internalize incremental coordination types"
```

---

### Task 4: Extract the Provisional Transition Builder

**Files:**

- Create: `include/boundary_mesh/transition/provisional_transition_builder.hpp`
- Create: `src/transition/provisional_transition_builder.cpp`
- Create: `tests/unit/transition/provisional_transition_builder_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**

- Produces:

```cpp
struct ProvisionalTransitionBuildInput {
    const GrowthFront *current_front{};
    const GrowthFront *candidate_front{};
    const std::vector<SurfaceFaceId> *retained_high_faces{};
    const LayerFaceSets *face_sets{};
    Scalar length_tolerance{1e-12};
};

Result<ProvisionalLayerTransition, TransitionTemplateError>
buildProvisionalTransition(const ProvisionalTransitionBuildInput &input);
```

- Consumes: Triangle/Quad incremental templates, high-neighbor selector, diagonal selection, and boundary ownership types.

- [ ] **Step 1: Write builder tests**

Add three small real-front fixtures: retained Triangle candidate produces a `RegularCandidate` proxy; stopped Triangle beside one retained neighbor produces a `SideTransition` whose rollback list contains that neighbor; stopped Quad beside one retained neighbor emits one diagonal requirement and TopCap/SideTransition owners with the same dependency.

- [ ] **Step 2: Verify RED**

Run:

```powershell
cmake -S . -B build
cmake --build build --target boundary_mesh_provisional_transition_builder_test -j 4
```

Expected: build failure because the builder header/function is missing.

- [ ] **Step 3: Move the existing anonymous helpers and builder**

Move `cellKey`, `edgeKey`, `oriented`, `faceIds`, `retainedFace`, `appendOwnedTriangle`, `appendFrontFace`, and the provisional construction loop. Replace the external `optional<TransitionTemplateError>` side channel with direct `Result::failure(error)` returns. Use `buildTriangleSideTransition()` for Triangle low faces.

- [ ] **Step 4: Verify GREEN and collision consumers**

Run:

```powershell
cmake --build build --target boundary_mesh_provisional_transition_builder_test boundary_mesh_transition_boundary_checker_test boundary_mesh_layer_transition_resolver_test -j 4
ctest --test-dir build -R "provisional_transition_builder|transition_boundary_checker|layer_transition_resolver" --output-on-failure
```

Expected: all tests pass.

- [ ] **Step 5: Commit**

```powershell
git add include/boundary_mesh/transition/provisional_transition_builder.hpp src/transition/provisional_transition_builder.cpp tests/unit/transition/provisional_transition_builder_test.cpp CMakeLists.txt tests/CMakeLists.txt
git commit -m "refactor: extract provisional transition builder"
```

---

### Task 5: Extract Incremental Topology Finalization

**Files:**

- Create: `include/boundary_mesh/boundary_layer/incremental_topology_finalizer.hpp`
- Create: `src/boundary_layer/incremental_topology_finalizer.cpp`
- Create: `tests/unit/boundary_layer/incremental_topology_finalizer_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**

- Produces:

```cpp
Result<RegularLayerGrowthResult, IncrementalLayerGrowthError>
finalizeIncrementalLayerTopology(
    const SurfaceMesh &surface_mesh,
    const GrowthFront &initial_front,
    RegularLayerGrowthResult regular);
```

- Consumes: new Triangle side template, incremental Quad templates, diagonal table, and high-neighbor selector.

- [ ] **Step 1: Add finalizer-level tests**

Extract representative assertions from `incremental_layer_transition_pipeline_test.cpp`: a zero-layer Quad becomes exactly two top triangles without a volume cell; a one-layer Quad replaces its top Hexa with the expected Pyramid/Tetra cap; a Triangle/Quad height step produces the expected side-transition cell types; farfield output excludes stale BoundaryLayerInterface faces and includes the finalized top.

- [ ] **Step 2: Verify RED**

Run:

```powershell
cmake -S . -B build
cmake --build build --target boundary_mesh_incremental_topology_finalizer_test -j 4
```

Expected: build failure because the new BoundaryLayer header/target source is absent.

- [ ] **Step 3: Move finalizer code and private helpers**

Move `FinalQuad`, `FinalTriangle`, layer-record lookup, top triangle ownership filtering, farfield remapping, and all of `finalizeIncrementalLayerTopology()` into the new source. Replace `buildTriangleTransition({..., 1, ...})` with `buildTriangleSideTransition({source_face_id, low_layer, low, high, high_edge})`; remove post-call metadata layer rewriting because the new template owns that responsibility.

- [ ] **Step 4: Verify GREEN and pipeline behavior**

Run:

```powershell
cmake --build build --target boundary_mesh_incremental_topology_finalizer_test boundary_mesh_incremental_layer_transition_pipeline_test -j 4
ctest --test-dir build -R "incremental_topology_finalizer|incremental_layer_transition_pipeline" --output-on-failure
```

Expected: both tests pass with unchanged mesh counts and topology assertions.

- [ ] **Step 5: Commit**

```powershell
git add include/boundary_mesh/boundary_layer/incremental_topology_finalizer.hpp src/boundary_layer/incremental_topology_finalizer.cpp tests/unit/boundary_layer/incremental_topology_finalizer_test.cpp CMakeLists.txt tests/CMakeLists.txt
git commit -m "refactor: extract incremental topology finalizer"
```

---

### Task 6: Move Complete Generation into the BoundaryLayer Module

**Files:**

- Create: `include/boundary_mesh/boundary_layer/incremental_boundary_layer_generator.hpp`
- Create: `src/boundary_layer/incremental_boundary_layer_generator.cpp`
- Create: `include/boundary_mesh/boundary_layer/boundary_layer_generator.hpp`
- Create: `src/boundary_layer/boundary_layer_generator.cpp`
- Modify: `src/cli/boundary_mesh_command.cpp`
- Modify: `CMakeLists.txt`
- Modify: pipeline tests that include the old paths.

**Interfaces:**

- Produces: unchanged `generateBoundaryLayers(...)` public signature at the new include path.
- Produces: unchanged `generateIncrementalBoundaryLayers(...)` signature at the new internal include path.
- Consumes: `BoundaryMesh::Growth`, `BoundaryMesh::Transition`, and `BoundaryMesh::MultiNormal`.

- [ ] **Step 1: Change pipeline tests to the desired public include path**

Replace imports of `boundary_mesh/transition/boundary_layer_generator.hpp` with `boundary_mesh/boundary_layer/boundary_layer_generator.hpp` and add a CMake module-boundary script verifying no `src/transition` file defines either complete-generation function.

- [ ] **Step 2: Verify RED**

Run:

```powershell
cmake -S . -B build
cmake --build build --target boundary_mesh_boundary_layer_generation_pipeline_test -j 4
```

Expected: compile failure because the new public header does not exist.

- [ ] **Step 3: Move the two generators and reduce incremental orchestration**

Move `generateBoundaryLayers()` unchanged. Move `generateIncrementalBoundaryLayers()` and retain only rejection merging, stopped-front carry, resolver invocation, error conversion, regular generation, and finalizer invocation. Call the extracted builder through:

```cpp
input.build_provisional = [&effective_current, &candidate](
    const auto &retained, const auto &sets) {
    return buildProvisionalTransition({
        &effective_current, &candidate.next_front, &retained, &sets, 1e-12});
};
```

Convert builder template errors directly into `IncrementalLayerGrowthError`.

- [ ] **Step 4: Create the real BoundaryLayer target**

Define:

```cmake
add_library(boundary_mesh_boundary_layer STATIC
    src/boundary_layer/boundary_layer_generator.cpp
    src/boundary_layer/incremental_boundary_layer_generator.cpp
    src/boundary_layer/incremental_topology_finalizer.cpp
)
add_library(BoundaryMesh::BoundaryLayer ALIAS boundary_mesh_boundary_layer)
target_link_libraries(boundary_mesh_boundary_layer PUBLIC
    BoundaryMesh::Growth
    BoundaryMesh::Transition
    BoundaryMesh::MultiNormal
)
```

Make CLI and complete pipeline tests link the real BoundaryLayer target. Confirm Transition links Growth/Core/Surface only as required by its headers and implementations.

- [ ] **Step 5: Verify GREEN and dependency boundaries**

Run:

```powershell
cmake -S . -B build
cmake --build build --target boundary_mesh_boundary_layer_generation_pipeline_test boundary_mesh_incremental_layer_transition_pipeline_test boundary_mesh_cli_support -j 4
ctest --test-dir build -R "boundary_layer_generation_pipeline|incremental_layer_transition_pipeline|boundary_layer_target_boundary" --output-on-failure
```

Expected: all focused builds and tests pass; CMake reports no dependency cycle.

- [ ] **Step 6: Commit**

```powershell
git add include/boundary_mesh/boundary_layer src/boundary_layer src/cli/boundary_mesh_command.cpp CMakeLists.txt tests/CMakeLists.txt tests
git commit -m "refactor: introduce boundary layer orchestration module"
```

---

### Task 7: Delete the Reserved Path and Obsolete Tests

**Files:**

- Delete: reserved production files, old generic template files, old transition generator files, and obsolete tests listed in the design.
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**

- Removes: every reserved API and old include path listed in design section 6.
- Preserves: new incremental template, resolver, builder, finalizer, and public BoundaryLayer interfaces.

- [ ] **Step 1: Add a source-removal assertion script**

Create `tests/cmake/no_reserved_transition_api_test.cmake`. It must scan non-historical production headers, sources, CMake, README, and the active algorithm document and fail on `generateReservedLayerTransition`, `makeReservedTrialProfiles`, `ReservedTransitionLayerCount`, `FaceLayerState`, `CoordinatedTransitionFace`, `buildQuadTransition`, `buildTriangleTransition`, or old transition generator include paths.

- [ ] **Step 2: Verify RED**

Run:

```powershell
cmake -DPROJECT_ROOT=$PWD -P tests/cmake/no_reserved_transition_api_test.cmake
```

Expected: failure listing current reserved declarations and sources.

- [ ] **Step 3: Delete obsolete sources, headers, and tests**

Use `apply_patch` deletions for the exact files approved in the design. Remove their CMake source entries, executable targets, and CTest registrations. Delete old `transition/boundary_layer_generator` and `transition/incremental_boundary_layer_generator` files after Tasks 5–6 have moved their active content.

- [ ] **Step 4: Remove residual old template usage**

Ensure all active callers use `buildTriangleSideTransition`, `buildQuadTopCap`, `buildQuadSideTransition`, or `buildQuadAdjacentSideTransition`. Remove generic `SourceTransitionResult`, `TriangleTransitionInput`, and `QuadTransitionInput` declarations.

- [ ] **Step 5: Verify source removal and transition tests**

Run:

```powershell
cmake -S . -B build
cmake --build build --target boundary_mesh_transition boundary_mesh_boundary_layer -j 4
cmake -DPROJECT_ROOT=$PWD -P tests/cmake/no_reserved_transition_api_test.cmake
ctest --test-dir build -R "transition|boundary_layer" --output-on-failure
```

Expected: build succeeds, removal script exits zero, and every remaining matching test passes.

- [ ] **Step 6: Commit**

```powershell
git add -A CMakeLists.txt tests include/boundary_mesh/transition src/transition include/boundary_mesh/boundary_layer src/boundary_layer
git commit -m "refactor: remove reserved layer transition path"
```

---

### Task 8: Update Active Documentation

**Files:**

- Modify: `README.md`
- Delete: `docs/reserved_layer_transition.md`
- Create: `docs/incremental_boundary_layer_transition.md`

**Interfaces:**

- Documents: `generateBoundaryLayers()` at `boundary_mesh/boundary_layer/boundary_layer_generator.hpp` as the only complete-generation entry point.
- Removes: claims that reserved profiles, generic multi-layer templates, or reserved orchestration remain available.

- [ ] **Step 1: Make the source-removal script cover active docs**

Run the Task 7 script before editing documentation and confirm it reports README and the old active document as remaining matches.

- [ ] **Step 2: Rewrite the architecture overview**

Document the dependency chain `CLI → BoundaryLayer → {Growth, Transition, MultiNormal}`, the per-layer fixed-point process, carried accepted stops, canonical diagonals, collision ownership, and final topology materialization. Preserve useful incremental explanations from the first half of the old document and remove its reserved compatibility section.

- [ ] **Step 3: Verify documentation and links**

Run:

```powershell
cmake -DPROJECT_ROOT=$PWD -P tests/cmake/no_reserved_transition_api_test.cmake
rg -n "transition/(boundary_layer_generator|incremental_boundary_layer_generator)\.hpp|generateReservedLayerTransition|makeReservedTrialProfiles" README.md docs/incremental_boundary_layer_transition.md include src tests CMakeLists.txt
```

Expected: script exits zero and `rg` prints no matches.

- [ ] **Step 4: Commit**

```powershell
git add README.md docs/reserved_layer_transition.md docs/incremental_boundary_layer_transition.md tests/cmake/no_reserved_transition_api_test.cmake
git commit -m "docs: describe incremental boundary layer architecture"
```

---

### Task 9: Full Verification

**Files:**

- Modify only if verification exposes a regression: the smallest owning component and its focused test.

**Interfaces:**

- Verifies all requirements in the approved design.

- [ ] **Step 1: Cleanly reconfigure and build all targets**

Run:

```powershell
cmake -S . -B build
cmake --build build -j 4
```

Expected: exit code zero and no newly introduced compiler warnings.

- [ ] **Step 2: Run the complete test suite**

Run:

```powershell
ctest --test-dir build --output-on-failure
```

Expected: 100% tests passed, zero failed.

- [ ] **Step 3: Check architecture and stale symbols**

Run:

```powershell
cmake -DPROJECT_ROOT=$PWD -P tests/cmake/boundary_layer_target_boundary_test.cmake
cmake -DPROJECT_ROOT=$PWD -P tests/cmake/no_reserved_transition_api_test.cmake
rg -n "generateReservedLayerTransition|ReservedTransitionLayerCount|makeReservedTrialProfiles|CoordinatedTransitionFace|buildQuadTransition|buildTriangleTransition" include src tests CMakeLists.txt README.md docs/incremental_boundary_layer_transition.md
```

Expected: both scripts exit zero and `rg` returns no matches.

- [ ] **Step 4: Inspect the final file responsibilities**

Run:

```powershell
Get-ChildItem include/boundary_mesh/boundary_layer,src/boundary_layer,include/boundary_mesh/transition,src/transition | Select-Object FullName
(Get-Content src/boundary_layer/incremental_boundary_layer_generator.cpp).Count
git diff --check HEAD~1
```

Expected: generators exist only under `boundary_layer`; transition contains only transition mechanics; the incremental generator is materially smaller than its original 1113 lines; diff check reports no whitespace errors outside known submodule metadata issues.

- [ ] **Step 5: Commit any verification-only fixes**

If Step 1–4 required changes, first add a focused failing regression test, implement the smallest correction, repeat the full verification, then commit:

```powershell
git add -A
git commit -m "fix: resolve transition consolidation regressions"
```

If no changes were required, do not create an empty commit.
