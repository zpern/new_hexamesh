# Multi-Normal Regular-Layer Pipeline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a production `generateBoundaryLayers()` pipeline that grows regular layers from the multi-normal transformed Wall front, merges both volume stages, and emits the true final top and farfield surfaces.

**Architecture:** Keep `generateRegularLayers()` as the pure regular-growth primitive and add a focused orchestration module beside it. The orchestrator passes the transformed front directly into the existing regular generator, merges through the explicit transition-interface mapping, and returns both stage diagnostics and final artifacts. Branch-aware exposed-surface identity is fixed at the shared collision/boundary key so every downstream surface builder preserves split branches.

**Tech Stack:** C++17, CMake/CTest, Eigen geometry types, existing CGNS reader and legacy VTK writers.

## Global Constraints

- Only faces tagged `Wall` enter multi-normal and regular-layer growth.
- `transformed_front` is regular layer zero; the transition is not counted in the requested regular layer count.
- Vertex identity within a layer is `(source_vertex_id, branch_id)`.
- Every split branch inherits the profile of its `source_vertex_id`.
- The regular mesh starts with transformed-front vertices in unchanged order.
- Final output contains merged volume, original Farfield faces, and the real exposed boundary-layer top.
- Multi-normal debug files are disabled by default.
- Existing `generateRegularLayers()` behavior and tests remain compatible.

---

### Task 1: Preserve branch identity in exposed surfaces

**Files:**
- Modify: `include/boundary_mesh/spatial/triangle_contact.hpp`
- Modify: `src/spatial/collision_index.cpp`
- Modify: `src/growth/farfield_boundary_builder.cpp`
- Test: `tests/integration/regular_layer_growth_pipeline_test.cpp`

**Interfaces:**
- Consumes: `GrowthFrontVertex::source_vertex_id`, `branch_id`, and layer number.
- Produces: `CollisionVertexKey { source_vertex_id, branch_id, layer }` equality used by collision and boundary surface assembly.

- [ ] **Step 1: Write a failing split-branch surface test**

Add a small exposed-boundary fixture containing two vertices with equal source
ID and layer but different branch IDs. Call the farfield/top builder and assert
that both output vertices remain present and are referenced independently.

```cpp
if (branch_zero_output_id == branch_one_output_id)
{
    return 30;
}
```

- [ ] **Step 2: Run the focused test and verify the collision**

Run:
`ctest --test-dir build -R boundary_mesh_regular_layer_growth_pipeline_test --output-on-failure`

Expected: FAIL because `sameKey()` currently compares only source ID and layer.

- [ ] **Step 3: Add branch identity to the shared key**

Extend `CollisionVertexKey` and every construction site:

```cpp
struct CollisionVertexKey
{
    VertexId source_vertex_id{};
    std::uint32_t branch_id{};
    std::uint32_t layer{};
};
```

Update equality/order helpers to compare all three fields. Original-surface
Farfield vertices use branch zero. Front-derived keys copy `branch_id` from the
corresponding `GrowthFrontVertex`.

- [ ] **Step 4: Run collision, regular growth, and exposed-boundary tests**

Run:
`ctest --test-dir build -R "collision|regular_layer_growth" --output-on-failure`

Expected: all selected tests PASS.

- [ ] **Step 5: Commit**

```text
git add include/boundary_mesh/spatial/triangle_contact.hpp src/spatial/collision_index.cpp src/growth/farfield_boundary_builder.cpp tests/integration/regular_layer_growth_pipeline_test.cpp
git commit -m "fix: preserve multi-normal branches in exposed surfaces"
```

### Task 2: Add the unified boundary-layer generator

**Files:**
- Create: `include/boundary_mesh/growth/boundary_layer_generator.hpp`
- Create: `src/growth/boundary_layer_generator.cpp`
- Create: `tests/integration/boundary_layer_generation_pipeline_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: complete `SurfaceMesh`, `SurfaceTopology`, `GrowthPatch`, initial `GrowthFront`, source profiles, `MultiNormalOptions`, and `RegularLayerGrowthOptions`.
- Produces: `generateBoundaryLayers(...) -> Result<BoundaryLayerGenerationResult, BoundaryLayerGenerationError>`.

- [ ] **Step 1: Write the failing orchestration test**

Build a concave triangulated Wall fixture that activates multi-normal splitting.
Request one regular layer and assert:

```cpp
const auto result = generateBoundaryLayers(
    surface, topology, patch, initial_front, profiles,
    multi_normal_options, regular_options);
assert(result.hasValue());
assert(result.value().transition.applied);
assert(result.value().regular.mesh.vertices.size() >=
       result.value().transition.transformed_front.vertices.size());
assert(result.value().mesh.cells.size() ==
       result.value().transition.transition_cells.cells.size() +
       result.value().regular.mesh.cells.size());
```

Also compare every transformed-front position with the same-index leading
regular mesh vertex and verify transition metadata precedes RegularLayer
metadata in the merged mesh.

- [ ] **Step 2: Register, configure, and run the test**

Run:
`cmake -S . -B build -DBOUNDARY_MESH_BUILD_TESTS=ON && cmake --build build --config Release --target boundary_mesh_boundary_layer_generation_pipeline_test`

Expected: build FAIL because the new header/function does not exist.

- [ ] **Step 3: Define stage-qualified result and error types**

Define:

```cpp
struct BoundaryLayerGenerationResult
{
    VolumeMesh mesh;
    MultiNormalTransitionResult transition;
    RegularLayerGrowthResult regular;
    SurfaceMesh top_surface;
    SurfaceMesh farfield_boundary;
};

using BoundaryLayerGenerationError = std::variant<
    MultiNormalError,
    RegularLayerGrowthError,
    MultiNormalMergeError,
    SpatialError>;
```

Expose one free function with the exact consumed arguments listed above.

- [ ] **Step 4: Implement minimal orchestration**

Call `generateMultiNormalTransition(initial_front, multi_normal_options)`, then
call `generateRegularLayers()` with `transition.transformed_front`. Pass the
complete original surface/topology/patch so source profiles and non-Wall
boundary context remain available. Finally call
`mergeMultiNormalAndRegularMeshes(transition, regular.mesh)` and copy regular
diagnostics into the unified result.

Do not alter or pre-triangulate `transformed_front` in this module; the
multi-normal generator owns that topology.

- [ ] **Step 5: Run the new and existing multi-normal tests**

Run:
`ctest --test-dir build -R "boundary_layer_generation|multi_normal_growth" --output-on-failure -C Release`

Expected: all selected tests PASS.

- [ ] **Step 6: Commit**

```text
git add CMakeLists.txt tests/CMakeLists.txt include/boundary_mesh/growth/boundary_layer_generator.hpp src/growth/boundary_layer_generator.cpp tests/integration/boundary_layer_generation_pipeline_test.cpp
git commit -m "feat: add unified boundary-layer generation pipeline"
```

### Task 3: Build explicit final-top and combined farfield outputs

**Files:**
- Create: `include/boundary_mesh/growth/exposed_surface_builder.hpp`
- Create: `src/growth/exposed_surface_builder.cpp`
- Modify: `src/growth/boundary_layer_generator.cpp`
- Modify: `tests/integration/boundary_layer_generation_pipeline_test.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `ExposedBoundaryTracker` data already committed by regular growth, or the regular result's exposed-interface representation after a minimal result extension.
- Produces: separate `SurfaceMesh top_surface` and `SurfaceMesh farfield_boundary` using branch-aware vertex keys.

- [ ] **Step 1: Add failing zero-layer and one-layer output assertions**

For a zero-regular-layer multi-normal run, assert that `top_surface.faces`
matches `transition.transformed_front.faces` in count and topology. For a
one-layer run, assert the top uses accepted next-layer coordinates. In both
runs assert original Wall faces are absent from `farfield_boundary`, original
Farfield face count is retained, and each top face is appended once.

- [ ] **Step 2: Run the focused test**

Run:
`ctest --test-dir build -R boundary_mesh_boundary_layer_generation_pipeline_test --output-on-failure -C Release`

Expected: FAIL because the unified result has no correct independent top yet.

- [ ] **Step 3: Factor the exposed-interface surface builder**

Implement one helper that converts exposed faces into a surface with reversed
interface orientation and branch-aware keys. Reuse it from the existing
farfield builder so top-only and combined output cannot diverge. When no
regular layer is accepted, seed exposure from `transformed_front` rather than
the original Wall face list.

- [ ] **Step 4: Populate unified result surfaces**

Set `result.top_surface` from the transformed/regular exposed interface. Set
`result.farfield_boundary` to original `Farfield` faces plus exactly that top
surface. Do not use original Wall geometry for the zero-layer transition case.

- [ ] **Step 5: Run output and regular-growth regression tests**

Run:
`ctest --test-dir build -R "boundary_layer_generation|regular_layer_growth|cgns_cli_pipeline" --output-on-failure -C Release`

Expected: all selected tests PASS.

- [ ] **Step 6: Commit**

```text
git add CMakeLists.txt include/boundary_mesh/growth/exposed_surface_builder.hpp src/growth/exposed_surface_builder.cpp src/growth/boundary_layer_generator.cpp tests/integration/boundary_layer_generation_pipeline_test.cpp
git commit -m "feat: expose final boundary-layer top surface"
```

### Task 4: Route the production CGNS command through the unified pipeline

**Files:**
- Modify: `src/cli/boundary_mesh_command.cpp`
- Modify: `tests/integration/cgns_cli_pipeline_test.cpp`

**Interfaces:**
- Consumes: existing CLI growth parameters plus multi-normal enablement and transition settings.
- Produces: `<prefix>_boundary_layer.vtk`, `<prefix>_farfield_boundary.vtk`, and `<prefix>_boundary_layer_top.vtk`.

- [ ] **Step 1: Extend the CLI test with failing production-output assertions**

Enable multi-normal in the fixture invocation and assert all three final files
exist and are non-empty. Assert command output reports transition and regular
cell counts. Retain the reversed-input determinism comparison for all files.

- [ ] **Step 2: Run the CLI test**

Run:
`ctest --test-dir build -R boundary_mesh_cgns_cli_pipeline_test --output-on-failure -C Release`

Expected: FAIL because the command still invokes `generateRegularLayers()` and
does not write the top file.

- [ ] **Step 3: Replace manual regular growth with the unified call**

Construct `MultiNormalOptions` from command options, call
`generateBoundaryLayers()` after the existing Wall-only patch/front builders,
and write `result.mesh`, `result.farfield_boundary`, and `result.top_surface`.
The complete `surface` remains the first input so original Farfield faces are
available; only the patch/front selected by `GrowthPatchBuilder` enter growth.

- [ ] **Step 4: Report stage counts and preserve debug gating**

Print `transition_cells`, `regular_cells`, and total `volume_cells`. Wire the
existing multi-normal debug option without enabling it by default.

- [ ] **Step 5: Run CLI and full unit/integration suite**

Run:
`ctest --test-dir build --output-on-failure -C Release`

Expected: 100% tests passed.

- [ ] **Step 6: Commit**

```text
git add src/cli/boundary_mesh_command.cpp tests/integration/cgns_cli_pipeline_test.cpp
git commit -m "feat: use multi-normal pipeline for CGNS boundary layers"
```

### Task 5: Validate five layers on `2dot5_cf.cgns`

**Files:**
- Generated: `build/real_case/2dot5_cgns_5_layers/2dot5_boundary_layer.vtk`
- Generated: `build/real_case/2dot5_cgns_5_layers/2dot5_farfield_boundary.vtk`
- Generated: `build/real_case/2dot5_cgns_5_layers/2dot5_boundary_layer_top.vtk`
- Generated when debug enabled: `build/real_case/2dot5_cgns_5_layers/multi_normal_transition.vtk`
- Generated when debug enabled: `build/real_case/2dot5_cgns_5_layers/multi_normal_front.vtk`

**Interfaces:**
- Consumes: `C:/Users/zpern/Desktop/todo/jiuyuan-quailty/test_case/2dot5_cf/2dot5_cf.cgns` and adjacent BC mapping (`Far: 1,2`, `Wall: 3,4`).
- Produces: final acceptance artifacts and recorded quality/count diagnostics.

- [ ] **Step 1: Rebuild Release and re-run all tests**

Run:
`cmake --build build --config Release && ctest --test-dir build -C Release --output-on-failure`

Expected: build succeeds and 100% tests pass.

- [ ] **Step 2: Run the real case with five regular layers**

Run the built CLI with the real CGNS input, `--layer-count 5`, multi-normal
enabled, `--first-height 0.01`, `--growth-ratio 1.2`, and output prefix
`build/real_case/2dot5_cgns_5_layers/2dot5`. Keep the configured quality,
neighbor-difference, and isotropic thresholds visible in captured output.

Expected: exit code zero and all three final VTK files are non-empty.

- [ ] **Step 3: Inspect generated topology and cell metadata**

Run the repository's existing VTK/mesh validation utilities on the merged
volume and top surface. Record transition-cell count, regular-cell count,
accepted layer counts, stop reasons, invalid/reversed cell count, collision
step reductions, and duplicate interface vertex count.

Expected: zero invalid/reversed cells, zero duplicate transition/regular
interface vertices, and a closed consistent transition-to-regular interface.

- [ ] **Step 4: Compare the zero-layer transition baseline**

Run the same input with `--layer-count 0` and compare its transition cell/front
counts with the previously validated BLMesh-equivalent result (50 transition
cells and no bad multi-normal topology points). Any difference must be traced
before accepting the five-layer output.

- [ ] **Step 5: Commit only source-controlled verification changes**

Do not commit generated VTK files. If Step 3 required a new deterministic
validation test, add only that named test file and its CMake registration, run
it once more, and commit them with message
`test: validate five-layer 2dot5 boundary mesh`. If no source-controlled file
was added, this task intentionally ends without another commit.
