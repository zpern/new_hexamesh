# Explicit Multi-Normal Transition Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make multi-normal processing an explicit pre-layer stage, triangulate affected Quads by minimum worst equiangular skewness, optionally write its intermediate VTK artifacts, and merge its cells with regular-layer cells through shared interface vertices.

**Architecture:** Keep detection, split planning, topology building, triangulation, displacement, and tetra construction as independently testable growth components. The public orchestration function composes them and optionally delegates artifact writing to the always-available Legacy VTK IO target. Regular-layer generation accepts a transformed front but contains no multi-normal logic; a specialized merge function welds its initial-front vertex prefix to the transition mesh's upper vertex IDs.

**Tech Stack:** C++17, CMake, Eigen, `std::variant`, existing `Result`, surface equiangular-skewness evaluator, Legacy VTK writer, CTest.

## Global Constraints

- The caller explicitly invokes multi-normal processing after initial topology/front construction and before regular-layer generation.
- Only multi-normal branch vertices move during the transition.
- A Quad is triangulated only when at least one corner is a split branch vertex.
- Select the diagonal minimizing the maximum equiangular skewness of its two triangles.
- Score differences at most `1e-12` are ties; the diagonal incident to the smallest topology vertex ID wins.
- Transition volume construction remains tetra-only and follows the existing BLMesh-derived triangle cases.
- Debug output is disabled by default; enabled output writes `multi_normal_transition.vtk` and `multi_normal_front.vtk` unless filenames are overridden.
- Transition and regular meshes share transformed-front interface vertices after merge.
- Continue supporting `-DBOUNDARY_MESH_ENABLE_CGNS_IO=OFF`.

---

## File Structure

- `include/boundary_mesh/growth/multi_normal_quad_triangulator.hpp`: standalone affected-Quad triangulation interface.
- `src/growth/multi_normal_quad_triangulator.cpp`: skewness scoring, deterministic tie break, and metadata expansion.
- `include/boundary_mesh/growth/multi_normal_transition_generator.hpp`: explicit public stage and debug options.
- `src/growth/multi_normal_transition_generator.cpp`: orchestration and optional VTK output.
- `include/boundary_mesh/growth/multi_normal_mesh_merge.hpp`: specialized transition/regular merge interface and error types.
- `src/growth/multi_normal_mesh_merge.cpp`: shared-interface connectivity remapping.
- Existing topology and transition builders remain focused on their current responsibilities.

### Task 1: Make Legacy VTK IO Available Without CGNS

**Files:**
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`
- Test: `tests/unit/io/legacy_vtk_writer_test.cpp`

**Interfaces:**
- Produces: `BoundaryMesh::IO` and both `writeLegacyVtk()` overloads when `BOUNDARY_MESH_ENABLE_CGNS_IO=OFF`.

- [ ] **Step 1: Add a no-CGNS linkage assertion**

Keep `boundary_mesh_legacy_vtk_writer_test` registered when CGNS is disabled and link it to `BoundaryMesh::IO`. Configure a fresh no-IO-named build with CGNS disabled.

- [ ] **Step 2: Verify RED**

Run:

```powershell
cmake -S . -B build-no-cgns-vtk -DBUILD_TESTING=ON -DBOUNDARY_MESH_ENABLE_CGNS_IO=OFF
```

Expected: configure fails because `BoundaryMesh::IO` is not defined while the VTK writer test is enabled.

- [ ] **Step 3: Split unconditional and conditional IO sources**

Define `boundary_mesh_io` unconditionally with:

```cmake
add_library(boundary_mesh_io STATIC src/io/legacy_vtk_writer.cpp)
add_library(BoundaryMesh::IO ALIAS boundary_mesh_io)
target_link_libraries(boundary_mesh_io PUBLIC BoundaryMesh::Core)
```

Inside `if(BOUNDARY_MESH_ENABLE_CGNS_IO)`, add CGNS sources through `target_sources()` and add the private CGNS dependency. Preserve CLI/CGNS executable conditionals.

- [ ] **Step 4: Verify GREEN**

Run configure, build `boundary_mesh_legacy_vtk_writer_test`, and execute its CTest entry. Expected: configure succeeds and the test passes with CGNS disabled.

- [ ] **Step 5: Commit**

```powershell
git add CMakeLists.txt tests/CMakeLists.txt
git commit -m "build: expose legacy vtk io without cgns"
```

### Task 2: Triangulate Split Quads as a Standalone Topology Stage

**Files:**
- Create: `include/boundary_mesh/growth/multi_normal_quad_triangulator.hpp`
- Create: `src/growth/multi_normal_quad_triangulator.cpp`
- Create: `tests/unit/growth/multi_normal_quad_triangulator_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `const MultiNormalTopology &`.
- Produces: `Result<MultiNormalTopology, MultiNormalError> triangulateMultiNormalQuads(const MultiNormalTopology &topology)`.

- [ ] **Step 1: Write the failing behavioral test**

Construct four cases using real `GrowthFrontVertex` coordinates and flags:

```cpp
const auto result = triangulateMultiNormalQuads(topology);
```

Assert that an unaffected Quad remains one Quad; a skewed affected Quad becomes the two triangles for the lower worst-skewness diagonal; a square tie selects the diagonal incident to the minimum vertex ID; and both output triangles duplicate the original `source_face_id` and `TransitionFaceOrigin` data.

- [ ] **Step 2: Verify RED**

Build the new target. Expected: compilation fails because `multi_normal_quad_triangulator.hpp` and `triangulateMultiNormalQuads()` do not exist.

- [ ] **Step 3: Implement minimal triangulation**

For each affected Quad, form both triangle pairs, call:

```cpp
triangleEquiangularSkewness(points)
```

score each pair with `std::max`, compare with tolerance `1e-12`, and use the minimum-ID corner tie rule. Append two faces and aligned metadata. Return `InvalidMultiNormalTopology` if both candidates are invalid.

- [ ] **Step 4: Verify GREEN**

Run the focused test and existing topology-builder test. Expected: both pass.

- [ ] **Step 5: Commit**

```powershell
git add include/boundary_mesh/growth/multi_normal_quad_triangulator.hpp src/growth/multi_normal_quad_triangulator.cpp tests/unit/growth/multi_normal_quad_triangulator_test.cpp CMakeLists.txt tests/CMakeLists.txt
git commit -m "feat: triangulate split quads by skewness"
```

### Task 3: Add the Explicit Multi-Normal Stage and Debug Artifacts

**Files:**
- Create: `include/boundary_mesh/growth/multi_normal_transition_generator.hpp`
- Create: `src/growth/multi_normal_transition_generator.cpp`
- Modify: `include/boundary_mesh/growth/multi_normal_types.hpp`
- Modify: `include/boundary_mesh/growth/multi_normal_error.hpp`
- Modify: `include/boundary_mesh/growth/multi_normal_transition_builder.hpp`
- Modify: `src/growth/multi_normal_transition_builder.cpp`
- Modify: `tests/unit/growth/multi_normal_transition_builder_test.cpp`
- Create: `tests/integration/multi_normal_transition_generator_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Produces: `generateMultiNormalTransition(const GrowthFront &, const MultiNormalOptions &) -> Result<MultiNormalTransitionResult, MultiNormalError>`.
- Produces: `MultiNormalTransitionResult::transformed_front_volume_vertex_ids`.

- [ ] **Step 1: Write failing orchestration tests**

Assert that a cube-corner Quad input is split, triangulated, displaced, produces no `omitted_quad_transitions`, and reaches tetra construction. Assert default options create no files. With debug enabled and a temporary directory, assert both configured VTK files exist and contain `DATASET UNSTRUCTURED_GRID`.

- [ ] **Step 2: Verify RED**

Build and run the new integration target. Expected: compilation fails because the explicit generator/debug configuration is absent, or the old orchestration still returns omitted Quads.

- [ ] **Step 3: Compose the explicit stage**

Implement this exact order:

```cpp
evaluate front
buildIncidentFaceFans
planMultiNormalSplits
buildMultiNormalTopology
triangulateMultiNormalQuads
buildMultiNormalTransition
write debug artifacts when enabled
```

Move `prepareMultiNormalTransition()` orchestration out of the transition builder and replace its public name with `generateMultiNormalTransition()`. Store `upper_ids` in `transformed_front_volume_vertex_ids`. Remove the Quad omission branch from transition construction; unaffected Quads perform no transition action.

- [ ] **Step 4: Add debug failure propagation**

Extend `MultiNormalError` with a debug-output failure containing the underlying `VtkWriteError`. Convert `GrowthFront` to `SurfaceMesh` without changing vertex or face order, then call both `writeLegacyVtk()` overloads.

- [ ] **Step 5: Verify GREEN**

Run transition-builder, Quad-triangulator, explicit-generator, and Legacy VTK tests. Expected: all pass; no file is written by default.

- [ ] **Step 6: Commit**

```powershell
git add include/boundary_mesh/growth src/growth tests CMakeLists.txt
git commit -m "feat: expose explicit multi-normal transition stage"
```

### Task 4: Make Regular-Layer Generation Pure and Accept Transformed Fronts

**Files:**
- Modify: `include/boundary_mesh/growth/regular_layer_growth.hpp`
- Modify: `include/boundary_mesh/growth/regular_layer_growth_error.hpp`
- Modify: `src/growth/regular_layer_generator.cpp`
- Modify: `tests/integration/multi_normal_growth_pipeline_test.cpp`
- Modify: `tests/integration/regular_layer_growth_pipeline_test.cpp`

**Interfaces:**
- Consumes: a caller-supplied original or transformed `GrowthFront`.
- Produces: a regular-only `RegularLayerGrowthResult::mesh` whose vertex prefix matches the supplied front.

- [ ] **Step 1: Rewrite the pipeline test first**

Explicitly call `generateMultiNormalTransition()`, pass its `transformed_front` to `generateRegularLayers()`, and assert the regular result contains no transition-role cells. Add a transformed-front fixture with repeated source vertex IDs and repeated source face IDs and assert one accepted layer is recorded per source entity.

- [ ] **Step 2: Verify RED**

Run the pipeline test. Expected: failure because current `validInitialMapping()` rejects the transformed front and the generator still runs multi-normal internally.

- [ ] **Step 3: Remove internal multi-normal behavior**

Remove `RegularLayerGrowthOptions::multi_normal`, `RegularLayerGrowthResult::omitted_quad_transitions`, `MultiNormalTransitionFailure`, the internal `prepareMultiNormalTransition()` call, and transition mesh seeding. Initialize the regular mesh directly from the supplied front.

- [ ] **Step 4: Validate source membership instead of exact equality**

Accept repeated transformed vertices/faces when each source ID exists in the patch/profile/constraint tables. Initialize source-level records once per original patch entity and keep accepted layer counts as the maximum layer number, not the number of transformed faces.

- [ ] **Step 5: Verify GREEN**

Run multi-normal pipeline plus all regular-layer, coordination, and collision tests. Expected: all pass.

- [ ] **Step 6: Commit**

```powershell
git add include/boundary_mesh/growth/regular_layer_growth.hpp include/boundary_mesh/growth/regular_layer_growth_error.hpp src/growth/regular_layer_generator.cpp tests/integration
git commit -m "refactor: separate regular growth from multi-normal stage"
```

### Task 5: Merge Through Shared Interface Vertices

**Files:**
- Create: `include/boundary_mesh/growth/multi_normal_mesh_merge.hpp`
- Create: `src/growth/multi_normal_mesh_merge.cpp`
- Create: `tests/unit/growth/multi_normal_mesh_merge_test.cpp`
- Modify: `tests/integration/multi_normal_growth_pipeline_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Produces: `mergeMultiNormalAndRegularMeshes(const MultiNormalTransitionResult &, const VolumeMesh &) -> Result<VolumeMesh, MultiNormalMergeError>`.

- [ ] **Step 1: Write failing merge tests**

Build a transition mesh whose upper IDs are nonzero and a regular mesh whose first vertices match the transformed front. Assert that the merged mesh does not duplicate those interface vertices, regular connectivity references the transition upper IDs, later regular vertices are appended once, and metadata order is transition then regular. Add invalid prefix, invalid connectivity, metadata mismatch, and overflow cases.

- [ ] **Step 2: Verify RED**

Build the new test. Expected: compilation fails because the merge API is absent.

- [ ] **Step 3: Implement checked remapping**

Create a regular-ID mapping where IDs below `transformed_front.vertices.size()` map through `transformed_front_volume_vertex_ids`; later IDs map to newly appended output IDs. Visit `Tetra`, `Pyramid`, `Prism`, and `Hexa` connectivity and replace every ID through the mapping. Reject inconsistent sizes, nonmatching prefix coordinates, invalid references, metadata mismatch, and overflow.

- [ ] **Step 4: Verify GREEN**

Run the merge test and explicit end-to-end pipeline. Expected: the final mesh contains transition and regular cells sharing interface IDs.

- [ ] **Step 5: Commit**

```powershell
git add include/boundary_mesh/growth/multi_normal_mesh_merge.hpp src/growth/multi_normal_mesh_merge.cpp tests/unit/growth/multi_normal_mesh_merge_test.cpp tests/integration/multi_normal_growth_pipeline_test.cpp CMakeLists.txt tests/CMakeLists.txt
git commit -m "feat: merge multi-normal and regular layer meshes"
```

### Task 6: Final Verification

**Files:**
- Verify all modified files.

- [ ] **Step 1: Check formatting and branch state**

```powershell
git diff --check
git status --short --branch
```

Expected: no whitespace errors and only intentional changes before the final commit.

- [ ] **Step 2: Run a fresh complete no-CGNS build**

```powershell
cmake -S . -B build-no-io -DBUILD_TESTING=ON -DBOUNDARY_MESH_ENABLE_CGNS_IO=OFF
cmake --build build-no-io --config Release
ctest --test-dir build-no-io -C Release --output-on-failure
```

Expected: configure/build exit zero and every test passes.

- [ ] **Step 3: Verify commits and clean status**

```powershell
git log --oneline -8
git status --short --branch
```

Expected: the implementation remains on `codex/multi-normal-topology-transition` with a clean worktree.
