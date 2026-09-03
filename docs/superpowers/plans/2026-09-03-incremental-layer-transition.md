# Incremental Layer Transition Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace two-layer reserved post-processing with a transactional `n→n+1` transition solver that uses separate corner-suppression seeds and transition-low-face sets, converges after collision rollback, and exposes an all-triangle outer surface.

**Architecture:** Keep `RegularLayerStepper`, growth geometry, and low-level collision primitives in `BoundaryMesh::BoundaryLayer`; implement transition topology, top-cap replacement, fixed-point resolution, and full pipeline orchestration in `BoundaryMesh::Transition` to avoid a library cycle. Each layer is assembled in a provisional transaction and committed only after corner suppression, template generation, and joint exposed-boundary collision checks reach a fixed point.

**Tech Stack:** C++17, CMake, CTest, Eigen-based geometry types, existing `Result<T,E>`, `VolumeMesh`, `GrowthFront`, `CollisionIndex`, and Pyramid/Tetra transition templates.

## Global Constraints

- Do not add two trial layers to requested profiles.
- Apply the same transition workflow to every `n→n+1` step, including `0→1`.
- Any directly adjacent source faces must differ by at most one accepted layer.
- `corner_suppression_seeds` drive corner suppression; `transition_low_faces` contain every low face that requires transition construction.
- Initial non-corner stops and collision rollbacks enter both sets; corner-suppressed faces enter only `transition_low_faces`.
- A Quad column has exactly one replaceable outer cap: its current top Hexa is decomposed into `5 Pyramid + 2 Tetra`; when the column advances, the previous cap becomes a complete Hexa and the new top Hexa becomes the cap.
- Every Quad layer face, including layer 0, has one canonical diagonal shared by its exposed triangulation, top-Hexa cap, and side-transition template; forced transition topology takes priority over local quality.
- The complete final boundary-layer top is triangular.
- A layer transaction commits only after the retained regular cells, top caps, and side transitions have no illegal contact with the original surface, historical exposed boundary, or one another.
- Regular candidates, provisional caps, and provisional side transitions participate in one joint fixed-point collision loop; only the stable resolved topology is committed.
- Preserve `source_face_id`, `layer`, and `cell_role` for every committed cell, and preserve `mesh.cells.size() == mesh.metadata.size()`.

---

## File Structure

- Create `include/boundary_mesh/transition/incremental_transition_types.hpp`: stop origins, the two named face sets, ownership records, provisional cap records, and stable-layer result types.
- Create `include/boundary_mesh/transition/layer_quad_diagonal_table.hpp` and `src/transition/layer_quad_diagonal_table.cpp`: one canonical diagonal per `(source_face_id, layer)` shared by source/top faces, caps, and side transitions.
- Create `include/boundary_mesh/transition/quad_high_neighbor_selector.hpp` and `src/transition/quad_high_neighbor_selector.cpp`: deterministic Quad high-edge option enumeration and quality selection.
- Create `include/boundary_mesh/transition/incremental_transition_templates.hpp` and `src/transition/incremental_transition_templates.cpp`: one-layer Triangle side templates, Quad top-cap decomposition, and Quad side templates without reserved-layer arithmetic.
- Create `include/boundary_mesh/transition/transition_boundary_checker.hpp` and `src/transition/transition_boundary_checker.cpp`: joint exposed-surface construction, collision ownership, and rollback-face extraction.
- Create `include/boundary_mesh/transition/layer_transition_resolver.hpp` and `src/transition/layer_transition_resolver.cpp`: `corner_suppression_seeds`/`transition_low_faces` fixed-point solver.
- Create `include/boundary_mesh/transition/incremental_boundary_layer_generator.hpp` and `src/transition/incremental_boundary_layer_generator.cpp`: per-layer orchestration and atomic commit.
- Modify `src/transition/boundary_layer_generator.cpp` and `src/cli/boundary_mesh_command.cpp`: route production generation through the incremental path without adding reserved layers.
- Modify `CMakeLists.txt` and `tests/CMakeLists.txt`: compile new sources and register focused unit/integration tests.
- Retain `reserved_layer_transition.*` and its tests until the new pipeline passes; remove or deprecate them only in the final cleanup task.

---

### Task 1: Named Transaction State and Stop-Set Semantics

**Files:**
- Create: `include/boundary_mesh/transition/incremental_transition_types.hpp`
- Create: `tests/unit/transition/incremental_transition_types_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Produces: `StopOrigin`, `LayerStopState`, `LayerFaceSets`, `addInitialStop`, `addCornerSuppressedFace`, and `addCollisionRollback`.
- Consumes: `SurfaceFaceId` and `FaceStopReason` from the existing growth types.

- [ ] **Step 1: Register and write a failing stop-set test**

Add a CMake target named `boundary_mesh_incremental_transition_types_test`, then test the exact membership contract:

```cpp
#include <cassert>
#include <boundary_mesh/transition/incremental_transition_types.hpp>

using namespace boundary_mesh;

int main()
{
    LayerFaceSets sets;
    addInitialStop(sets, {10, 3, StopOrigin::Quality});
    addCornerSuppressedFace(sets, {11, 3, StopOrigin::CornerSuppression});
    addCollisionRollback(sets, {12, 3, StopOrigin::TransitionCollision});

    assert((sets.corner_suppression_seeds ==
            std::vector<SurfaceFaceId>{10, 12}));
    assert((sets.transition_low_faces ==
            std::vector<SurfaceFaceId>{10, 11, 12}));
    assert(sets.states.size() == 3);
}
```

- [ ] **Step 2: Run the test and verify RED**

Run:

```powershell
cmake --build build --target boundary_mesh_incremental_transition_types_test
```

Expected: compilation fails because `incremental_transition_types.hpp` and the named APIs do not exist.

- [ ] **Step 3: Implement sorted, duplicate-free set updates**

Define:

```cpp
enum class StopOrigin
{
    RequestedLimit,
    Quality,
    Collision,
    SlidingProjection,
    IsotropicStop,
    CornerSuppression,
    TransitionCollision
};

struct LayerStopState
{
    SurfaceFaceId source_face_id{};
    std::uint32_t completed_layer{};
    StopOrigin origin{};
};

struct LayerFaceSets
{
    std::vector<SurfaceFaceId> corner_suppression_seeds;
    std::vector<SurfaceFaceId> transition_low_faces;
    std::vector<LayerStopState> states;
};
```

Implement all three add functions with one private `insertSortedUnique` helper. `addInitialStop` and `addCollisionRollback` insert into both vectors; `addCornerSuppressedFace` inserts only into `transition_low_faces`. Re-adding a source face must not duplicate its ID or state.

- [ ] **Step 4: Run the focused test and verify GREEN**

Run:

```powershell
cmake --build build --target boundary_mesh_incremental_transition_types_test
ctest --test-dir build -R incremental_transition_types --output-on-failure
```

Expected: build succeeds and one test passes.

- [ ] **Step 5: Commit**

```powershell
git add include/boundary_mesh/transition/incremental_transition_types.hpp tests/unit/transition/incremental_transition_types_test.cpp tests/CMakeLists.txt
git commit -m "feat: add incremental transition face sets"
```

---

### Task 2: Canonical Layer Diagonals, One-Layer Quad Cap, and `0→1` Templates

**Files:**
- Create: `include/boundary_mesh/transition/incremental_transition_templates.hpp`
- Create: `src/transition/incremental_transition_templates.cpp`
- Create: `include/boundary_mesh/transition/layer_quad_diagonal_table.hpp`
- Create: `src/transition/layer_quad_diagonal_table.cpp`
- Create: `tests/unit/transition/incremental_transition_templates_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`
- Reference: `src/transition/quad_transition_template.cpp`
- Reference: `src/transition/triangle_transition_template.cpp`

**Interfaces:**
- Produces: `LayerQuadFaceKey`, `LayerQuadDiagonalTable::resolve(...)`, `buildQuadTopCap(const QuadTopCapInput&)`, `buildQuadSideTransition(const QuadSideTransitionInput&)`, and `buildTriangleSideTransition(const TriangleSideTransitionInput&)`.
- Consumes: existing `QuadDiagonalSelection`, `SourceTransitionResult`, `VolumeCell`, `CellMetadata`, and mesh points.

- [ ] **Step 1: Write failing tests for a one-layer cap and a zero-to-one side transition**

Use a unit cube with bottom IDs `{0,1,2,3}`, top IDs `{4,5,6,7}`, and center ID `8`. Assert:

```cpp
const auto cap = buildQuadTopCap({
    7, 1, {0,1,2,3}, {4,5,6,7}, &points, 8, 1e-12});
assert(cap.hasValue());
assert(cap.value().created_vertices.size() == 1);
assert(cap.value().volume_cells.size() == 7);
assert(countCells<Pyramid>(cap.value().volume_cells) == 5);
assert(countCells<Tetra>(cap.value().volume_cells) == 2);
assert(cap.value().top_faces.size() == 2);

const auto side = buildQuadSideTransition({
    7, 0, {0,1,2,3}, {4,5,6,7}, {0},
    cap.value().diagonal, &points, 1e-12});
assert(side.hasValue());
assert(!side.value().volume_cells.empty());
assert(!side.value().top_faces.empty());
assert(side.value().low_diagonal == cap.value().diagonal);
```

Also assert every cap metadata entry has `source_face_id == 7`, `layer == 1`, and `cell_role == CellRole::ReservedLayerTransition` until a dedicated incremental role is introduced.

Add a layer-0 case that resolves `(source_face_id=7, layer=0)`, triangulates the source Quad, then passes the returned canonical diagonal to `buildQuadSideTransition`; assert both use identical diagonal endpoints. Registering the opposite forced diagonal for the same key must return `ConflictingLayerQuadDiagonal`.

- [ ] **Step 2: Run the test and verify RED**

Run `cmake --build build --target boundary_mesh_incremental_transition_templates_test`.

Expected: compilation fails because the new one-layer template API does not exist.

- [ ] **Step 3: Extract the current cap decomposition without reserved-layer counts**

Implement `LayerQuadDiagonalTable` as a sorted table keyed by `{source_face_id, layer}`. Its `resolve(key, quad, optional_forced_diagonal)` returns the existing value when compatible, stores a forced value before considering quality, otherwise stores `chooseQuadDiagonal(quad, tolerance)`, and rejects an opposite forced value for an existing key.

Implement `QuadTopCapInput` with explicit `bottom`, `top`, `layer`, and resolved `QuadDiagonal`; copy the existing eight-point center, five-Pyramid, and two-Tetra logic. Do not call `regularLayerCount()`, do not accept `trial_layers`, and do not select a second diagonal inside the builder.

Implement side-template inputs using explicit low/high vertex arrays, selected high-edge indices, and the already resolved low-face `QuadDiagonal`. Reuse the existing connectivity for single high edge, adjacent double high edge, and Triangle single-high-edge cases. A zero-layer low face is valid whenever the required high vertices exist. The side builder must never call `chooseQuadDiagonal()` independently.

- [ ] **Step 4: Run new and legacy template tests**

Run:

```powershell
cmake --build build --target boundary_mesh_incremental_transition_templates_test boundary_mesh_quad_hexa_decomposition_test boundary_mesh_quad_side_transition_template_test boundary_mesh_triangle_transition_template_test
ctest --test-dir build -R "incremental_transition_templates|quad_hexa_decomposition|quad_side_transition|triangle_transition" --output-on-failure
```

Expected: all selected tests pass.

- [ ] **Step 5: Commit**

```powershell
git add include/boundary_mesh/transition/incremental_transition_templates.hpp src/transition/incremental_transition_templates.cpp include/boundary_mesh/transition/layer_quad_diagonal_table.hpp src/transition/layer_quad_diagonal_table.cpp tests/unit/transition/incremental_transition_templates_test.cpp CMakeLists.txt tests/CMakeLists.txt
git commit -m "feat: add one-layer transition templates"
```

---

### Task 3: Deterministic Quad High-Neighbor Selection

**Files:**
- Create: `include/boundary_mesh/transition/quad_high_neighbor_selector.hpp`
- Create: `src/transition/quad_high_neighbor_selector.cpp`
- Create: `tests/unit/transition/quad_high_neighbor_selector_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Produces: `selectQuadHighNeighbors(const QuadHighNeighborSelectionInput&) -> Result<QuadHighNeighborSelection, TransitionTemplateError>`, including `required_low_diagonal`.
- Consumes: one Quad's local edge-to-neighbor mapping, current retained-high set, vertex IDs/points, and the one-layer template builder from Task 2.

- [ ] **Step 1: Write a table-driven failing test for 0–4 high edges**

Cover these exact expected retained-edge counts:

```cpp
check({}, {});
check({edge(0, 20)}, {0});
check({edge(0, 20), edge(1, 21)}, {0, 1});
check({edge(0, 20), edge(2, 22)}, {0});
check({edge(0, 20), edge(1, 21), edge(2, 22)}, {0, 1});
check({edge(0, 20), edge(1, 21), edge(2, 22), edge(3, 23)}, {0, 1});
```

Use symmetric geometry so ties occur and assert that sorted neighbor IDs, then local edge indices, select `{0}` or `{0,1}`. Repeat with reversed input order and assert identical output.

For adjacent double-high input, assert `required_low_diagonal` matches the existing four-cell topology. Supply a pre-registered opposite low diagonal and assert that candidate is rejected rather than emitting mismatched triangles.

- [ ] **Step 2: Run the test and verify RED**

Run `cmake --build build --target boundary_mesh_quad_high_neighbor_selector_test`.

Expected: compilation fails because `selectQuadHighNeighbors` does not exist.

- [ ] **Step 3: Implement option enumeration, scoring, and tie-breaking**

Implement the candidate rules exactly:

```cpp
0 highs -> empty candidate
1 high  -> that edge
2 adjacent -> both edges
2 opposite -> two one-edge candidates
3 highs -> each adjacent pair contained in the input
4 highs -> {0,1}, {1,2}, {2,3}, {3,0}
```

For each candidate, derive its optional forced low-face diagonal, resolve it through `LayerQuadDiagonalTable`, build the corresponding local side template with that exact diagonal, and score the maximum equiangle skewness of every triangular face of every candidate cell. Compare successful scores first, then sorted retained neighbor IDs, then sorted local edge indices. Reject candidates conflicting with an existing canonical diagonal; return `FaceEvaluationError` only if every remaining legal candidate fails geometric evaluation.

- [ ] **Step 4: Run focused and diagonal tests**

Run:

```powershell
cmake --build build --target boundary_mesh_quad_high_neighbor_selector_test boundary_mesh_quad_diagonal_test
ctest --test-dir build -R "quad_high_neighbor_selector|quad_diagonal" --output-on-failure
```

Expected: all selected tests pass.

- [ ] **Step 5: Commit**

```powershell
git add include/boundary_mesh/transition/quad_high_neighbor_selector.hpp src/transition/quad_high_neighbor_selector.cpp tests/unit/transition/quad_high_neighbor_selector_test.cpp CMakeLists.txt tests/CMakeLists.txt
git commit -m "feat: select quad high neighbors by quality"
```

---

### Task 4: Corner Suppression Without Recursive Seeding

**Files:**
- Create: `include/boundary_mesh/transition/corner_suppression.hpp`
- Create: `src/transition/corner_suppression.cpp`
- Create: `tests/unit/transition/corner_suppression_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`
- Reference: `src/growth/termination_propagator.cpp:402`

**Interfaces:**
- Produces: `applyCornerSuppression(const CornerSuppressionInput&) -> Result<CornerSuppressionResult, TransitionCoordinationError>`.
- Consumes: `LayerFaceSets`, front edge/vertex incidence, retained-high face IDs, and Task 3 selection.

- [ ] **Step 1: Write a failing non-recursion test**

Construct a three-Quad strip where face 10 is an initial low seed, suppressing face 11 at a non-contact corner while face 12 is adjacent only to face 11. Assert:

```cpp
assert(result.removed_high_faces == std::vector<SurfaceFaceId>{11});
assert(result.face_sets.corner_suppression_seeds ==
       std::vector<SurfaceFaceId>{10});
assert(result.face_sets.transition_low_faces ==
       std::vector<SurfaceFaceId>({10, 11}));
assert(contains(result.retained_high_faces, 12));
```

Add cases for adjacent double-high and four-high Quad selection; assert only faces excluded by the selected topology are removed.

- [ ] **Step 2: Run the test and verify RED**

Run `cmake --build build --target boundary_mesh_corner_suppression_test`.

Expected: compilation fails because the corner suppression API does not exist.

- [ ] **Step 3: Implement seed-only processing**

Build current edge uses and vertex uses from the `n`-layer front. Iterate a snapshot of `corner_suppression_seeds`; never append corner-suppressed faces to that vector. For every removed high face call:

```cpp
addCornerSuppressedFace(
    result.face_sets,
    {source_face_id, input.completed_layer,
     StopOrigin::CornerSuppression});
```

Sort and deduplicate `removed_high_faces` before returning. Triangle seeds retain the existing single-high-edge rule. Quad seeds use Task 3.

- [ ] **Step 4: Run focused coordination tests**

Run:

```powershell
cmake --build build --target boundary_mesh_corner_suppression_test boundary_mesh_termination_propagator_test
ctest --test-dir build -R "corner_suppression|termination_propagator" --output-on-failure
```

Expected: both tests pass; the legacy test confirms no accidental change to the old API during extraction.

- [ ] **Step 5: Commit**

```powershell
git add include/boundary_mesh/transition/corner_suppression.hpp src/transition/corner_suppression.cpp tests/unit/transition/corner_suppression_test.cpp CMakeLists.txt tests/CMakeLists.txt
git commit -m "feat: add non-recursive corner suppression"
```

---

### Task 5: Joint Boundary Collision Ownership and Rollback

**Files:**
- Create: `include/boundary_mesh/transition/transition_boundary_checker.hpp`
- Create: `src/transition/transition_boundary_checker.cpp`
- Create: `tests/unit/transition/transition_boundary_checker_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`
- Reference: `src/growth/layer_collision_checker.cpp`
- Reference: `src/growth/exposed_boundary.cpp`

**Interfaces:**
- Produces: `BoundaryOwnerRole`, `LayerBoundaryOwner`, `OwnedBoundaryTriangle`, and `TransitionBoundaryChecker::findRollbackFaces(...)`.
- Consumes: retained regular candidates, top-cap results, side-transition results, original `CollisionIndex`, and historical `ExposedBoundaryTracker`.

- [ ] **Step 1: Write failing ownership tests**

Create fixtures where a side-transition triangle intersects an obstacle and depends on high faces `{21,22}`. Assert:

```cpp
const auto rollback = checker.findRollbackFaces(input);
assert(rollback.hasValue());
assert((rollback.value() == std::vector<SurfaceFaceId>{21,22}));
```

Add a retained-high/top-cap collision returning its single high owner, and a self-collision returning the sorted union of both candidate owners. Add a legal shared-edge contact and assert it does not request rollback.

Add a low Quad whose cap and side transition share a face. Assert the boundary assembler cancels the two matching triangles as internal faces; construct a deliberately opposite side diagonal and assert `ConflictingLayerQuadDiagonal` is returned before collision indexing.

- [ ] **Step 2: Run the test and verify RED**

Run `cmake --build build --target boundary_mesh_transition_boundary_checker_test`.

Expected: compilation fails because the owned-boundary checker does not exist.

- [ ] **Step 3: Implement complete joint collision assembly**

Triangulate every outer face and attach:

```cpp
struct LayerBoundaryOwner
{
    SurfaceFaceId source_face_id{};
    std::uint32_t layer{};
    BoundaryOwnerRole role{};
    std::vector<SurfaceFaceId> rollback_high_faces;
};
```

Build one candidate index containing retained regular, cap, and side-transition exposed triangles. Query it against the original surface, the historical exposed boundary, and itself. Use existing topological vertex keys so shared vertices/edges/faces remain legal contacts. Return the sorted unique union of `rollback_high_faces` for every illegal contact.

- [ ] **Step 4: Run focused and existing collision tests**

Run:

```powershell
cmake --build build --target boundary_mesh_transition_boundary_checker_test boundary_mesh_layer_collision_checker_test boundary_mesh_exposed_boundary_test
ctest --test-dir build -R "transition_boundary_checker|layer_collision_checker|exposed_boundary" --output-on-failure
```

Expected: all selected tests pass.

- [ ] **Step 5: Commit**

```powershell
git add include/boundary_mesh/transition/transition_boundary_checker.hpp src/transition/transition_boundary_checker.cpp tests/unit/transition/transition_boundary_checker_test.cpp CMakeLists.txt tests/CMakeLists.txt
git commit -m "feat: detect collisions across complete transition boundary"
```

---

### Task 6: Fixed-Point Layer Transition Resolver

**Files:**
- Create: `include/boundary_mesh/transition/layer_transition_resolver.hpp`
- Create: `src/transition/layer_transition_resolver.cpp`
- Create: `tests/unit/transition/layer_transition_resolver_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Produces: `LayerTransitionResolver::resolve(const LayerTransitionInput&) -> Result<StableLayerTransition, LayerTransitionError>`.
- Consumes: Tasks 1–5, the current front, provisional `LayerStepResult`, global vertex IDs, the transaction's `LayerQuadDiagonalTable`, and historical collision state.

- [ ] **Step 1: Write a failing two-set fixed-point test**

Use a scripted collision checker or a small geometric fixture where the first transition build rolls back high face 30, then the rebuilt transition is valid. Assert:

```cpp
assert(result.hasValue());
assert(result.value().iterations == 2);
assert(!contains(result.value().retained_high_faces, 30));
assert(contains(result.value().face_sets.corner_suppression_seeds, 30));
assert(contains(result.value().face_sets.transition_low_faces, 30));
assert(result.value().all_top_faces_are_triangles);
```

Add a case where corner suppression removes face 31 and assert 31 appears only in `transition_low_faces`. Add a reversed face-order fixture and assert identical retained IDs and cell connectivity.

- [ ] **Step 2: Run the test and verify RED**

Run `cmake --build build --target boundary_mesh_layer_transition_resolver_test`.

Expected: compilation fails because `LayerTransitionResolver` does not exist.

- [ ] **Step 3: Implement the monotone fixed-point loop**

Implement this control flow literally:

```cpp
while (true)
{
    applyCornerSuppression(transaction);
    rebuildAllTopCapsAndTransitions(transaction);
    const auto rollback = boundary_checker.findRollbackFaces(transaction);
    if (!rollback.hasValue()) return failure(rollback.error());
    if (rollback.value().empty()) break;
    for (const SurfaceFaceId id : rollback.value())
    {
        eraseRetainedHighFace(transaction, id);
        addCollisionRollback(
            transaction.face_sets,
            {id, input.current_front.layer,
             StopOrigin::TransitionCollision});
    }
    ++transaction.iterations;
}
```

`rebuildAllTopCapsAndTransitions` must clear every provisional cap/side cell and the provisional diagonal table, then rebuild them from the full `transition_low_faces`. Resolve one canonical diagonal for every Quad layer face before building either its cap/source triangulation or side transition, then pass that stored value to every consumer. Run one joint collision query over retained regular candidates, provisional caps, and provisional side transitions. Reject a final direct-neighbor layer difference above one, conflicting forced diagonals, a low face without a legal template, non-triangular top output, or mismatched cell metadata.

When rollback is empty, copy the exact per-face decision into:

```cpp
struct ResolvedTransitionTopology
{
    SurfaceFaceId source_face_id{};
    std::uint32_t layer{};
    TransitionTemplateKind template_kind{};
    std::optional<QuadDiagonal> low_diagonal;
    std::vector<SurfaceFaceId> dependent_high_faces;
};
```

Return this vector in `StableLayerTransition`; do not resolve diagonals or choose templates again after leaving the fixed-point loop.

- [ ] **Step 4: Run resolver and template suites**

Run:

```powershell
cmake --build build --target boundary_mesh_layer_transition_resolver_test
ctest --test-dir build -R "layer_transition_resolver|incremental_transition_templates|corner_suppression|transition_boundary_checker" --output-on-failure
```

Expected: all selected tests pass.

- [ ] **Step 5: Commit**

```powershell
git add include/boundary_mesh/transition/layer_transition_resolver.hpp src/transition/layer_transition_resolver.cpp tests/unit/transition/layer_transition_resolver_test.cpp CMakeLists.txt tests/CMakeLists.txt
git commit -m "feat: resolve layer transitions to a fixed point"
```

---

### Task 7: Transactional Incremental Generator and Replaceable Caps

**Files:**
- Create: `include/boundary_mesh/transition/incremental_boundary_layer_generator.hpp`
- Create: `src/transition/incremental_boundary_layer_generator.cpp`
- Create: `tests/integration/incremental_layer_transition_pipeline_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Produces: `generateIncrementalBoundaryLayers(...) -> Result<RegularLayerGrowthResult, IncrementalLayerGrowthError>`.
- Consumes: `RegularLayerStepper`, constraint/profile builders, Task 6 resolver, `LayerVertexTable`, `ExposedBoundaryTracker`, and existing final boundary builders.

- [ ] **Step 1: Write failing `0→1` and cap-replacement integration tests**

Create a one-Quad patch requesting one layer. Assert there is no committed Hexa, there are exactly five Pyramids and two Tetra, and `top_surface.faces` contains exactly two Triangle variants.

Create the same patch requesting two layers. Assert:

```cpp
assert(countCells<Hexa>(result.mesh.cells) == 1);
assert(countCells<Pyramid>(result.mesh.cells) == 5);
assert(countCells<Tetra>(result.mesh.cells) == 2);
assert(maxMetadataLayer(result.mesh.metadata) == 2);
assert(allTriangles(result.top_surface.faces));
```

The single Hexa is layer 1 restored from the old cap; only layer 2 remains decomposed.

Add a two-Quad `0→1` step and a `1→2` step where a low Quad also owns a side transition. For both layers, extract the low-interface diagonal from cap/source triangles and from side-template triangles and assert the endpoint pairs are identical.

- [ ] **Step 2: Run the test and verify RED**

Run `cmake --build build --target boundary_mesh_incremental_layer_transition_pipeline_test`.

Expected: compilation fails because the incremental generator does not exist.

- [ ] **Step 3: Implement per-layer provisional assembly and atomic commit**

Move or reuse the non-transition initialization helpers from `regular_layer_generator.cpp` without introducing a `BoundaryLayer → Transition` dependency. For each loop iteration:

```cpp
auto provisional = stepper.step(current_front, profiles, constraints,
                                sliding_surfaces, options);
auto stable = resolver.resolve(makeLayerTransitionInput(
    current_front, provisional.value(), committed_state));
if (!stable.hasValue()) return failure(stable.error());
commitStableLayer(committed_state, stable.value());
current_front = buildContinuingFront(stable.value());
```

Store each Quad column's cap as replaceable committed state. When the column advances, remove its seven old cap cells and center vertex from the logical cap store, append one complete Hexa for the previous layer, and install the new seven-cell cap. Build the committed cap and side cells directly from `StableLayerTransition::resolved_topology`; the commit path must not call `chooseQuadDiagonal()` or the high-neighbor selector. Flatten logical regular cells plus current caps into `RegularLayerGrowthResult::mesh` only after generation finishes, so no in-place cell-index deletion is required.

- [ ] **Step 4: Run focused integration and regular-growth regression tests**

Run:

```powershell
cmake --build build --target boundary_mesh_incremental_layer_transition_pipeline_test boundary_mesh_regular_layer_growth_pipeline_test
ctest --test-dir build -R "incremental_layer_transition_pipeline|regular_layer_growth_pipeline" --output-on-failure
```

Expected: both tests pass.

- [ ] **Step 5: Commit**

```powershell
git add include/boundary_mesh/transition/incremental_boundary_layer_generator.hpp src/transition/incremental_boundary_layer_generator.cpp tests/integration/incremental_layer_transition_pipeline_test.cpp CMakeLists.txt tests/CMakeLists.txt
git commit -m "feat: generate incremental layer transitions"
```

---

### Task 8: Multi-Step Staircases and Collision Rollback Integration

**Files:**
- Modify: `tests/integration/incremental_layer_transition_pipeline_test.cpp`
- Modify: `src/transition/incremental_boundary_layer_generator.cpp`
- Modify: `src/transition/layer_transition_resolver.cpp`

**Interfaces:**
- Consumes: Task 7 public generator.
- Produces: verified multi-step staircase behavior and stable collision rollback through the production path.

- [ ] **Step 1: Add failing multi-step and rollback integration cases**

Build a four-region Quad strip whose requested counts are `{1,2,3,4}`. Assert every shared edge has accepted-layer difference at most one, while the first and last region differ by three. Assert all top faces are Triangle and every interior Triangle has two owners.

For every transition-low Quad in the strip, assert its registered `(source_face_id, layer)` diagonal equals the diagonal recovered from both the low cap/source triangulation and the attached side-transition interface.

Add geometry that makes the first chosen side transition collide with the historical exposed boundary. Assert the dependent high cell is absent, its low face appears in both named sets in resolver diagnostics, the second iteration is collision-free, and unrelated high cells remain.

- [ ] **Step 2: Run the integration test and verify RED**

Run:

```powershell
cmake --build build --target boundary_mesh_incremental_layer_transition_pipeline_test
ctest --test-dir build -R incremental_layer_transition_pipeline --output-on-failure
```

Expected: at least one new assertion fails because cross-layer staircase or rollback propagation is incomplete.

- [ ] **Step 3: Implement only the missing staircase and rollback behavior**

Ensure initial requested limits are not globally flattened through `propagateInitial(..., 1)`. Retain only the direct-neighbor invariant check at each stable transaction. Feed every transition-collision rollback through `addCollisionRollback`, rerun corner suppression for the newly appended seeds, and rebuild all transitions from the complete `transition_low_faces`.

- [ ] **Step 4: Run all transition and layer-coordination tests**

Run:

```powershell
cmake --build build
ctest --test-dir build -R "transition|layer_coordination|termination_propagator" --output-on-failure
```

Expected: all selected tests pass.

- [ ] **Step 5: Commit**

```powershell
git add tests/integration/incremental_layer_transition_pipeline_test.cpp src/transition/incremental_boundary_layer_generator.cpp src/transition/layer_transition_resolver.cpp
git commit -m "feat: support multi-step boundary-layer staircases"
```

---

### Task 9: Switch Production Entry Points and Remove Reserved-Layer Expansion

**Files:**
- Modify: `src/transition/boundary_layer_generator.cpp`
- Modify: `include/boundary_mesh/transition/boundary_layer_generator.hpp`
- Modify: `src/cli/boundary_mesh_command.cpp`
- Modify: `tests/integration/transition/boundary_layer_generation_pipeline_test.cpp`
- Modify: `tests/integration/reserved_layer_transition_pipeline_test.cpp`
- Modify: `docs/reserved_layer_transition.md`

**Interfaces:**
- Produces: production `generateBoundaryLayers()` and CLI behavior backed by Task 7.
- Consumes: `generateIncrementalBoundaryLayers()`.

- [ ] **Step 1: Write failing production-entry assertions**

Update the boundary-layer generation pipeline fixture to request one Quad layer and assert the final mesh contains the one-layer `5 Pyramid + 2 Tetra` cap, not a reserved trial result. Add an assertion that the maximum accepted layer equals the user request exactly.

Update the former reserved-pipeline test to exercise `generateBoundaryLayers()` or rename it to `incremental_layer_transition_pipeline_test`; remove expectations that requested counts are increased by two.

- [ ] **Step 2: Run production tests and verify RED**

Run:

```powershell
cmake --build build --target boundary_mesh_boundary_layer_generation_pipeline_test boundary_mesh_reserved_layer_transition_pipeline_test
ctest --test-dir build -R "boundary_layer_generation_pipeline|reserved_layer_transition_pipeline" --output-on-failure
```

Expected: assertions fail because the production entry or CLI still uses `generateReservedLayerTransition()` and `makeReservedTrialProfiles()`.

- [ ] **Step 3: Route production generation through the incremental generator**

Replace the regular-only call inside `generateBoundaryLayers()` with `generateIncrementalBoundaryLayers()`. Replace the CLI call to `generateReservedLayerTransition()` with `generateBoundaryLayers()`, adapting only result field names. Do not invoke `makeReservedTrialProfiles()` anywhere on the production path.

Update `docs/reserved_layer_transition.md` with a leading notice that it describes the legacy algorithm and link to the new design specification; do not rewrite historical connectivity descriptions in this task.

- [ ] **Step 4: Run production and CLI tests**

Run:

```powershell
cmake --build build
ctest --test-dir build -R "boundary_layer_generation_pipeline|incremental_layer_transition_pipeline|cgns_cli_pipeline" --output-on-failure
```

Expected: all selected tests pass; when CGNS is disabled, only registered tests run.

- [ ] **Step 5: Commit**

```powershell
git add src/transition/boundary_layer_generator.cpp include/boundary_mesh/transition/boundary_layer_generator.hpp src/cli/boundary_mesh_command.cpp tests/integration/transition/boundary_layer_generation_pipeline_test.cpp tests/integration/reserved_layer_transition_pipeline_test.cpp docs/reserved_layer_transition.md
git commit -m "feat: enable incremental transition generation"
```

---

### Task 10: Deprecate the Legacy Entry and Run Full Validation

**Files:**
- Modify: `include/boundary_mesh/transition/reserved_layer_transition.hpp`
- Modify: `docs/reserved_layer_transition.md`

**Interfaces:**
- Consumes: all preceding tasks.
- Produces: a deprecated but buildable legacy API plus a clean build and complete test suite.

- [ ] **Step 1: Record remaining legacy reserved API callers**

Run:

```powershell
rg -n "makeReservedTrialProfiles|generateReservedLayerTransition|ReservedTransitionLayerCount" include src apps tests
```

Expected: output identifies only the retained legacy implementation/tests and no production entry point or CLI caller.

- [ ] **Step 2: Deprecate the retained compatibility entry point**

Keep the legacy source and tests for compatibility. Add `[[deprecated("use generateBoundaryLayers incremental transition path")]]` immediately before the public `generateReservedLayerTransition` declaration. Update the legacy document notice to state that this API remains for compatibility tests but is not used by production generation.

- [ ] **Step 3: Configure and build from the current source tree**

Run:

```powershell
cmake -S . -B build
cmake --build build --config Debug
```

Expected: both commands exit with code 0 and no compilation errors.

- [ ] **Step 4: Run the complete suite**

Run:

```powershell
ctest --test-dir build -C Debug --output-on-failure
```

Expected: 100% tests passed, 0 tests failed.

- [ ] **Step 5: Verify repository invariants and diff hygiene**

Run:

```powershell
git diff --check
git status --short
```

Expected: `git diff --check` prints nothing. `git status --short` lists only intended implementation changes, or is empty after the final commit.

- [ ] **Step 6: Commit legacy deprecation**

```powershell
git add include/boundary_mesh/transition/reserved_layer_transition.hpp docs/reserved_layer_transition.md
git commit -m "docs: deprecate reserved layer transition path"
```
