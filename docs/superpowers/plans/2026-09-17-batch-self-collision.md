# Batch Self-Collision Detection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace per-triangle self-collision queries with one deterministic batch pass that tests each unordered `CollisionTriangle` pair at most once and reports profiling counters.

**Architecture:** Add a focused `BatchSelfCollisionDetector` in the spatial module. It caches triangle AABBs, sorts primitives on the longest overall axis, performs a one-dimensional sweep followed by full AABB and existing exact-contact checks, and returns sorted illegal owner IDs. This is the pair-once and interval-pruning core of the `blmesh` approach without importing its legacy classes; a later octree refinement is warranted only if diagnostics show excessive sweep candidates.

**Tech Stack:** C++17, Eigen geometry types, existing `Result`, `Aabb`, `CollisionTriangle`, and `hasIllegalTriangleContact` APIs, CMake/CTest.

## Global Constraints

- Preserve the exact collision semantics implemented by `hasIllegalTriangleContact()`.
- Do not persist the current-layer self-collision structure across layers.
- Do not modify user-owned uncommitted CLI, generator timing, HDF5, editor, or JSON configuration changes.
- Use first height `0.1`, growth ratio `1.2`, and 20 layers for the final 2dot5 Release benchmark.

---

### Task 1: Specify batch pair semantics and diagnostics

**Files:**
- Create: `include/boundary_mesh/spatial/batch_self_collision_detector.hpp`
- Create: `tests/unit/spatial/batch_self_collision_detector_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `CollisionTriangle`, `SpatialError`, and `Result`.
- Produces: `BatchSelfCollisionDiagnostics`, `BatchSelfCollisionResult`, and `BatchSelfCollisionDetector::detect(const std::vector<CollisionTriangle>&)`.

- [ ] **Step 1: Register a failing unit test executable**

Add `boundary_mesh_batch_self_collision_detector_test` linked to `BoundaryMesh::Spatial` in `tests/CMakeLists.txt`.

- [ ] **Step 2: Write tests against the desired public API**

Cover empty input, separated triangles, different-owner illegal intersection,
same-owner exclusion, and three simultaneously overlapping triangles. Assert
that `illegal_owner_ids` is sorted and unique, `unique_pairs` counts unordered
pairs, `same_owner_skips` is nonzero for same-owner geometry, and
`exact_tests <= unique_pairs`.

- [ ] **Step 3: Run the focused build and verify RED**

Run:

```powershell
cmake --build build-release --config Release --target boundary_mesh_batch_self_collision_detector_test
```

Expected: compilation fails because `batch_self_collision_detector.hpp` and its API do not exist.

- [ ] **Step 4: Add the minimal declarations**

Declare:

```cpp
struct BatchSelfCollisionDiagnostics {
    std::uint64_t triangle_count{}, sweep_pairs{}, unique_pairs{};
    std::uint64_t same_owner_skips{}, aabb_rejections{};
    std::uint64_t topology_rejections{}, exact_tests{};
    std::uint64_t illegal_owner_pairs{};
};

struct BatchSelfCollisionResult {
    std::vector<std::uint32_t> illegal_owner_ids;
    BatchSelfCollisionDiagnostics diagnostics;
};

class BatchSelfCollisionDetector {
public:
    static Result<BatchSelfCollisionResult, SpatialError> detect(
        const std::vector<CollisionTriangle>& triangles);
};
```

- [ ] **Step 5: Commit the red specification**

```powershell
git add include/boundary_mesh/spatial/batch_self_collision_detector.hpp tests/unit/spatial/batch_self_collision_detector_test.cpp tests/CMakeLists.txt
git commit -m "test: specify batch self collision detection"
```

### Task 2: Implement pair-once sweep and preserve contact semantics

**Files:**
- Create: `src/spatial/batch_self_collision_detector.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/unit/spatial/batch_self_collision_detector_test.cpp`

**Interfaces:**
- Consumes: the declarations from Task 1 and existing `makeAabb`, `overlaps`, and `hasIllegalTriangleContact`.
- Produces: a working deterministic batch detector.

- [ ] **Step 1: Extend the failing test with duplicate-contact and invalid-input cases**

Add cases proving that multiple triangle contacts between the same two owners
produce two owner IDs only, and a non-finite coordinate returns
`SpatialError::NonFiniteCoordinate`.

- [ ] **Step 2: Run the focused test and verify RED**

Expected: link failure for the unimplemented `detect()` method.

- [ ] **Step 3: Implement cached bounds and longest-axis ordering**

For every triangle, call `makeAabb`, reject zero-area triangles consistently
with `CollisionIndex::build`, cache `{primitive, bounds}`, select the axis with
the greatest overall extent, and stable-sort by `bounds.minimum[axis]` then
primitive ID.

- [ ] **Step 4: Implement pair-once sweep and cheap filters**

For each sorted primitive `i`, visit only later primitives `j`; break when
`min[j][axis] > max[i][axis]`. Count the pair once, skip equal owner IDs before
the exact predicate, reject non-overlapping three-axis AABBs, and call
`hasIllegalTriangleContact()` for the remainder. Propagate predicate errors.

- [ ] **Step 5: Produce deterministic owner output and diagnostics**

Insert illegal owner IDs into a temporary ordered set, count each unordered
owner pair once for `illegal_owner_pairs`, then copy IDs to the result vector.
Do not add a topology shortcut yet: retain `topology_rejections == 0` until a
separate parity test proves a shortcut safe.

- [ ] **Step 6: Build and run the focused test to verify GREEN**

```powershell
cmake --build build-release --config Release --target boundary_mesh_batch_self_collision_detector_test
ctest --test-dir build-release -C Release -R batch_self_collision_detector --output-on-failure
```

Expected: PASS.

- [ ] **Step 7: Run all spatial collision tests**

```powershell
ctest --test-dir build-release -C Release -R "(collision_index|triangle_contact|incremental_collision)" --output-on-failure
```

Expected: all selected tests pass.

- [ ] **Step 8: Commit the implementation**

```powershell
git add CMakeLists.txt src/spatial/batch_self_collision_detector.cpp tests/unit/spatial/batch_self_collision_detector_test.cpp
git commit -m "perf: batch self collision pairs"
```

### Task 3: Integrate the batch detector into layer filtering

**Files:**
- Modify: `src/growth/layer_collision_checker.cpp:566-617`
- Modify: `tests/unit/growth/layer_collision_checker_test.cpp`

**Interfaces:**
- Consumes: `BatchSelfCollisionDetector::detect()`.
- Produces: the same stopped-candidate set without building and querying a `CollisionIndex` per triangle.

- [ ] **Step 1: Add a regression case with two colliding candidates and one unaffected candidate**

Assert the same compacted front/cell result expected from the current
self-collision behavior, including stable owner-to-candidate mapping.

- [ ] **Step 2: Run the layer collision test and verify the new API is unused**

Use a temporary assertion or compile-time call expectation in the test helper
so the test fails until `filterSelfCollisions()` routes through the batch API.

- [ ] **Step 3: Replace the per-primitive query loop**

Keep candidate triangle construction unchanged. Call `detect(all)`, propagate
its `SpatialError`, allocate `stopped`, and mark each returned owner ID after
checking it is within the candidate count. Remove the temporary
`CollisionIndex::build(all)` and `queryIllegalContacts()` loop.

- [ ] **Step 4: Run focused growth tests to verify GREEN**

```powershell
cmake --build build-release --config Release --target boundary_mesh_layer_collision_checker_test boundary_mesh_collision_growth_pipeline_test
ctest --test-dir build-release -C Release -R "(layer_collision_checker|collision_growth_pipeline)" --output-on-failure
```

Expected: PASS with unchanged candidate decisions.

- [ ] **Step 5: Run the complete test suite**

```powershell
ctest --test-dir build-release -C Release --output-on-failure
```

Expected: all tests pass.

- [ ] **Step 6: Commit the integration**

```powershell
git add src/growth/layer_collision_checker.cpp tests/unit/growth/layer_collision_checker_test.cpp
git commit -m "perf: batch current layer self collisions"
```

### Task 4: Benchmark 2dot5 and decide whether an octree refinement is justified

**Files:**
- Modify only if required for existing diagnostics: `src/growth/regular_layer_generator.cpp` (preserve and build on the user's timing changes; do not overwrite them)
- Create: benchmark output outside the repository under the existing 2dot5 case directory.

**Interfaces:**
- Consumes: Release CLI and the existing 2dot5 CGNS input.
- Produces: timing and cell-count comparison against the 471.9-second baseline.

- [ ] **Step 1: Build the Release CLI**

Use the repository's existing Release build directory and CLI target discovered
from CMake rather than reconfiguring unrelated options.

- [ ] **Step 2: Run 20 layers**

Input:

```text
C:\Users\zpern\Desktop\todo\jiuyuan-quailty\test_case\2dot5_cf\2dot5_cf.cgns
```

Parameters: first height `0.1`, growth ratio `1.2`, layer count `20`.

- [ ] **Step 3: Compare correctness and timings**

Confirm all 20 per-layer cell counts equal the previous optimized run. Report
total time, self-collision time, worst layer, and detector counters. If
`sweep_pairs` remains much larger than `exact_tests`, propose an octree-local
sweep as a measured second phase; otherwise keep the simpler global sweep.

- [ ] **Step 4: Run final repository checks**

```powershell
git diff --check
git status --short
```

Confirm only intended files plus the pre-existing user changes remain.
