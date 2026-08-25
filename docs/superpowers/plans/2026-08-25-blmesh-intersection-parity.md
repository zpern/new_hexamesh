# BLMesh Multi-Normal Intersection Parity Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the generic multi-normal collision resolver with BLMesh-equivalent geometric-contact and bad-point length-resolution behavior.

**Architecture:** Add a private BLMesh-compatible triangle intersection checker beside the resolver, preserving coordinate-based shared-point cases and returning bad topology vertex IDs. Keep the public resolver interface, but extend its result to distinguish ALM fallback from accepted lengths so the generator never publishes zero-height split branches as success.

**Tech Stack:** C++17, Eigen point/vector types, existing Result error transport, CMake/CTest, vendored TiGER geometry predicates where BLMesh uses them.

## Global Constraints

- Work only on `codex/multi-normal-topology-transition`.
- Preserve BLMesh loop order, constants, exact coordinate equality, and shared-point branching.
- Write and run a failing test before each production behavior change.
- Do not commit machine-specific paths or temporary real-case harnesses.

---

### Task 1: Port BLMesh triangle-contact semantics

**Files:**
- Create: `include/boundary_mesh/growth/blmesh_intersection_checker.hpp`
- Create: `src/growth/blmesh_intersection_checker.cpp`
- Create: `tests/unit/growth/blmesh_intersection_checker_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Produces: `bool blmeshTrianglesIntersect(const TrianglePoints &, const TrianglePoints &)`.
- Semantics: coordinate-based `same_count` cases copied from `MNormal/include/intersection_check.hpp::checkIntersect()`.

- [ ] **Step 1: Write failing shared-contact tests**

Create triangles that are identical, share an edge, share one vertex without crossing, share one vertex with a non-shared edge crossing, and properly intersect without shared vertices. Assert BLMesh results: `false, false, false, true, true`.

- [ ] **Step 2: Verify RED**

Run `cmake --build build --config Release --target boundary_mesh_blmesh_intersection_checker_test` and confirm compilation fails because the checker interface does not exist.

- [ ] **Step 3: Port the minimal checker**

Copy BLMesh's `same_count` loop and branches. Use the same TiGER `tri_tri_overlap_test_3d` and `lin_tri_intersect3d` calls for zero- and one-shared-point cases. Return false for two or three shared coordinates.

- [ ] **Step 4: Verify GREEN and commit**

Run the focused test and commit as `feat: port BLMesh triangle intersection semantics`.

---

### Task 2: Port bad-point surface checking

**Files:**
- Modify: `include/boundary_mesh/growth/multi_normal_intersection_resolver.hpp`
- Modify: `src/growth/multi_normal_intersection_resolver.cpp`
- Modify: `tests/unit/growth/multi_normal_intersection_resolver_test.cpp`

**Interfaces:**
- Produces internally: `checkOuterSurfaceIntersection(...) -> set<size_t>` containing bad topology vertex IDs.
- Consumes the Task 1 checker for precise triangle pairs.

- [ ] **Step 1: Write failing bottom/top legal-contact tests**

Construct a split triangle whose two stationary corners make the displaced top share a bottom edge geometrically. Assert no bad points and no shrinking. Confirm the current resolver shrinks or zeroes it.

- [ ] **Step 2: Verify RED**

Run `ctest --test-dir build -C Release -R boundary_mesh_multi_normal_intersection_resolver_test --output-on-failure` and confirm the new assertion fails.

- [ ] **Step 3: Port outer-surface assembly and bad-point collection**

Build BLMesh-equivalent bottom and top triangle collections, query possible pairs, apply Task 1 semantics, and insert all three query-face vertex IDs only when the BLMesh checker reports intersection. Remove the current bad-face expansion path.

- [ ] **Step 4: Verify GREEN and commit**

Run checker and resolver tests and commit as `fix: port BLMesh multi-normal surface checking`.

---

### Task 3: Port length smoothing, shrinking, and ALM fallback

**Files:**
- Modify: `include/boundary_mesh/growth/multi_normal_intersection_resolver.hpp`
- Modify: `include/boundary_mesh/growth/multi_normal_types.hpp`
- Modify: `src/growth/multi_normal_intersection_resolver.cpp`
- Modify: `src/growth/multi_normal_transition_generator.cpp`
- Modify: `tests/unit/growth/multi_normal_intersection_resolver_test.cpp`
- Modify: `tests/integration/multi_normal_transition_generator_test.cpp`

**Interfaces:**
- `ResolvedMultiNormalLengths` adds `bool fallback_to_single_normal`.
- Generator returns `applied == false`, unchanged front, and empty transition mesh when ALM fallback is requested.

- [ ] **Step 1: Write failing bad-point-only and fallback tests**

Assert shrinking changes only explicitly bad points, neighbor smoothing applies the BLMesh `1.1` cap, and an unresolved ALM candidate returns fallback rather than accepted coincident branches.

- [ ] **Step 2: Verify RED**

Run resolver and generator tests and confirm the fallback assertion fails against current zero-retry acceptance.

- [ ] **Step 3: Port BLMesh length functions**

Mechanically implement neighbor rebuilding, queue-based smoothing, bad-point `0.8` shrinking, bad-point zeroing, 20 shrink iterations, and ALM fallback at the zero-step boundary.

- [ ] **Step 4: Verify GREEN and commit**

Run focused tests and commit as `fix: port BLMesh multi-normal length resolution`.

---

### Task 4: Add the 2dot5 target-point regression

**Files:**
- Create temporarily, then delete: `tests/manual/multi_normal_pls_compare.cpp`
- Modify temporarily, then restore: `CMakeLists.txt`
- Extend: `tests/unit/growth/multi_normal_intersection_resolver_test.cpp` with the extracted local eight-face patch.

**Interfaces:**
- Fixture root coordinate: `(205.9503, -600, 3.3557019)`.
- Expected: three branch points, each with nonzero displacement and initial resolved length `0.01`; BLMesh reference requires zero shrink iterations.

- [ ] **Step 1: Add the extracted local-patch regression and verify RED**

Use the eight incident PLS triangles around source point 38 and assert all three transformed branches move. Confirm current behavior leaves branches coincident.

- [ ] **Step 2: Make only parity corrections exposed by the fixture**

Correct adapter ordering or point mapping only when comparison with the original BLMesh run identifies a concrete mismatch.

- [ ] **Step 3: Run the full PLS case**

Run original BLMesh and the port with height `0.01`; compare target branch coordinates, bad-point count, shrink count, transformed surface, and transition cells.

- [ ] **Step 4: Remove the harness, verify, and commit**

Remove temporary source/target, run `git diff --check`, and commit the permanent regression as `test: cover 2dot5 BLMesh multi-normal parity`.

---

### Task 5: Full verification

**Files:**
- Modify only for in-scope corrections exposed by verification.

- [ ] **Step 1: Run complete Release verification**

Run `cmake -S . -B build -DBOUNDARY_MESH_ENABLE_CGNS_IO=OFF`, `cmake --build build --config Release`, and `ctest --test-dir build -C Release --output-on-failure`. Require zero failures.

- [ ] **Step 2: Confirm clean delivery state**

Run `git diff --check`, `git status --short --branch`, and inspect the final commit list. Require no temporary harness or uncommitted files.
