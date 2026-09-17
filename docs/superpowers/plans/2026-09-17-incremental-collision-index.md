# Incremental Collision Index Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace repeated global boundary-index construction with a persistent `CollisionTriangle` Octree and local external-patch collision trials.

**Architecture:** Keep the original surface immutable, maintain committed exposed-boundary triangles in an incrementally updated Octree, and bulk-build a separate candidate index per transition iteration. Stable primitive and group IDs support atomic face/group replacement; exact contact classification remains in `hasIllegalTriangleContact`.

**Tech Stack:** C++17, Eigen geometry types, CMake/CTest, existing `Result<T, SpatialError>` and collision-contact code.

## Global Constraints

- Do not import `BLFront`, `MBLNode`, `OctreeAgent`, or global triangle storage from `blmesh`.
- Do not change the semantics of `hasIllegalTriangleContact` or sliding-contact permissions.
- Keep original, committed-history, and provisional-candidate geometry in separate indexes.
- Primitive IDs are monotonic and never reused; group operations are atomic.
- Unknown group erasure is an explicit error.
- Query and report results are sorted and deduplicated for deterministic output.
- Preserve the user's existing uncommitted changes in CLI, growth, and transition files; inspect the live diff before editing overlapping files.

---

## File Structure

- Create `include/boundary_mesh/spatial/incremental_collision_index.hpp`: public stable-ID, group, options, diagnostics, query, and rebuild API.
- Create `src/spatial/incremental_collision_index.cpp`: primitive store, Octree insertion/deletion/query, tombstones, root expansion, and rebuilding.
- Create `tests/unit/spatial/incremental_collision_index_test.cpp`: deterministic mutation and immutable-index parity tests.
- Modify `include/boundary_mesh/spatial/spatial_error.hpp`: add explicit missing-group error.
- Modify `CMakeLists.txt` and `tests/CMakeLists.txt`: compile and register the new module/test.
- Modify `include/boundary_mesh/growth/exposed_boundary.hpp` and `src/growth/exposed_boundary.cpp`: transactional committed-boundary index ownership.
- Modify `tests/unit/growth/exposed_boundary_test.cpp`: index synchronization and atomic failure tests.
- Modify `include/boundary_mesh/growth/layer_collision_checker.hpp`, `src/growth/layer_collision_checker.cpp`, and call sites in `src/growth/regular_layer_generator.cpp`: consume the persistent historical index.
- Modify `include/boundary_mesh/transition/transition_boundary_checker.hpp` and `src/transition/transition_boundary_checker.cpp`: cached indexes and one combined inspection report.
- Modify `include/boundary_mesh/transition/layer_transition_resolver.hpp`, `src/transition/layer_transition_resolver.cpp`, `src/boundary_layer/incremental_boundary_layer_generator.cpp`: one scan per iteration and group-local external trials.
- Modify transition/growth tests and add `benchmarks/incremental_collision_index_benchmark.cpp`: correctness and performance evidence.

### Task 1: Stable-ID incremental index core

**Files:**
- Create: `include/boundary_mesh/spatial/incremental_collision_index.hpp`
- Create: `src/spatial/incremental_collision_index.cpp`
- Create: `tests/unit/spatial/incremental_collision_index_test.cpp`
- Modify: `include/boundary_mesh/spatial/spatial_error.hpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `CollisionTriangle`, `Aabb`, `makeAabb`, `hasIllegalTriangleContact`, and `Result<T, SpatialError>`.
- Produces: `CollisionPrimitiveId`, `CollisionGroupId`, `CollisionPrimitiveGroup`, `IncrementalCollisionIndexOptions`, `CollisionIndexDiagnostics`, and `IncrementalCollisionIndex`.

- [ ] **Step 1: Register a failing empty/index mutation test**

Add a CMake test target and write a test that constructs an empty index, inserts one group, queries a crossing triangle, erases the group, and verifies that a second erase fails:

```cpp
const auto empty = IncrementalCollisionIndex::build({});
assert(empty.hasValue());
auto index = std::move(empty.value());
const CollisionGroupId group{7};
assert(index.insertGroup({group, {horizontalTriangle()}}).hasValue());
assert(index.queryIllegalContacts(verticalTriangle()).size() == 1);
assert(index.eraseGroup(group).hasValue());
assert(index.queryIllegalContacts(verticalTriangle()).empty());
const auto missing = index.eraseGroup(group);
assert(!missing.hasValue());
assert(missing.error() == SpatialError::MissingPrimitiveGroup);
```

Add `src/spatial/incremental_collision_index.cpp` to `boundary_mesh_spatial` and register `boundary_mesh_incremental_collision_index_test` linked to `BoundaryMesh::Spatial`.

- [ ] **Step 2: Build to verify the test fails**

Run:

```powershell
cmake --build build --config Debug --target boundary_mesh_incremental_collision_index_test
```

Expected: compilation fails because `incremental_collision_index.hpp` and its types do not exist.

- [ ] **Step 3: Add the public types and minimal linear implementation**

Define the exact public API:

```cpp
using CollisionPrimitiveId = std::uint64_t;
using CollisionGroupId = std::uint64_t;

struct CollisionPrimitiveGroup {
    CollisionGroupId id{};
    std::vector<CollisionTriangle> triangles;
};

struct IncrementalCollisionIndexOptions {
    std::size_t maximum_depth{12};
    std::size_t target_leaf_capacity{64};
    std::size_t rebuild_leaf_capacity{1024};
    Scalar rebuild_inactive_ratio{Scalar{0.30}};
    Scalar root_padding{Scalar{0.05}};
};

struct CollisionIndexDiagnostics {
    std::uint64_t inserts{}, erases{}, queries{};
    std::uint64_t broad_phase_candidates{}, exact_tests{};
    std::uint64_t rebuilds{}, root_expansions{};
    std::size_t active_primitives{}, inactive_primitives{};
    std::size_t maximum_leaf_load{};
};
```

Expose `build`, `insertGroup`, `eraseGroup`, `queryCandidates`,
`queryIllegalContacts`, `primitive`, `diagnostics`, and `rebuildIfDegraded`.
Implement the first passing version with a stable `std::vector` store,
`std::map<CollisionGroupId, std::vector<CollisionPrimitiveId>>`, active flags,
linear AABB filtering, and exact contact checks. Validate every triangle before
mutating and add `SpatialError::MissingPrimitiveGroup`.

- [ ] **Step 4: Run the focused test**

```powershell
cmake --build build --config Debug --target boundary_mesh_incremental_collision_index_test
ctest --test-dir build -C Debug -R boundary_mesh_incremental_collision_index_test --output-on-failure
```

Expected: build succeeds and 1/1 test passes.

- [ ] **Step 5: Commit the API and linear reference behavior**

```powershell
git add CMakeLists.txt tests/CMakeLists.txt include/boundary_mesh/spatial/spatial_error.hpp include/boundary_mesh/spatial/incremental_collision_index.hpp src/spatial/incremental_collision_index.cpp tests/unit/spatial/incremental_collision_index_test.cpp
git commit -m "feat: add stable incremental collision index API"
```

### Task 2: Octree broad phase, tombstones, and rebuilding

**Files:**
- Modify: `src/spatial/incremental_collision_index.cpp`
- Modify: `include/boundary_mesh/spatial/incremental_collision_index.hpp`
- Modify: `tests/unit/spatial/incremental_collision_index_test.cpp`

**Interfaces:**
- Consumes: Task 1's complete public API.
- Produces: the same API backed by an incremental Octree; no caller changes.

- [ ] **Step 1: Add failing parity and degradation tests**

Add deterministic tests that:

1. insert 200 separated groups and compare every query's business owners with a fresh `CollisionIndex` over active triangles;
2. erase every third group, rebuild, and verify IDs and results are unchanged;
3. insert outside the initial root and assert `root_expansions == 1`;
4. force a low `rebuild_leaf_capacity`, call `rebuildIfDegraded()`, and assert it returns `true` once and `false` immediately afterward;
5. pass two triangles where the second is degenerate and assert the complete group is rejected with no active primitives.

Use a fixed arithmetic sequence for coordinates rather than nondeterministic randomness.

- [ ] **Step 2: Run the test to expose missing Octree behavior**

```powershell
cmake --build build --config Debug --target boundary_mesh_incremental_collision_index_test
ctest --test-dir build -C Debug -R boundary_mesh_incremental_collision_index_test --output-on-failure
```

Expected: assertions for root expansion/rebuild diagnostics fail against the linear implementation.

- [ ] **Step 3: Implement the private Octree**

Add private `StoredPrimitive` and `Node` types:

```cpp
struct StoredPrimitive {
    CollisionTriangle triangle;
    Aabb bounds;
    CollisionGroupId group{};
    bool active{};
};

struct Node {
    Aabb bounds;
    std::array<std::size_t, 8> children{};
    std::vector<CollisionPrimitiveId> primitives;
    std::size_t inactive_count{};
    bool leaf{true};
};
```

Implement these invariants:

- validate and compute all group AABBs before allocating IDs;
- expand/rebuild the root before committing an out-of-root group;
- store a primitive in every overlapping leaf;
- mark erased store entries inactive without reusing IDs;
- deduplicate query candidates by primitive ID;
- sort query results before exact checks;
- rebuild exclusively from active store entries while preserving IDs;
- recompute `maximum_leaf_load` after mutation batches.

- [ ] **Step 4: Run spatial tests**

```powershell
cmake --build build --config Debug --target boundary_mesh_incremental_collision_index_test boundary_mesh_spatial_collision_index_test boundary_mesh_spatial_triangle_contact_test
ctest --test-dir build -C Debug -R "boundary_mesh_(incremental_collision_index|spatial_collision_index|spatial_triangle_contact)_test" --output-on-failure
```

Expected: all 3 tests pass.

- [ ] **Step 5: Commit Octree behavior**

```powershell
git add include/boundary_mesh/spatial/incremental_collision_index.hpp src/spatial/incremental_collision_index.cpp tests/unit/spatial/incremental_collision_index_test.cpp
git commit -m "feat: back incremental collisions with octree"
```

### Task 3: Transactional exposed-boundary index

**Files:**
- Modify: `include/boundary_mesh/growth/exposed_boundary.hpp`
- Modify: `src/growth/exposed_boundary.cpp`
- Modify: `tests/unit/growth/exposed_boundary_test.cpp`

**Interfaces:**
- Consumes: `IncrementalCollisionIndex`, `CollisionPrimitiveGroup`.
- Produces: `Result<std::monostate, SpatialError> ExposedBoundaryTracker::apply(...)` and `const IncrementalCollisionIndex &collisionIndex() const noexcept`.

- [ ] **Step 1: Write failing synchronization tests**

After every existing `prepare/apply`, query both:

```cpp
const auto fresh_triangles = tracker.collisionTriangles();
const auto fresh = CollisionIndex::build(fresh_triangles.value());
const auto incremental_hits = tracker.collisionIndex().queryIllegalContacts(probe);
const auto fresh_hits = fresh.value().queryIllegalContacts(probe);
assert(incremental_hits.size() == fresh_hits.size());
```

Also prepare an update containing a valid erasure plus a degenerate insertion;
assert `apply()` fails and both `faces()` and `collisionIndex().diagnostics().active_primitives`
remain unchanged.

- [ ] **Step 2: Build to verify API failure**

```powershell
cmake --build build --config Debug --target boundary_mesh_exposed_boundary_test
```

Expected: compilation fails because `apply()` does not return `Result` and `collisionIndex()` does not exist.

- [ ] **Step 3: Implement face-to-group synchronization**

Add:

```cpp
IncrementalCollisionIndex collision_index_;
std::map<BoundaryFaceKey, CollisionGroupId> collision_groups_;
CollisionGroupId next_collision_group_id_{1};
```

Extract one helper that triangulates a `BoundaryFace` into the exact metadata
currently produced by `collisionTriangles()`. Use it both for full export and
incremental insertion. In `apply()`:

1. validate that every erased face and collision group exists;
2. triangulate and validate every inserted face without mutating state;
3. reserve all primitive, group, and face-map storage required by the plan;
4. apply the now-infallible group erasures and insertions;
5. update the face map only after the collision-index mutation completes.

Add a private debug invariant check that every face key has exactly one active
group. If an operation after validation can still return `SpatialError`, add a
rollback journal containing erased groups and inserted group IDs and restore it
before returning failure.

Update every call site to check the returned `Result` and propagate
`SpatialError` through the existing growth error path.

- [ ] **Step 4: Run boundary and growth-failure tests**

```powershell
cmake --build build --config Debug --target boundary_mesh_exposed_boundary_test boundary_mesh_regular_layer_growth_failure_test boundary_mesh_collision_growth_failure_test
ctest --test-dir build -C Debug -R "boundary_mesh_(exposed_boundary|regular_layer_growth_failure|collision_growth_failure)_test" --output-on-failure
```

Expected: all 3 tests pass.

- [ ] **Step 5: Commit transactional history indexing**

```powershell
git add include/boundary_mesh/growth/exposed_boundary.hpp src/growth/exposed_boundary.cpp src/growth/regular_layer_generator.cpp tests/unit/growth/exposed_boundary_test.cpp
git commit -m "feat: index exposed boundary incrementally"
```

### Task 4: Reuse history in regular obstacle checks

**Files:**
- Modify: `include/boundary_mesh/growth/layer_collision_checker.hpp`
- Modify: `src/growth/layer_collision_checker.cpp`
- Modify: `src/growth/regular_layer_generator.cpp`
- Modify: `tests/unit/growth/layer_collision_checker_test.cpp`
- Modify: `tests/integration/collision_growth_pipeline_test.cpp`

**Interfaces:**
- Consumes: `ExposedBoundaryTracker::collisionIndex()`.
- Produces: `filterAgainstObstacles` overloads that query `const IncrementalCollisionIndex &historical` without building it.

- [ ] **Step 1: Add a failing reuse assertion**

Add a diagnostic accessor/test seam and create a history tracker containing an
obstacle. Call `filterAgainstObstacles()` twice, assert both outputs match, and
assert the historical diagnostics report increased queries but unchanged
`rebuilds` and `active_primitives`.

- [ ] **Step 2: Run the focused test**

```powershell
cmake --build build --config Debug --target boundary_mesh_layer_collision_checker_test
ctest --test-dir build -C Debug -R boundary_mesh_layer_collision_checker_test --output-on-failure
```

Expected: test fails because the checker still builds a temporary immutable history index.

- [ ] **Step 3: Replace history construction with persistent queries**

Change both obstacle overloads so their history branch is:

```cpp
stopped[index] = hitsIndex(triangles.value(), original_surface) ||
    hitsIndex(triangles.value(), exposed_boundary.collisionIndex());
```

Add an overload/template of `hitsIndex` for `IncrementalCollisionIndex` that
uses `queryIllegalContacts`. Remove the two calls to
`exposed_boundary.collisionTriangles()` and `CollisionIndex::build()`.

- [ ] **Step 4: Run collision-growth tests**

```powershell
cmake --build build --config Debug --target boundary_mesh_layer_collision_checker_test boundary_mesh_collision_growth_pipeline_test
ctest --test-dir build -C Debug -R "boundary_mesh_(layer_collision_checker|collision_growth_pipeline)_test" --output-on-failure
```

Expected: both tests pass with identical stopped faces.

- [ ] **Step 5: Commit history reuse**

```powershell
git add include/boundary_mesh/growth/layer_collision_checker.hpp src/growth/layer_collision_checker.cpp src/growth/regular_layer_generator.cpp tests/unit/growth/layer_collision_checker_test.cpp tests/integration/collision_growth_pipeline_test.cpp
git commit -m "perf: reuse exposed boundary collision index"
```

### Task 5: One transition inspection per provisional boundary

**Files:**
- Modify: `include/boundary_mesh/transition/transition_boundary_checker.hpp`
- Modify: `src/transition/transition_boundary_checker.cpp`
- Modify: `src/transition/layer_transition_resolver.cpp`
- Modify: `tests/unit/transition/transition_boundary_checker_test.cpp`
- Modify: `tests/unit/transition/layer_transition_resolver_test.cpp`

**Interfaces:**
- Consumes: persistent historical index and existing prior-transition triangles.
- Produces: `TransitionCollisionReport inspect(const TransitionBoundaryInput&)` and one scan per resolver iteration.

- [ ] **Step 1: Add failing combined-report tests**

Create a fixture with one colliding external patch and one rollback dependency.
Assert a single `inspect()` returns both:

```cpp
assert(report.value().colliding_owners.size() == 1);
assert(report.value().colliding_owners[0].role == BoundaryOwnerRole::ExternalPatch);
assert(report.value().rollback_faces == std::vector<SurfaceFaceId>{expected});
```

Add a test diagnostic counter to the checker/resolver fixture and assert one
complete inspection for a no-rollback iteration.

- [ ] **Step 2: Build to verify the report API is absent**

```powershell
cmake --build build --config Debug --target boundary_mesh_transition_boundary_checker_test boundary_mesh_layer_transition_resolver_test
```

Expected: compilation fails because `TransitionCollisionReport` and `inspect()` do not exist.

- [ ] **Step 3: Implement unified inspection and cached history**

Define:

```cpp
struct TransitionCollisionReport {
    std::vector<LayerBoundaryOwner> colliding_owners;
    std::vector<SurfaceFaceId> rollback_faces;
};
```

Refactor the existing `findRollbackFacesImpl` traversal into `inspect()`. During
the same traversal, append unique colliding owners and rollback faces. Query
`input.historical_index` directly instead of rebuilding from
`historical_boundary`. Build the prior-transition immutable index once when
`prior_transition_boundary` changes, store it beside that boundary in
`IncrementalBoundaryLayerGenerator`, and pass a pointer through
`LayerTransitionInput`; never rebuild it inside `inspect()`. Keep compatibility wrappers temporarily if tests or
callers require them, implemented by delegating to `inspect()`.

In the resolver, keep the first report and use its rollback faces at the end of
the iteration. Re-run `inspect()` only after a patch group has actually changed.

- [ ] **Step 4: Run transition regressions**

```powershell
cmake --build build --config Debug --target boundary_mesh_transition_boundary_checker_test boundary_mesh_layer_transition_resolver_test boundary_mesh_incremental_layer_transition_pipeline_test
ctest --test-dir build -C Debug -R "boundary_mesh_(transition_boundary_checker|layer_transition_resolver|incremental_layer_transition_pipeline)_test" --output-on-failure
```

Expected: all 3 tests pass; existing owner and rollback ordering remains stable.

- [ ] **Step 5: Commit unified transition inspection**

```powershell
git add include/boundary_mesh/transition/transition_boundary_checker.hpp src/transition/transition_boundary_checker.cpp src/transition/layer_transition_resolver.cpp tests/unit/transition/transition_boundary_checker_test.cpp tests/unit/transition/layer_transition_resolver_test.cpp
git commit -m "perf: inspect transition collisions once"
```

### Task 6: Candidate groups and local external-patch trials

**Files:**
- Modify: `include/boundary_mesh/transition/provisional_transition_builder.hpp`
- Modify: `src/transition/provisional_transition_builder.cpp`
- Modify: `include/boundary_mesh/transition/layer_transition_resolver.hpp`
- Modify: `src/transition/layer_transition_resolver.cpp`
- Modify: `src/boundary_layer/incremental_boundary_layer_generator.cpp`
- Modify: `tests/unit/transition/layer_transition_resolver_test.cpp`
- Modify: `tests/integration/incremental_layer_transition_pipeline_test.cpp`

**Interfaces:**
- Consumes: incremental group insertion, erasure, and ignore-group queries.
- Produces: stable candidate group IDs and a patch-only builder callback for local distance trials.

- [ ] **Step 1: Add a failing many-safe-patches test**

Construct a resolver fixture with at least 100 terminal external patches and no
collision. Instrument provisional full builds and patch-only builds, then assert:

```cpp
assert(stable.hasValue());
assert(full_build_count == 1);
assert(patch_trial_count == 0);
assert(inspection_count == 1);
```

Add a second fixture with one colliding external patch and assert only that
patch is passed to the local trial callback during bracketing/binary search.

- [ ] **Step 2: Run tests to confirm current global rebuild behavior fails**

```powershell
cmake --build build --config Debug --target boundary_mesh_layer_transition_resolver_test
ctest --test-dir build -C Debug -R boundary_mesh_layer_transition_resolver_test --output-on-failure
```

Expected: counter assertions fail because `build_provisional` rebuilds the complete boundary during trials.

- [ ] **Step 3: Expose candidate groups and patch-only construction**

Extend `ProvisionalLayerTransition` with group metadata mapping owner identity to
the candidate index group. Add a callback to `LayerTransitionInput`:

```cpp
std::function<Result<std::vector<OwnedBoundaryTriangle>, LayerTransitionError>(
    SurfaceFaceId, Scalar)> build_external_patch;
```

Build the complete candidate index once after initial provisional construction.
Use the initial combined report to select colliding external owners. For each
trial, generate only that patch, query all fixed indexes plus candidate index
with `ignored_group`, and replace the group only when the trial is accepted.
After accepted replacements, derive rollback information from affected owners;
perform one final combined inspection only when a group changed.

- [ ] **Step 4: Run transition and boundary-layer tests**

```powershell
cmake --build build --config Debug --target boundary_mesh_layer_transition_resolver_test boundary_mesh_incremental_layer_transition_pipeline_test boundary_mesh_regular_layer_growth_pipeline_test
ctest --test-dir build -C Debug -R "boundary_mesh_(layer_transition_resolver|incremental_layer_transition_pipeline|regular_layer_growth_pipeline)_test" --output-on-failure
```

Expected: all 3 tests pass and the safe-patch fixture records one full build, zero trials.

- [ ] **Step 5: Commit local patch trials**

```powershell
git add include/boundary_mesh/transition/provisional_transition_builder.hpp src/transition/provisional_transition_builder.cpp include/boundary_mesh/transition/layer_transition_resolver.hpp src/transition/layer_transition_resolver.cpp src/boundary_layer/incremental_boundary_layer_generator.cpp tests/unit/transition/layer_transition_resolver_test.cpp tests/integration/incremental_layer_transition_pipeline_test.cpp
git commit -m "perf: validate external patches locally"
```

### Task 7: Diagnostics, benchmark, and full verification

**Files:**
- Create: `benchmarks/incremental_collision_index_benchmark.cpp`
- Modify: `tests/CMakeLists.txt`
- Modify: `src/growth/regular_layer_generator.cpp`
- Modify: `src/transition/layer_transition_resolver.cpp`
- Modify: `docs/design/modules/layer-transition.md`

**Interfaces:**
- Consumes: all completed index diagnostics and resolver counters.
- Produces: aggregate debug output and a non-CTest benchmark executable.

- [ ] **Step 1: Add the benchmark executable and acceptance checks**

Register `boundary_mesh_incremental_collision_index_benchmark` without
`add_test`. The program creates 60,000 representative boundary triangles,
performs one build, 10 percent erase/insert churn, and 60,000 queries. Print:

```text
build_ms=...
update_ms=...
query_ms=...
rebuilds=...
exact_tests=...
maximum_leaf_load=...
```

Return nonzero if query results differ from a fresh immutable-index sample.

- [ ] **Step 2: Build and run the benchmark baseline**

```powershell
cmake --build build --config Release --target boundary_mesh_incremental_collision_index_benchmark
& .\build\tests\Release\boundary_mesh_incremental_collision_index_benchmark.exe
```

Expected: exit code 0 with all six metrics printed. Record the output in the implementation handoff; do not encode machine-specific timing as a unit-test assertion.

- [ ] **Step 3: Replace temporary timing prints with aggregate diagnostics**

Remove the uncommitted `temporary layer stage` and per-external-face logging
only after preserving any useful counters in one layer summary. Under the
existing debug-log option, print one deterministic line containing history
queries/rebuilds, candidate full inspections, patch trials, broad-phase
candidates, and exact tests.

Update the transition design documentation to describe persistent history and
local patch validation.

- [ ] **Step 4: Run complete verification**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
cmake --build build --config Release --target boundary_mesh_cli
```

Expected: complete Debug suite passes and Release CLI builds.

Run the supplied 59,381-face configuration using the same invocation that
produced the baseline log. Capture a new log and verify:

```text
historical_global_builds_after_initialization=0
complete_transition_scans_per_iteration<=1
safe_external_patch_trials=0
layer_10_candidate_rejections_ms<20000
layer_10_elapsed_ms<45000
```

Also compare per-layer added-cell counts and run the project's existing volume,
skewness, and illegal-intersection validation scripts on the resulting mesh.

- [ ] **Step 5: Commit diagnostics and documentation**

```powershell
git add benchmarks/incremental_collision_index_benchmark.cpp tests/CMakeLists.txt src/growth/regular_layer_generator.cpp src/transition/layer_transition_resolver.cpp docs/design/modules/layer-transition.md
git commit -m "test: benchmark incremental collision indexing"
```

## Final Review Checklist

- [ ] `git diff` contains no accidental changes to user-owned configuration or `.vscode` files.
- [ ] Every incremental query parity test matches a freshly rebuilt immutable index.
- [ ] Historical boundary updates are atomic on all error paths.
- [ ] Transition owner, rollback, topology, and cell-count regressions are unchanged.
- [ ] The real case meets correctness checks before performance claims are made.
- [ ] The final profiler/log evidence identifies any remaining cost in regular self-collision separately from transition checking.
