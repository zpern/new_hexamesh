# BLMesh Multi-Normal Behavioral Parity Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the simplified multi-normal planner and fixed transition displacement with a function-by-function port of BLMesh MNormal splitting, virtual-sphere strategy selection, per-vertex height resolution, and surface-intersection retry behavior.

**Architecture:** Keep `generateMultiNormalTransition()` as the public stage and place the mechanically ported BLMesh algorithms behind private `boundary_mesh::blmesh_compat` components. Convert ordered `IncidentFaceFan` data into compatibility inputs, return the selected strategy as `VertexSplitPlan`, then resolve a per-branch length field before invoking the existing BLMesh-compatible Triangle Tetra builder.

**Tech Stack:** C++17, Eigen, CMake/CTest, existing BoundaryMesh spatial collision index, legacy VTK debug writer, CGNS real-case harness.

## Global Constraints

- Work only on branch `codex/multi-normal-topology-transition` in its existing isolated worktree.
- Port BLMesh control flow and candidate order mechanically; only names, ownership, ID types, namespace, errors, and interfaces may change.
- Do not reference BLMesh through an absolute build path or require an external BLMesh installation.
- `plane_skewness_threshold`, `convex_skewness_threshold`, and `maximum_strategy_count` must affect runtime behavior.
- Non-split vertices have zero displacement during the multi-normal transition stage.
- Affected Quads are triangulated after topology construction with the established skewness/tie rule.
- Transition volume construction remains Tetra-only and preserves `BuildCandidateVolume()` connectivity.
- Debug output remains disabled by default.
- Every production behavior is introduced through a failing test first.

---

### Task 1: Freeze BLMesh Splitter Enumeration as Parity Fixtures

**Files:**
- Create: `include/boundary_mesh/growth/detail/blmesh_splitter.hpp`
- Create: `src/growth/blmesh_splitter.cpp`
- Create: `tests/unit/growth/blmesh_splitter_parity_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Produces: `class BlmeshSplitter { public: const std::vector<std::set<std::size_t>> &combinations(const std::set<std::size_t> &splitters); };`
- Preserves: BLMesh `Splitter::getSpliiter()` subset membership and ordering.

- [ ] **Step 1: Record reference results**

Build a test table from the original `MNormal/include/Splitter.h` and implementation for candidate sets `{}`, `{1}`, `{1,3}`, and `{0,2,4}`. Store the expected ordered vectors explicitly in `blmesh_splitter_parity_test.cpp`; do not calculate expectations using the port.

- [ ] **Step 2: Write the failing test**

For every table entry, call `BlmeshSplitter{}.combinations(input)` and compare every ordered `std::set<std::size_t>` with the recorded BLMesh output. Also call the same input twice and verify cached and uncached results are identical.

- [ ] **Step 3: Verify RED**

Run:

```powershell
cmake --build build --config Release --target boundary_mesh_blmesh_splitter_parity_test
```

Expected: compilation fails because `blmesh_splitter.hpp` has no implementation target yet.

- [ ] **Step 4: Mechanically port `Splitter`**

Copy the original recursive subset generation and cache lookup into `boundary_mesh::detail::BlmeshSplitter`. Replace `int` ridge indices with `std::size_t`, replace the singleton with an owned object, and preserve insertion and returned-vector order.

- [ ] **Step 5: Verify GREEN and regressions**

Run:

```powershell
cmake --build build --config Release --target boundary_mesh_blmesh_splitter_parity_test
ctest --test-dir build -C Release -R "boundary_mesh_(blmesh_splitter_parity|multi_normal_split_planner)_test" --output-on-failure
```

Expected: both tests pass.

- [ ] **Step 6: Commit**

```powershell
git add CMakeLists.txt tests/CMakeLists.txt include/boundary_mesh/growth/detail/blmesh_splitter.hpp src/growth/blmesh_splitter.cpp tests/unit/growth/blmesh_splitter_parity_test.cpp
git commit -m "feat: port BLMesh splitter enumeration"
```

---

### Task 2: Port BLMesh Geometry Functions with Numeric Parity

**Files:**
- Create: `include/boundary_mesh/growth/detail/blmesh_geometry.hpp`
- Create: `src/growth/blmesh_geometry.cpp`
- Create: `tests/unit/growth/blmesh_geometry_parity_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Produces: `Scalar blmeshMinCos(const Vector3 &, const std::vector<Vector3> &);`
- Produces: `Result<Vector3, MultiNormalError> blmeshMostNormal(const std::vector<Vector3> &);`
- Produces internal three-normal circle-center candidate with BLMesh candidate ordering.

- [ ] **Step 1: Write numeric parity fixtures**

Record original BLMesh outputs for one normal, two normals at 60 degrees, three non-coplanar normals, four normals with tied pair candidates, and a nearly collinear triplet. Assert direction components and minimum cosine within `1e-12`.

- [ ] **Step 2: Verify RED**

Build `boundary_mesh_blmesh_geometry_parity_test`; expect failure because the compatibility functions are absent.

- [ ] **Step 3: Mechanically port geometry code**

Copy `getMinCos`, `getMostNormal`, and the three-normal circle-center calculation statement-for-statement. Replace `BLVector` operators with Eigen operations, retain loop nesting and strict comparison behavior, and return `InvalidMultiNormalTopology` for non-finite or zero candidates.

- [ ] **Step 4: Verify GREEN**

Run the geometry parity test and existing growth-direction tests. Expected: all pass.

- [ ] **Step 5: Commit**

Commit the two compatibility files, parity test, and build-list updates as `feat: port BLMesh multi-normal geometry`.

---

### Task 3: Port Virtual-Sphere Data, Boundary Validation, and Hashing

**Files:**
- Create: `include/boundary_mesh/growth/detail/blmesh_virtual_sphere.hpp`
- Create: `src/growth/blmesh_virtual_sphere.cpp`
- Create: `tests/unit/growth/blmesh_virtual_sphere_parity_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Produces: private equivalents of BLMesh virtual point, virtual triangle, boundary, mesh, and mesh hash structures.
- Consumes: `Vector3`, splitter IDs, cyclic classifications, manifold edges, and manifold coordinates.
- Produces: `bool isBoundaryValid() const`, valid point/extra point counts, triangles, branch normals, and source splitter mapping used by the adapter.

- [ ] **Step 1: Write failing boundary fixtures**

Create explicit valid two-branch, valid three-branch, open boundary, duplicated-edge, and non-manifold-edge fixtures. Record original BLMesh `isBoundaryValid()`, point counts, and triangle adjacency for each.

- [ ] **Step 2: Verify RED**

Build the new test and confirm missing compatibility types cause the expected failure.

- [ ] **Step 3: Port storage and boundary construction**

Mechanically copy `VirtualSphereBoundary`, `VirtualSphereMesh`, `BoundaryTriangulation`, and `VirtualSphereMeshHasher` functions required by `ComplexNode::SplitNode()` and strategy generation. Rename types and replace global IDs, but preserve point insertion, triangle insertion, sorting, set/map iteration, and hash semantics.

- [ ] **Step 4: Port manifold validation**

Copy `addBoundaryTriangles()` and `isBoundaryValid()` without algorithmic simplification. Convert exceptions into `Result` only at the outer adapter; internal parity code may preserve invariant assertions where BLMesh uses them.

- [ ] **Step 5: Verify GREEN**

Run virtual-sphere, splitter, and geometry parity tests. Expected: all pass.

- [ ] **Step 6: Commit**

Commit as `feat: port BLMesh virtual sphere boundary`.

---

### Task 4: Port Strategy Optimizers and Final Mesh Selection

**Files:**
- Create: `include/boundary_mesh/growth/detail/blmesh_virtual_sphere_generator.hpp`
- Create: `src/growth/blmesh_virtual_sphere_generator.cpp`
- Create: `tests/unit/growth/blmesh_virtual_sphere_strategy_parity_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Produces: `BlmeshVirtualSphereGenerator::addStrategy`, `cutStrategies`, `generate`, `finalMesh`, and `problemCount`.
- Consumes: compatibility virtual-sphere strategies from Task 3 and geometry scoring from Task 2.

- [ ] **Step 1: Capture original strategy results**

Use fixed two-, three-, and four-branch virtual-sphere inputs. Record original strategy count before/after trimming, selected mesh hash, selected skewness, valid point count, extra point count, and final Triangle connectivity.

- [ ] **Step 2: Write and run failing test**

Assert the port produces the recorded values for `maximum_strategy_count` values 1, 2, and 20. Verify RED because the generator is absent.

- [ ] **Step 3: Port the strategy pipeline**

Mechanically port `VirtualSphereMeshStrategy`, `VirtualSphereMeshGenerator`, `MeshEvaluation`, `PointOptimizer`, `TopologyOptimizer`, `mergeoptimizer`, `combineoptimizer`, `mostnormaloptimizer`, and directly required helper functions. Keep optimizer order, strict comparisons, strategy trimming order, mesh hashing, and final selection unchanged.

- [ ] **Step 4: Verify GREEN**

Run all four BLMesh compatibility parity tests. Expected: all pass with numeric tolerance `1e-12` where floating-point comparison is required.

- [ ] **Step 5: Commit**

Commit as `feat: port BLMesh virtual sphere strategies`.

---

### Task 5: Replace the Simplified Split Planner

**Files:**
- Modify: `src/growth/multi_normal_split_planner.cpp`
- Modify: `include/boundary_mesh/growth/multi_normal_types.hpp`
- Modify: `include/boundary_mesh/growth/multi_normal_error.hpp`
- Rewrite: `tests/unit/growth/multi_normal_split_planner_test.cpp`
- Create: `tests/unit/growth/blmesh_split_plan_parity_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Keeps: `planMultiNormalSplits(const GrowthFront &, const std::vector<IncidentFaceFan> &, const MultiNormalOptions &)`.
- Uses: Tasks 1-4 compatibility components.
- Produces: `VertexSplitPlan` with BLMesh branch and splitter order.

- [ ] **Step 1: Write failing option-use tests**

Add fixtures where changing only `plane_skewness_threshold` changes retained splitter subsets and where changing only `maximum_strategy_count` changes the selected strategy. Verify the current simplified planner fails both assertions.

- [ ] **Step 2: Write end-to-end local parity tests**

For convex-only, convex-plus-plane, more-than-16 candidates, multiple strategies, invalid manifold, and no-improvement fans, compare recorded BLMesh selected status, splitter neighbor IDs, face groups, branch count/order, and branch directions within `1e-12`.

- [ ] **Step 3: Mechanically port `ComplexNode::SplitNode()`**

Replace the all-convex grouping with the exact ridge-sign calculation, `plane_skewness_threshold + count * 0.01` loop, `BlmeshSplitter` enumeration, convex/plane filtering, cyclic classification rotation, most-normal calculation, manifold construction, boundary validation, strategy insertion, trimming, and generation.

- [ ] **Step 4: Adapt the final virtual mesh**

Map BLMesh virtual point global indices to `SplitBranch`, preserve selected branch order, map classification face indices back to `IncidentFaceSector::face_index`, and map splitter indices to `previous_vertex`. Return a structured error for inconsistent mappings.

- [ ] **Step 5: Verify GREEN**

Run all compatibility tests, both planner tests, topology-builder tests, and quad-triangulator tests. Expected: all pass.

- [ ] **Step 6: Commit**

Commit as `fix: restore BLMesh multi-normal split parity`.

---

### Task 6: Introduce Per-Vertex Transition Length Fields

**Files:**
- Create: `include/boundary_mesh/growth/multi_normal_length_field.hpp`
- Create: `src/growth/multi_normal_length_field.cpp`
- Create: `tests/unit/growth/multi_normal_length_field_parity_test.cpp`
- Modify: `include/boundary_mesh/growth/multi_normal_types.hpp`
- Modify: `src/growth/multi_normal_transition_builder.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Add to `MultiNormalOptions`: source-vertex first-layer heights or a callback resolving height by `source_vertex_id`.
- Produces: `ResolvedMultiNormalLengths` containing topology-vertex lengths and iteration diagnostics.
- Keeps `transition_height` only as an explicit uniform fallback for compatibility; resolved per-source heights take precedence.

- [ ] **Step 1: Write failing initialization tests**

Verify split branches inherit their source point's height, non-split vertices are zero, duplicate split branches start equal, local-edge limiting matches `FixedLength()`, and neighbor smoothing enforces BLMesh's 1.1 upper bound.

- [ ] **Step 2: Verify RED**

Run the new focused target and confirm it fails because the length-field API is absent.

- [ ] **Step 3: Port length initialization and smoothing**

Mechanically port `FixedLength`, `InitLengthField`, `RebuildPointNeighbors`, and `SmoothLengthField`. Preserve queue initialization, neighbor iteration order, strict `>` comparisons, 0.3/0.8 local-edge scaling, and 0.1 high-ratio propagation.

- [ ] **Step 4: Apply resolved lengths**

Change transition point construction from one `transition_height` to `length[topology_id] * normalized(direction)`. Keep ordinary upper points geometrically coincident with their bottoms and preserve existing Tetra distinct-coordinate filtering.

- [ ] **Step 5: Verify GREEN**

Run length-field and transition-builder tests. Expected: exact vector parity for fixtures and all previous connectivity tests pass.

- [ ] **Step 6: Commit**

Commit as `feat: port BLMesh multi-normal length field`.

---

### Task 7: Port Surface-Intersection Resolution

**Files:**
- Create: `include/boundary_mesh/growth/multi_normal_intersection_resolver.hpp`
- Create: `src/growth/multi_normal_intersection_resolver.cpp`
- Create: `tests/unit/growth/multi_normal_intersection_resolver_test.cpp`
- Modify: `include/boundary_mesh/growth/multi_normal_error.hpp`
- Modify: `src/growth/multi_normal_transition_generator.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Produces: `resolveMultiNormalLengths(const MultiNormalTopology &, std::vector<Scalar>, const MultiNormalOptions &) -> Result<ResolvedMultiNormalLengths, MultiNormalError>`.
- Uses: existing `CollisionIndex` and Triangle contact policy to implement BLMesh bottom/top surface checks.
- Reports: bad face IDs, bad topology vertex IDs, iteration count, and zero-retry use.

- [ ] **Step 1: Write a reproducible failing collision test**

Construct two non-adjacent Triangle patches whose displaced fronts properly intersect at the initial height. Assert the current pipeline accepts the collision; this is the required RED reproduction.

- [ ] **Step 2: Add BLMesh iteration parity fixture**

Record BLMesh length vectors, bad faces, and bad points for every iteration of the fixture. Assert the resolver produces the same sequence and accepts at the same iteration.

- [ ] **Step 3: Port surface building and intersection checks**

Mechanically port `BuildLayerPoints`, `CheckSurfaceIntersection`, and `CheckOuterSurfaceIntersection`. Use `CollisionIndex` only as the acceleration backend; preserve BLMesh's candidate Triangle set, bottom/top inclusion, adjacency exceptions, and bad-point collection.

- [ ] **Step 4: Port shrink, zero, and retry**

Copy `ShrinkLengthField`, `ZeroLengthField`, and `ResolveLengthField`: 20 shrink iterations at factor `0.8`, smoothing after every change, one zero-length retry, then structured failure.

- [ ] **Step 5: Rebuild transition geometry only from accepted lengths**

Run the resolver after topology and affected-Quad triangulation but before final transition-cell construction. Use the accepted vector for both `transition_cells` and `transformed_front`.

- [ ] **Step 6: Verify GREEN**

Run resolver, transition-builder, quad, topology, and pipeline tests. Expected: the synthetic collision is rejected or resolved at the BLMesh-matching iteration and no regression fails.

- [ ] **Step 7: Commit**

Commit as `fix: resolve multi-normal transition intersections`.

---

### Task 8: Expand Debug Outputs for Candidate Diagnosis

**Files:**
- Modify: `include/boundary_mesh/growth/multi_normal_types.hpp`
- Modify: `src/growth/multi_normal_transition_generator.cpp`
- Modify: `tests/unit/growth/multi_normal_types_test.cpp`
- Create: `tests/integration/multi_normal_debug_output_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Keeps existing accepted `transition_filename` and `front_filename`.
- Adds opt-in candidate-iteration output with deterministic names `multi_normal_candidate_00.vtk`, etc.

- [ ] **Step 1: Write failing default-off and enabled tests**

Verify default options create no files. With candidate output enabled, verify one file per attempted resolver iteration plus accepted transition/front files.

- [ ] **Step 2: Implement minimal debug extension**

Write rejected candidate surfaces using the existing legacy VTK writer. Do not retain candidate meshes unless debug output is enabled.

- [ ] **Step 3: Verify GREEN**

Run debug-output and type tests. Expected: deterministic file set and no default output.

- [ ] **Step 4: Commit**

Commit as `feat: expose multi-normal collision diagnostics`.

---

### Task 9: Verify `2dot5_cf` Against BLMesh

**Files:**
- Create temporarily, then delete: `tests/manual/multi_normal_real_case.cpp`
- Modify temporarily, then restore: `CMakeLists.txt`
- Persist only if needed: `tests/integration/multi_normal_real_case_regression_test.cpp` with a small checked-in fixture, not the external CGNS file.

**Interfaces:**
- Exercises: `readCgnsSurface -> topology -> GrowthFront -> generateMultiNormalTransition` with regular-layer count zero.

- [ ] **Step 1: Produce BLMesh reference diagnostics**

Run the original BLMesh MNormal path on `C:\Users\zpern\Desktop\todo\jiuyuan-quailty\test_case\2dot5_cf\2dot5_cf.cgns` with the same thresholds and initial first-layer height source. Record split source IDs, splitter IDs, branch directions, accepted per-point lengths, retry counts, transition cell count, and transformed-front VTK.

- [ ] **Step 2: Run the new implementation**

Enable multi-normal, set regular-layer count to zero, enable accepted and candidate debug output, and run the same case.

- [ ] **Step 3: Compare parity and validity**

Require identical split/source mappings, splitter ordering, branch grouping, retry counts, and accepted length values within `1e-12`. Verify `omitted_quads=0`, every Tetra has positive nonzero signed volume, and `CollisionIndex` finds no illegal non-adjacent transformed-front or transition-boundary contact.

- [ ] **Step 4: Preserve a small regression**

Extract the smallest local patch reproducing every splitter/interaction pattern found in `2dot5`; add it as a normal integration fixture so CI does not depend on the external CGNS path.

- [ ] **Step 5: Remove the temporary harness**

Delete `tests/manual/multi_normal_real_case.cpp`, restore the temporary CMake block, and verify `git diff --check`.

- [ ] **Step 6: Commit**

Commit only the small regression fixture and test as `test: cover 2dot5 multi-normal intersections`.

---

### Task 10: Full Build, Regular-Layer Integration, and Branch Verification

**Files:**
- Modify only if failures require in-scope corrections from previous tasks.

**Interfaces:**
- Verifies the public explicit transition/regular/merge workflow.

- [ ] **Step 1: Run full no-CGNS Release tests**

```powershell
cmake -S . -B build -DBOUNDARY_MESH_ENABLE_CGNS_IO=OFF
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Expected: 100% pass.

- [ ] **Step 2: Run CGNS Release build and real case**

```powershell
cmake -S . -B build-cgns -DBOUNDARY_MESH_ENABLE_CGNS_IO=ON
cmake --build build-cgns --config Release
```

Expected: build exit code zero; run the Task 9 real-case command and confirm all parity/validity checks pass.

- [ ] **Step 3: Run regular-layer integration**

Generate at least one regular layer from the collision-resolved `transformed_front`, merge it with transition cells, and verify metadata counts, shared vertex mapping, exposed surface validity, and absence of illegal collision.

- [ ] **Step 4: Verify repository state**

```powershell
git diff --check
git status --short --branch
git log --oneline --decorate -12
```

Expected: no uncommitted source changes, no temporary harness, and all task commits present on `codex/multi-normal-topology-transition`.
