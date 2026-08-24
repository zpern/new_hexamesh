# Multi-Normal Topology Transition Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Port BLMesh-compatible multi-normal vertex splitting and Triangle transition-volume generation into `new_boundaryMesh`, directly produce the post-transition mixed surface, and deliberately omit but diagnose Quad-based transition volumes.

**Architecture:** Implement the feature as a sequence of pure growth-module transformations: ordered mixed-face fans, split planning, topology rewriting/stitching, bottom-ID merging and split-only displacement, Triangle Tetra construction, and result integration. Quad surface topology remains active, but its transition-volume builder is an isolated no-op that records `OmittedQuadTransition` entries for later user implementation.

**Tech Stack:** C++17, Eigen vector types already exposed by `boundary_mesh/core/types.hpp`, the existing `Result<T, E>` error model, CMake/CTest, and native `VolumeMesh`/`GrowthFront` types.

## Global Constraints

- Create implementation branch `codex/multi-normal-topology-transition` in an isolated worktree before editing production code.
- Preserve BLMesh point-ID ordering for all Triangle decomposition and stitching tie breaks.
- Do not copy legacy `BLVector`, global caches, mutable singletons, or threshold macros.
- Only split vertex copies receive nonzero transition displacement; ordinary points remain fixed.
- Build `transformed_front` directly from rewritten surface connectivity and accepted moved coordinates; never extract it from transition-cell faces.
- Do not implement Quad transition-volume decomposition. Retain its new surface and record an omission diagnostic instead.
- Use TDD for every task: observe the focused test fail for the intended missing behavior before implementation.

---

## File Structure

Create focused growth components:

- `include/boundary_mesh/growth/multi_normal_types.hpp`: public configuration, split/source mappings, omission diagnostics, and transition result.
- `include/boundary_mesh/growth/multi_normal_error.hpp`: structured fan, topology, and Triangle-transition errors.
- `include/boundary_mesh/growth/incident_face_fan.hpp` and `src/growth/incident_face_fan.cpp`: ordered mixed Triangle/Quad fan construction.
- `include/boundary_mesh/growth/multi_normal_split_planner.hpp` and `src/growth/multi_normal_split_planner.cpp`: BLMesh-compatible complex-node selection, splitter enumeration, and branch directions.
- `include/boundary_mesh/growth/multi_normal_topology_builder.hpp` and `src/growth/multi_normal_topology_builder.cpp`: vertex copying, face remapping, and split-edge triangle-strip stitching.
- `include/boundary_mesh/growth/multi_normal_transition_builder.hpp` and `src/growth/multi_normal_transition_builder.cpp`: bottom-ID context, split-only displacement, Triangle Tetra generation, Quad omission, validation, and orchestration.
- `tests/unit/growth/incident_face_fan_test.cpp`: mixed fan ordering and error tests.
- `tests/unit/growth/multi_normal_split_planner_test.cpp`: split-plan selection tests.
- `tests/unit/growth/multi_normal_topology_builder_test.cpp`: surface rewriting and stitching tests.
- `tests/unit/growth/multi_normal_transition_builder_test.cpp`: BLMesh Triangle cases, Quad omission, and direct new-surface tests.
- `tests/integration/multi_normal_growth_pipeline_test.cpp`: result handoff to regular Triangle/Quad growth.

Modify:

- `include/boundary_mesh/growth/growth_front.hpp`: attach branch identity and generated-face origin without changing ordinary-front defaults.
- `include/boundary_mesh/growth/regular_layer_growth.hpp`: expose transition cells and omission records in the final result.
- `include/boundary_mesh/growth/regular_layer_generator.hpp` and `src/growth/regular_layer_generator.cpp`: invoke the transition before the ordinary layer loop when enabled.
- `CMakeLists.txt` and `tests/CMakeLists.txt`: register sources and focused tests.

---

### Task 1: Create the Isolated Implementation Branch and Public Types

**Files:**
- Create: `include/boundary_mesh/growth/multi_normal_types.hpp`
- Create: `include/boundary_mesh/growth/multi_normal_error.hpp`
- Modify: `include/boundary_mesh/growth/growth_front.hpp`
- Test: `tests/unit/growth/multi_normal_types_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Produces: `MultiNormalOptions`, `SplitVertexMapping`, `TransitionFaceOrigin`, `OmittedQuadTransition`, `MultiNormalTransitionResult`, `MultiNormalError`.
- Produces: optional `branch_id` and generated-face origin fields with defaults that preserve all existing aggregate construction.

- [ ] **Step 1: Create an isolated worktree and branch**

Run from the current repository:

```powershell
git worktree add ..\new_boundaryMesh-multi-normal -b codex/multi-normal-topology-transition
```

Expected: a new worktree on branch `codex/multi-normal-topology-transition`. Run every remaining implementation command in that worktree.

- [ ] **Step 2: Write the failing public-type test**

Add a test that constructs the intended result and verifies the Quad omission carries enough information for manual completion:

```cpp
MultiNormalTransitionResult result;
result.omitted_quad_transitions.push_back(OmittedQuadTransition{
    SurfaceFaceId{7},
    {VertexId{4}, VertexId{5}, VertexId{6}, VertexId{8}},
    {VertexId{1}, VertexId{2}, VertexId{3}, VertexId{4}},
    std::array<bool, 4>{true, false, true, false}});

if (result.omitted_quad_transitions[0].source_face_id != SurfaceFaceId{7} ||
    !result.omitted_quad_transitions[0].moved_corners[0]) return 1;
```

Also construct an ordinary `GrowthFrontVertex` and ordinary `GrowthFront` face to prove new fields default to unsplit/non-generated state.

- [ ] **Step 3: Register and run the test to verify RED**

Run:

```powershell
cmake --build build --config Release --target boundary_mesh_multi_normal_types_test
```

Expected: compile failure because the multi-normal types do not exist.

- [ ] **Step 4: Add the minimal types**

Use these public shapes:

```cpp
struct MultiNormalOptions
{
    bool enabled{false};
    Scalar transition_height{0};
    Scalar split_skewness_threshold{0.80};
    Scalar plane_skewness_threshold{-0.10};
    Scalar convex_skewness_threshold{0.20};
    std::size_t maximum_strategy_count{20};
};

struct SplitVertexMapping
{
    VertexId transformed_vertex_id{};
    VertexId source_vertex_id{};
    std::uint32_t branch_id{};
};

struct OmittedQuadTransition
{
    SurfaceFaceId source_face_id{};
    std::array<VertexId, 4> topology_vertex_ids{};
    std::array<VertexId, 4> bottom_vertex_ids{};
    std::array<bool, 4> moved_corners{};
};
```

`MultiNormalTransitionResult` owns `VolumeMesh transition_cells`, `GrowthFront transformed_front`, mappings, origins, and omission records. Define one `MultiNormalError` variant containing the exact structured failures consumed by later tasks: invalid input mapping, non-manifold fan, disconnected fan, inconsistent winding, invalid split topology, non-finite displacement, degenerate Triangle transition, and inverted Triangle transition.

- [ ] **Step 5: Run the focused test and baseline type tests**

Run:

```powershell
cmake --build build --config Release --target boundary_mesh_multi_normal_types_test boundary_mesh_growth_front_test
ctest --test-dir build -C Release -R "boundary_mesh_(multi_normal_types|growth_front)_test" --output-on-failure
```

Expected: both tests pass.

- [ ] **Step 6: Commit**

```powershell
git add include/boundary_mesh/growth/multi_normal_types.hpp include/boundary_mesh/growth/multi_normal_error.hpp include/boundary_mesh/growth/growth_front.hpp tests/unit/growth/multi_normal_types_test.cpp tests/CMakeLists.txt
git commit -m "feat: add multi-normal transition types"
```

---

### Task 2: Build Ordered Mixed Triangle/Quad Incident Fans

**Files:**
- Create: `include/boundary_mesh/growth/incident_face_fan.hpp`
- Create: `src/growth/incident_face_fan.cpp`
- Test: `tests/unit/growth/incident_face_fan_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `GrowthFront`, `FrontEvaluation`.
- Produces: `Result<std::vector<IncidentFaceFan>, MultiNormalError> buildIncidentFaceFans(...)`.

- [ ] **Step 1: Write the failing mixed-fan tests**

Define:

```cpp
struct IncidentFaceSector
{
    std::size_t face_index{};
    VertexId previous_vertex{};
    VertexId next_vertex{};
    Vector3 unit_normal{Vector3::Zero()};
};

struct IncidentFaceFan
{
    VertexId center_vertex{};
    bool closed{};
    std::vector<IncidentFaceSector> sectors;
};
```

Test a center shared by one Triangle and two Quads and assert sectors are linked by real perimeter edges only. Add separate tests for branched, disconnected, and oppositely wound fans.

- [ ] **Step 2: Run to verify RED**

Run the `boundary_mesh_incident_face_fan_test` target. Expected: compile failure because `buildIncidentFaceFans` is missing.

- [ ] **Step 3: Implement deterministic fan walking**

For each face corner, record its predecessor and successor. Link sectors when one sector's successor edge continues another sector's predecessor edge. Walk from the unique boundary start for open fans or from the sector with the smallest `(source_face_id, face_index)` for closed fans. Never create a Quad diagonal.

- [ ] **Step 4: Verify GREEN and regression**

Run the focused fan test plus `boundary_mesh_front_adjacency_test` and `boundary_mesh_front_evaluator_test`. Expected: pass.

- [ ] **Step 5: Commit**

```powershell
git add include/boundary_mesh/growth/incident_face_fan.hpp src/growth/incident_face_fan.cpp tests/unit/growth/incident_face_fan_test.cpp CMakeLists.txt tests/CMakeLists.txt
git commit -m "feat: order mixed-face incident fans"
```

---

### Task 3: Plan BLMesh-Compatible Multi-Normal Splits

**Files:**
- Create: `include/boundary_mesh/growth/multi_normal_split_planner.hpp`
- Create: `src/growth/multi_normal_split_planner.cpp`
- Test: `tests/unit/growth/multi_normal_split_planner_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: ordered fans, positions, `MultiNormalOptions`.
- Produces: `Result<std::vector<VertexSplitPlan>, MultiNormalError> planMultiNormalSplits(...)`.

- [ ] **Step 1: Write failing split-plan tests**

Cover a planar fan that remains unsplit, a convex 90-degree ridge that splits into two contiguous groups, and a three-region corner whose selected branch directions each maximize minimum visibility within their group. Assert plan results are unchanged by face-container permutation after fan ordering.

- [ ] **Step 2: Run to verify RED**

Expected: compile failure because `VertexSplitPlan` and planner are absent.

- [ ] **Step 3: Implement BLMesh geometry primitives**

Add internal functions equivalent to `getMinCos`, `getMostNormal`, and the three-normal circle-center candidate, using `Vector3`, finite checks, and normalized optional results. Candidate ranking is maximum minimum cosine, then deterministic source-face ID order.

- [ ] **Step 4: Implement splitter enumeration and branch grouping**

Port the `PLANE_SKEWNESS`, `CONVEX_SKEWNESS`, `MAX_PLAIN_RIDGE`, strategy-count, and one-percent improvement semantics into named option-backed code. Enumerate splitter subsets in deterministic edge-ID order, build contiguous face groups, calculate group normals, and accept only a quality-improving plan.

- [ ] **Step 5: Verify GREEN**

Run the focused planner tests and existing `boundary_mesh_growth_direction_test`. Expected: pass with no warning output.

- [ ] **Step 6: Commit**

```powershell
git add include/boundary_mesh/growth/multi_normal_split_planner.hpp src/growth/multi_normal_split_planner.cpp tests/unit/growth/multi_normal_split_planner_test.cpp CMakeLists.txt tests/CMakeLists.txt
git commit -m "feat: plan multi-normal vertex splits"
```

---

### Task 4: Rewrite the Surface and Stitch Split Edges

**Files:**
- Create: `include/boundary_mesh/growth/multi_normal_topology_builder.hpp`
- Create: `src/growth/multi_normal_topology_builder.cpp`
- Test: `tests/unit/growth/multi_normal_topology_builder_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: input front and split plans.
- Produces: `Result<MultiNormalTopology, MultiNormalError> buildMultiNormalTopology(...)`.

- [ ] **Step 1: Write failing one-ended split test**

Use a Triangle and Quad sharing edge `(A, B)`. Split `A` into two branches and assert the original faces retain their types with branch-specific vertex IDs while exactly one stitching `Triangle{A0, A1, B}` is added.

- [ ] **Step 2: Run to verify RED**

Expected: compile failure for the missing topology builder.

- [ ] **Step 3: Implement vertex copying and face remapping**

Create branch copies in source vertex ID then branch order. Retain ordinary vertices once. Remap every face corner from its plan's face-group membership. Preserve ordinary source face IDs and attach complete origins to generated stitching faces.

- [ ] **Step 4: Write the failing two-ended stitching test**

Construct unequal ordered branch chains at both endpoints. Assert the output triangle count is `left_count + right_count`, all triangles have consistent winding, and the interleaving matches BLMesh's accumulated `abs(dot(n0,n1)-1)` cost with point-ID tie breaks.

- [ ] **Step 5: Implement ordered-chain dynamic programming**

Port `OrderDirectedTriangleChain` and `FindSmoothestInterleaving` as private functions using `Vector3` normals and stable topology IDs. Reject closed, branched, or disconnected chains rather than indexing a fallback element.

- [ ] **Step 6: Verify GREEN and commit**

Run the topology test plus fan tests, then commit:

```powershell
git add include/boundary_mesh/growth/multi_normal_topology_builder.hpp src/growth/multi_normal_topology_builder.cpp tests/unit/growth/multi_normal_topology_builder_test.cpp CMakeLists.txt tests/CMakeLists.txt
git commit -m "feat: build and stitch multi-normal topology"
```

---

### Task 5: Build the Transition Context and Split-Only New Surface

**Files:**
- Create: `include/boundary_mesh/growth/multi_normal_transition_builder.hpp`
- Create: `src/growth/multi_normal_transition_builder.cpp`
- Test: `tests/unit/growth/multi_normal_transition_builder_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `MultiNormalTopology`, transition height.
- Produces internally: merged bottom IDs, bottom points, split-copy mask, displaced topology points.
- Produces publicly: `buildMultiNormalTransition(...)`.

- [ ] **Step 1: Write the failing bottom-merge/displacement test**

Create two branch copies sharing source position plus ordinary neighbors. Assert both copies map to one bottom ID, both move along separate directions, and every ordinary output position is bitwise unchanged.

- [ ] **Step 2: Run to verify RED**

Expected: missing transition builder failure.

- [ ] **Step 3: Implement `BuildPreGenContext` semantics**

Merge bottom IDs by source vertex identity rather than approximate coordinate hashing. Mark topology IDs that participate in a face with repeated bottom IDs. Assign nonzero height only to those IDs and build the transformed surface directly from `MultiNormalTopology::faces` and displaced positions.

- [ ] **Step 4: Verify direct-surface behavior**

Assert no cell-face enumeration helper is called or exposed and the transformed face list exactly equals topology output. Run the focused test; expected pass.

- [ ] **Step 5: Commit**

```powershell
git add include/boundary_mesh/growth/multi_normal_transition_builder.hpp src/growth/multi_normal_transition_builder.cpp tests/unit/growth/multi_normal_transition_builder_test.cpp CMakeLists.txt tests/CMakeLists.txt
git commit -m "feat: displace split-only transition surface"
```

---

### Task 6: Port `BuildCandidateVolume` Triangle Tetra Rules

**Files:**
- Modify: `src/growth/multi_normal_transition_builder.cpp`
- Test: `tests/unit/growth/multi_normal_transition_builder_test.cpp`

**Interfaces:**
- Consumes: ordered Triangle topology IDs, merged bottom IDs, bottom/upper global volume IDs, split-copy mask.
- Produces: zero or more oriented `Tetra` cells and `CellMetadata`.

- [ ] **Step 1: Write failing tests for every bottom-ID case**

Add fixtures for:

```text
[A,A,A] -> one Tetra
[A,A,B] with B fixed -> BLMesh ID-ordered branch
[A,A,B] with B moving -> BLMesh additional-Tetra branch
[A,B,C] with none moving -> no transition cell
[A,B,C] with one/two/three moving -> exact BLMesh ID-ordered Tetra sets
```

Repeat cases with permuted topology array positions but the same stable IDs and assert equivalent canonical Tetra sets.

- [ ] **Step 2: Run to verify RED**

Expected: missing cells or incorrect counts, not a fixture error.

- [ ] **Step 3: Implement the exact reference branches**

Translate `MNormalMesh::BuildCandidateVolume()` lines 1346-1497. Preserve its minimum-lower-ID selection, `reverse_order` comparison, repeated-bottom branch, third-moving-point extra Tetra, and distinct-four-coordinate guard. Convert each Tetra to positive winding by one deterministic swap only after confirming the reference connectivity.

- [ ] **Step 4: Verify GREEN**

Run focused transition tests. Expected: every canonical cell set and winding assertion passes.

- [ ] **Step 5: Commit**

```powershell
git add src/growth/multi_normal_transition_builder.cpp tests/unit/growth/multi_normal_transition_builder_test.cpp
git commit -m "feat: port triangle multi-normal transition cells"
```

---

### Task 7: Omit Quad Volumes but Preserve and Diagnose Their New Surface

**Files:**
- Modify: `src/growth/multi_normal_transition_builder.cpp`
- Test: `tests/unit/growth/multi_normal_transition_builder_test.cpp`

**Interfaces:**
- Produces: `OmittedQuadTransition` for each affected Quad and no corresponding volume cell.

- [ ] **Step 1: Write the failing affected-Quad test**

Use a Quad with one moved split corner. Assert:

```cpp
result.transition_cells.cells.empty();
result.omitted_quad_transitions.size() == 1;
result.transformed_front.faces contains the remapped Quad;
result.transformed_front.vertices[split].position != source_position;
```

Also assert an unchanged Quad creates neither a transition cell nor an omission record.

- [ ] **Step 2: Run to verify RED**

Expected: missing omission record or incorrect rollback of the new surface.

- [ ] **Step 3: Implement the reserved Quad branch**

For a Quad, calculate the moved-corner mask. If empty, return without a cell or diagnostic. Otherwise append source face ID, ordered topology IDs, merged bottom IDs, and mask to `omitted_quad_transitions`; do not triangulate, choose a diagonal, or reject the topology.

- [ ] **Step 4: Verify GREEN and commit**

```powershell
cmake --build build --config Release --target boundary_mesh_multi_normal_transition_builder_test
ctest --test-dir build -C Release -R boundary_mesh_multi_normal_transition_builder_test --output-on-failure
git add src/growth/multi_normal_transition_builder.cpp tests/unit/growth/multi_normal_transition_builder_test.cpp
git commit -m "feat: report omitted quad transition volumes"
```

---

### Task 8: Integrate the Transition Before Regular Growth

**Files:**
- Modify: `include/boundary_mesh/growth/regular_layer_growth.hpp`
- Modify: `include/boundary_mesh/growth/regular_layer_generator.hpp`
- Modify: `src/growth/regular_layer_generator.cpp`
- Test: `tests/integration/multi_normal_growth_pipeline_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `MultiNormalOptions` through `RegularLayerGrowthOptions`.
- Produces: transition cells, omission diagnostics, and regular cells grown from `transformed_front`.

- [ ] **Step 1: Write the failing integration test**

Create a mixed Triangle/Quad patch with one accepted complex split. Enable multi-normal transition and assert:

- transition Tetra cells precede regular-layer cells in the result;
- affected Quad omission diagnostics are retained;
- the transformed stitching Triangle enters regular growth;
- the remapped Quad remains a Quad and later generates a Hexa;
- branch copies map back to one source vertex without duplicate source-level growth records.

- [ ] **Step 2: Run to verify RED**

Expected: multi-normal options have no effect and transition outputs are absent.

- [ ] **Step 3: Integrate transactionally**

In `RegularLayerGenerator::generate`, build profiles and constraints from original source entities, run the transition once before the ordinary loop, seed the volume mesh with its bottom/upper vertices and Triangle transition cells, then start `current_front` from `transformed_front`. Deduplicate `LayerVertexRecord` and `VertexGrowthRecord` by source vertex ID while retaining all branch global IDs.

For generated stitching faces, use their complete origin records for reporting and derive continuation limits as the minimum allowed layer count of contributing source faces. Do not index the original `SurfaceMesh::face_tags` with a synthetic face ID.

- [ ] **Step 4: Verify focused integration GREEN**

Run the new pipeline test, `boundary_mesh_regular_layer_growth_pipeline_test`, and `boundary_mesh_regular_layer_growth_failure_test`. Expected: pass.

- [ ] **Step 5: Commit**

```powershell
git add include/boundary_mesh/growth/regular_layer_growth.hpp include/boundary_mesh/growth/regular_layer_generator.hpp src/growth/regular_layer_generator.cpp tests/integration/multi_normal_growth_pipeline_test.cpp tests/CMakeLists.txt
git commit -m "feat: integrate multi-normal transition growth"
```

---

### Task 9: Full Verification and Branch Handoff

**Files:**
- Modify only if verification exposes a tested defect in the feature branch.

**Interfaces:**
- Produces: verified branch `codex/multi-normal-topology-transition` ready for user review, without merging to `master`.

- [ ] **Step 1: Configure and build from scratch**

Run:

```powershell
cmake -S . -B build-multi-normal -DBUILD_TESTING=ON
cmake --build build-multi-normal --config Release
```

Expected: successful Release build with no new compiler warnings.

- [ ] **Step 2: Run the complete test suite**

```powershell
ctest --test-dir build-multi-normal -C Release --output-on-failure
```

Expected: 100% tests passed.

- [ ] **Step 3: Verify repository and branch state**

```powershell
git status --short
git branch --show-current
git log --oneline --decorate -10
```

Expected: clean worktree, branch `codex/multi-normal-topology-transition`, and the task commits above. Do not merge the branch.

- [ ] **Step 4: Hand off the reserved Quad boundary**

Report the exact function implementing the Quad no-op, the `OmittedQuadTransition` fields, the focused test target, and the count/details of omitted Quad regions in any exercised fixture so the user can add their decomposition without tracing the rest of the pipeline.
