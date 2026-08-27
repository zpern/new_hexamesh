# Internal/Non-Internal Surface Topology Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build independent internal and non-internal edge incidence and neighbor layers while retaining globally unique geometric edges.

**Architecture:** Keep `EdgeId`, `Edge`, `FaceEdgeIds`, and `vertexFaces()` global. Replace the two-face edge model with a public dual-layer `EdgeFaceIds`, use optional per-edge neighbors, and make the builder validate manifoldness/orientation per layer while enforcing closedness only for non-internal faces.

**Tech Stack:** C++17, CMake, CTest, existing `boundary_mesh::Result` and strong ID types.

## Global Constraints

- Reuse `SurfaceBoundaryKind::Internal`; every other boundary kind is non-internal.
- Preserve the order and semantics of existing input/face validation errors.
- Do not split `EdgeId`, `edges()`, `faceEdges()`, or `vertexFaces()`.
- Do not introduce a lossy legacy incidence view or invalid-ID sentinel.
- Preserve unrelated working-tree changes and stage only task-owned hunks.

---

### Task 1: Publish the layered topology contract

**Files:**
- Modify: `include/boundary_mesh/mesh/mesh_surface_topology.hpp`
- Modify: `include/boundary_mesh/mesh/mesh_surface_topology_error.hpp`
- Modify: `tests/unit/mesh/surface_topology_types_test.cpp`

**Interfaces:**
- Consumes: existing strong ID types and `SurfaceTopology` accessors.
- Produces: `OptionalSurfaceFaceId`, dual-layer `EdgeFaceIds`, optional triangle/quad neighbor arrays.

- [ ] **Step 1: Write the failing public-type test**

Add:

```cpp
static_assert(std::is_same_v<
    OptionalSurfaceFaceId,
    std::optional<SurfaceFaceId>>);
static_assert(std::is_same_v<
    TriangleNeighborIds,
    std::array<OptionalSurfaceFaceId, 3>>);
static_assert(std::is_same_v<
    QuadNeighborIds,
    std::array<OptionalSurfaceFaceId, 4>>);

const EdgeFaceIds incidence{
    {SurfaceFaceId{1}, SurfaceFaceId{2}},
    {SurfaceFaceId{3}, std::nullopt}};
if (incidence.non_internal_faces[1] != SurfaceFaceId{2} ||
    incidence.internal_faces[0] != SurfaceFaceId{3} ||
    incidence.internal_faces[1].has_value())
    return 1;
```

- [ ] **Step 2: Verify the old API fails**

Run: `cmake --build build --config Debug --target boundary_mesh_surface_topology_types_test`

Expected: compilation fails because the optional and named incidence types do not exist.

- [ ] **Step 3: Implement the public types and comments**

Add `<optional>` and define:

```cpp
using OptionalSurfaceFaceId = std::optional<SurfaceFaceId>;

struct EdgeFaceIds
{
    std::array<OptionalSurfaceFaceId, 2> non_internal_faces{};
    std::array<OptionalSurfaceFaceId, 2> internal_faces{};

    bool operator==(const EdgeFaceIds &other) const noexcept
    {
        return non_internal_faces == other.non_internal_faces &&
               internal_faces == other.internal_faces;
    }
};

using TriangleNeighborIds =
    std::array<OptionalSurfaceFaceId, 3>;
using QuadNeighborIds =
    std::array<OptionalSurfaceFaceId, 4>;
```

Document `edgeFaces()` as dual-layer incidence, `faceNeighbors()` as optional same-layer neighbors, and `BoundaryEdge` as a non-internal opening.

- [ ] **Step 4: Verify the public contract passes**

Run: `cmake --build build --config Debug --target boundary_mesh_surface_topology_types_test`

Run: `ctest --test-dir build -C Debug -R "^boundary_mesh_surface_topology_types_test$" --output-on-failure`

Expected: 1/1 test passes.

- [ ] **Step 5: Commit the contract**

```powershell
git add -- include/boundary_mesh/mesh/mesh_surface_topology.hpp include/boundary_mesh/mesh/mesh_surface_topology_error.hpp tests/unit/mesh/surface_topology_types_test.cpp
git commit -m "refactor: expose layered surface edge incidence"
```

### Task 2: Build independent incidence layers

**Files:**
- Modify: `src/mesh/surface_topology_builder.cpp`
- Modify: `tests/unit/mesh/surface_topology_builder_test.cpp`
- Modify: `tests/unit/mesh/surface_topology_edge_error_test.cpp`

**Interfaces:**
- Consumes: Task 1 types and `SurfaceBoundaryKind::Internal`.
- Produces: per-layer manifold/orientation checks, non-internal closedness, and same-layer neighbors.

- [ ] **Step 1: Add successful topology tests**

Add an `internalTag()` helper and fixtures asserting:

```text
single Internal triangle -> success, three open internal edges, nullopt neighbors
closed wall tetrahedron + one attached Internal face -> success, shared 2+1 incidence
two wall + two Internal faces on one edge -> success, complete 2+2 incidence
wall-wall-Internal neighbor -> wall faces mutual, Internal nullopt
vertexFaces() -> retains both topology classes
```

For each fixture assert exact stable face IDs and edge incidence arrays, not just success.

- [ ] **Step 2: Add layer-specific error tests**

Add exact variant and face-ID assertions for:

```text
three Internal faces on one edge        -> NonManifoldEdge
two same-direction Internal faces       -> InconsistentOrientation
Internal vs Non-Internal same direction -> no orientation error
one Non-Internal face on an edge        -> BoundaryEdge
three Non-Internal faces on one edge    -> NonManifoldEdge
```

Where necessary, place internal cases beside a closed non-internal shell so another deterministic error cannot occur first.

- [ ] **Step 3: Verify the new cases fail**

Run: `cmake --build build --config Debug --target boundary_mesh_surface_topology_builder_test boundary_mesh_surface_topology_edge_error_test`

Expected: compilation/assertion failure because the builder uses one incidence list and assumes two faces per edge.

- [ ] **Step 4: Add builder-only incidence types**

```cpp
struct IncidenceLayer
{
    std::vector<SurfaceFaceId> faces;
    std::vector<int> directions;
};

struct EdgeIncidence
{
    IncidenceLayer non_internal;
    IncidenceLayer internal;
};

bool isInternalFaceTag(const SurfaceBoundaryTag &tag) noexcept
{
    return tag.kind == SurfaceBoundaryKind::Internal;
}
```

Change `appendEdge()` and `appendFace()` to accept `bool is_internal` and `std::vector<EdgeIncidence>&`. Select one layer, reject only its third face, and compare directions only within it. New geometric edges receive one blank `EdgeIncidence`.

- [ ] **Step 5: Classify faces without moving validation**

In the second build scan compute and pass:

```cpp
const bool is_internal =
    isInternalFaceTag(mesh.face_tags[face_index]);
```

Keep the first-stage validation order unchanged and keep registering every face in `vertex_faces`.

- [ ] **Step 6: Check only non-internal closedness**

```cpp
const auto &faces = edge_incidence[edge_index].non_internal.faces;
if (faces.size() == 1)
    return BuildResult::failure(SurfaceTopologyError{
        BoundaryEdge{edges[edge_index].vertex_ids, faces[0]}});
```

Materialize both layers without truncation:

```cpp
EdgeFaceIds output;
for (std::size_t i = 0; i < incidence.non_internal.faces.size(); ++i)
    output.non_internal_faces[i] = incidence.non_internal.faces[i];
for (std::size_t i = 0; i < incidence.internal.faces.size(); ++i)
    output.internal_faces[i] = incidence.internal.faces[i];
```

- [ ] **Step 7: Generate same-layer optional neighbors**

Make `makeNeighbors()` accept `bool is_internal`, select the matching field, return the other populated face when two exist, and return `nullopt` for a lone internal face. Classify the current face from `mesh.face_tags` at the call site.

- [ ] **Step 8: Verify topology behavior**

Run: `ctest --test-dir build -C Debug -R "^boundary_mesh_surface_topology_(types|validation|builder|edge_error)_test$" --output-on-failure`

Expected: 4/4 tests pass.

- [ ] **Step 9: Commit the builder behavior**

Inspect the pre-existing blank-line diff first, then stage only intended hunks:

```powershell
git add -- src/mesh/surface_topology_builder.cpp tests/unit/mesh/surface_topology_builder_test.cpp tests/unit/mesh/surface_topology_edge_error_test.cpp
git commit -m "feat: split internal surface topology incidence"
```

### Task 3: Adapt optional-neighbor consumers

**Files:**
- Modify: `src/growth/termination_propagator.cpp`
- Modify: `src/transition/transition_layer_coordinator.cpp`
- Modify: `tests/unit/growth/termination_propagator_test.cpp`
- Modify: `tests/unit/transition/transition_layer_coordinator_test.cpp`

**Interfaces:**
- Consumes: optional `FaceNeighborIds` from Tasks 1-2.
- Produces: neighbor traversal that ignores missing neighbors while preserving local-edge positions.

- [ ] **Step 1: Add failing consumer regressions**

Build topologies through `SurfaceTopologyBuilder` and assert an internal open patch neither creates `SurfaceFaceId{0}` as a false neighbor nor breaks `TerminationPropagator::build` or `TransitionLayerCoordinator::coordinate`.

- [ ] **Step 2: Verify consumer tests fail**

Run: `cmake --build build --config Debug --target boundary_mesh_termination_propagator_test boundary_mesh_transition_layer_coordinator_test`

Expected: compilation fails at conversion from optional arrays, or the new false-neighbor assertions fail.

- [ ] **Step 3: Adapt `TerminationPropagator`**

Return `std::vector<OptionalSurfaceFaceId>` from its helper:

```cpp
return std::visit([](const auto &value) {
    return std::vector<OptionalSurfaceFaceId>{
        value.begin(), value.end()};
}, neighbors);
```

Skip `!neighbor.has_value()` before dereferencing in both loops. Retain vector length so `local_neighbors[local]` remains aligned with `local_edges[local]`.

- [ ] **Step 4: Adapt `TransitionLayerCoordinator`**

Return `std::vector<OptionalSurfaceFaceId>` from `neighborIds()`. Preserve its size as `local_edge_count`, then use:

```cpp
if (ids[local].has_value() &&
    findFace(faces, *ids[local]) != nullptr)
    face.neighbors.push_back(Neighbor{local, *ids[local]});
```

- [ ] **Step 5: Verify both consumers**

Run: `ctest --test-dir build -C Debug -R "^(boundary_mesh_termination_propagator_test|boundary_mesh_transition_layer_coordinator_test)$" --output-on-failure`

Expected: 2/2 tests pass.

- [ ] **Step 6: Commit the adaptation**

```powershell
git add -- src/growth/termination_propagator.cpp src/transition/transition_layer_coordinator.cpp tests/unit/growth/termination_propagator_test.cpp tests/unit/transition/transition_layer_coordinator_test.cpp
git commit -m "fix: handle open surface neighbors downstream"
```

### Task 4: Full verification and compatibility audit

**Files:**
- Modify only if the audit discovers a direct consumer missed above.

**Interfaces:**
- Consumes: complete implementation.
- Produces: verified build/test result and external compatibility report.

- [ ] **Step 1: Search for stale assumptions**

Run: `rg -n "EdgeFaceIds|edgeFaces\(|FaceNeighborIds|faceNeighbors\(" include src tests benchmarks`

Expected: all neighbor consumers handle optional IDs; no edge consumer assumes two total faces.

- [ ] **Step 2: Build everything**

Run: `cmake --build build --config Debug`

Expected: exit code 0.

- [ ] **Step 3: Run everything**

Run: `ctest --test-dir build -C Debug --output-on-failure`

Expected: all discovered tests pass.

- [ ] **Step 4: Inspect repository state**

Run: `git diff --check`

Run: `git status --short`

Run: `git log -4 --oneline`

Expected: no whitespace errors; unrelated pre-existing changes remain intact and unstaged.

- [ ] **Step 5: Report API compatibility**

Report `EdgeFaceIds` changing from one two-ID array to two optional arrays, and neighbor elements changing from `SurfaceFaceId` to `optional<SurfaceFaceId>`. Confirm that `vertexFaces()`, global `EdgeId`, and error variants remain compatible.
