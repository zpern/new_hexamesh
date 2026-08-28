# Confirmed High-Edge Corner Constraint Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Delay high-edge selection until the next candidate layer proves which neighbor is truly higher, then constrain every face incident to the selected edge's non-contact corners.

**Architecture:** `TerminationPropagator` will schedule a face that accepts its final layer and resolve its high edge during the next layer, before that layer is committed. `TransitionLayerCoordinator` will independently validate final trial counts so invalid externally supplied states cannot reach transition templates.

**Tech Stack:** C++17, existing `Result`/`std::variant` error model, CMake, CTest, legacy VTK real-case CLI verification.

## Global Constraints

- Multi-normal remains disabled for the `2dot5_cf` regression command.
- The selected high edge is the first confirmed next-layer neighbor in source-face local-edge order.
- Triangle non-contact one-ring faces and Quad non-contact two-corner one-ring faces must not exceed the low face's trial-layer count.
- Both Triangle and Quad must finish with at most one high edge.
- Preserve `DirectStop`; new propagated limits use `NeighborConstraint`.
- Do not change growth geometry, collision evaluation, quality thresholds, transition templates, or `max_layer_diff`.
- Keep reserved growth capped at `requested_layers + 2`; confirmation uses the already existing `k + 1` candidate and never requests `k + 2` relative to the stopped face.
- If the continuing front becomes empty while pending faces remain, those faces have no possible next-layer edge neighbor; ending the loop is valid and requires no additional trial layer.

---

## File Structure

- Modify `tests/unit/growth/termination_propagator_test.cpp`: reproduce early edge-selection drift and verify next-layer confirmation for Quad and Triangle.
- Modify `src/growth/termination_propagator.cpp`: defer present stop cells, resolve pending faces from the next candidate front, and normalize pending IDs on every return path.
- Modify `include/boundary_mesh/transition/transition_layer_coordinator.hpp`: add a structured final corner-layer violation error.
- Modify `src/transition/transition_layer_coordinator.cpp`: enforce one high edge and validate non-contact vertex one-rings from final trial counts.
- Modify `tests/unit/transition/transition_layer_coordinator_test.cpp`: cover valid and invalid Triangle/Quad final states and remove the obsolete adjacent-double-high expectation.
- Verify, but do not permanently modify, `src/transition/reserved_layer_transition.cpp`: ordinary reserved generation must continue to consume the coordinator result before constructing templates.

### Task 1: Reproduce delayed high-edge selection in the growth unit test

**Files:**
- Modify: `tests/unit/growth/termination_propagator_test.cpp`
- Test: `tests/unit/growth/termination_propagator_test.cpp`

**Interfaces:**
- Consumes: `TerminationPropagator::filterSingleHighEdgeCandidates(...)`
- Produces: regression expectations for `pending_stop_cells`, `allowed_layer_count`, and filtered next-front source IDs.

- [ ] **Step 1: Replace the current same-layer capped-neighbor assertion with a two-call Quad regression**

Add an octahedral corner-fan helper at file scope (the same geometry is repeated
in the coordinator test because the test executables do not share helpers):

```cpp
SurfaceMesh makeDelayedSelectionMesh()
{
    SurfaceMesh mesh;
    mesh.vertices = {
        {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0},
        {0,0,1}, {1,0,1}, {1,1,1}, {0,1,1}};
    mesh.faces = {
        Quad{{0,3,2,1}}, Quad{{4,5,6,7}},
        Triangle{{0,1,5}}, Triangle{{0,5,4}},
        Quad{{1,2,6,5}}, Quad{{2,3,7,6}},
        Quad{{3,0,4,7}}};
    mesh.face_tags.resize(
        mesh.faces.size(), {SurfaceBoundaryKind::Wall, 1});
    return mesh;
}
```

Build its topology, patch, front, profiles, constraints, and propagator using
the same builders already used by the test. Quad `1` is the low face, face `5`
is its local-edge-2 neighbor, and Triangle `2` touches it only at non-contact
vertex `5`. In layer 5, make face `1` present at its final allowed layer and verify
it is only scheduled:

```cpp
const SurfaceMesh delayed_mesh = makeDelayedSelectionMesh();
const auto delayed_topology =
    SurfaceTopologyBuilder{}.build(delayed_mesh);
assert(delayed_topology.hasValue());
const auto delayed_patch = GrowthPatchBuilder{}.build(
    delayed_mesh, delayed_topology.value());
assert(delayed_patch.hasValue());
const auto delayed_front = GrowthFrontBuilder{}.buildInitial(
    delayed_mesh, delayed_patch.value());
assert(delayed_front.hasValue());
std::vector<SourceVertexGrowthProfile> delayed_source_profiles;
for (const PatchVertex &vertex : delayed_patch.value().vertices())
    delayed_source_profiles.push_back(
        {vertex.source_vertex_id, {0.1, 1.0, 10}});
const auto delayed_profiles = GrowthProfileBuilder{}.build(
    delayed_patch.value(), delayed_source_profiles);
assert(delayed_profiles.hasValue());
auto delayed_constraints = buildFaceLayerConstraints(
    delayed_patch.value(), delayed_front.value(),
    delayed_profiles.value());
assert(delayed_constraints.hasValue());
delayed_constraints.value().find(1)->allowed_layer_count = 5;
delayed_constraints.value().find(1)->limit_kind =
    FaceLayerLimitKind::NeighborConstraint;
const auto delayed_propagator = TerminationPropagator::build(
    delayed_patch.value(), delayed_topology.value());
assert(delayed_propagator.hasValue());

LayerStepResult layer5;
layer5.layer = 5;
layer5.next_front = delayed_front.value();
layer5.next_front.layer = 5;
layer5.previous_front_face_indices = {0,1,2,3,4,5,6};
layer5.previous_front_vertex_indices.resize(
    layer5.next_front.vertices.size());
for (std::size_t index = 0;
     index < layer5.previous_front_vertex_indices.size(); ++index)
    layer5.previous_front_vertex_indices[index] = index;

std::vector<SurfaceFaceId> delayed_pending;
const auto layer5_filtered =
    delayed_propagator.value().filterSingleHighEdgeCandidates(
        delayed_front.value(), layer5, delayed_constraints.value(), 1,
        delayed_pending);
assert(layer5_filtered.hasValue());
assert((delayed_pending == std::vector<SurfaceFaceId>{1}));
assert(delayed_constraints.value().find(2)->allowed_layer_count == 10);
assert(delayed_constraints.value().find(5)->allowed_layer_count == 10);
```

Construct layer 6 with edge neighbor `5` and corner-only face `2` attempting to
survive. Process pending face `1`; edge `2` is confirmed from neighbor `5`, then
corner-only face `2` is capped and filtered:

```cpp
LayerStepResult layer6;
layer6.layer = 6;
layer6.next_front = layer5_filtered.value().next_front;
layer6.next_front.layer = 6;
const SurfaceFace face2 = layer6.next_front.faces[2];
const SurfaceFace face5 = layer6.next_front.faces[5];
layer6.next_front.faces = {face2, face5};
layer6.next_front.source_face_ids = {2,5};
layer6.previous_front_face_indices = {2,5};
layer6.previous_front_vertex_indices.resize(
    layer6.next_front.vertices.size());
for (std::size_t index = 0;
     index < layer6.previous_front_vertex_indices.size(); ++index)
    layer6.previous_front_vertex_indices[index] = index;

const auto layer6_filtered =
    delayed_propagator.value().filterSingleHighEdgeCandidates(
        layer5_filtered.value().next_front, layer6,
        delayed_constraints.value(), 1, delayed_pending);
assert(layer6_filtered.hasValue());
assert(delayed_pending.empty());
assert(delayed_constraints.value().find(2)->allowed_layer_count == 5);
assert((layer6_filtered.value().next_front.source_face_ids ==
        std::vector<SurfaceFaceId>{5}));
```

- [ ] **Step 2: Add a Triangle delayed-selection regression**

Reuse the tetrahedral Triangle mesh later in the test. Set Triangle `0` to stop
after layer 5, call once with all faces present, and assert it is pending without
immediate corner limiting:

```cpp
triangle_constraints.value().find(0)->allowed_layer_count = 5;
triangle_constraints.value().find(0)->limit_kind =
    FaceLayerLimitKind::NeighborConstraint;
std::vector<SurfaceFaceId> triangle_pending;
const auto triangle_layer5 =
    triangle_propagator.value().filterSingleHighEdgeCandidates(
        triangle_front.value(), triangle_candidates,
        triangle_constraints.value(), 1, triangle_pending);
assert(triangle_layer5.hasValue());
assert((triangle_pending == std::vector<SurfaceFaceId>{0}));
```

Construct layer 6 by removing only low face `0`. For Triangle `0` with vertex
order `{0,2,1}`, local edge 0 selects neighbor `3`; neighbors `1` and `2` both
touch the opposite vertex and must be filtered:

```cpp
LayerStepResult triangle_layer6;
triangle_layer6.layer = 6;
triangle_layer6.next_front = triangle_layer5.value().next_front;
triangle_layer6.next_front.layer = 6;
triangle_layer6.next_front.faces.erase(
    triangle_layer6.next_front.faces.begin());
triangle_layer6.next_front.source_face_ids.erase(
    triangle_layer6.next_front.source_face_ids.begin());
triangle_layer6.previous_front_face_indices = {1,2,3};
triangle_layer6.previous_front_vertex_indices.resize(
    triangle_layer6.next_front.vertices.size());
for (std::size_t index = 0;
     index < triangle_layer6.previous_front_vertex_indices.size(); ++index)
    triangle_layer6.previous_front_vertex_indices[index] = index;

const auto triangle_confirmed =
    triangle_propagator.value().filterSingleHighEdgeCandidates(
        triangle_layer5.value().next_front, triangle_layer6,
        triangle_constraints.value(), 1, triangle_pending);
assert(triangle_confirmed.hasValue());
assert(triangle_pending.empty());
assert(triangle_constraints.value().find(1)->allowed_layer_count == 5);
assert(triangle_constraints.value().find(2)->allowed_layer_count == 5);
assert((triangle_confirmed.value().next_front.source_face_ids ==
        std::vector<SurfaceFaceId>{3}));
```

- [ ] **Step 3: Run the focused test and verify RED**

Before running, add the no-high-neighbor boundary case. Schedule face `0` at
layer 5, then pass an empty layer-6 candidate and verify the call succeeds,
clears pending, and does not lower any unrelated face. This proves delayed
confirmation does not require an extra trial layer when the continuing front is
empty.

```cpp
LayerStepResult empty_layer6;
empty_layer6.layer = 6;
empty_layer6.next_front.layer = 6;
std::vector<SurfaceFaceId> no_high_pending{0};
const auto no_high = triangle_propagator.value()
    .filterSingleHighEdgeCandidates(
        triangle_layer5.value().next_front, empty_layer6,
        triangle_constraints.value(), 1, no_high_pending);
assert(no_high.hasValue());
assert(no_high_pending.empty());
```

Run:

```powershell
cmake --build .\build --config Release --target boundary_mesh_termination_propagator_test -j 1
ctest --test-dir .\build -C Release -R '^boundary_mesh_termination_propagator_test$' --output-on-failure
```

Expected: FAIL because the layer-5 call currently selects and constrains an edge immediately instead of returning pending face `1`/`0`.

- [ ] **Step 4: Commit the failing regression tests**

```powershell
git add -- tests/unit/growth/termination_propagator_test.cpp
git commit -m "test: reproduce high-edge selection drift"
```

### Task 2: Defer present stop cells and resolve confirmed next-layer edges

**Files:**
- Modify: `src/growth/termination_propagator.cpp:402-559`
- Test: `tests/unit/growth/termination_propagator_test.cpp`

**Interfaces:**
- Consumes: existing `pending_stop_cells` reference and `NeighborEntry::edge_rules` local-edge order.
- Produces: the unchanged public signature `filterSingleHighEdgeCandidates(...)`, with pending IDs now meaning “accepted final layer; resolve against the next candidate layer.”

- [ ] **Step 1: Simplify `StopCell` to confirmed non-present faces**

Replace the local type with:

```cpp
struct StopCell
{
    SurfaceFaceId source_face_id{};
    std::uint32_t completed_layer{};
};
```

Convert pending IDs at function entry using:

```cpp
for (const SurfaceFaceId pending_id : pending_stop_cells)
    stop_cells.push_back({pending_id, step.layer - 1});
pending_stop_cells.clear();
```

- [ ] **Step 2: Schedule present final-layer faces instead of selecting immediately**

Replace the current `candidate_present` stop-cell insertion with:

```cpp
if (!candidate_present)
{
    stop_cells.push_back({source_face_id, step.layer - 1});
}
else if (constraint->allowed_layer_count == step.layer &&
         constraint->limit_kind != FaceLayerLimitKind::Requested)
{
    pending_stop_cells.push_back(source_face_id);
}
```

Keep the existing duplicate check, but compare only `source_face_id`.

- [ ] **Step 3: Select only from the confirmed next candidate front**

Remove the `candidate_present`/allowed-limit branch from edge selection. A rule qualifies only when its neighbor is in `output.next_front.source_face_ids`:

```cpp
const NeighborEntry::EdgeRule *selected = nullptr;
for (const NeighborEntry::EdgeRule &rule : entry->edge_rules)
{
    if (std::find(
            output.next_front.source_face_ids.begin(),
            output.next_front.source_face_ids.end(),
            rule.neighbor) != output.next_front.source_face_ids.end())
    {
        selected = &rule;
        break;
    }
}
if (selected == nullptr)
    continue;
```

Keep the existing lowering loop over `selected->non_contact_corner_faces` and the subsequent `propagate()` call.

- [ ] **Step 4: Normalize pending IDs before every successful return**

Add a local lambda near function entry:

```cpp
const auto normalizePending = [&pending_stop_cells]()
{
    std::sort(pending_stop_cells.begin(), pending_stop_cells.end());
    pending_stop_cells.erase(
        std::unique(
            pending_stop_cells.begin(), pending_stop_cells.end()),
        pending_stop_cells.end());
};
```

Before the early `if (!constrained)` return, call `normalizePending()`. After adding propagation-created pending IDs, call it again instead of duplicating the sort/unique block.

- [ ] **Step 5: Run the focused growth test and verify GREEN**

Run:

```powershell
cmake --build .\build --config Release --target boundary_mesh_termination_propagator_test -j 1
ctest --test-dir .\build -C Release -R '^boundary_mesh_termination_propagator_test$' --output-on-failure
```

Expected: PASS.

- [ ] **Step 6: Run neighboring growth tests**

Run:

```powershell
ctest --test-dir .\build -C Release -R 'boundary_mesh_(termination_propagator|layer_coordination_pipeline|regular_layer_generator)_test' --output-on-failure
```

Expected: all selected tests PASS.

- [ ] **Step 7: Commit the growth fix**

```powershell
git add -- src/growth/termination_propagator.cpp tests/unit/growth/termination_propagator_test.cpp
git commit -m "fix: confirm high edges from next-layer candidates"
```

### Task 3: Add final corner-layer invariant validation

**Files:**
- Modify: `include/boundary_mesh/transition/transition_layer_coordinator.hpp`
- Modify: `src/transition/transition_layer_coordinator.cpp`
- Modify: `tests/unit/transition/transition_layer_coordinator_test.cpp`
- Test: `tests/unit/transition/transition_layer_coordinator_test.cpp`

**Interfaces:**
- Produces: `TransitionCornerLayerViolation` in `TransitionCoordinationError`.
- Consumes: `SurfaceTopology::faceEdges()`, `edges()`, and `vertexFaces()` plus final trial-layer input.

- [ ] **Step 1: Write failing coordinator tests**

Add this closed octahedron helper. Face `0` is the low face; face `4` is its
edge-1 neighbor; face `2` touches face `0` only at vertex `0`:

```cpp
SurfaceMesh makeCornerFanMesh()
{
    SurfaceMesh mesh;
    mesh.vertices = {
        {0,0,1}, {0,0,-1}, {1,0,0}, {0,1,0},
        {-1,0,0}, {0,-1,0}};
    mesh.faces = {
        Triangle{{0,2,3}}, Triangle{{0,3,4}},
        Triangle{{0,4,5}}, Triangle{{0,5,2}},
        Triangle{{1,3,2}}, Triangle{{1,4,3}},
        Triangle{{1,5,4}}, Triangle{{1,2,5}}};
    mesh.face_tags.resize(
        mesh.faces.size(), {SurfaceBoundaryKind::Wall, 1});
    return mesh;
}
```

Build its topology and patch, then add an error assertion where face `0` has
one high edge through face `4`, while corner-only face `2` also exceeds it:

```cpp
const SurfaceMesh corner_fan = makeCornerFanMesh();
const auto corner_topology =
    SurfaceTopologyBuilder{}.build(corner_fan);
assert(corner_topology.hasValue());
const auto corner_patch = GrowthPatchBuilder{}.build(
    corner_fan, corner_topology.value());
assert(corner_patch.hasValue());
const auto corner_violation = coordinator.coordinate(
    corner_patch.value(), corner_topology.value(),
    {{0,2}, {1,2}, {2,3}, {3,2},
     {4,3}, {5,2}, {6,2}, {7,2}});
assert(!corner_violation.hasValue());
assert(std::holds_alternative<TransitionCornerLayerViolation>(
    corner_violation.error()));
```

Update the adjacent-two-high test: two final high edges are no longer accepted for a Quad.

```cpp
assert(!adjacent_two_high.hasValue());
assert(std::holds_alternative<MultipleTransitionHighEdges>(
    adjacent_two_high.error()));
```

Add a valid version by reducing corner-only face `2` to 2 trial layers:

```cpp
const auto valid_corner_fan = coordinator.coordinate(
    corner_patch.value(), corner_topology.value(),
    {{0,2}, {1,2}, {2,2}, {3,2},
     {4,3}, {5,2}, {6,2}, {7,2}});
assert(valid_corner_fan.hasValue());
```

- [ ] **Step 2: Run the coordinator test and verify RED**

Run:

```powershell
cmake --build .\build --config Release --target boundary_mesh_transition_layer_coordinator_test -j 1
ctest --test-dir .\build -C Release -R '^boundary_mesh_transition_layer_coordinator_test$' --output-on-failure
```

Expected: compile failure because `TransitionCornerLayerViolation` is not defined, followed by behavioral failure until validation is implemented.

- [ ] **Step 3: Add the structured error type**

In `transition_layer_coordinator.hpp`, add:

```cpp
struct TransitionCornerLayerViolation
{
    SurfaceFaceId low_source_face_id{};
    std::size_t high_edge_local_index{};
    VertexId non_contact_vertex_id{};
    SurfaceFaceId violating_source_face_id{};
};
```

Append it to `TransitionCoordinationError`:

```cpp
using TransitionCoordinationError = std::variant<
    MissingTransitionFaceState,
    DuplicateTransitionFaceState,
    UncoordinatedTransitionLayerDifference,
    MultipleTransitionHighEdges,
    TransitionCornerLayerViolation>;
```

- [ ] **Step 4: Reject more than one final high edge for every face type**

Replace the Quad-specific allowance with:

```cpp
if (high_count > 1)
    return CoordinationResult::failure(
        MultipleTransitionHighEdges{
            face.layers.source_face_id,
            high_edges});
```

Remove the adjacent-two-edge special case.

- [ ] **Step 5: Validate non-contact vertex one-rings**

After identifying the sole high edge, derive its endpoint IDs from the face's local edge ID:

```cpp
const auto local_edges = faceEdgeIds(
    topology.faceEdges()[face_index]);
const Edge &selected_edge = topology.edges()[
    static_cast<std::size_t>(local_edges[high_edges[0]])];
```

Collect unique endpoints of every other local edge that are not endpoints of `selected_edge`. For each resulting non-contact vertex, scan `topology.vertexFaces()[vertex]`. Skip the low face and selected high neighbor. If an incident patch face has a greater `trial_layers` count than the low face, return:

```cpp
return CoordinationResult::failure(
    TransitionCornerLayerViolation{
        face.layers.source_face_id,
        high_edges[0],
        vertex,
        incident});
```

Use `findFace(faces, incident)` so non-patch faces are ignored deterministically.

- [ ] **Step 6: Run the coordinator test and verify GREEN**

Run:

```powershell
cmake --build .\build --config Release --target boundary_mesh_transition_layer_coordinator_test -j 1
ctest --test-dir .\build -C Release -R '^boundary_mesh_transition_layer_coordinator_test$' --output-on-failure
```

Expected: PASS.

- [ ] **Step 7: Commit final validation**

```powershell
git add -- include/boundary_mesh/transition/transition_layer_coordinator.hpp src/transition/transition_layer_coordinator.cpp tests/unit/transition/transition_layer_coordinator_test.cpp
git commit -m "fix: validate high-edge corner layer limits"
```

### Task 4: Verify integration behavior and the real case

**Files:**
- Verify: `src/transition/reserved_layer_transition.cpp`
- Verify: `tests/integration/reserved_layer_transition_pipeline_test.cpp`
- Verify: `tests/integration/layer_coordination_pipeline_test.cpp`
- Generated only: `build/real_case/2dot5_confirmed_high_edge/*`

**Interfaces:**
- Consumes: fixed growth propagation and final coordinator validation.
- Produces: verified ordinary reserved-layer output with no non-contact corner violation.

- [ ] **Step 1: Build and run transition/growth integration tests**

Run:

```powershell
cmake --build .\build --config Release -j 1
ctest --test-dir .\build -C Release -R 'boundary_mesh_(termination_propagator|transition_layer_coordinator|reserved_layer_transition_pipeline|layer_coordination_pipeline)_test' --output-on-failure
```

Expected: all selected tests PASS.

- [ ] **Step 2: Run the full test suite**

Run:

```powershell
ctest --test-dir .\build -C Release --output-on-failure
```

Expected: 100% tests passed.

- [ ] **Step 3: Regenerate `2dot5_cf` without multi-normal**

Run:

```powershell
.\build\Release\boundary_mesh_cli.exe `
  --input 'C:\Users\zpern\Desktop\todo\jiuyuan-quailty\test_case\2dot5_cf\2dot5_cf.cgns' `
  --first-height 0.1 `
  --growth-ratio 1.2 `
  --layer-count 20 `
  --maximum-skewness 1 `
  --isotropic-height 1.0 `
  --output-prefix '.\build\real_case\2dot5_confirmed_high_edge\2dot5_cf'
```

Expected: exit code 0, `transition_cells=0`, and three VTK outputs are created.

- [ ] **Step 4: Inspect source faces 19357 and 19358 in memory**

Use a temporary diagnostic at the successful pipeline boundary, without committing it, to print:

```text
source face 19357 trial layers
source face 19357 final high edge
all non-contact corner incident source-face trial layers
source face 19358 trial layers
```

Expected: if face `19357` retains a high edge, every non-contact-corner incident face has no more trial layers than face `19357`; specifically the previous `19357:18` / `19358:19` violation is absent.

- [ ] **Step 5: Remove the temporary diagnostic and rebuild the CLI**

Run:

```powershell
cmake --build .\build --config Release --target boundary_mesh_cli -j 1
git diff --check
```

Expected: build succeeds, no diagnostic output remains in source, and `git diff --check` reports no whitespace errors.

- [ ] **Step 6: Review only intended changes and commit verification adjustments**

Do not commit generated VTK files. If no integration-test source changes were required, make no extra commit. Otherwise:

```powershell
git add -- tests/integration/reserved_layer_transition_pipeline_test.cpp tests/integration/layer_coordination_pipeline_test.cpp
git commit -m "test: cover confirmed high-edge integration"
```
