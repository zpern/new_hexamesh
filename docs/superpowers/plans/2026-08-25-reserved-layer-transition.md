# Reserved-Layer Transition Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Generate a conforming mixed Prism/Hexa/Pyramid/Tetra boundary-layer volume and a directly authored triangular top surface by reserving the outer two trial layers.

**Architecture:** Keep `RegularLayerGenerator` responsible for trial growth and its existing quality/collision transaction. Add a focused `transition` module that augments requested profiles by two layers, coordinates final per-face trial counts, filters ordinary cells from the trial mesh, applies source-local transition templates, and directly emits the boundary-layer top. The transition builder consumes existing `RegularLayerGrowthResult`, `GrowthPatch`, and `SurfaceTopology` data without changing the mesh primitive types.

**Tech Stack:** C++17, CMake 3.20+, existing `boundary_mesh` `Result`, `SurfaceMesh`, `VolumeMesh`, Growth and Surface libraries, CTest.

## Global Constraints

- Trial growth attempts exactly two layers beyond each requested vertex layer count, subject to existing quality, collision, isotropic-stop, and propagated-stop rules.
- `regular_layers = max(0, trial_layers - 2)` and `occupied_layers = max(0, trial_layers - 1)`.
- Edge-neighbor occupied-layer difference is at most one, and every low source face has at most one high edge before template construction.
- Every penultimate Quad Hexa is decomposed; every penultimate Triangle Prism is retained unchanged.
- The outermost trial Prism/Hexa is geometry-only and is never committed unchanged.
- A zero/one trial-layer pair creates no geometric step or transition volume.
- Pyramid/Tetra quality thresholds and transition-cell intersection checks remain excluded.
- Per-source top Triangles are written directly; do not recover them by scanning volume-cell boundary faces.
- Preserve deterministic source-face order, vertex-ID tie breaks, and outward face orientation.

---

## File Structure

- `include/boundary_mesh/transition/reserved_layer_growth.hpp`: profile augmentation and layer-count formulas.
- `src/transition/reserved_layer_growth.cpp`: overflow-safe `N + 2` trial-profile construction.
- `include/boundary_mesh/transition/transition_layer_coordinator.hpp`: coordinated face state and high-edge representation.
- `src/transition/transition_layer_coordinator.cpp`: occupied-height propagation and single-high-edge enforcement.
- `include/boundary_mesh/transition/quad_diagonal.hpp`: deterministic Quad diagonal result.
- `src/transition/quad_diagonal.cpp`: minimax Triangle-skewness selection.
- `include/boundary_mesh/transition/transition_templates.hpp`: source-local template inputs and results.
- `src/transition/triangle_transition_template.cpp`: Triangle keep/delete/side-Pyramid cases.
- `src/transition/quad_transition_template.cpp`: penultimate-Hexa and side Pyramid/Tetra cases.
- `include/boundary_mesh/transition/reserved_layer_transition.hpp`: public pipeline and error model.
- `src/transition/reserved_layer_transition.cpp`: atomic assembly, ordinary-cell filtering, and top-surface collection.
- `tests/unit/transition/*.cpp`: focused count, coordination, diagonal, and template tests.
- `tests/integration/reserved_layer_transition_pipeline_test.cpp`: mixed source end-to-end behavior.
- `CMakeLists.txt`, `tests/CMakeLists.txt`: Transition target and test registration.

---

### Task 1: Trial-profile augmentation and layer-state formulas

**Files:**
- Create: `include/boundary_mesh/transition/reserved_layer_growth.hpp`
- Create: `src/transition/reserved_layer_growth.cpp`
- Create: `tests/unit/transition/reserved_layer_growth_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `SourceVertexGrowthProfile` from `growth/growth_profile.hpp`.
- Produces: `FaceLayerState`, `regularLayerCount`, `occupiedLayerCount`, and `makeReservedTrialProfiles` used by all later tasks.

- [ ] **Step 1: Write the failing unit test**

```cpp
#include <cassert>
#include <limits>
#include <vector>

#include <boundary_mesh/transition/reserved_layer_growth.hpp>

using namespace boundary_mesh;

int main()
{
    assert(regularLayerCount(0) == 0);
    assert(regularLayerCount(1) == 0);
    assert(regularLayerCount(7) == 5);
    assert(occupiedLayerCount(0) == 0);
    assert(occupiedLayerCount(1) == 0);
    assert(occupiedLayerCount(7) == 6);

    const std::vector<SourceVertexGrowthProfile> requested{
        {VertexId{4}, VertexGrowthProfile{0.1, 1.2, 20}}};
    const auto trial = makeReservedTrialProfiles(requested);
    assert(trial.hasValue());
    assert(trial.value()[0].profile.layer_count == 22);

    const std::vector<SourceVertexGrowthProfile> overflow{
        {VertexId{9}, VertexGrowthProfile{
             0.1, 1.0, std::numeric_limits<std::uint32_t>::max()}}};
    assert(!makeReservedTrialProfiles(overflow).hasValue());
}
```

- [ ] **Step 2: Register and run the failing test**

Add `src/transition/reserved_layer_growth.cpp` to a new `boundary_mesh_transition` static library linked publicly to `BoundaryMesh::BoundaryLayer`, `BoundaryMesh::Surface`, and `BoundaryMesh::Core`. Add alias `BoundaryMesh::Transition`. Register `boundary_mesh_reserved_layer_growth_test` linked to it.

Run:

```powershell
cmake --build build --target boundary_mesh_reserved_layer_growth_test
```

Expected: FAIL because the new header/functions do not exist.

- [ ] **Step 3: Implement the public count API**

```cpp
namespace boundary_mesh
{
    inline constexpr std::uint32_t ReservedTransitionLayerCount{2};

    struct FaceLayerState
    {
        SurfaceFaceId source_face_id{};
        std::uint32_t trial_layers{};
        std::uint32_t occupied_layers{};
        std::uint32_t regular_layers{};
    };

    struct ReservedLayerCountOverflow
    {
        VertexId source_vertex_id{};
        std::uint32_t requested_layers{};
    };

    std::uint32_t regularLayerCount(std::uint32_t trial_layers) noexcept;
    std::uint32_t occupiedLayerCount(std::uint32_t trial_layers) noexcept;

    Result<std::vector<SourceVertexGrowthProfile>, ReservedLayerCountOverflow>
    makeReservedTrialProfiles(
        const std::vector<SourceVertexGrowthProfile> &requested);
}
```

Implement the formulas with guarded unsigned subtraction. Reject a profile when `layer_count > UINT32_MAX - 2`; otherwise copy the profile and add exactly two.

- [ ] **Step 4: Run the unit test**

```powershell
cmake --build build --target boundary_mesh_reserved_layer_growth_test
ctest --test-dir build -R boundary_mesh_reserved_layer_growth_test --output-on-failure
```

Expected: build succeeds and one test passes.

- [ ] **Step 5: Commit**

```powershell
git add CMakeLists.txt tests/CMakeLists.txt include/boundary_mesh/transition/reserved_layer_growth.hpp src/transition/reserved_layer_growth.cpp tests/unit/transition/reserved_layer_growth_test.cpp
git commit -m "feat: add reserved trial layer counts"
```

---

### Task 2: Occupied-height and single-high-edge coordination

**Files:**
- Create: `include/boundary_mesh/transition/transition_layer_coordinator.hpp`
- Create: `src/transition/transition_layer_coordinator.cpp`
- Create: `tests/unit/transition/transition_layer_coordinator_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: sorted source-face trial counts, `GrowthPatch`, and `SurfaceTopology`.
- Produces: `CoordinatedTransitionFace` records with zero or one `high_edge_local_index`.

- [ ] **Step 1: Write tests for the required coordination cases**

Create a two-face shared-edge fixture and a three-face fan fixture. Assert:

```cpp
const auto zero_one = coordinator.coordinate(
    patch, topology, {{face0, 0}, {face1, 1}});
assert(zero_one.hasValue());
assert(zero_one.value()[0].occupied_layers == 0);
assert(!zero_one.value()[0].high_edge_local_index.has_value());

const auto one_two = coordinator.coordinate(
    patch, topology, {{face0, 1}, {face1, 2}});
assert(one_two.hasValue());
assert(one_two.value()[0].occupied_layers == 0);
assert(one_two.value()[0].high_edge_local_index.has_value());

const auto two_high_edges = coordinator.coordinate(
    fan_patch, fan_topology,
    {{low, 2}, {high0, 3}, {high1, 3}});
assert(two_high_edges.hasValue());
assert(countHighEdges(two_high_edges.value(), low) <= 1);
```

Also test missing face records and an edge-neighbor occupied difference greater than one as explicit failures.

- [ ] **Step 2: Run the test to verify it fails**

```powershell
cmake --build build --target boundary_mesh_transition_layer_coordinator_test
```

Expected: FAIL because `TransitionLayerCoordinator` is undefined.

- [ ] **Step 3: Implement coordinator types and deterministic propagation**

```cpp
struct CoordinatedTransitionFace
{
    FaceLayerState layers;
    std::optional<std::size_t> high_edge_local_index;
};

struct MissingTransitionFaceState { SurfaceFaceId source_face_id{}; };
struct UncoordinatedLayerDifference
{
    SurfaceFaceId first{};
    SurfaceFaceId second{};
};

using TransitionCoordinationError = std::variant<
    MissingTransitionFaceState,
    UncoordinatedLayerDifference>;

class TransitionLayerCoordinator
{
public:
    Result<std::vector<CoordinatedTransitionFace>,
           TransitionCoordinationError>
    coordinate(
        const GrowthPatch &patch,
        const SurfaceTopology &topology,
        const std::vector<std::pair<SurfaceFaceId, std::uint32_t>>
            &trial_layers) const;
};
```

Build a source-edge adjacency table in source-face order. Use a queue ordered by `SurfaceFaceId`. First lower the higher trial count until occupied differences are at most one. Then, when a low face has multiple high edges, lower all but the lowest-source-ID high neighbor by one trial layer and enqueue affected neighbors. Recompute `regular_layers` and `occupied_layers` after every reduction. Never increase a count.

- [ ] **Step 4: Run coordination tests**

```powershell
cmake --build build --target boundary_mesh_transition_layer_coordinator_test
ctest --test-dir build -R boundary_mesh_transition_layer_coordinator_test --output-on-failure
```

Expected: all coordination cases pass deterministically, including reversed input-record order.

- [ ] **Step 5: Commit**

```powershell
git add CMakeLists.txt tests/CMakeLists.txt include/boundary_mesh/transition/transition_layer_coordinator.hpp src/transition/transition_layer_coordinator.cpp tests/unit/transition/transition_layer_coordinator_test.cpp
git commit -m "feat: coordinate reserved transition heights"
```

---

### Task 3: Deterministic Quad diagonal selection

**Files:**
- Create: `include/boundary_mesh/transition/quad_diagonal.hpp`
- Create: `src/transition/quad_diagonal.cpp`
- Create: `tests/unit/transition/quad_diagonal_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: four oriented Vertex IDs, their points, and a length tolerance.
- Produces: `QuadDiagonal::ZeroTwo` or `QuadDiagonal::OneThree` plus the two oriented Triangles.

- [ ] **Step 1: Write the failing minimax and tie-break tests**

```cpp
const OrientedQuad quad{
    {VertexId{10}, VertexId{11}, VertexId{12}, VertexId{13}},
    {Point3{0,0,0}, Point3{2,0,0}, Point3{1.8,1,0}, Point3{0,1,0}}};
const auto chosen = chooseQuadDiagonal(quad, 1e-12);
assert(chosen.hasValue());
assert(chosen.value().worst_skewness <= 1.0);

const OrientedQuad square{
    {VertexId{4}, VertexId{1}, VertexId{3}, VertexId{2}},
    {Point3{0,0,0}, Point3{1,0,0}, Point3{1,1,0}, Point3{0,1,0}}};
const auto tie = chooseQuadDiagonal(square, 1e-12);
assert(tie.hasValue());
assert(tie.value().diagonal == QuadDiagonal::OneThree);
```

The square expects diagonal IDs `(1,2)`, the lexicographically smaller endpoint pair.

- [ ] **Step 2: Run the test to verify it fails**

```powershell
cmake --build build --target boundary_mesh_quad_diagonal_test
```

Expected: FAIL because `chooseQuadDiagonal` does not exist.

- [ ] **Step 3: Implement the selector using existing Triangle skewness**

```cpp
enum class QuadDiagonal { ZeroTwo, OneThree };

struct QuadDiagonalSelection
{
    QuadDiagonal diagonal{};
    std::array<Triangle, 2> triangles{};
    Scalar worst_skewness{};
};

Result<QuadDiagonalSelection, FaceEvaluationError>
chooseQuadDiagonal(const OrientedQuad &quad, Scalar length_tolerance);
```

Evaluate both Triangle pairs with `triangleEquiangularSkewness`. Minimize the maximum pair score; for equal scores compare sorted diagonal endpoint ID pairs. Preserve the input Quad winding in the returned Triangles.

- [ ] **Step 4: Run the diagonal and existing skewness tests**

```powershell
cmake --build build --target boundary_mesh_quad_diagonal_test boundary_mesh_surface_face_skewness_test
ctest --test-dir build -R "boundary_mesh_(quad_diagonal|surface_face_skewness)_test" --output-on-failure
```

Expected: both tests pass.

- [ ] **Step 5: Commit**

```powershell
git add CMakeLists.txt tests/CMakeLists.txt include/boundary_mesh/transition/quad_diagonal.hpp src/transition/quad_diagonal.cpp tests/unit/transition/quad_diagonal_test.cpp
git commit -m "feat: select quad transition diagonals"
```

---

### Task 4: Triangle transition templates

**Files:**
- Create: `include/boundary_mesh/transition/transition_templates.hpp`
- Create: `src/transition/triangle_transition_template.cpp`
- Create: `tests/unit/transition/triangle_transition_template_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: source ID, trial count, optional high-edge index, oriented per-layer Triangle vertex IDs, and shared output vertices.
- Produces: `SourceTransitionResult { volume_cells, metadata, top_faces }`.

- [ ] **Step 1: Write failing tests for zero, one, and side-step cases**

Assert these exact behaviors:

```cpp
const auto zero = buildTriangleTransition(triangleInput(0, std::nullopt));
assert(zero.hasValue());
assert(zero.value().volume_cells.empty());
assert(zero.value().top_faces == std::vector<Triangle>{{{0,1,2}}});

const auto one = buildTriangleTransition(triangleInput(1, std::nullopt));
assert(one.hasValue());
assert(one.value().volume_cells.empty());
assert(one.value().top_faces.size() == 1);

const auto step = buildTriangleTransition(triangleInput(2, 0));
assert(step.hasValue());
assert(step.value().volume_cells.size() == 2); // retained penultimate Prism + Pyramid
assert(std::holds_alternative<Prism>(step.value().volume_cells[0]));
assert(std::get<Pyramid>(step.value().volume_cells[1]).vertex_ids ==
       std::array<VertexId,5>{3,4,7,6,5});
```

Use a fixture whose edge 0 is `(v0,v1)`, low IDs are `(3,4,5)`, and high edge IDs are `(6,7)`.

- [ ] **Step 2: Run the test to verify it fails**

```powershell
cmake --build build --target boundary_mesh_triangle_transition_template_test
```

Expected: FAIL because the template API is absent.

- [ ] **Step 3: Implement the source-local result and Triangle cases**

```cpp
struct SourceTransitionResult
{
    SurfaceFaceId source_face_id{};
    std::vector<Point3> created_vertices;
    std::vector<VolumeCell> volume_cells;
    std::vector<CellMetadata> metadata;
    std::vector<Triangle> top_faces;
};

struct TriangleTransitionInput
{
    SurfaceFaceId source_face_id{};
    std::uint32_t trial_layers{};
    std::optional<std::size_t> high_edge_local_index;
    std::vector<std::array<VertexId, 3>> layer_vertex_ids;
};

struct QuadTransitionInput
{
    SurfaceFaceId source_face_id{};
    std::uint32_t trial_layers{};
    std::optional<std::size_t> high_edge_local_index;
    std::vector<std::array<VertexId, 4>> layer_vertex_ids;
    const std::vector<Point3> *mesh_vertices{};
    Scalar length_tolerance{1e-12};
};

struct InvalidTransitionTemplateInput
{
    SurfaceFaceId source_face_id{};
};

using TransitionTemplateResult = Result<
    SourceTransitionResult,
    std::variant<InvalidTransitionTemplateInput, FaceEvaluationError>>;

TransitionTemplateResult buildTriangleTransition(
    const TriangleTransitionInput &input);
TransitionTemplateResult buildQuadTransition(
    const QuadTransitionInput &input);
```

For zero and one trial layers without a real high edge, emit the initial Triangle and no volume. For two or more trial layers, retain ordinary Prisms through the penultimate layer. With a high edge, append `Pyramid(v0_low,v1_low,v1_high,v0_high,v2_low)` after cyclically rotating the source so its high edge is local edge 0. Mark retained Prisms `RegularLayer` and the Pyramid `Transition`.

- [ ] **Step 4: Run the Triangle tests**

```powershell
cmake --build build --target boundary_mesh_triangle_transition_template_test
ctest --test-dir build -R boundary_mesh_triangle_transition_template_test --output-on-failure
```

Expected: all zero/one/regular/high-edge cases pass for all three local edge indices.

- [ ] **Step 5: Commit**

```powershell
git add CMakeLists.txt tests/CMakeLists.txt include/boundary_mesh/transition/transition_templates.hpp src/transition/triangle_transition_template.cpp tests/unit/transition/triangle_transition_template_test.cpp
git commit -m "feat: build triangle reserved-layer transitions"
```

---

### Task 5: Penultimate Quad-Hexa decomposition

**Files:**
- Create: `src/transition/quad_transition_template.cpp`
- Create: `tests/unit/transition/quad_hexa_decomposition_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `QuadTransitionInput` and `chooseQuadDiagonal`.
- Produces: five Pyramids, two Tetrahedra, one center vertex, and two selected top Triangles for every Quad with at least two trial layers.

- [ ] **Step 1: Write the failing canonical decomposition test**

Use a unit cube with bottom IDs `0..3`, top IDs `4..7`, and no high edge. Assert:

```cpp
const auto result = buildQuadTransition(unitCubeQuadInput(2, std::nullopt));
assert(result.hasValue());
assert(result.value().created_vertices.size() == 1);
assert(result.value().volume_cells.size() == 7);
assert(countCells<Pyramid>(result.value()) == 5);
assert(countCells<Tetra>(result.value()) == 2);
assert(result.value().top_faces.size() == 2);
for (const auto &meta : result.value().metadata)
    assert(meta.role == CellRole::Transition);
```

Also assert the first Pyramid is `{b0,b1,b2,b3,c}` and the remaining four use the exact side-face orders from the spec.

- [ ] **Step 2: Run the test to verify it fails**

```powershell
cmake --build build --target boundary_mesh_quad_hexa_decomposition_test
```

Expected: FAIL because `buildQuadTransition` is not implemented.

- [ ] **Step 3: Implement no-high-edge Quad cases**

For zero or one trial layer, do not create a center or volume; call `chooseQuadDiagonal` on the initial Quad and emit its two Triangles. For two or more trial layers, copy ordinary Hexas through layer `L-2`, compute the penultimate Hexa center as the arithmetic mean of its eight vertices, append the five specified Pyramids, choose the top diagonal, and append the two specified Tetrahedra. Return the chosen two top Triangles for later side processing.

- [ ] **Step 4: Run Quad decomposition and diagonal tests**

```powershell
cmake --build build --target boundary_mesh_quad_hexa_decomposition_test boundary_mesh_quad_diagonal_test
ctest --test-dir build -R "boundary_mesh_quad_(hexa_decomposition|diagonal)_test" --output-on-failure
```

Expected: both diagonal choices produce exactly five Pyramids and two Tetrahedra with deterministic IDs.

- [ ] **Step 5: Commit**

```powershell
git add CMakeLists.txt tests/CMakeLists.txt src/transition/quad_transition_template.cpp tests/unit/transition/quad_hexa_decomposition_test.cpp
git commit -m "feat: decompose penultimate quad hexas"
```

---

### Task 6: Quad single-high-edge side template and direct top faces

**Files:**
- Modify: `src/transition/quad_transition_template.cpp`
- Create: `tests/unit/transition/quad_side_transition_template_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: the selected low-top diagonal and unique high edge from `QuadTransitionInput`.
- Produces: canonical one-Pyramid/one-Tetra side transition and four directly authored top Triangles.

- [ ] **Step 1: Write the failing `a-d` canonical test**

```cpp
const auto result = buildQuadTransition(
    canonicalHighEdgeInput(/* diagonal a-d, high edge a-b */));
assert(result.hasValue());
assert(result.value().side_cells.size() == 2);
assert(std::get<Pyramid>(result.value().side_cells[0]).vertex_ids ==
       std::array<VertexId,5>{e,f,a,b,d});
assert(std::get<Tetra>(result.value().side_cells[1]).vertex_ids ==
       std::array<VertexId,4>{a,c,d,e});
assert(result.value().top_faces == std::vector<Triangle>{
    Triangle{{a,e,c}}, Triangle{{e,d,c}},
    Triangle{{e,f,d}}, Triangle{{f,b,d}}});
```

Repeat the assertion after rotating the source Quad through all four high-edge indices, comparing rotation-normalized topology rather than raw fixture IDs.

- [ ] **Step 2: Run the test to verify it fails**

```powershell
cmake --build build --target boundary_mesh_quad_side_transition_template_test
```

Expected: FAIL because the side template is not emitted.

- [ ] **Step 3: Implement canonicalization and both diagonal cases**

Rotate the source so its high edge is local `a-b`. If the already-selected diagonal is `a-d`, emit exactly:

```text
Pyramid(e,f,a,b,d)
Tetra(a,c,d,e)
top: aec, edc, efd, fbd
```

For the alternate diagonal, reflect the canonical pattern so the Pyramid consumes the Triangle adjacent to `a-b` and the Tetra consumes the other Triangle. Rotate resulting IDs back to source orientation. Do not call `chooseQuadDiagonal` again. Append both cells with `CellRole::Transition`.

- [ ] **Step 4: Run all Quad template tests**

```powershell
cmake --build build --target boundary_mesh_quad_side_transition_template_test boundary_mesh_quad_hexa_decomposition_test
ctest --test-dir build -R "boundary_mesh_quad_(side_transition_template|hexa_decomposition)_test" --output-on-failure
```

Expected: canonical and all rotated cases pass; every source returns its own final top faces.

- [ ] **Step 5: Commit**

```powershell
git add tests/CMakeLists.txt src/transition/quad_transition_template.cpp tests/unit/transition/quad_side_transition_template_test.cpp
git commit -m "feat: add quad side transition template"
```

---

### Task 7: Atomic reserved-layer transition pipeline

**Files:**
- Create: `include/boundary_mesh/transition/reserved_layer_transition.hpp`
- Create: `src/transition/reserved_layer_transition.cpp`
- Create: `tests/integration/reserved_layer_transition_pipeline_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: original surface/topology/patch, requested profiles, growth options, and existing `RegularLayerGenerator`.
- Produces: `ReservedLayerTransitionResult { VolumeMesh mesh; SurfaceMesh boundary_layer_top; coordinated_faces; trial_growth; }`.

- [ ] **Step 1: Write an end-to-end mixed-source test**

Build a small Wall patch containing adjacent Triangle and Quad sources with requested counts that yield trial cases 0, 1, 2, and 3 via deterministic fixture geometry/options. Assert:

```cpp
const auto result = generateReservedLayerTransition(
    surface, topology, patch, initial_front, requested_profiles, options);
assert(result.hasValue());
assert(result.value().trial_growth.faces.size() == patch.sourceFaceIds().size());
assert(noCommittedOutermostOrdinaryCells(result.value()));
assert(everyLowFaceHasAtMostOneHighEdge(result.value().coordinated_faces));
assert(result.value().mesh.cells.size() == result.value().mesh.metadata.size());
assert(allFacesAreTriangles(result.value().boundary_layer_top));
assert(topFacesAreInSourceFaceOrder(result.value()));
```

Include focused scenarios for `0<->1` (no transition volume), `1<->2` Triangle (one Pyramid), `1<->2` Quad (one Pyramid plus one Tetra), a Quad with no high edge and two trial layers (five Pyramids plus two Tetra), and a penultimate Triangle Prism that remains present.

- [ ] **Step 2: Run the integration test to verify it fails**

```powershell
cmake --build build --target boundary_mesh_reserved_layer_transition_pipeline_test
```

Expected: FAIL because the public pipeline is undefined.

- [ ] **Step 3: Implement the public error and result model**

```cpp
struct ReservedLayerTransitionResult
{
    VolumeMesh mesh;
    SurfaceMesh boundary_layer_top;
    std::vector<CoordinatedTransitionFace> coordinated_faces;
    RegularLayerGrowthResult trial_growth;
};

struct TransitionTemplateFailure
{
    SurfaceFaceId source_face_id{};
};

using ReservedLayerTransitionError = std::variant<
    ReservedLayerCountOverflow,
    RegularLayerGrowthError,
    TransitionCoordinationError,
    FaceEvaluationError,
    TransitionTemplateFailure>;
```

Expose `generateReservedLayerTransition(...)` with the same surface/topology/patch/front/profile/options inputs as `generateRegularLayers`.

- [ ] **Step 4: Implement atomic assembly**

Call `makeReservedTrialProfiles`, then existing `generateRegularLayers`. Coordinate the returned per-source `accepted_layer_count` values. Build a new output `VolumeMesh` rather than mutating `trial_growth.mesh`: copy only ordinary cells below each coordinated transition base, invoke the Triangle or Quad source template, append created center vertices with checked `VertexId` conversion, remap template-local IDs, and append cells/metadata only after the entire source result succeeds. Concatenate source `top_faces` in source-face order and copy referenced points into a compact `SurfaceMesh` tagged `BoundaryLayerInterface`.

- [ ] **Step 5: Run the integration test and existing growth regressions**

```powershell
cmake --build build --target boundary_mesh_reserved_layer_transition_pipeline_test boundary_mesh_regular_layer_growth_pipeline_test boundary_mesh_layer_coordination_pipeline_test
ctest --test-dir build -R "boundary_mesh_(reserved_layer_transition|regular_layer_growth|layer_coordination)_pipeline_test" --output-on-failure
```

Expected: all three pipeline tests pass; existing regular growth behavior is unchanged when called directly.

- [ ] **Step 6: Commit**

```powershell
git add CMakeLists.txt tests/CMakeLists.txt include/boundary_mesh/transition/reserved_layer_transition.hpp src/transition/reserved_layer_transition.cpp tests/integration/reserved_layer_transition_pipeline_test.cpp
git commit -m "feat: assemble reserved-layer transitions"
```

---

### Task 8: CLI pipeline adoption and full verification

**Files:**
- Modify: `src/cli/boundary_mesh_command.cpp`
- Modify: `tests/integration/cgns_cli_pipeline_test.cpp`
- Modify: `docs/design/roadmap.md`

**Interfaces:**
- Consumes: `generateReservedLayerTransition` from Task 7.
- Produces: CLI VTK volume containing transition cells and boundary-layer interface output from `boundary_layer_top`.

- [ ] **Step 1: Extend the CLI test before changing the CLI**

Add a fixture invocation requesting `N` layers and assert that the written VTK contains only committed ordinary cells plus Transition-role Pyramid/Tetra cells, never unchanged layer `N+1` or `N+2` cells. Assert the boundary-layer interface cell count equals `boundary_layer_top.faces.size()`.

- [ ] **Step 2: Run the CLI test to verify it fails**

```powershell
cmake --build build --target boundary_mesh_cgns_cli_pipeline_test
ctest --test-dir build -R boundary_mesh_cgns_cli_pipeline_test --output-on-failure
```

Expected: FAIL because the CLI still calls regular growth directly.

- [ ] **Step 3: Switch the CLI orchestration**

Replace the direct `generateRegularLayers(...)` call with `generateReservedLayerTransition(...)`. Pass `result.mesh` to the existing volume writer and use `result.boundary_layer_top` wherever the CLI previously consumed `growth.farfield_boundary` as the boundary-layer interface. Preserve original Farfield faces when assembling final surface output.

- [ ] **Step 4: Update roadmap status and documented exclusions**

Mark the reserved-layer Pyramid/Tetra transition feature implemented. State explicitly that transition quality, transition intersection checks, multiple-high-edge templates, and global template optimization remain deferred.

- [ ] **Step 5: Run full verification**

```powershell
cmake --build build
ctest --test-dir build --output-on-failure
```

Expected: build succeeds and every registered test passes.

- [ ] **Step 6: Commit**

```powershell
git add src/cli/boundary_mesh_command.cpp tests/integration/cgns_cli_pipeline_test.cpp docs/design/roadmap.md
git commit -m "feat: enable reserved-layer transition pipeline"
```
