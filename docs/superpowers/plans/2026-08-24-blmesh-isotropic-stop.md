# BLMesh-Style Isotropic Stop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the face-area isotropic-stop test with a BLMesh-style node-centered, multi-scale, neighbor-consensus test for triangular and quadrilateral growth fronts.

**Architecture:** Add a focused `IsotropicStopEvaluator` that computes per-face perimeter scales, per-node BLMesh candidate states, immutable one-ring consensus, and per-face stop flags. `RegularLayerStepper` evaluates the complete eligible candidate front once, then converts evaluator flags into the existing accepted-stop events only for quality-accepted cells.

**Tech Stack:** C++17, Eigen geometry types already exposed by BoundaryMesh Core, CMake/CTest, existing `Result` and growth-front types.

## Global Constraints

- Preserve `RegularLayerGrowthOptions::isotropic_height`, CLI option `--isotropic-height`, default value `1`, and `FaceStopReason::IsotropicHeightReached`.
- Recompute the unweighted global mean face scale for every eligible active layer.
- Use perimeter edges only: three for triangles and four for quadrilaterals; do not use diagonals or `sqrt(face_area)`.
- Use fixed BLMesh constants `0.95`, `1.3`, `1.8`, blends `(0.1, 0.9)` and `(0.3, 0.7)`.
- Evaluate node consensus from an immutable candidate snapshot so results do not depend on iteration order.
- Accept the current quality-valid cell before recording isotropic termination for its next layer.
- Do not alter collision, quality-rejection, requested-layer, or neighbor-layer propagation semantics.

---

## File Structure

- Create `include/boundary_mesh/growth/isotropic_stop_evaluator.hpp`: evaluator result type and public growth-module interface.
- Create `src/growth/isotropic_stop_evaluator.cpp`: validation, face scales, BLMesh node test, neighbor consensus, and face aggregation.
- Create `tests/unit/growth/isotropic_stop_evaluator_test.cpp`: focused geometry and consensus unit tests.
- Modify `src/growth/regular_layer_stepper.cpp`: replace `sqrt(area)` calculation with evaluator output.
- Modify `tests/unit/growth/regular_layer_stepper_test.cpp`: update prism/hexa behavioral expectations and prove accepted-stop integration.
- Modify `CMakeLists.txt` and `tests/CMakeLists.txt`: compile and register the new component/test.
- Modify `docs/design/modules/isotropic-height-stop.md` and `README.md`: document the new formula and semantics.

---

### Task 1: Add the BLMesh-Style Isotropic Evaluator

**Files:**
- Create: `include/boundary_mesh/growth/isotropic_stop_evaluator.hpp`
- Create: `src/growth/isotropic_stop_evaluator.cpp`
- Create: `tests/unit/growth/isotropic_stop_evaluator_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `GrowthFront current_front`, `GrowthFront candidate_front`, `FrontAdjacency adjacency`, and positive finite `Scalar isotropic_height`.
- Produces: `Result<IsotropicStopEvaluation, InvalidLayerFrontMapping> IsotropicStopEvaluator::evaluate(...) const`.
- `IsotropicStopEvaluation` exposes `vertex_candidates`, `vertex_stops`, and `face_stops` as equally indexed `std::vector<bool>` collections for deterministic testing and stepper consumption.

- [ ] **Step 1: Register the new source and failing unit-test target**

Add `src/growth/isotropic_stop_evaluator.cpp` beside `front_adjacency.cpp` in `boundary_mesh_boundary_layer`. Add this target to `tests/CMakeLists.txt`:

```cmake
add_executable(
    boundary_mesh_isotropic_stop_evaluator_test
    unit/growth/isotropic_stop_evaluator_test.cpp
)
target_link_libraries(
    boundary_mesh_isotropic_stop_evaluator_test
    PRIVATE BoundaryMesh::BoundaryLayer
)
add_test(
    NAME boundary_mesh_isotropic_stop_evaluator_test
    COMMAND boundary_mesh_isotropic_stop_evaluator_test
)
```

- [ ] **Step 2: Write the public interface and a regular triangle/quad equivalence test**

Create the header with this interface:

```cpp
#pragma once

#include <vector>

#include <boundary_mesh/core/result.hpp>
#include <boundary_mesh/growth/front_adjacency.hpp>
#include <boundary_mesh/growth/growth_front.hpp>
#include <boundary_mesh/growth/regular_layer_growth_error.hpp>

namespace boundary_mesh
{
    struct IsotropicStopEvaluation
    {
        std::vector<bool> vertex_candidates;
        std::vector<bool> vertex_stops;
        std::vector<bool> face_stops;
    };

    class IsotropicStopEvaluator
    {
    public:
        Result<IsotropicStopEvaluation, InvalidLayerFrontMapping>
        evaluate(
            const GrowthFront &current_front,
            const GrowthFront &candidate_front,
            const FrontAdjacency &adjacency,
            Scalar isotropic_height) const;
    };
}
```

In the test, construct a disconnected front containing one unit equilateral triangle and one unit square. Give every candidate vertex a normal displacement of `1.1`, build adjacency with `buildFrontAdjacency`, and assert both faces stop at `isotropic_height == 1.0`. The displacement is above the ordinary BLMesh boundary `1 / 0.95` while remaining below both hard guards:

```cpp
const auto result = IsotropicStopEvaluator{}.evaluate(
    current, candidate, buildFrontAdjacency(current).value(), Scalar{1});
if (!result.hasValue() ||
    result.value().face_stops != std::vector<bool>({true, true}))
{
    return 1;
}
```

- [ ] **Step 3: Run the new test to verify RED**

Run:

```powershell
cmake -S . -B build
cmake --build build --config Debug --target boundary_mesh_isotropic_stop_evaluator_test
```

Expected: compilation/link failure because `IsotropicStopEvaluator::evaluate` has no implementation.

- [ ] **Step 4: Implement face scales and regular candidate decisions minimally**

Implement an internal scale type and perimeter walker:

```cpp
struct FaceScales
{
    Scalar average{};
    Scalar geometric{};
    Scalar minimum{};
};

template <class Face>
FaceScales faceScales(const Face &face, const GrowthFront &front)
{
    Scalar sum = 0;
    Scalar product = 1;
    Scalar minimum = std::numeric_limits<Scalar>::max();
    for (std::size_t local = 0; local < face.vertex_ids.size(); ++local)
    {
        const auto first = static_cast<std::size_t>(face.vertex_ids[local]);
        const auto second = static_cast<std::size_t>(
            face.vertex_ids[(local + 1) % face.vertex_ids.size()]);
        const Scalar length =
            (front.vertices[second].position -
             front.vertices[first].position).norm();
        sum += length;
        product *= length;
        minimum = std::min(minimum, length);
    }
    const Scalar count = static_cast<Scalar>(face.vertex_ids.size());
    return {sum / count, std::pow(product, Scalar{1} / count), minimum};
}
```

Compute `G` as the mean of face `average` values. For each vertex and every incident face, apply:

```cpp
const Scalar scaled_height = side_length / isotropic_height;
const Scalar blended = std::max(
    Scalar{0.1} * global_average + Scalar{0.9} * scales.average,
    Scalar{0.3} * global_average + Scalar{0.7} * scales.average);

if (blended > Scalar{0.95} * scaled_height)
    candidate = false;
if (scaled_height > Scalar{1.3} * scales.geometric ||
    scaled_height > Scalar{1.8} * scales.minimum)
    candidate = true;
```

Confirm a vertex only when all entries in `adjacency.vertex_neighbors[vertex]` are candidates. Mark a face stopped when any of its vertices is confirmed.

- [ ] **Step 5: Run the focused test to verify GREEN**

Run:

```powershell
cmake --build build --config Debug --target boundary_mesh_isotropic_stop_evaluator_test
ctest --test-dir build -C Debug -R "^boundary_mesh_isotropic_stop_evaluator_test$" --output-on-failure
```

Expected: one test passes.

- [ ] **Step 6: Add RED tests for continuation and both hard-stop guards**

Extend the unit test with separate return codes for these real geometries:

```cpp
// Unit triangle and square displaced by 0.5: neither stops.
assert(evaluate(displacementHalf).face_stops ==
       std::vector<bool>({false, false}));

// Slender triangle where H exceeds 1.8 * Lmin: stops.
assert(evaluate(slenderTriangle).face_stops ==
       std::vector<bool>({true}));

// Slender quad where H exceeds 1.8 * Lmin: stops.
assert(evaluate(slenderQuad).face_stops ==
       std::vector<bool>({true}));

// An anisotropic face where H exceeds 1.3 * Lgeo while it does not
// exceed 1.8 * Lmin: stops through the geometric guard.
assert(evaluate(geometricGuard).face_stops ==
       std::vector<bool>({true}));
```

Choose explicit dimensions and assert the preconditions numerically in the test so each guard is isolated rather than inferred from the final boolean.

- [ ] **Step 7: Temporarily disable each missing branch and verify the new tests fail for the intended reason**

Run the test after adding each case before its matching production branch. Expected: the specific return code for continuation, minimum, or geometric behavior is reported. Do not combine production changes until each new assertion has demonstrated RED.

- [ ] **Step 8: Complete the guard implementation and verify GREEN**

Implement only the branches exercised by Step 6, then run:

```powershell
cmake --build build --config Debug --target boundary_mesh_isotropic_stop_evaluator_test
ctest --test-dir build -C Debug -R "^boundary_mesh_isotropic_stop_evaluator_test$" --output-on-failure
```

Expected: one test passes with exit code zero.

- [ ] **Step 9: Add RED tests for immutable neighborhood consensus and invalid mappings**

Create a connected two-face front whose center candidate has at least one non-candidate neighbor. Assert:

```cpp
if (!evaluation.vertex_candidates[center] ||
    evaluation.vertex_stops[center] ||
    evaluation.face_stops[center_face])
{
    return consensus_failure;
}
```

Permute vertex and face order, rebuild adjacency, and assert the source-face keyed result is unchanged. Also pass mismatched current/candidate vertex counts and assert `InvalidLayerFrontMapping{candidate.layer}` is returned.

- [ ] **Step 10: Run the consensus tests to verify RED, implement snapshot consensus/validation, then verify GREEN**

The implementation must compute all `vertex_candidates` first, then fill `vertex_stops` in a second loop without mutating candidates. Validate current/candidate face types, connectivity, adjacency vector sizes, finite positive heights, and finite positive face scales before calculations.

Run:

```powershell
cmake --build build --config Debug --target boundary_mesh_isotropic_stop_evaluator_test
ctest --test-dir build -C Debug -R "^boundary_mesh_(front_adjacency|isotropic_stop_evaluator)_test$" --output-on-failure
```

Expected: both tests pass.

- [ ] **Step 11: Commit Task 1**

```powershell
git add CMakeLists.txt tests/CMakeLists.txt `
  include/boundary_mesh/growth/isotropic_stop_evaluator.hpp `
  src/growth/isotropic_stop_evaluator.cpp `
  tests/unit/growth/isotropic_stop_evaluator_test.cpp
git commit -m "feat: evaluate BLMesh-style isotropic stops"
```

---

### Task 2: Integrate Node-Consensus Stops into Regular Layer Stepping

**Files:**
- Modify: `src/growth/regular_layer_stepper.cpp`
- Modify: `tests/unit/growth/regular_layer_stepper_test.cpp`

**Interfaces:**
- Consumes: `IsotropicStopEvaluation::face_stops` indexed by the compact eligible front.
- Produces: existing `LayerStepResult::accepted_stopped_faces` events with `layer == target_layer + 1` and reason `IsotropicHeightReached`.

- [ ] **Step 1: Replace area-normalized test expectations with equal triangle/quad semantics**

In `regular_layer_stepper_test.cpp`, remove the old triangle threshold based on `0.25 / sqrt(0.5)`. Use geometrically comparable regular fronts and heights so both prism and hexa cases assert:

```cpp
options.isotropic_height = Scalar{1};
// Above the normalized ordinary boundary 1 / 0.95, current cell is accepted and
// accepted_stopped_faces contains one IsotropicHeightReached event.
```

Keep the scaled-front test but make it assert that doubling the perimeter scale while holding actual height fixed continues growth.

- [ ] **Step 2: Run the stepper test to verify RED**

Run:

```powershell
cmake --build build --config Debug --target boundary_mesh_regular_layer_stepper_test
ctest --test-dir build -C Debug -R "^boundary_mesh_regular_layer_stepper_test$" --output-on-failure
```

Expected: failure in the triangular equivalence assertion because the existing implementation still divides by `sqrt(area)`.

- [ ] **Step 3: Evaluate isotropic flags once after candidate geometry is built**

Include the new evaluator and insert:

```cpp
const auto isotropic = IsotropicStopEvaluator{}.evaluate(
    eligible.front,
    candidate_front,
    adjacency_result.value(),
    options.isotropic_height);
if (!isotropic.hasValue())
{
    return StepResult::failure(isotropic.error());
}
```

Delete `averageSideLength` and every use of `current_base_area` and
`std::sqrt(current_base_area)` from the isotropic decision.

- [ ] **Step 4: Convert flags only for quality-accepted faces**

Inside the existing `quality.value().acceptable` branch, preserve
`accepted_eligible_faces.push_back(...)` and replace the old ratio block with:

```cpp
if (isotropic.value().face_stops[eligible_face_index])
{
    output.accepted_stopped_faces.push_back(
        FaceStopEvent{
            previous_face_index,
            current_front.source_face_ids[previous_face_index],
            target_layer + 1,
            FaceStopReason::IsotropicHeightReached});
}
```

This ordering ensures rejected candidates never receive an accepted-stop event.

- [ ] **Step 5: Run focused tests to verify GREEN**

Run:

```powershell
cmake --build build --config Debug --target `
  boundary_mesh_isotropic_stop_evaluator_test `
  boundary_mesh_regular_layer_stepper_test `
  boundary_mesh_regular_layer_growth_pipeline_test `
  boundary_mesh_layer_coordination_pipeline_test
ctest --test-dir build -C Debug `
  -R "^boundary_mesh_(isotropic_stop_evaluator|regular_layer_stepper|regular_layer_growth_pipeline|layer_coordination_pipeline)_test$" `
  --output-on-failure
```

Expected: all four tests pass.

- [ ] **Step 6: Confirm the obsolete formula is gone**

Run:

```powershell
rg -n "averageSideLength|sqrt\(current_base_area\)|isotropic_ratio" src/growth/regular_layer_stepper.cpp
```

Expected: no matches.

- [ ] **Step 7: Commit Task 2**

```powershell
git add src/growth/regular_layer_stepper.cpp `
  tests/unit/growth/regular_layer_stepper_test.cpp
git commit -m "fix: use node consensus for isotropic stopping"
```

---

### Task 3: Update Documentation and Verify the Whole Project

**Files:**
- Modify: `docs/design/modules/isotropic-height-stop.md`
- Modify: `README.md`

**Interfaces:**
- Consumes: final evaluator semantics and unchanged CLI/API names.
- Produces: user-facing explanation of the BLMesh-style formula, constants, triangle/quad treatment, and accepted-current-layer behavior.

- [ ] **Step 1: Update the module design documentation**

Replace the old formula with:

```text
For every active face:
  Lavg = mean(perimeter edge lengths)
  Lgeo = geometric_mean(perimeter edge lengths)
  Lmin = min(perimeter edge lengths)

G = mean(Lavg over active faces)
H = actual candidate side length / isotropic_height
Lblend = max(0.1 G + 0.9 Lavg, 0.3 G + 0.7 Lavg)

continue when Lblend > 0.95 H unless
H > 1.3 Lgeo or H > 1.8 Lmin
```

Document that triangle faces use three perimeter edges, quad faces use four,
and a node stops only after direct-neighbor candidate consensus.

- [ ] **Step 2: Update README troubleshooting text**

Explain that `stop_isotropic_height` means the current cell was accepted and
the next layer was suppressed after BLMesh-style local/global scale and
neighbor-consensus evaluation. Remove any remaining claim that the threshold
means `height / sqrt(area)`.

- [ ] **Step 3: Run documentation and diff hygiene checks**

Run:

```powershell
rg -n "sqrt\(.*area|average_side_length / sqrt|isotropic_ratio" README.md docs/design/modules/isotropic-height-stop.md
git diff --check
```

Expected: no obsolete formula matches and `git diff --check` exits zero.

- [ ] **Step 4: Build and run the complete configured test suite**

Run:

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

Expected: build exits zero and CTest reports zero failed tests.

- [ ] **Step 5: Re-run the 2dot5_cf 20-layer diagnostic**

Use the same input and growth parameters recorded in
`docs/plans/height-smoothing-state-separation.md`, but write to a new output
prefix so the user's existing VTK files are preserved:

```powershell
.\build\Debug\boundary_mesh_cli.exe `
  --input "C:\Users\zpern\Desktop\todo\jiuyuan-quailty\test_case\2dot5_cf\2dot5_cf.cgns" `
  --first-height 0.1 `
  --growth-ratio 1.2 `
  --layer-count 20 `
  --maximum-skewness 0.95 `
  --isotropic-height 1.0 `
  --output-prefix ".\build\real_case\2dot5_cf_blmesh_isotropic"
```

Expected: exit code zero, 20-layer progress output, finite VTK coordinates,
and reported `stop_isotropic_height` count. Record the old/new volume-cell and
stop-reason counts in the final handoff; do not require identical counts because
the intended stopping behavior changed.

- [ ] **Step 6: Inspect the original reported lineage**

Trace the cell lineage beginning with the old source path
`6283 -> 64397 -> 122292 -> 179825 -> 236806 -> 293082 -> 348509 -> 402909`
against the new output by shared bottom/top connectivity. Report whether the
corresponding source region continues beyond layer 8 and, if it later stops,
which aggregate stopping mechanism is consistent with the new run. Do not
assume VTK cell IDs remain stable after the algorithm change.

- [ ] **Step 7: Commit Task 3**

```powershell
git add README.md docs/design/modules/isotropic-height-stop.md
git commit -m "docs: explain BLMesh-style isotropic stopping"
```

---

## Final Verification Checklist

- [ ] Every new evaluator behavior was observed failing before its production branch was added.
- [ ] Triangle and quad regular-scale equivalence passes.
- [ ] Geometric and minimum guards pass for both relevant face shapes.
- [ ] Immutable neighbor consensus and permutation determinism pass.
- [ ] Stepper accepts current cells before emitting isotropic accepted-stop events.
- [ ] No production isotropic decision uses face area.
- [ ] Full Debug build and configured CTest suite pass with zero failures.
- [ ] The new real-case output uses a distinct prefix and preserves existing artifacts.
- [ ] Final handoff includes fresh test counts and old/new real-case statistics.
