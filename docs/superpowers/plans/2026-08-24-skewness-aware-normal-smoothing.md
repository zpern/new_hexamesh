# Skewness-Aware Normal Smoothing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a bounded local direction search that reduces incident candidate-cell maximum skewness above 0.8 without changing smoothed layer heights or final quality acceptance.

**Architecture:** Keep baseline direction and height smoothing intact, then pass the frozen height field to a dedicated skewness direction refiner. The refiner evaluates local prism/hexahedron trials synchronously, accepts only strict minimax improvements that satisfy existing safety constraints, and returns diagnostics that the stepper and generator aggregate.

**Tech Stack:** C++17, Eigen vectors, existing `Result` type, existing prism/hexahedron evaluators, CMake/CTest.

## Global Constraints

- Direction refinement is a soft optimization; `maximum_skewness` remains the final hard acceptance gate.
- The activation threshold is exactly `0.8` by default.
- The first and second angular offsets are 5 and 2 degrees, respectively.
- Each search level samples six uniformly spaced azimuths and includes the unchanged direction.
- Search runs for at most two levels and stops after a non-improving first level.
- `actual_heights` must be bit-for-bit identical with refinement enabled and disabled for the same smoothing input.
- Vertex updates within a level are synchronous and independent of traversal order.
- Degenerate, reversed, locally inverted, non-finite, or visibility-violating trials are rejected locally; they do not turn a valid baseline into a fatal smoothing error.
- Existing collision, stop-reason, and final quality-filter behavior must remain unchanged.
- Preserve unrelated working-tree content, including the existing untracked `.superpowers/` directory.

---

## File Structure

- Create `include/boundary_mesh/growth/growth_field_smoothing_options.hpp`: public refinement options and diagnostics shared by smoother, stepper, and generator.
- Create `include/boundary_mesh/growth/skewness_direction_refiner.hpp`: focused refinement input/output interface.
- Create `src/growth/skewness_direction_refiner.cpp`: tangent candidates, local cell assembly, minimax comparison, safety checks, synchronous two-level search.
- Create `tests/unit/growth/skewness_direction_refiner_test.cpp`: focused geometry and deterministic-search tests.
- Modify `include/boundary_mesh/growth/growth_field_smoother.hpp`: accept options and return refinement diagnostics.
- Modify `src/growth/growth_field_smoother.cpp`: preserve baseline height computation and invoke refinement afterward.
- Modify `include/boundary_mesh/growth/regular_layer_growth.hpp`: expose options and aggregate diagnostics in growth types.
- Modify `src/growth/regular_layer_stepper.cpp`: pass refinement options and attach step diagnostics.
- Modify `src/growth/regular_layer_generator.cpp`: accumulate step diagnostics.
- Modify `tests/unit/growth/growth_field_smoother_test.cpp`: disabled parity and frozen-height regression tests.
- Modify `tests/unit/growth/regular_layer_stepper_test.cpp`: integration and final-filter regression tests.
- Modify `tests/unit/growth/regular_layer_growth_types_test.cpp`: default option and diagnostic type checks.
- Modify `tests/CMakeLists.txt` and root `CMakeLists.txt`: compile and register the new component/test.
- Modify `benchmarks/cgns_pipeline_benchmark.cpp`: report activation, update, before/after maximum skewness, and runtime comparison inputs.

---

### Task 1: Public Options and Diagnostics

**Files:**
- Create: `include/boundary_mesh/growth/growth_field_smoothing_options.hpp`
- Modify: `include/boundary_mesh/growth/regular_layer_growth.hpp`
- Modify: `tests/unit/growth/regular_layer_growth_types_test.cpp`

**Interfaces:**
- Produces: `SkewnessNormalOptimizationOptions`, `GrowthFieldSmoothingOptions`, and `GrowthFieldSmoothingDiagnostics`.
- Produces: `RegularLayerGrowthOptions::field_smoothing` and `RegularLayerGrowthResult::smoothing_diagnostics`.

- [ ] **Step 1: Write failing public-default tests**

Append checks to `tests/unit/growth/regular_layer_growth_types_test.cpp`:

```cpp
const RegularLayerGrowthOptions options;
if (!options.field_smoothing.skewness.enabled ||
    options.field_smoothing.skewness.activation_skewness != Scalar{0.8} ||
    options.field_smoothing.skewness.first_angle_degrees != Scalar{5} ||
    options.field_smoothing.skewness.second_angle_degrees != Scalar{2} ||
    options.field_smoothing.skewness.azimuth_samples != 6 ||
    options.field_smoothing.skewness.maximum_levels != 2)
{
    return 13;
}

const RegularLayerGrowthResult empty_result;
if (empty_result.smoothing_diagnostics.activated_vertices != 0 ||
    empty_result.smoothing_diagnostics.updated_vertices != 0 ||
    empty_result.smoothing_diagnostics.maximum_skewness_before != Scalar{0} ||
    empty_result.smoothing_diagnostics.maximum_skewness_after != Scalar{0})
{
    return 14;
}
```

- [ ] **Step 2: Build the focused test and verify compilation fails**

Run:

```powershell
cmake --build build --config Release --target boundary_mesh_regular_layer_growth_types_test
```

Expected: compilation fails because `field_smoothing` and `smoothing_diagnostics` do not exist.

- [ ] **Step 3: Add the public option and diagnostic types**

Create `include/boundary_mesh/growth/growth_field_smoothing_options.hpp`:

```cpp
#pragma once

#include <cstddef>

#include <boundary_mesh/core/types.hpp>

namespace boundary_mesh
{
    struct SkewnessNormalOptimizationOptions
    {
        bool enabled{true};
        Scalar activation_skewness{0.8};
        Scalar first_angle_degrees{5};
        Scalar second_angle_degrees{2};
        std::size_t azimuth_samples{6};
        std::size_t maximum_levels{2};
        Scalar improvement_tolerance{1e-12};
    };

    struct GrowthFieldSmoothingOptions
    {
        SkewnessNormalOptimizationOptions skewness;
    };

    struct GrowthFieldSmoothingDiagnostics
    {
        std::size_t activated_vertices{};
        std::size_t updated_vertices{};
        Scalar maximum_skewness_before{};
        Scalar maximum_skewness_after{};
    };
}
```

Include it from `regular_layer_growth.hpp`, then extend the public structs:

```cpp
struct RegularLayerGrowthOptions
{
    VolumeCellQualityOptions cell_quality;
    GrowthFieldSmoothingOptions field_smoothing;
    std::uint32_t max_neighbor_layer_difference{1};
    Scalar isotropic_height{1};
};

struct RegularLayerGrowthResult
{
    // existing fields remain in their current order
    GrowthFieldSmoothingDiagnostics smoothing_diagnostics;
};
```

- [ ] **Step 4: Rebuild and run the focused test**

Run:

```powershell
cmake --build build --config Release --target boundary_mesh_regular_layer_growth_types_test
ctest --test-dir build -C Release -R boundary_mesh_regular_layer_growth_types_test --output-on-failure
```

Expected: build succeeds and the one selected test passes.

- [ ] **Step 5: Commit the type slice**

```powershell
git add -- include/boundary_mesh/growth/growth_field_smoothing_options.hpp include/boundary_mesh/growth/regular_layer_growth.hpp tests/unit/growth/regular_layer_growth_types_test.cpp
git commit -m "feat: add skewness smoothing options"
```

---

### Task 2: Local Skewness Direction Refiner

**Files:**
- Create: `include/boundary_mesh/growth/skewness_direction_refiner.hpp`
- Create: `src/growth/skewness_direction_refiner.cpp`
- Create: `tests/unit/growth/skewness_direction_refiner_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `GrowthFront`, `FrontEvaluation`, `FrontAdjacency`, fixed `std::vector<Scalar>` heights, baseline `std::vector<Vector3>` directions, and `SkewnessNormalOptimizationOptions`.
- Produces: `refineDirectionsForSkewness(...) -> SkewnessDirectionRefinement`, containing directions and diagnostics.

- [ ] **Step 1: Register an empty focused test target and write geometry fixtures**

Add `tests/unit/growth/skewness_direction_refiner_test.cpp` with a triangular fan fixture whose center direction can be tilted to reduce an incident prism's skewness. Add helpers that independently assemble `PrismPoints` from lower positions, directions, and fixed heights, then compute the incident maximum through `evaluatePrism()`.

The test's central assertions must have this exact shape:

```cpp
const auto refined = refineDirectionsForSkewness(
    front,
    evaluation,
    adjacency.value(),
    baseline,
    fixed_heights,
    options);

if (refined.diagnostics.activated_vertices == 0 ||
    refined.diagnostics.updated_vertices == 0 ||
    !(refined.diagnostics.maximum_skewness_after <
      refined.diagnostics.maximum_skewness_before) ||
    !(incidentMaximum(front, refined.directions, fixed_heights, 0) <
      incidentMaximum(front, baseline, fixed_heights, 0)))
{
    return 1;
}
```

Register it in `tests/CMakeLists.txt` as `boundary_mesh_skewness_direction_refiner_test`, linked to `BoundaryMesh::BoundaryLayer`.

- [ ] **Step 2: Build and verify the new test fails**

Run:

```powershell
cmake -S . -B build
cmake --build build --config Release --target boundary_mesh_skewness_direction_refiner_test
```

Expected: compilation fails because the refiner header/function does not exist.

- [ ] **Step 3: Define the focused interface**

Create `include/boundary_mesh/growth/skewness_direction_refiner.hpp`:

```cpp
#pragma once

#include <vector>

#include <boundary_mesh/core/types.hpp>
#include <boundary_mesh/growth/front_adjacency.hpp>
#include <boundary_mesh/growth/front_evaluation.hpp>
#include <boundary_mesh/growth/growth_field_smoothing_options.hpp>
#include <boundary_mesh/growth/growth_front.hpp>

namespace boundary_mesh
{
    struct SkewnessDirectionRefinement
    {
        std::vector<Vector3> directions;
        GrowthFieldSmoothingDiagnostics diagnostics;
    };

    SkewnessDirectionRefinement refineDirectionsForSkewness(
        const GrowthFront &front,
        const FrontEvaluation &evaluation,
        const FrontAdjacency &adjacency,
        const std::vector<Vector3> &baseline_directions,
        const std::vector<Scalar> &fixed_actual_heights,
        const SkewnessNormalOptimizationOptions &options);
}
```

- [ ] **Step 4: Implement cell assembly and lexicographic objectives**

In `src/growth/skewness_direction_refiner.cpp`, implement internal helpers with these concrete forms:

```cpp
struct LocalObjective
{
    Scalar maximum{};
    Scalar average{};
    bool valid{};
};

bool better(
    const LocalObjective &candidate,
    const LocalObjective &current,
    Scalar candidate_alignment,
    Scalar current_alignment,
    Scalar tolerance)
{
    if (!candidate.valid) return false;
    if (candidate.maximum < current.maximum - tolerance) return true;
    if (std::abs(candidate.maximum - current.maximum) > tolerance) return false;
    if (candidate.average < current.average - tolerance) return true;
    if (std::abs(candidate.average - current.average) > tolerance) return false;
    return candidate_alignment > current_alignment + tolerance;
}
```

For every incident triangle, assemble lower indices `[0..2]` and upper indices `[3..5]`; for every incident quad, assemble lower `[0..3]` and upper `[4..7]`. Treat anything other than `VolumeCellValidity::Valid`, and any evaluator failure, as `valid == false`. Do not compare against `maximum_skewness` here.

- [ ] **Step 5: Implement deterministic tangent sampling and synchronous levels**

Build a deterministic tangent basis by selecting the Cartesian axis least aligned with the current direction:

```cpp
const std::array<Vector3, 3> axes{
    Vector3::UnitX(), Vector3::UnitY(), Vector3::UnitZ()};
const auto axis = *std::min_element(
    axes.begin(), axes.end(),
    [&](const Vector3 &a, const Vector3 &b)
    {
        return std::abs(direction.dot(a)) < std::abs(direction.dot(b));
    });
const Vector3 tangent = direction.cross(axis).normalized();
const Vector3 bitangent = direction.cross(tangent).normalized();
```

Generate candidates using
`cos(angle) * direction + sin(angle) * (cos(azimuth) * tangent + sin(azimuth) * bitangent)`.
At each level, read only `level_start`, write `level_next`, and swap after every active vertex is evaluated. Activate only when the baseline local maximum is strictly above `activation_skewness`. Count a vertex as updated once even if both levels improve it.

Reuse the smoother's safety constants by moving them into named internal helpers if necessary; do not relax the current visibility or maximum-deviation rules.

- [ ] **Step 6: Build and run the improving-case test**

Run:

```powershell
cmake --build build --config Release --target boundary_mesh_skewness_direction_refiner_test
ctest --test-dir build -C Release -R boundary_mesh_skewness_direction_refiner_test --output-on-failure
```

Expected: the focused test passes and reports no output.

- [ ] **Step 7: Add threshold, fallback, invalid-trial, and permutation cases**

Extend the test executable with separately numbered checks:

```cpp
SkewnessNormalOptimizationOptions inactive = options;
inactive.activation_skewness = Scalar{1};
const auto unchanged = refineDirectionsForSkewness(
    front, evaluation, adjacency.value(), baseline, fixed_heights, inactive);
if (!sameDirections(unchanged.directions, baseline) ||
    unchanged.diagnostics.activated_vertices != 0) return 2;

SkewnessNormalOptimizationOptions disabled = options;
disabled.enabled = false;
const auto bypassed = refineDirectionsForSkewness(
    front, evaluation, adjacency.value(), baseline, fixed_heights, disabled);
if (!sameDirections(bypassed.directions, baseline)) return 3;
```

Add one fixture where all tilted trials invert a prism and assert exact baseline fallback. Permute face order and assert matching directions and diagnostics. Add a fixture whose quality-improving tilt violates visibility and assert rejection.

- [ ] **Step 8: Run focused and evaluator regressions**

Run:

```powershell
cmake --build build --config Release --target boundary_mesh_skewness_direction_refiner_test boundary_mesh_prism_evaluator_test boundary_mesh_hexa_evaluator_test
ctest --test-dir build -C Release -R "boundary_mesh_(skewness_direction_refiner|prism_evaluator|hexa_evaluator)_test" --output-on-failure
```

Expected: all three tests pass.

- [ ] **Step 9: Commit the refiner slice**

```powershell
git add -- CMakeLists.txt tests/CMakeLists.txt include/boundary_mesh/growth/skewness_direction_refiner.hpp src/growth/skewness_direction_refiner.cpp tests/unit/growth/skewness_direction_refiner_test.cpp
git commit -m "feat: refine growth normals by local skewness"
```

---

### Task 3: Integrate Refinement Without Changing Heights

**Files:**
- Modify: `include/boundary_mesh/growth/growth_field_smoother.hpp`
- Modify: `include/boundary_mesh/growth/growth_field_smoothing_error.hpp`
- Modify: `src/growth/growth_field_smoother.cpp`
- Modify: `tests/unit/growth/growth_field_smoother_test.cpp`

**Interfaces:**
- Consumes: Task 2's `refineDirectionsForSkewness()`.
- Produces: `GrowthFieldSmoother::smooth(..., const GrowthFieldSmoothingOptions &options = {})` and `SmoothedGrowthFields::diagnostics`.

- [ ] **Step 1: Write disabled-parity and frozen-height tests**

In `growth_field_smoother_test.cpp`, call the smoother twice with identical input:

```cpp
GrowthFieldSmoothingOptions disabled;
disabled.skewness.enabled = false;
const auto baseline_result = GrowthFieldSmoother{}.smooth(
    front, evaluation, adjacency.value(), raw,
    reference_heights, provisional_heights, disabled);

GrowthFieldSmoothingOptions enabled;
enabled.skewness.activation_skewness = Scalar{0};
const auto refined_result = GrowthFieldSmoother{}.smooth(
    front, evaluation, adjacency.value(), raw,
    reference_heights, provisional_heights, enabled);

if (!baseline_result.hasValue() || !refined_result.hasValue()) return 13;
if (baseline_result.value().actual_heights !=
    refined_result.value().actual_heights) return 14;
```

Also assert disabled diagnostics are zero and enabled diagnostics are populated when the fixture activates.

Add one invalid-options check:

```cpp
GrowthFieldSmoothingOptions invalid = enabled;
invalid.skewness.activation_skewness = Scalar{1.1};
const auto invalid_result = GrowthFieldSmoother{}.smooth(
    front, evaluation, adjacency.value(), raw,
    reference_heights, provisional_heights, invalid);
if (invalid_result.hasValue() ||
    std::get_if<InvalidSkewnessNormalOptimizationOptions>(
        &invalid_result.error()) == nullptr)
{
    return 15;
}
```

- [ ] **Step 2: Build and verify signature/field compilation failure**

Run:

```powershell
cmake --build build --config Release --target boundary_mesh_growth_field_smoother_test
```

Expected: compilation fails because the options overload and diagnostics field do not exist.

- [ ] **Step 3: Extend the smoother interface**

Update `SmoothedGrowthFields` and `smooth()`:

```cpp
struct SmoothedGrowthFields
{
    std::uint32_t layer{};
    std::vector<Vector3> directions;
    std::vector<Scalar> actual_heights;
    GrowthFieldSmoothingDiagnostics diagnostics;
};

Result<SmoothedGrowthFields, GrowthFieldSmoothingError> smooth(
    const GrowthFront &front,
    const FrontEvaluation &evaluation,
    const FrontAdjacency &adjacency,
    const GrowthDirections &raw_directions,
    const std::vector<Scalar> &reference_heights,
    const std::vector<Scalar> &provisional_heights,
    const GrowthFieldSmoothingOptions &options = {}) const;
```

Add a concrete validation error to
`growth_field_smoothing_error.hpp` and its error variant:

```cpp
struct InvalidSkewnessNormalOptimizationOptions
{
    Scalar activation_skewness{};
    Scalar first_angle_degrees{};
    Scalar second_angle_degrees{};
    std::size_t azimuth_samples{};
    std::size_t maximum_levels{};
    Scalar improvement_tolerance{};
};
```

At the start of `smooth()`, reject enabled options unless the trigger is finite
and in `[0, 1]`, both angles are finite and in `(0, 90)`, samples and levels are
nonzero, `maximum_levels <= 2`, and the improvement tolerance is finite and
non-negative. Return `InvalidSkewnessNormalOptimizationOptions` populated from
the received values. Disabled options bypass this validation because none of
their search fields are consumed.

- [ ] **Step 4: Invoke refinement strictly after height computation**

Keep the current direction loop and height loop unchanged. Immediately before the success return, invoke:

```cpp
auto refinement = refineDirectionsForSkewness(
    front,
    evaluation,
    adjacency,
    current,
    actual_heights,
    options.skewness);

return SmoothingResult::success(SmoothedGrowthFields{
    front.layer,
    std::move(refinement.directions),
    std::move(actual_heights),
    refinement.diagnostics});
```

This ordering is mandatory: never recompute `actual_heights` from refined directions.

- [ ] **Step 5: Run smoother and refiner tests**

Run:

```powershell
cmake --build build --config Release --target boundary_mesh_growth_field_smoother_test boundary_mesh_skewness_direction_refiner_test
ctest --test-dir build -C Release -R "boundary_mesh_(growth_field_smoother|skewness_direction_refiner)_test" --output-on-failure
```

Expected: both tests pass, including exact height-vector equality.

- [ ] **Step 6: Commit the integration slice**

```powershell
git add -- include/boundary_mesh/growth/growth_field_smoother.hpp include/boundary_mesh/growth/growth_field_smoothing_error.hpp src/growth/growth_field_smoother.cpp tests/unit/growth/growth_field_smoother_test.cpp
git commit -m "feat: integrate skewness refinement after height smoothing"
```

---

### Task 4: Stepper Wiring and Growth Diagnostics

**Files:**
- Modify: `include/boundary_mesh/growth/regular_layer_growth.hpp`
- Modify: `src/growth/regular_layer_stepper.cpp`
- Modify: `src/growth/regular_layer_generator.cpp`
- Modify: `tests/unit/growth/regular_layer_stepper_test.cpp`
- Modify: `tests/unit/growth/regular_layer_growth_types_test.cpp`

**Interfaces:**
- Consumes: `RegularLayerGrowthOptions::field_smoothing` and `SmoothedGrowthFields::diagnostics`.
- Produces: `LayerStepResult::smoothing_diagnostics` and aggregated `RegularLayerGrowthResult::smoothing_diagnostics`.

- [ ] **Step 1: Write stepper disabled/enabled integration checks**

Extend `regular_layer_stepper_test.cpp` with a skewed fixture and two runs. Assert that both results retain the existing final validity/filter semantics while diagnostics differ:

```cpp
RegularLayerGrowthOptions disabled_options;
disabled_options.field_smoothing.skewness.enabled = false;
const auto disabled_step = RegularLayerStepper{}.step(
    front, profiles, constraints, disabled_options);

RegularLayerGrowthOptions enabled_options = disabled_options;
enabled_options.field_smoothing.skewness.enabled = true;
enabled_options.field_smoothing.skewness.activation_skewness = Scalar{0};
const auto enabled_step = RegularLayerStepper{}.step(
    front, profiles, constraints, enabled_options);

if (!disabled_step.hasValue() || !enabled_step.hasValue()) return 30;
if (disabled_step.value().smoothing_diagnostics.updated_vertices != 0 ||
    enabled_step.value().smoothing_diagnostics.activated_vertices == 0)
{
    return 31;
}
```

Retain the existing strict `maximum_skewness = 0.05` assertion to prove the optimizer does not bypass final rejection.

- [ ] **Step 2: Build and verify the missing diagnostics failure**

Run:

```powershell
cmake --build build --config Release --target boundary_mesh_regular_layer_stepper_test
```

Expected: compilation fails because `LayerStepResult::smoothing_diagnostics` does not exist.

- [ ] **Step 3: Add step diagnostics and pass options**

Add the field to `LayerStepResult`:

```cpp
GrowthFieldSmoothingDiagnostics smoothing_diagnostics;
```

Pass options at the existing smoother call:

```cpp
const auto field_result = GrowthFieldSmoother{}.smooth(
    eligible.front,
    front_evaluation.value(),
    adjacency_result.value(),
    direction_result.value(),
    reference_heights,
    provisional_heights,
    options.field_smoothing);
```

After success, assign:

```cpp
output.smoothing_diagnostics = field_result.value().diagnostics;
```

- [ ] **Step 4: Aggregate diagnostics in the generator**

At every accepted `LayerStepResult`, accumulate counts and maxima:

```cpp
output.smoothing_diagnostics.activated_vertices +=
    quality_step.value().smoothing_diagnostics.activated_vertices;
output.smoothing_diagnostics.updated_vertices +=
    quality_step.value().smoothing_diagnostics.updated_vertices;
output.smoothing_diagnostics.maximum_skewness_before = std::max(
    output.smoothing_diagnostics.maximum_skewness_before,
    quality_step.value().smoothing_diagnostics.maximum_skewness_before);
output.smoothing_diagnostics.maximum_skewness_after = std::max(
    output.smoothing_diagnostics.maximum_skewness_after,
    quality_step.value().smoothing_diagnostics.maximum_skewness_after);
```

Place accumulation exactly once per generated step, before later collision or termination filtering can create duplicate paths.

- [ ] **Step 5: Run stepper, generator, and type regressions**

Run:

```powershell
cmake --build build --config Release --target boundary_mesh_regular_layer_stepper_test boundary_mesh_regular_layer_generator_test boundary_mesh_regular_layer_growth_types_test
ctest --test-dir build -C Release -R "boundary_mesh_regular_layer_(stepper|generator|growth_types)_test" --output-on-failure
```

Expected: all selected tests pass; the strict skewness filter still reports `SkewnessExceeded` where it did before.

- [ ] **Step 6: Commit the pipeline wiring**

```powershell
git add -- include/boundary_mesh/growth/regular_layer_growth.hpp src/growth/regular_layer_stepper.cpp src/growth/regular_layer_generator.cpp tests/unit/growth/regular_layer_stepper_test.cpp tests/unit/growth/regular_layer_growth_types_test.cpp
git commit -m "feat: report skewness smoothing diagnostics"
```

---

### Task 5: Benchmark Reporting and Full Verification

**Files:**
- Modify: `benchmarks/cgns_pipeline_benchmark.cpp`
- Modify: `README.md`

**Interfaces:**
- Consumes: aggregate `RegularLayerGrowthResult::smoothing_diagnostics`.
- Produces: human-readable benchmark/configuration reporting and user documentation.

- [ ] **Step 1: Add benchmark diagnostic output**

After the existing stop-count output, print:

```cpp
const auto &smoothing = growth.value().smoothing_diagnostics;
std::cout << "smoothing_activated_vertices="
          << smoothing.activated_vertices << '\n'
          << "smoothing_updated_vertices="
          << smoothing.updated_vertices << '\n'
          << "smoothing_maximum_skewness_before="
          << smoothing.maximum_skewness_before << '\n'
          << "smoothing_maximum_skewness_after="
          << smoothing.maximum_skewness_after << '\n';
```

Run the benchmark once with `options.field_smoothing.skewness.enabled = false`
and once with it enabled, using the existing wall-clock measurement path. Label both modes in output so runtime and stop counts can be compared directly.

- [ ] **Step 2: Document behavior and defaults**

Add a README subsection near the existing `maximum_skewness` documentation stating:

```text
Skewness-aware normal smoothing is a pre-quality optimization. Candidate
vertices activate when their incident maximum equiangular skewness exceeds
0.8. The search changes direction only; layer heights and the final
--maximum-skewness acceptance rule are unchanged. Disable the optimization for
baseline performance comparisons.
```

Document the 5/2 degree, six-azimuth, two-level defaults and the four diagnostic names.

- [ ] **Step 3: Build all affected targets**

Run:

```powershell
cmake -S . -B build
cmake --build build --config Release
```

Expected: complete Release build succeeds with no compiler errors.

- [ ] **Step 4: Run the complete test suite**

Run:

```powershell
ctest --test-dir build -C Release --output-on-failure
```

Expected: 100% of tests pass, including the new refiner test.

- [ ] **Step 5: Run the representative A/B benchmark**

Run the configured benchmark command for the repository's representative CGNS input. Capture both enabled and disabled sections. Acceptance checks:

```text
enabled smoothing_maximum_skewness_after <= enabled smoothing_maximum_skewness_before
enabled stop_skewness_exceeded <= disabled stop_skewness_exceeded, unless another
existing validity/collision constraint becomes the earlier stop reason
height invariance remains covered by the exact unit assertion
both elapsed times are recorded
```

If no representative CGNS input is available locally, run the benchmark target's help/input validation path, report that the performance measurement is pending external input, and do not claim a runtime improvement.

- [ ] **Step 6: Inspect the final diff and configuration parity**

Run:

```powershell
git diff --check
git status --short
git diff HEAD~4 -- include/boundary_mesh/growth src/growth tests/unit/growth benchmarks/cgns_pipeline_benchmark.cpp README.md CMakeLists.txt tests/CMakeLists.txt
```

Expected: no whitespace errors; only scoped files appear; `.superpowers/` remains untouched.

- [ ] **Step 7: Commit benchmark and documentation**

```powershell
git add -- benchmarks/cgns_pipeline_benchmark.cpp README.md
git commit -m "docs: report skewness smoothing performance"
```

- [ ] **Step 8: Record final evidence for review**

Record in the handoff:

```text
branch and final commit
Release build result
full CTest passed/total count
disabled/enabled maximum skewness
disabled/enabled skewness stop count
disabled/enabled elapsed time
any representative-input limitation
```

Do not merge the branch. Leave it available for user evaluation.
