# Iterative Sliding Normal Constraint Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Constrain Symmetry/Internal directions before smoothing and after every synchronous smoothing round so constrained directions drive subsequent smoothing and height prediction.

**Architecture:** `RegularLayerStepper` builds `SlidingConstraints` before invoking `GrowthFieldSmoother`. The smoother accepts an optional immutable constraint pointer, applies it to the initial buffer, every tentative synchronous buffer, and skewness-refined output, while preserving the existing no-constraint call behavior and final position projection.

**Tech Stack:** C++17, Eigen, CMake/CTest, existing sliding-surface and growth-field modules.

## Global Constraints

- Do not rebuild sliding surfaces inside `GrowthFieldSmoother`.
- Constrain at the current front position.
- Synchronous rounds must never expose unconstrained tentative directions to the next round.
- Constraint failures remain typed and do not fall back silently.
- Keep the final stepper direction constraint and final position projection.
- Use test-first red-green development.

---

### Task 1: Constraint-aware growth-field smoothing

**Files:**
- Modify: `include/boundary_mesh/growth/growth_field_smoother.hpp`
- Modify: `include/boundary_mesh/growth/growth_field_smoothing_error.hpp`
- Modify: `src/growth/growth_field_smoother.cpp`
- Modify: `tests/unit/growth/growth_field_smoother_test.cpp`

**Interfaces:**
- Consumes: `SlidingConstraints::constrainDirection(std::size_t, const Point3 &, const Vector3 &) const`.
- Produces: optional final parameter `const SlidingConstraints *sliding_constraints = nullptr` on `GrowthFieldSmoother::smooth(...)` and `SlidingGrowthFieldConstraintFailure { std::size_t front_vertex_index; VertexId source_vertex_id; std::uint32_t layer; GrowthDirectionError cause; }` in `GrowthFieldSmoothingError`.

- [ ] **Step 1: Write a failing curved-constraint test**

In `growth_field_smoother_test.cpp`, build a `SlidingSurfaceSet` containing a curved indexed surface and assign its region to selected fan vertices. Build `SlidingConstraints` with `SlidingConstraintBuilder`. Call the wished-for smoother overload with the constraint pointer. Assert every constrained result is tangent by verifying that a second `constrainDirection()` changes it by at most `1e-12`.

Also compare a neighboring unconstrained vertex against a run where only the final returned direction is constrained; assert the iterative result differs, proving constrained intermediate directions influenced the next round. Keep the existing no-constraint call and equality expectations unchanged.

- [ ] **Step 2: Verify RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_growth_field_smoother_test -j 8
```

Expected: compile failure because `smooth(...)` does not accept a `SlidingConstraints` provider.

- [ ] **Step 3: Add the typed error and optional API**

Add the required include for `GrowthDirectionError` and define:

```cpp
struct SlidingGrowthFieldConstraintFailure
{
    std::size_t front_vertex_index{};
    VertexId source_vertex_id{};
    std::uint32_t layer{};
    GrowthDirectionError cause;
};
```

Append it to `GrowthFieldSmoothingError`. Forward-declare or include `SlidingConstraints` in the smoother header and add the optional pointer after `options`.

- [ ] **Step 4: Apply constraints at every synchronization boundary**

Add an internal helper returning `Result<std::vector<Vector3>, GrowthFieldSmoothingError>`:

```cpp
constrainDirections(
    const GrowthFront &front,
    const SlidingConstraints *constraints,
    std::vector<Vector3> directions);
```

For a null pointer, return the buffer unchanged. Otherwise constrain each direction using the corresponding current front position and wrap any error in `SlidingGrowthFieldConstraintFailure`.

Invoke it:

1. immediately after `validateAndInitialize()`;
2. after every complete tentative `next` buffer and before `current.swap(next)`;
3. after `refineDirectionsForSkewness()` and before constructing `SmoothedGrowthFields`.

Do not constrain one vertex in-place while other tentative vertices are still being calculated.

- [ ] **Step 5: Verify GREEN and regressions**

```powershell
cmake --build build --config Debug --target boundary_mesh_growth_field_smoother_test boundary_mesh_regular_layer_stepper_test boundary_mesh_sliding_constraints_test -j 8
ctest --test-dir build -C Debug -R 'boundary_mesh_(growth_field_smoother|regular_layer_stepper|sliding_constraints)_test' --output-on-failure
```

Expected: all selected tests pass.

---

### Task 2: Stepper orchestration and full verification

**Files:**
- Modify: `src/growth/regular_layer_stepper.cpp:340-420`
- Modify: `tests/unit/growth/regular_layer_stepper_test.cpp`

**Interfaces:**
- Consumes: Task 1 optional smoother constraint provider.
- Produces: stepper order `build constraints -> smooth with constraints -> defensive final direction constraint -> final position projection`; no public stepper signature changes.

- [ ] **Step 1: Write a failing stepper integration assertion**

Extend the existing curved-sliding stepper fixture so a vertex receives a raw direction with a substantial normal component. Assert the resulting candidate direction is tangent and that its neighbor/candidate position matches constraint-aware smoothing rather than post-only projection.

- [ ] **Step 2: Verify RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_regular_layer_stepper_test -j 8
ctest --test-dir build -C Debug -R '^boundary_mesh_regular_layer_stepper_test$' --output-on-failure
```

Expected: the new iterative-influence assertion fails with the current post-only order.

- [ ] **Step 3: Move constraint construction before smoothing**

In `regular_layer_stepper.cpp`, build `SlidingConstraints` immediately after raw directions/heights are available and before `GrowthFieldSmoother::smooth()`. Pass `&sliding.value()` as the final smoother argument. Preserve existing typed builder failure handling, final `constrainDirection()`, and `projectPosition()`.

- [ ] **Step 4: Verify targeted GREEN**

Run the Task 2 Step 2 commands. Expected: the stepper test passes.

- [ ] **Step 5: Run full builds and tests**

```powershell
cmake --build build --config Debug -j 8
cmake --build build --config Release -j 8
ctest --test-dir build -C Debug --output-on-failure
ctest --test-dir build -C Release --output-on-failure
```

Expected: 75/75 tests pass in each configuration.

- [ ] **Step 6: Run plane and curve**

Run both real cases with `--first-height 0.1 --growth-ratio 1.2 --layer-count 20 --maximum-skewness 1 --isotropic-height 1` and temporary output prefixes.

Expected plane: first layer remains `11302`. Expected curve: first two layers remain `4746`; compare locally inverted/collision/neighbor stops against `69/199/4235` and report exact new totals.

- [ ] **Step 7: Commit and inspect**

```powershell
git add -- include/boundary_mesh/growth/growth_field_smoother.hpp include/boundary_mesh/growth/growth_field_smoothing_error.hpp src/growth/growth_field_smoother.cpp src/growth/regular_layer_stepper.cpp tests/unit/growth/growth_field_smoother_test.cpp tests/unit/growth/regular_layer_stepper_test.cpp
git commit -m "fix: constrain sliding normals during smoothing"
git diff --check
git status --short
```
