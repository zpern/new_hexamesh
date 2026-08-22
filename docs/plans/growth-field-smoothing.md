# Growth Field Smoothing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace angle-weighted extrusion directions with the approved multi-candidate normal selector, smooth every active front's directions with the reference numerical strategy, and smooth per-vertex layer heights with inherited actual heights.

**Architecture:** Introduce a state-bearing `GrowthFrontVertex`, rebuild deterministic one-ring adjacency from each compact active front, keep raw direction selection in `computeGrowthDirections()`, and isolate normal/height smoothing in `GrowthFieldSmoother`. `RegularLayerStepper` orchestrates these pure components before candidate quality evaluation; collision filtering continues to compact the state-bearing front without owning smoothing logic.

**Tech Stack:** C++17, Eigen3, CMake 3.20+, MSVC 17.14, CTest, existing `Result<T, E>` error model.

## Global Constraints

- Triangle and Quad normals follow the existing input Wall right-hand ordering; do not reverse `FrontEvaluation::unit_normal`.
- Every active Triangle or Quad contributes exactly one unit face normal.
- Only the current compact active front participates in direction and height smoothing.
- Normal-smoothing constants remain internal: 25°, 30°, 10°, 1°, 1.7, 15, 2/3, 0.7, 0.9985, 40 bisection steps, four full rounds, and 100 maximum rounds.
- Height smoothing runs once per layer with logistic coefficient `0.5` and clamps each result to `[0.5, 1.5]` times its base height.
- Layer 1 base height is `first_height`; later base heights are the previous accepted `actual_height * growth_ratio`.
- Public fields and error alternatives receive Chinese `//` comments.
- No backward-compatible parallel `GrowthFront` vertex arrays are retained.
- `.superpowers/` is never staged or committed.
- Each task follows RED, GREEN, focused regression, and an independent commit.

---

## File Structure

**New public headers**

- `include/boundary_mesh/growth/front_adjacency.hpp`: immutable per-layer vertex-neighbor and incident-face tables plus builder declaration.
- `include/boundary_mesh/growth/front_adjacency_error.hpp`: adjacency validation errors.
- `include/boundary_mesh/growth/growth_field_smoother.hpp`: smoothed field result and public smoother interface.
- `include/boundary_mesh/growth/growth_field_smoothing_error.hpp`: direction/height smoothing errors.

**New implementation files**

- `src/growth/front_adjacency.cpp`: deterministic active-front adjacency construction.
- `src/growth/growth_field_smoother.cpp`: reference numerical normal iteration and synchronous height smoothing.

**Primary modified files**

- `include/boundary_mesh/growth/growth_front.hpp`: add `GrowthFrontVertex` and replace parallel point arrays.
- `include/boundary_mesh/growth/growth_direction.hpp`: expose per-vertex selected direction, visibility, and complex-corner flag.
- `include/boundary_mesh/growth/growth_direction_error.hpp`: add candidate-construction diagnostics.
- `src/growth/growth_direction.cpp`: implement lower/simple/naive/center/geometry candidate chain.
- `src/growth/growth_front_builder.cpp`: initialize state-bearing layer-0 vertices.
- `src/growth/front_evaluator.cpp`: read positions and mappings from `GrowthFrontVertex`.
- `src/growth/regular_layer_stepper.cpp`: build adjacency, select/smooth fields, inherit heights, and preserve state through compaction.
- `src/growth/regular_layer_generator.cpp`: write only front positions into `VolumeMesh` and preserve state through collision stages.
- `src/growth/symmetry_constraint_builder.cpp`, `src/growth/layer_collision_checker.cpp`, `src/growth/face_layer_constraint.cpp`, `src/growth/termination_propagator.cpp`: migrate front point access without changing their algorithms.
- `CMakeLists.txt`, `tests/CMakeLists.txt`: register the two new sources and focused tests.

**New tests**

- `tests/unit/growth/front_adjacency_test.cpp`
- `tests/unit/growth/growth_field_smoother_test.cpp`

**Migrated tests**

- All current tests that construct or inspect `GrowthFront` listed by `Select-String -Pattern 'GrowthFront|source_vertex_ids|vertex_boundaries'`.

---

### Task 1: Migrate GrowthFront to state-bearing vertices

**Files:**
- Modify: `include/boundary_mesh/growth/growth_front.hpp`
- Modify: `src/growth/growth_front_builder.cpp`
- Modify: `src/growth/front_evaluator.cpp`
- Modify: `src/growth/symmetry_constraint_builder.cpp`
- Modify: `src/growth/face_layer_constraint.cpp`
- Modify: `src/growth/termination_propagator.cpp`
- Modify: `src/growth/layer_collision_checker.cpp`
- Modify: `src/growth/regular_layer_generator.cpp`
- Modify: `tests/unit/growth/growth_front_test.cpp`
- Modify: `tests/unit/growth/front_evaluator_test.cpp`
- Modify: all other tests that directly access the removed parallel arrays

**Interfaces:**
- Produces: `GrowthFrontVertex` and `GrowthFront::vertices` as `std::vector<GrowthFrontVertex>`.
- Preserves: `GrowthFront::faces` and `GrowthFront::source_face_ids`.

- [ ] **Step 1: Change the GrowthFront type test first**

Update `tests/unit/growth/growth_front_test.cpp` to require state on each point:

```cpp
const GrowthFrontVertex &vertex = front.value().vertices[0];
if ((vertex.position - mesh.vertices[vertex.source_vertex_id]).norm() != 0 ||
    (vertex.root_position - vertex.position).norm() != 0 ||
    vertex.direction.norm() != 0 ||
    vertex.actual_height != Scalar{0} ||
    vertex.visibility_cosine != Scalar{1} ||
    vertex.complex_corner)
{
    return 20;
}
```

- [ ] **Step 2: Build the focused test and observe RED**

Run:

```powershell
cmake --build build --config Debug --target boundary_mesh_growth_front_test
```

Expected: compile failure because `GrowthFrontVertex` and its fields do not exist.

- [ ] **Step 3: Add the new public vertex structure**

Implement in `growth_front.hpp`:

```cpp
struct GrowthFrontVertex
{
    Point3 position{Point3::Zero()}; // 当前活动层坐标
    Point3 root_position{Point3::Zero()}; // 第 0 层 Wall 源点坐标
    VertexId source_vertex_id{}; // 输入 Wall 顶点编号
    FrontVertexBoundary boundary; // 当前点继承的边界约束
    Vector3 direction{Vector3::Zero()}; // 最近一次平滑后的单位生长方向
    Scalar actual_height{}; // 最近一层真正采用的推出步长
    Scalar visibility_cosine{1}; // 原始法向对最不利关联面的点积
    bool complex_corner{}; // 是否属于低可见性复杂角点
};

struct GrowthFront
{
    std::uint32_t layer{}; // 当前层号，第 0 层对应输入 Wall
    std::vector<GrowthFrontVertex> vertices; // 当前紧凑活动点及生成状态
    std::vector<SurfaceFace> faces; // 顶点编号索引当前 vertices
    std::vector<SurfaceFaceId> source_face_ids; // 局部面到源 Wall 面的映射
};
```

Initialize every layer-0 vertex with identical `position` and `root_position`, zero direction/height, visibility one, and `complex_corner=false`.

- [ ] **Step 4: Migrate consumers mechanically and preserve behavior**

Use these exact access mappings:

```cpp
front.vertices[index]
    -> front.vertices[index].position
front.source_vertex_ids[index]
    -> front.vertices[index].source_vertex_id
front.vertex_boundaries[index]
    -> front.vertices[index].boundary
```

When copying or compacting a point, copy the complete `GrowthFrontVertex`, not selected fields. When writing `VolumeMesh`, append only `.position`.

- [ ] **Step 5: Run all pre-existing tests**

Run:

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

Expected: all existing tests pass with unchanged algorithms.

- [ ] **Step 6: Commit the migration**

```powershell
git add include/boundary_mesh/growth/growth_front.hpp src tests CMakeLists.txt
git diff --cached --check
git commit -m "refactor: store growth state on front vertices"
```

---

### Task 2: Build deterministic active-front adjacency

**Files:**
- Create: `include/boundary_mesh/growth/front_adjacency.hpp`
- Create: `include/boundary_mesh/growth/front_adjacency_error.hpp`
- Create: `src/growth/front_adjacency.cpp`
- Create: `tests/unit/growth/front_adjacency_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `const GrowthFront &`.
- Produces: `Result<FrontAdjacency, FrontAdjacencyError> buildFrontAdjacency(const GrowthFront &front)`.

- [ ] **Step 1: Add a failing mixed Triangle/Quad adjacency test**

The test must build a compact front whose Triangle and Quad share an edge and assert:

```cpp
const auto result = buildFrontAdjacency(front);
if (!result.hasValue()) return 1;
if (result.value().vertex_neighbors[0] !=
        std::vector<std::size_t>{1, 2, 3} ||
    result.value().vertex_incident_faces[0] !=
        std::vector<std::size_t>{0, 1})
{
    return 2;
}
```

Add a second front containing only one of the two faces and verify that removed-face-only neighbors disappear.

- [ ] **Step 2: Register and run the test to observe RED**

Run:

```powershell
cmake --build build --config Debug --target boundary_mesh_front_adjacency_test
```

Expected: missing header or unresolved target until the interface is added.

- [ ] **Step 3: Implement deterministic construction**

For every face edge `(a,b)`, append `b` to `a`, append `a` to `b`, and append the face index once to every face vertex. Validate all point references before indexing. Sort and erase duplicates in every neighbor and incident-face row.

Use errors carrying the current layer, face index, source face, and invalid vertex id:

```cpp
struct InvalidFrontAdjacencyReference
{
    std::uint32_t layer{}; // 当前活动层号
    std::size_t front_face_index{}; // 非法活动面下标
    SurfaceFaceId source_face_id{}; // 对应输入 Wall 面编号
    VertexId vertex_id{}; // 越界的紧凑顶点编号
};
```

- [ ] **Step 4: Run focused and existing growth tests**

```powershell
cmake --build build --config Debug --target boundary_mesh_front_adjacency_test
ctest --test-dir build -C Debug -R "boundary_mesh_(front_adjacency|growth_front|front_evaluator)_test" --output-on-failure
```

Expected: all selected tests pass.

- [ ] **Step 5: Commit adjacency**

```powershell
git add CMakeLists.txt tests/CMakeLists.txt include/boundary_mesh/growth/front_adjacency*.hpp src/growth/front_adjacency.cpp tests/unit/growth/front_adjacency_test.cpp
git diff --cached --check
git commit -m "feat: build active front adjacency"
```

---

### Task 3: Implement multi-candidate raw direction selection

**Files:**
- Modify: `include/boundary_mesh/growth/growth_direction.hpp`
- Modify: `include/boundary_mesh/growth/growth_direction_error.hpp`
- Rewrite: `src/growth/growth_direction.cpp`
- Rewrite: `tests/unit/growth/growth_direction_test.cpp`
- Modify: `tests/integration/growth_front_pipeline_test.cpp`

**Interfaces:**
- Consumes: `GrowthFront`, `FrontEvaluation`, and `FrontAdjacency`.
- Produces: `GrowthDirections` containing one `GrowthDirectionSelection` per active point.

- [ ] **Step 1: Define tests for candidate priority and visibility**

Require this result shape:

```cpp
struct GrowthDirectionSelection
{
    Vector3 value{Vector3::Zero()}; // 选中的原始单位方向
    Scalar visibility_cosine{}; // 对全部活动关联面的最小点积
    bool complex_corner{}; // 可见性是否低于 cos(30°)
};

struct GrowthDirections
{
    std::uint32_t layer{}; // 方向所属层号
    std::vector<GrowthDirectionSelection> vertices; // 与前沿活动点一一对应
};
```

Add fixtures that verify:

- a planar mixed patch selects the simple average;
- a nonzero previous `GrowthFrontVertex::direction` is selected when its minimum dot exceeds `cos(30°)`;
- a 3:1 duplicated-direction corner uses 25° grouped `NaiveNormal` without count dominance;
- generalized Quad center construction uses only the previous and next boundary vertices;
- a low-visibility fallback returns the best finite candidate and sets `complex_corner`.

- [ ] **Step 2: Run the direction target and observe RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_growth_direction_test
```

Expected: compile failure because the selector still exposes angle-weighted `values` and has no adjacency input.

- [ ] **Step 3: Implement visibility and fast candidates**

Use:

```cpp
Scalar visibility(
    const Vector3 &candidate,
    const std::vector<std::size_t> &incident_faces,
    const FrontEvaluation &evaluation)
{
    Scalar result = Scalar{1};
    for (const std::size_t face_index : incident_faces)
        result = std::min(result,
            candidate.dot(evaluation.faces[face_index].value.unit_normal));
    return result;
}
```

Implement previous, simple, and 25° grouped naive candidates with stable face order and strict threshold comparisons.

- [ ] **Step 4: Implement CenterNormal and GeometryNormal**

Port the reference center and circle-center helpers as private functions. For either face type, locate the current vertex in its ordered `vertex_ids`, then use `(local-1) mod count` and `(local+1) mod count` as its two real boundary neighbors. Do not use a Quad diagonal.

Try candidates in this exact order and stop on the first strict threshold success:

```text
previous cos(30°), simple cos(30°), naive cos(30°),
center cos(10°), geometry cos(1°)
```

If none passes, return the valid candidate with maximum visibility.

- [ ] **Step 5: Run focused and pipeline tests**

```powershell
cmake --build build --config Debug --target boundary_mesh_growth_direction_test boundary_mesh_growth_front_pipeline_test
ctest --test-dir build -C Debug -R "boundary_mesh_(growth_direction|growth_front_pipeline)_test" --output-on-failure
```

Expected: both tests pass.

- [ ] **Step 6: Commit raw direction selection**

```powershell
git add include/boundary_mesh/growth/growth_direction*.hpp src/growth/growth_direction.cpp tests/unit/growth/growth_direction_test.cpp tests/integration/growth_front_pipeline_test.cpp
git diff --cached --check
git commit -m "feat: select visible growth directions"
```

---

### Task 4: Add reference numerical normal smoothing

**Files:**
- Create: `include/boundary_mesh/growth/growth_field_smoother.hpp`
- Create: `include/boundary_mesh/growth/growth_field_smoothing_error.hpp`
- Create: `src/growth/growth_field_smoother.cpp`
- Create: `tests/unit/growth/growth_field_smoother_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Produces: `GrowthFieldSmoother::smooth(...)` and `SmoothedGrowthFields`.
- Consumes: the selected raw directions from Task 3 and base heights supplied by Task 5.

- [ ] **Step 1: Add failing deterministic numerical tests**

Declare the target API in the test:

```cpp
struct SmoothedGrowthFields
{
    std::uint32_t layer{}; // 平滑结果所属的当前活动层
    std::vector<Vector3> directions; // 每个活动点的平滑单位方向
    std::vector<Scalar> actual_heights; // 每个活动点的本层实际步长
};

const auto result = GrowthFieldSmoother{}.smooth(
    front, evaluation, adjacency, raw_directions, base_heights);
```

Use a fixed nonplanar four-point fan and assert component values to `1e-12`. Repeat with permuted face insertion order that leaves point numbering fixed and require identical output. Add invalid zero-length-neighbor and nonfinite-input cases.

- [ ] **Step 2: Run the focused test and observe RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_growth_field_smoother_test
```

Expected: missing header or target.

- [ ] **Step 3: Implement one synchronous normal round**

Implement reference weights:

```text
distance_ratio   = average_squared_distance / squared_distance
alignment        = abs(unit_edge dot neighbor_direction)
beita_scale      = neighbor_visibility * 2 / pi
influence        = pow(distance_ratio, 3 + 2 * alignment)
                   / (beita_scale * beita_scale)
weight           = 0.5 + influence / (1 + influence)
```

Normalize the weighted sum, add and renormalize the previous-layer direction when `front.layer > 0`, and use the approved strength:

```text
ratio = |position - root_position| / minimum_incident_face_edge
strength = 2 + (pow(1.7, ratio) - 1) * 15
```

- [ ] **Step 4: Implement deviation, visibility, and dynamic iteration**

Apply 40 bisection steps for the `2/3` deviation cap. Preserve the reference dot threshold `35 * 0.8 * pi / 180`, perform at most 11 `candidate + 0.7 * original` retries, and restore the round's original direction if retries fail.

Use two direction buffers. Process all points in rounds 0 through 3. Afterwards process only points whose previous-round dot is below `0.9985` and their one-ring neighbors. Stop at 100 rounds or when the tracked active count equals the count ten records earlier after more than 12 records.

- [ ] **Step 5: Keep height output temporarily equal to base height**

Until Task 5 adds the synchronous height formula, validate every base height and copy it to `actual_heights`. This keeps the component callable and independently testable.

- [ ] **Step 6: Run focused tests**

```powershell
cmake --build build --config Debug --target boundary_mesh_growth_field_smoother_test
ctest --test-dir build -C Debug -R "boundary_mesh_(growth_field_smoother|growth_direction)_test" --output-on-failure
```

Expected: all selected tests pass.

- [ ] **Step 7: Commit normal smoothing**

```powershell
git add CMakeLists.txt tests/CMakeLists.txt include/boundary_mesh/growth/growth_field*.hpp src/growth/growth_field_smoother.cpp tests/unit/growth/growth_field_smoother_test.cpp
git diff --cached --check
git commit -m "feat: smooth active front directions"
```

---

### Task 5: Add inherited synchronous height smoothing to the layer stepper

**Files:**
- Modify: `src/growth/growth_field_smoother.cpp`
- Modify: `tests/unit/growth/growth_field_smoother_test.cpp`
- Modify: `src/growth/regular_layer_stepper.cpp`
- Modify: `include/boundary_mesh/growth/regular_layer_growth_error.hpp`
- Rewrite: `tests/unit/growth/regular_layer_stepper_test.cpp`
- Modify: `tests/integration/regular_layer_growth_pipeline_test.cpp`

**Interfaces:**
- Consumes: `GrowthFrontVertex::actual_height`, `VertexGrowthProfile`, and smoothed directions.
- Produces: candidate and next-front vertices carrying the current layer's direction and actual height.

- [ ] **Step 1: Add failing height inheritance tests**

For a planar front with `first_height=0.1` and `growth_ratio=2`, verify layer 1 begins from `0.1`. Set accepted layer-1 vertices to known actual heights such as `0.08`, then verify layer 2 bases are `0.16`, not `0.2`.

For a nonuniform neighbor fixture, compute the expected value with:

```cpp
const Scalar predicted =
    ((neighbor_position + neighbor_base * neighbor_direction)
        - current_position).dot(current_direction);
const Scalar relative = (predicted - current_base) / current_base;
const Scalar correction =
    Scalar{1} / (Scalar{1} + std::exp(Scalar{-0.5} * relative))
    - Scalar{0.5};
const Scalar expected = std::clamp(
    current_base * (Scalar{1} + correction),
    Scalar{0.5} * current_base,
    Scalar{1.5} * current_base);
```

- [ ] **Step 2: Run focused tests and observe RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_growth_field_smoother_test boundary_mesh_regular_layer_stepper_test
```

Expected: assertions fail because profiles still use the closed-form height and the smoother still copies base heights.

- [ ] **Step 3: Implement stable one-round height smoothing**

First build every predicted neighbor position from the immutable base-height buffer and final smoothed direction buffer. Then compute every actual height into a separate output buffer. Evaluate the logistic in a branch-stable form for positive and negative exponents, validate finite positive results, and clamp to `[0.5,1.5] * base`.

- [ ] **Step 4: Orchestrate fields in RegularLayerStepper**

After compacting eligible faces:

```text
evaluate front -> build adjacency -> select raw directions
-> compute base heights -> smooth fields -> pre-extrude
```

Use `first_height` when `target_layer == 1`; otherwise require a positive finite `current_vertex.actual_height` and multiply it by `growth_ratio`.

Before moving each candidate point, assign:

```cpp
candidate_vertex.direction = fields.directions[index];
candidate_vertex.actual_height = fields.actual_heights[index];
candidate_vertex.visibility_cosine = raw.vertices[index].visibility_cosine;
candidate_vertex.complex_corner = raw.vertices[index].complex_corner;
candidate_vertex.position +=
    candidate_vertex.actual_height * candidate_vertex.direction;
```

- [ ] **Step 5: Preserve complete point state through quality and collision compaction**

Ensure `compactFaces`, `TerminationPropagator::filterCandidates`, `LayerCollisionChecker`, and generator commits copy entire `GrowthFrontVertex` values. A point shared by accepted and rejected faces persists if at least one accepted face uses it.

- [ ] **Step 6: Run growth regression tests**

```powershell
cmake --build build --config Debug --target boundary_mesh_regular_layer_stepper_test boundary_mesh_regular_layer_growth_pipeline_test boundary_mesh_collision_growth_pipeline_test boundary_mesh_layer_coordination_pipeline_test
ctest --test-dir build -C Debug -R "boundary_mesh_(regular_layer|collision_growth|layer_coordination).*test" --output-on-failure
```

Expected: all selected tests pass with updated deterministic coordinates and unchanged stop semantics.

- [ ] **Step 7: Commit height smoothing and integration**

```powershell
git add include/boundary_mesh/growth/regular_layer_growth_error.hpp src/growth/growth_field_smoother.cpp src/growth/regular_layer_stepper.cpp src/growth/regular_layer_generator.cpp src/growth/layer_collision_checker.cpp src/growth/termination_propagator.cpp tests
git diff --cached --check
git commit -m "feat: smooth inherited layer heights"
```

---

### Task 6: Full regression and real-case verification

**Files:**
- Modify only if verification reveals a defect in the files owned by Tasks 1–5.

**Interfaces:**
- Verifies the complete public pipeline; produces no new API.

- [ ] **Step 1: Run Debug full regression**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

Expected: 100% tests pass.

- [ ] **Step 2: Run Release full regression**

```powershell
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Expected: 100% tests pass.

- [ ] **Step 3: Verify IO-disabled configuration**

```powershell
cmake -S . -B build-no-io -DBOUNDARY_MESH_ENABLE_CGNS_IO=OFF
cmake --build build-no-io --config Debug
ctest --test-dir build-no-io -C Debug --output-on-failure
```

Expected: configure, build, and all registered non-IO tests pass.

- [ ] **Step 4: Run the approved real case**

Use the automatically derived `2dot5_cf.bc.txt` beside the CGNS input. Run the same one-layer command used by the current collision regression and overwrite the ASCII VTK output:

```powershell
& .\build\Release\boundary_mesh_cli.exe `
  --input "C:\Users\zpern\Desktop\todo\九院项目质量对标\test_case\2dot5_cf\2dot5_cf.cgns" `
  --first-height 0.1 `
  --growth-ratio 1.0 `
  --layer-count 1 `
  --maximum-skewness 0.95 `
  --max-neighbor-layer-difference 1 `
  --output-prefix ".\build\real_case\2dot5_cf_smoothed"
```

Record:

```text
volume_cells
stop_collision
growth_seconds
peak_working_set_bytes
```

Expected: the command succeeds, output points are finite, VTK contains volume cells and the farfield boundary, and stop diagnostics are internally consistent.

- [ ] **Step 5: Inspect repository cleanliness and commit verification-only fixes**

```powershell
git diff --check
git status --short --branch
```

Expected: only `.superpowers/` remains untracked. If a verification defect required code changes, stage only the owned files and commit with `fix: stabilize growth field smoothing` after rerunning the affected regression.
