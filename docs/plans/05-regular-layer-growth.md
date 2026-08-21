# Prism/Hexa Regular Layer Growth Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. The project owner has explicitly requested inline execution without subagents. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement transaction-safe, per-vertex-parameterized regular boundary-layer growth that generates Prism cells from Triangle faces and Hexa cells from Quad faces, stopping only the source face whose candidate fails stage 04 quality evaluation.

**Architecture:** A `GrowthProfileBuilder` validates and indexes external per-source-vertex parameters. A stateless `RegularLayerStepper` prepares exactly one accepted next front without modifying the final mesh. A `RegularLayerGenerator` repeatedly calls the stepper, atomically commits accepted vertices and cells, and returns the volume mesh plus explicit layer mappings and growth records.

**Tech Stack:** C++17, Eigen `Point3`/`Vector3`, CMake 3.20+, Visual Studio/MSBuild, CTest, existing `Result<T, E>`, `FrontEvaluator`, `computeGrowthDirections`, `evaluatePrism`, and `evaluateHexa`.

## Global Constraints

- Keep stage 05 on branch `feature/regular-layer-growth` and do not add the untracked `.superpowers/` directory.
- Use `apply_patch` for source edits.
- Add Chinese `//` comments to every new public struct field and enum value.
- Follow the public Prism order `0,1,2,3,4,5` and Hexa order `0,1,2,3,4,5,6,7` already defined in `mesh_volume.hpp`.
- Always grow along the current input face winding's right-hand direction; do not add a reverse-growth option.
- Do not apply symmetry direction constraints or position projection in stage 05.
- Do not add collision detection, stop propagation, layer coordination, or Pyramid/Tetra transition generation.
- Treat `layer_count == 0` as valid.
- Treat quality rejection as a normal per-face stop; treat evaluator `failure` as a program-level failure that aborts the current layer.
- A candidate vertex is committed only when at least one accepted face references it.
- Run each task's focused Debug test before the full Debug regression suite.

## File Map

### New public headers

- `include/boundary_mesh/growth/growth_profile.hpp`: per-vertex input values and immutable source-vertex lookup table.
- `include/boundary_mesh/growth/growth_profile_error.hpp`: missing, duplicate, unknown, and invalid profile diagnostics.
- `include/boundary_mesh/growth/growth_profile_builder.hpp`: profile validation and table construction.
- `include/boundary_mesh/growth/regular_layer_growth.hpp`: options, stop/completion events, layer mapping, records, step result, and final result.
- `include/boundary_mesh/growth/regular_layer_growth_error.hpp`: stage 05 wrapper errors preserving lower-level causes.
- `include/boundary_mesh/growth/regular_layer_stepper.hpp`: one-layer transactional candidate interface.
- `include/boundary_mesh/growth/regular_layer_generator.hpp`: high-level complete-growth interface.

### New source files

- `src/growth/growth_profile_builder.cpp`: deterministic profile validation/indexing.
- `src/growth/regular_layer_stepper.cpp`: eligibility, direction, candidate, quality, compaction, and next-front logic.
- `src/growth/regular_layer_generator.cpp`: initialization, repeated stepping, atomic mesh commit, and final records.

### New tests

- `tests/unit/growth/growth_profile_test.cpp`: profile validation and layer-height calculation.
- `tests/unit/growth/regular_layer_stepper_test.cpp`: Triangle/Quad stepping, fixed ordering, eligibility, and local quality stop.
- `tests/unit/growth/regular_layer_growth_types_test.cpp`: status, event, error, and output type contracts.
- `tests/integration/regular_layer_growth_pipeline_test.cpp`: multi-layer mixed-face generation and output mappings.
- `tests/integration/regular_layer_growth_failure_test.cpp`: program-error atomicity and no-orphan-candidate behavior.

### Existing build files

- `CMakeLists.txt`: add three stage 05 sources and link `BoundaryMesh::Quality` into `BoundaryMesh::BoundaryLayer`.
- `tests/CMakeLists.txt`: register five stage 05 executables.
- `docs/design/roadmap.md`: mark stage 05 complete only after all Debug and Release tests pass.

---

### Task 1: Validate and index per-vertex growth profiles

**Files:**
- Create: `include/boundary_mesh/growth/growth_profile.hpp`
- Create: `include/boundary_mesh/growth/growth_profile_error.hpp`
- Create: `include/boundary_mesh/growth/growth_profile_builder.hpp`
- Create: `src/growth/growth_profile_builder.cpp`
- Create: `tests/unit/growth/growth_profile_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `GrowthPatch::vertices()` and external `std::vector<SourceVertexGrowthProfile>`.
- Produces: `VertexGrowthProfile`, `SourceVertexGrowthProfile`, `GrowthProfileTable::find(VertexId)`, `GrowthProfileTable::height(VertexId, std::uint32_t)`, and `GrowthProfileBuilder::build(...)`.

- [ ] **Step 1: Add a failing profile validation test**

Create a test with a small helper that obtains a real `GrowthPatch`, then check one valid table plus every input error:

```cpp
const std::vector<SourceVertexGrowthProfile> valid{
    {VertexId{0}, {0.1, 1.0, 0}},
    {VertexId{1}, {0.2, 2.0, 3}},
    {VertexId{2}, {0.3, 0.5, 4}}};

const auto table = GrowthProfileBuilder{}.build(patch, valid);
if (!table.hasValue()) return 1;
if (table.value().find(VertexId{1}) == nullptr) return 2;
if (std::abs(table.value().height(VertexId{1}, 1).value() - 0.2) > 1e-12) return 3;
if (std::abs(table.value().height(VertexId{1}, 3).value() - 0.8) > 1e-12) return 4;

auto missing = valid;
missing.pop_back();
const auto missing_result = GrowthProfileBuilder{}.build(patch, missing);
if (missing_result.hasValue() ||
    std::get_if<MissingVertexGrowthProfile>(&missing_result.error()) == nullptr)
    return 5;

auto duplicate = valid;
duplicate.push_back(valid.front());
const auto duplicate_result = GrowthProfileBuilder{}.build(patch, duplicate);
if (duplicate_result.hasValue() ||
    std::get_if<DuplicateVertexGrowthProfile>(&duplicate_result.error()) == nullptr)
    return 6;

auto unknown = valid;
unknown.push_back({VertexId{99}, {0.1, 1.0, 1}});
const auto unknown_result = GrowthProfileBuilder{}.build(patch, unknown);
if (unknown_result.hasValue() ||
    std::get_if<UnknownVertexGrowthProfile>(&unknown_result.error()) == nullptr)
    return 7;

auto bad_height = valid;
bad_height[0].profile.first_height = 0.0;
const auto bad_height_result = GrowthProfileBuilder{}.build(patch, bad_height);
if (bad_height_result.hasValue() ||
    std::get_if<InvalidFirstHeight>(&bad_height_result.error()) == nullptr)
    return 8;

auto bad_ratio = valid;
bad_ratio[0].profile.growth_ratio =
    std::numeric_limits<Scalar>::infinity();
const auto bad_ratio_result = GrowthProfileBuilder{}.build(patch, bad_ratio);
if (bad_ratio_result.hasValue() ||
    std::get_if<InvalidGrowthRatio>(&bad_ratio_result.error()) == nullptr)
    return 9;
```

Also use `first_height = NaN`, `first_height = Inf`, `growth_ratio = 0`, and `growth_ratio = NaN` in separate checks. Check that a very large finite ratio produces `NonFiniteLayerHeight` when `height` evaluates a later layer.

- [ ] **Step 2: Register and run the RED test**

Add source and test targets:

```cmake
# root CMakeLists.txt, boundary_mesh_boundary_layer sources
src/growth/growth_profile_builder.cpp

# tests/CMakeLists.txt
add_executable(
    boundary_mesh_growth_profile_test
    unit/growth/growth_profile_test.cpp
)
target_link_libraries(
    boundary_mesh_growth_profile_test
    PRIVATE BoundaryMesh::BoundaryLayer
)
add_test(
    NAME boundary_mesh_growth_profile_test
    COMMAND boundary_mesh_growth_profile_test
)
```

Run:

```powershell
cmake --build build --config Debug --target boundary_mesh_growth_profile_test
```

Expected: compilation fails because `growth_profile_builder.hpp` does not exist.

- [ ] **Step 3: Define profile types and errors**

Add these exact public contracts:

```cpp
struct VertexGrowthProfile
{
    Scalar first_height{};       // 第一层生长步长
    Scalar growth_ratio{1};      // 相邻层步长倍率
    std::uint32_t layer_count{}; // 外部请求的最大层数
};

struct SourceVertexGrowthProfile
{
    VertexId source_vertex_id{}; // 输入 SurfaceMesh 的源顶点编号
    VertexGrowthProfile profile; // 该源顶点继承的生长参数
};

struct NonFiniteLayerHeight
{
    VertexId source_vertex_id{}; // 发生步长溢出的源顶点编号
    std::uint32_t layer{};       // 无法得到有限步长的目标层号
};

struct MissingVertexGrowthProfile
{
    VertexId source_vertex_id{}; // GrowthPatch 中缺少参数的源顶点
};

struct DuplicateVertexGrowthProfile
{
    VertexId source_vertex_id{}; // 被重复输入的源顶点
};

struct UnknownVertexGrowthProfile
{
    VertexId source_vertex_id{}; // 不属于当前 GrowthPatch 的输入顶点
};

struct InvalidFirstHeight
{
    VertexId source_vertex_id{}; // 参数非法的源顶点
    Scalar value{};              // 非有限或非正的首层高度
};

struct InvalidGrowthRatio
{
    VertexId source_vertex_id{}; // 参数非法的源顶点
    Scalar value{};              // 非有限或非正的增长率
};

using GrowthProfileError = std::variant<
    MissingVertexGrowthProfile,
    DuplicateVertexGrowthProfile,
    UnknownVertexGrowthProfile,
    InvalidFirstHeight,
    InvalidGrowthRatio>;
```

Keep `NonFiniteLayerHeight` outside `GrowthProfileError` because it can arise during a later layer rather than while validating raw input.

- [ ] **Step 4: Implement deterministic lookup and height calculation**

Expose:

```cpp
class GrowthProfileTable
{
public:
    const VertexGrowthProfile *find(VertexId source_vertex_id) const noexcept;

    Result<Scalar, NonFiniteLayerHeight>
    height(VertexId source_vertex_id, std::uint32_t layer) const;

    const std::vector<SourceVertexGrowthProfile> &entries() const noexcept;

private:
    friend class GrowthProfileBuilder;
    explicit GrowthProfileTable(
        std::vector<SourceVertexGrowthProfile> entries);
    std::vector<SourceVertexGrowthProfile> entries_; // 按源顶点编号升序保存
};

class GrowthProfileBuilder
{
public:
    Result<GrowthProfileTable, GrowthProfileError>
    build(
        const GrowthPatch &patch,
        const std::vector<SourceVertexGrowthProfile> &profiles) const;
};
```

Implement lookup with `std::lower_bound`. Implement height without integer exponent conversion:

```cpp
Scalar value = profile->first_height;
for (std::uint32_t current = 1; current < layer; ++current)
{
    value *= profile->growth_ratio;
    if (!std::isfinite(value))
    {
        return HeightResult::failure(
            NonFiniteLayerHeight{source_vertex_id, layer});
    }
}
return HeightResult::success(value);
```

Require `layer >= 1`; the stepper only asks for a target layer. Validate unknown lookup before dereferencing and represent it as `NonFiniteLayerHeight` only if the table invariant is already broken; the builder must prevent that state in normal use.

- [ ] **Step 5: Run focused and full tests**

Run:

```powershell
cmake --build build --config Debug --target boundary_mesh_growth_profile_test
ctest --test-dir build -C Debug -R boundary_mesh_growth_profile_test --output-on-failure
ctest --test-dir build -C Debug --output-on-failure
```

Expected: the focused test and all existing 22 tests pass.

- [ ] **Step 6: Commit Task 1**

```powershell
git add CMakeLists.txt tests/CMakeLists.txt include/boundary_mesh/growth/growth_profile.hpp include/boundary_mesh/growth/growth_profile_error.hpp include/boundary_mesh/growth/growth_profile_builder.hpp src/growth/growth_profile_builder.cpp tests/unit/growth/growth_profile_test.cpp
git diff --cached --check
git commit -m "feat: validate vertex growth profiles"
```

---

### Task 2: Define regular growth state, result, and error contracts

**Files:**
- Create: `include/boundary_mesh/growth/regular_layer_growth.hpp`
- Create: `include/boundary_mesh/growth/regular_layer_growth_error.hpp`
- Create: `tests/unit/growth/regular_layer_growth_types_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: profile types, `GrowthFront`, `VolumeMesh`, stage 03 errors, and stage 04 errors.
- Produces: all state/result/error types used by the stepper and generator.

- [ ] **Step 1: Write the public contract test**

Check exact defaults and variant alternatives:

```cpp
static_assert(std::is_same_v<
    decltype(RegularLayerGrowthResult{}.mesh), VolumeMesh>);

FaceGrowthRecord face;
if (face.status != FaceGrowthStatus::Active ||
    face.stop_reason != FaceStopReason::None ||
    face.accepted_layer_count != 0)
    return 1;

VertexGrowthRecord vertex;
if (vertex.accepted_layer_count != 0) return 2;

RegularLayerGrowthError error = VolumeVertexIdOverflow{42};
const auto *overflow = std::get_if<VolumeVertexIdOverflow>(&error);
if (overflow == nullptr || overflow->attempted_index != 42) return 3;
```

- [ ] **Step 2: Register and run the RED test**

Create `boundary_mesh_regular_layer_growth_types_test`, link it to `BoundaryMesh::BoundaryLayer`, and run its build target. Expected: missing-header compilation failure.

- [ ] **Step 3: Define statuses, records, events, and results**

Add the exact contracts approved by the design:

```cpp
enum class FaceGrowthStatus
{
    Active,    // 仍可尝试下一层
    Completed, // 因请求层数上限正常完成
    Stopped    // 因候选单元质量不合格提前停止
};

enum class FaceStopReason
{
    None,                     // 未完成或停止
    VertexLayerLimit,         // 至少一个面顶点达到请求上限
    DegenerateCandidate,      // 候选单元退化
    ReversedCandidate,        // 候选单元整体反转
    LocallyInvertedCandidate, // 候选单元局部翻转
    SkewnessExceeded          // 候选单元偏斜度超过阈值
};

struct FaceStopEvent
{
    std::size_t previous_front_face_index{}; // 对应上一层 Front 面下标
    SurfaceFaceId source_face_id{};           // 对应输入 Wall 面编号
    std::uint32_t layer{};                    // 首个未接受的目标层号
    FaceStopReason reason{FaceStopReason::None}; // 完成或停止原因
};

struct LayerStepResult
{
    std::uint32_t layer{}; // 本次目标层号
    GrowthFront next_front; // 只包含质量合格面的下一层 Front
    std::vector<std::size_t> previous_front_vertex_indices; // 下一层局部点到上一层局部点
    std::vector<std::size_t> previous_front_face_indices; // 下一层局部面到上一层局部面
    std::vector<FaceStopEvent> stopped_faces; // 质量不合格的源面
    std::vector<FaceStopEvent> completed_faces; // 达到层数上限的源面
};

struct LayerVertexRecord
{
    VertexId source_vertex_id{}; // 输入 Wall 顶点编号
    std::vector<VertexId> layer_vertex_ids; // 从第 0 层开始的实际体网格顶点编号
};

using LayerVertexTable = std::vector<LayerVertexRecord>;
```

Add `VertexGrowthRecord`, `FaceGrowthRecord`, `RegularLayerGrowthOptions`, and `RegularLayerGrowthResult` exactly as specified in the design document. `RegularLayerGrowthOptions` contains only `VolumeCellQualityOptions cell_quality` in stage 05.

- [ ] **Step 4: Define wrapper errors without losing lower-level causes**

```cpp
struct GrowthProfileFailure
{
    GrowthProfileError cause; // 参数整理阶段的具体失败
};

struct FrontEvaluationFailure
{
    std::uint32_t target_layer{}; // 本次尝试生成的目标层
    FrontEvaluationError cause;   // 阶段 03 的动态前沿错误
};

struct GrowthDirectionFailure
{
    std::uint32_t target_layer{}; // 本次尝试生成的目标层
    GrowthDirectionError cause;   // 阶段 03 的方向错误
};

struct CellEvaluationFailure
{
    SurfaceFaceId source_face_id{}; // 候选单元对应的源 Wall 面
    std::uint32_t layer{};           // 候选单元目标层
    VolumeCellEvaluationError cause; // 阶段 04 的具体失败
};

struct InvalidLayerFrontMapping
{
    std::uint32_t layer{}; // 映射不一致的当前层
};

struct VolumeVertexIdOverflow
{
    std::size_t attempted_index{}; // 无法转换为 VertexId 的体网格下标
};

using RegularLayerGrowthError = std::variant<
    GrowthProfileFailure,
    FrontEvaluationFailure,
    GrowthDirectionFailure,
    CellEvaluationFailure,
    NonFiniteLayerHeight,
    InvalidLayerFrontMapping,
    VolumeVertexIdOverflow>;
```

- [ ] **Step 5: Run tests and commit**

Run the focused type test and full Debug CTest. Expected: all tests pass. Then commit:

```powershell
git add tests/CMakeLists.txt include/boundary_mesh/growth/regular_layer_growth.hpp include/boundary_mesh/growth/regular_layer_growth_error.hpp tests/unit/growth/regular_layer_growth_types_test.cpp
git diff --cached --check
git commit -m "feat: define regular layer growth state"
```

---

### Task 3: Step one Triangle layer into a quality-checked Prism

**Files:**
- Create: `include/boundary_mesh/growth/regular_layer_stepper.hpp`
- Create: `src/growth/regular_layer_stepper.cpp`
- Create: `tests/unit/growth/regular_layer_stepper_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: current `GrowthFront`, validated `GrowthProfileTable`, and `RegularLayerGrowthOptions`.
- Produces: `RegularLayerStepper::step(...) -> Result<LayerStepResult, RegularLayerGrowthError>`.

- [ ] **Step 1: Write a RED single-Triangle test**

Use a layer-0 Triangle wound so its right-hand normal is `+Z`, with all three profiles `{0.25, 1.0, 2}`. Check:

```cpp
const auto result = RegularLayerStepper{}.step(
    front, profiles, RegularLayerGrowthOptions{});
if (!result.hasValue()) return 1;

const LayerStepResult &layer = result.value();
if (layer.layer != 1 ||
    layer.next_front.layer != 1 ||
    layer.next_front.vertices.size() != 3 ||
    layer.next_front.faces.size() != 1 ||
    layer.previous_front_vertex_indices !=
        std::vector<std::size_t>{0, 1, 2} ||
    layer.previous_front_face_indices !=
        std::vector<std::size_t>{0} ||
    !layer.stopped_faces.empty() ||
    !layer.completed_faces.empty())
    return 2;

for (std::size_t index = 0; index < 3; ++index)
{
    if ((layer.next_front.vertices[index] -
         (front.vertices[index] + Vector3{0, 0, 0.25})).norm() > 1e-12)
        return 3;
}

const Triangle &bottom = std::get<Triangle>(front.faces[0]);
const Triangle &top = std::get<Triangle>(layer.next_front.faces[0]);
const PrismPoints points{
    front.vertices[bottom.vertex_ids[0]],
    front.vertices[bottom.vertex_ids[1]],
    front.vertices[bottom.vertex_ids[2]],
    layer.next_front.vertices[top.vertex_ids[0]],
    layer.next_front.vertices[top.vertex_ids[1]],
    layer.next_front.vertices[top.vertex_ids[2]]};
const auto quality = evaluatePrism(points);
if (!quality.hasValue() || !quality.value().acceptable) return 4;
```

- [ ] **Step 2: Register dependencies and verify RED**

Add `src/growth/regular_layer_stepper.cpp` to `boundary_mesh_boundary_layer`. Add `BoundaryMesh::Quality` to that target's public links. Register `boundary_mesh_regular_layer_stepper_test`. Build the target. Expected: missing stepper header or unresolved method.

- [ ] **Step 3: Declare the stepper**

```cpp
class RegularLayerStepper
{
public:
    Result<LayerStepResult, RegularLayerGrowthError>
    step(
        const GrowthFront &current_front,
        const GrowthProfileTable &profiles,
        const RegularLayerGrowthOptions &options = {}) const;
};
```

- [ ] **Step 4: Implement eligibility before direction evaluation**

Validate all Front parallel arrays and face indices. Set `target_layer = current_front.layer + 1` with checked `std::uint32_t` arithmetic. For each face, visit its vertex IDs and require every source vertex profile to satisfy:

```cpp
current_front.layer < profile->layer_count
```

Put ineligible faces into `completed_faces` with `VertexLayerLimit`. Build a compact `eligible_front` from eligible faces only. If no face is eligible, return a successful empty `next_front` without calling `FrontEvaluator`.

- [ ] **Step 5: Compute current active directions and candidates**

Call:

```cpp
const auto evaluation = FrontEvaluator{}.evaluate(eligible_front);
const auto directions = computeGrowthDirections(
    eligible_front, evaluation.value());
```

Wrap failures as `FrontEvaluationFailure` and `GrowthDirectionFailure`. For each eligible local vertex, call `profiles.height(source_id, target_layer)` and compute:

```cpp
candidate = current + height * direction;
```

If the height result fails, return `NonFiniteLayerHeight`. Verify candidate coordinates are finite; otherwise return the same program-level error for that source point and target layer.

- [ ] **Step 6: Evaluate the candidate Prism and compact accepted output**

For a Triangle, assemble `PrismPoints` in fixed bottom/top order and call `evaluatePrism`. Wrap evaluator failure as `CellEvaluationFailure`. Convert successful rejection to a `FaceStopEvent` with this exact priority:

```cpp
switch (evaluation.validity)
{
case VolumeCellValidity::Degenerate:
    reason = FaceStopReason::DegenerateCandidate;
    break;
case VolumeCellValidity::Reversed:
    reason = FaceStopReason::ReversedCandidate;
    break;
case VolumeCellValidity::LocallyInverted:
    reason = FaceStopReason::LocallyInvertedCandidate;
    break;
case VolumeCellValidity::Valid:
    reason = FaceStopReason::SkewnessExceeded;
    break;
}
```

Only after all faces have been evaluated, collect vertices referenced by accepted faces, assign compact next-front IDs in previous-front vertex order, and populate both mapping arrays. Copy source IDs and `vertex_boundaries` for committed candidates. This guarantees no stopped-only candidate survives.

- [ ] **Step 7: Run tests and commit**

Run the focused stepper test and full Debug CTest. Expected: Triangle step succeeds and all regressions pass. Commit:

```powershell
git add CMakeLists.txt tests/CMakeLists.txt include/boundary_mesh/growth/regular_layer_stepper.hpp src/growth/regular_layer_stepper.cpp tests/unit/growth/regular_layer_stepper_test.cpp
git diff --cached --check
git commit -m "feat: step triangle growth into prism"
```

---

### Task 4: Add Quad/Hexa stepping, different layer counts, and local face stops

**Files:**
- Modify: `src/growth/regular_layer_stepper.cpp`
- Modify: `tests/unit/growth/regular_layer_stepper_test.cpp`

**Interfaces:**
- Consumes: the Task 3 stepper contract.
- Produces: complete mixed Triangle/Quad single-layer behavior.

- [ ] **Step 1: Add RED Quad and mixed-front cases**

Add a `+Z` Quad and check its four candidates, one next face, and a valid `HexaPoints` evaluation. Add a disconnected mixed front containing one Triangle and one Quad and check that both faces and all seven vertices survive with source mappings unchanged.

Use different first heights per vertex and assert each candidate moves by its own height along its computed direction. Use `growth_ratio = 2` and a layer-1 input Front to verify target-layer-2 heights are doubled.

- [ ] **Step 2: Add RED `4/5/6` eligibility behavior**

Construct a Triangle at `front.layer = 4` with vertex limits `4/5/6`. Assert:

```cpp
if (!result.hasValue() ||
    !result.value().next_front.faces.empty() ||
    result.value().completed_faces.size() != 1 ||
    result.value().completed_faces[0].reason !=
        FaceStopReason::VertexLayerLimit ||
    result.value().completed_faces[0].layer != 5)
    return failure_code;
```

Also construct two faces sharing vertices where one face contains a limit-0 vertex and the other remains eligible. Assert that only the eligible face influences directions and survives.

- [ ] **Step 3: Add RED local quality-stop cases**

Use a strict `maximum_skewness` to reject one deliberately skewed Quad while a neighboring regular Triangle remains acceptable. Assert:

- one `stopped_faces` event with `SkewnessExceeded`;
- one face in `next_front`;
- stopped-only candidate vertices are absent;
- shared candidate vertices survive when the accepted face references them.

Add explicit candidate configurations that stage 04 classifies as `Degenerate`, `Reversed`, and `LocallyInverted`, then check the exact mapped `FaceStopReason`.

- [ ] **Step 4: Implement Quad assembly and shared compaction**

Visit each eligible face:

```cpp
return std::visit(
    [&](const auto &face) -> CandidateDecision
    {
        using Face = std::decay_t<decltype(face)>;
        if constexpr (std::is_same_v<Face, Triangle>)
        {
            return evaluateTriangleCandidate(face, ...);
        }
        else
        {
            return evaluateQuadCandidate(face, ...);
        }
    },
    eligible_front.faces[face_index]);
```

For Quad, build `HexaPoints` as four current points followed by their four corresponding candidates and call `evaluateHexa`. Reuse one compaction path for accepted Triangle and Quad faces so mixed fronts remain deterministic.

- [ ] **Step 5: Run tests and commit**

Run the focused stepper test and full Debug CTest. Commit:

```powershell
git add src/growth/regular_layer_stepper.cpp tests/unit/growth/regular_layer_stepper_test.cpp
git diff --cached --check
git commit -m "feat: step mixed regular growth fronts"
```

---

### Task 5: Generate and atomically commit all regular layers

**Files:**
- Create: `include/boundary_mesh/growth/regular_layer_generator.hpp`
- Create: `src/growth/regular_layer_generator.cpp`
- Create: `tests/integration/regular_layer_growth_pipeline_test.cpp`
- Modify: `CMakeLists.txt`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `GrowthPatch`, initial `GrowthFront`, external profiles, options, and `RegularLayerStepper`.
- Produces: `RegularLayerGenerator::generate(...)` and free convenience function `generateRegularLayers(...)`.

- [ ] **Step 1: Write a RED multi-layer integration test**

Build an initial mixed Front with one Triangle and one Quad. Give every vertex `{0.1, 2.0, 2}`. Assert:

- the final mesh contains layer 0, layer 1, and layer 2 vertices;
- the Triangle produces two Prism cells;
- the Quad produces two Hexa cells;
- metadata layers are `1,1,2,2` in deterministic layer/face order;
- every cell metadata entry contains the correct source face;
- every `LayerVertexRecord` begins at layer 0 and has three IDs;
- all face records are `Completed` with `accepted_layer_count == 2`;
- all vertex records preserve requested profiles and report two accepted layers.

- [ ] **Step 2: Register and verify RED**

Add `src/growth/regular_layer_generator.cpp` to the BoundaryLayer target. Register `boundary_mesh_regular_layer_growth_pipeline_test`. Build it. Expected: missing generator header or unresolved function.

- [ ] **Step 3: Declare generator APIs**

```cpp
class RegularLayerGenerator
{
public:
    Result<RegularLayerGrowthResult, RegularLayerGrowthError>
    generate(
        const GrowthPatch &patch,
        const GrowthFront &initial_front,
        const std::vector<SourceVertexGrowthProfile> &profiles,
        const RegularLayerGrowthOptions &options = {}) const;
};

Result<RegularLayerGrowthResult, RegularLayerGrowthError>
generateRegularLayers(
    const GrowthPatch &patch,
    const GrowthFront &initial_front,
    const std::vector<SourceVertexGrowthProfile> &profiles,
    const RegularLayerGrowthOptions &options = {});
```

- [ ] **Step 4: Initialize layer 0 and records**

Validate that `initial_front.layer == 0`, all parallel arrays match, and source vertex/face IDs exactly match the Patch sets. Build the profile table; wrap failure as `GrowthProfileFailure`.

Copy `initial_front.vertices` into `result.mesh.vertices`. For each initial local vertex, create one `LayerVertexRecord` whose first entry is its volume vertex ID. Initialize vertex records from the validated profiles and initialize one active face record per initial source face.

Check every `std::size_t` to `VertexId` conversion:

```cpp
if (index > static_cast<std::size_t>(
        std::numeric_limits<VertexId>::max()))
{
    return GrowthResult::failure(
        VolumeVertexIdOverflow{index});
}
```

- [ ] **Step 5: Commit one successful LayerStepResult atomically**

Before changing `result`, create temporary vectors for new volume vertices, IDs, cells, metadata, and updated layer records. Check all target IDs first. Build each cell from the previous current-face order and the aligned next-face order:

```cpp
Prism{{old0, old1, old2, next0, next1, next2}}
Hexa{{old0, old1, old2, old3, next0, next1, next2, next3}}
```

Append the temporary buffers only after the entire layer converts successfully. Set metadata role to `CellRole::RegularLayer`, source face to the step mapping, and layer to `step.layer`.

Update completed/stopped face records after successful commit. Then replace current Front and its local-to-volume ID vector with the step result. Loop until the next Front has no faces.

- [ ] **Step 6: Run tests and commit**

Run the focused pipeline test and full Debug CTest. Commit:

```powershell
git add CMakeLists.txt tests/CMakeLists.txt include/boundary_mesh/growth/regular_layer_generator.hpp src/growth/regular_layer_generator.cpp tests/integration/regular_layer_growth_pipeline_test.cpp
git diff --cached --check
git commit -m "feat: generate regular prism and hexa layers"
```

---

### Task 6: Prove local stop output and program-error transaction behavior

**Files:**
- Create: `tests/integration/regular_layer_growth_failure_test.cpp`
- Modify: `src/growth/regular_layer_generator.cpp`
- Modify: `src/growth/regular_layer_stepper.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: completed Task 5 generator.
- Produces: verified no-orphan and no-half-layer guarantees plus complete diagnostic propagation.

- [ ] **Step 1: Add a RED no-orphan local-stop test**

Construct two disconnected source faces: one valid Triangle and one Quad rejected by strict skewness. Generate one requested layer. Assert that the output contains only the valid Triangle's three layer-1 points and one Prism, while the stopped Quad contributes neither a cell nor its four unshared candidate points. Assert the Triangle face is `Completed`, the Quad face is `Stopped`, and their accepted counts are `1` and `0`.

- [ ] **Step 2: Add RED program-error tests**

Cover these observable failures:

- malformed initial Front parallel-array sizes -> `InvalidLayerFrontMapping`;
- invalid raw profile -> `GrowthProfileFailure` preserving `InvalidFirstHeight`;
- degenerate active Front -> `FrontEvaluationFailure` preserving `DegenerateFrontFace`;
- finite first height whose later multiplication overflows -> `NonFiniteLayerHeight` with source ID and target layer;
- invalid `maximum_skewness` -> `CellEvaluationFailure` preserving `InvalidMaximumSkewness`.

For each failure, retain copies of the input Patch, Front, and profiles and assert they remain unchanged. The API returns no partial `RegularLayerGrowthResult`, so no failed layer is externally observable.

- [ ] **Step 3: Add deterministic mapping checks**

Run the same input twice and compare all output coordinates, cell variants and vertex IDs, metadata, layer tables, and records. Verify source-ID sorting does not change face winding or right-hand growth direction.

- [ ] **Step 4: Correct transaction ordering and diagnostics**

Move any result mutation found before a fallible operation into temporary buffers. Ensure the operation order is:

```text
validate current state
→ build eligible Front
→ evaluate Front and directions
→ calculate all candidate positions
→ evaluate every candidate cell
→ compact accepted candidates
→ validate all future IDs and mappings
→ append the complete layer
```

Never append a candidate point while candidate faces are still being evaluated.

- [ ] **Step 5: Run tests and commit**

Register `boundary_mesh_regular_layer_growth_failure_test`, run it, then run full Debug CTest. Commit:

```powershell
git add tests/CMakeLists.txt src/growth/regular_layer_stepper.cpp src/growth/regular_layer_generator.cpp tests/integration/regular_layer_growth_failure_test.cpp
git diff --cached --check
git commit -m "test: cover regular growth failure transactions"
```

---

### Task 7: Complete stage 05 regression, Release verification, and documentation

**Files:**
- Modify: `docs/design/roadmap.md`
- Modify: `docs/design/modules/regular-layer-growth.md` only if implementation names differ from the approved design.
- Modify: `docs/plans/05-regular-layer-growth.md` to mark executed checkboxes during implementation.

**Interfaces:**
- Consumes: all stage 05 implementation and tests.
- Produces: verified branch ready for user review and merge.

- [ ] **Step 1: Run formatting and stale-name scans**

```powershell
git diff --check
Get-ChildItem include,src,tests -Recurse -File |
    Select-String -Pattern 'SymmetryConstraints|projectPosition' |
    Where-Object { $_.Path -match 'regular_layer|growth_profile' }
```

Expected: `git diff --check` prints nothing; the stage 05 symmetry scan prints nothing.

- [ ] **Step 2: Run full Debug verification**

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

Expected: all existing 22 tests plus all five stage 05 tests pass.

- [ ] **Step 3: Run full Release verification**

```powershell
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Expected: the same complete test set passes in Release.

- [ ] **Step 4: Update roadmap completion status**

Change stage 05 status from `设计完成，待实施` to `已完成`. Keep the documented exclusions for symmetry, collision, stop propagation, coordination, and transition cells.

- [ ] **Step 5: Commit stage completion documentation**

```powershell
git add docs/design/roadmap.md docs/design/modules/regular-layer-growth.md docs/plans/05-regular-layer-growth.md
git diff --cached --check
git commit -m "docs: complete regular layer growth stage"
```

- [ ] **Step 6: Report final evidence for review**

Report the branch name, all stage 05 commits, Debug/Release passed-test counts, `git status --short --branch`, and the exact scope intentionally deferred to later stages. Do not merge into `master` until the user reviews the implementation.
