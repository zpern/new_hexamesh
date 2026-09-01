# Exact Cross-Zone Vertex Welding Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `readCgnsSurface()` deterministically merge exactly coincident vertices from different CGNS Zones when explicit connectivity is missing.

**Architecture:** Preserve all existing CGNS parsing and explicit `PointList/PointListDonor` validation. After explicit connections have populated the existing `DisjointSet`, perform a stable Zone/local-vertex scan using an exact normalized coordinate key and supplement the same disjoint set only across distinct Zone IDs; then reuse the current compact-ID pipeline.

**Tech Stack:** C++17, Eigen point types, cgnslib fixtures, existing `Result` errors, CMake/CTest.

## Global Constraints

- Merge only vertices whose source Zone IDs differ.
- Require exact equality of X, Y, and Z; never use a geometric tolerance.
- Treat `+0.0` and `-0.0` as equal.
- Never auto-merge duplicate coordinates within one Zone.
- Preserve explicit connectivity validation and deterministic compact Vertex IDs.
- Do not auto-repair face orientation or write changes back to CGNS.

---

### Task 1: Add missing-connectivity CGNS fixtures and failing tests

**Files:**
- Modify: `tests/helpers/cgns_fixture.hpp`
- Modify: `tests/helpers/cgns_fixture.cpp`
- Modify: `tests/unit/io/cgns_zone_merge_test.cpp`

**Interfaces:**
- Produces: `writeTwoZoneUnconnectedSurface(path, second_shared_x, use_negative_zero, duplicate_within_first_zone)`.
- Consumes: existing `readCgnsSurface(const std::filesystem::path&)`.

- [ ] **Step 1: Add a fixture declaration**

Add to `tests/helpers/cgns_fixture.hpp`:

```cpp
void writeTwoZoneUnconnectedSurface(
    const std::filesystem::path &path,
    double second_shared_x = 1.0,
    bool use_negative_zero = false,
    bool duplicate_within_first_zone = false);
```

- [ ] **Step 2: Implement the no-connection fixture**

In `tests/helpers/cgns_fixture.cpp`, create two unstructured TRI/QUAD-capable surface Zones without calling `cg_conn_write`. Zone 1 uses four quad vertices `(0,0,0)`, `(1,0,0)`, `(1,1,0)`, `(0,1,0)`; Zone 2 uses `(second_shared_x,0,-0.0 when requested)`, `(2,0,0)`, `(2,1,0)`, `(1,1,+0.0)`. When `duplicate_within_first_zone` is true, append a fifth unreferenced Zone-1 vertex equal to `(0,0,0)` so the output count proves same-Zone duplicates remain distinct.

- [ ] **Step 3: Add exact cross-Zone merge assertions**

In `cgns_zone_merge_test.cpp`, write `unconnected.cgns` and its four-section-compatible map using both Zones as Wall. Assert:

```cpp
const auto unconnected = readCgnsSurface(unconnected_path);
assert(unconnected.hasValue());
assert(unconnected.value().vertices.size() == 6);
assert(sharedVertexCount(
    unconnected.value().faces[0],
    unconnected.value().faces[1]) == 2);
```

Extract the existing shared-ID loop into `sharedVertexCount(first, second)` so the test stays readable.

- [ ] **Step 4: Add exactness, signed-zero, and same-Zone assertions**

Add three cases:

```cpp
writeTwoZoneUnconnectedSurface(near_path, std::nextafter(1.0, 2.0));
assert(readCgnsSurface(near_path).value().vertices.size() == 8);

writeTwoZoneUnconnectedSurface(zero_path, 1.0, true);
assert(readCgnsSurface(zero_path).value().vertices.size() == 6);

writeTwoZoneUnconnectedSurface(same_zone_path, 1.0, false, true);
assert(readCgnsSurface(same_zone_path).value().vertices.size() == 7);
```

Create the matching `.bc.txt` beside every fixture and include `<cmath>` for `std::nextafter`.

- [ ] **Step 5: Run the focused test and verify RED**

Run:

```powershell
cmake --build build --config Debug --target boundary_mesh_cgns_zone_merge_test
ctest --test-dir build -C Debug -R boundary_mesh_cgns_zone_merge_test --output-on-failure
```

Expected: test FAIL because the unconnected exact-coordinate fixture retains 8 vertices instead of 6.

---

### Task 2: Implement deterministic exact cross-Zone welding

**Files:**
- Modify: `src/io/cgns_surface_reader.cpp`
- Test: `tests/unit/io/cgns_zone_merge_test.cpp`
- Modify: `README.md`

**Interfaces:**
- Consumes: `ZoneInfo::{id,vertex_offset,vertex_count}`, temporary `mesh.vertices`, and `DisjointSet::merge(...)`.
- Produces: the unchanged public API `CgnsSurfaceResult readCgnsSurface(path)` with supplemental exact-coordinate unions.

- [ ] **Step 1: Define an exact normalized coordinate key**

Add an anonymous-namespace key and hash:

```cpp
struct ExactPointKey
{
    double x{};
    double y{};
    double z{};

    bool operator==(const ExactPointKey &other) const noexcept
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct ExactPointKeyHash
{
    std::size_t operator()(const ExactPointKey &key) const noexcept;
};

ExactPointKey exactPointKey(const Point3 &point) noexcept
{
    const auto normalize_zero = [](double value)
    {
        return value == 0.0 ? 0.0 : value;
    };
    return {normalize_zero(point.x()),
            normalize_zero(point.y()),
            normalize_zero(point.z())};
}
```

Implement the hash by combining `std::hash<double>` values. Equality, not the hash, remains authoritative.

- [ ] **Step 2: Supplement the existing disjoint set after explicit connections**

Immediately before compact-ID generation, scan `zones` in ascending numeric ID and local vertices in ascending ID. Store all prior occurrences per key so same-Zone duplicates do not hide a representative from another Zone:

```cpp
struct CoordinateOccurrence
{
    std::uint32_t zone_id{};
    std::size_t vertex_index{};
};

std::unordered_map<
    ExactPointKey,
    std::vector<CoordinateOccurrence>,
    ExactPointKeyHash> occurrences;

for (const ZoneInfo &zone : zones)
{
    for (std::size_t local = 0;
         local < static_cast<std::size_t>(zone.vertex_count);
         ++local)
    {
        const std::size_t vertex = zone.vertex_offset + local;
        auto &matches = occurrences[exactPointKey(mesh.vertices[vertex])];
        const auto prior = std::find_if(
            matches.begin(), matches.end(),
            [&](const CoordinateOccurrence &candidate)
            {
                return candidate.zone_id != zone.id;
            });
        if (prior != matches.end())
            connected_vertices.merge(prior->vertex_index, vertex);
        matches.push_back({zone.id, vertex});
    }
}
```

- [ ] **Step 3: Run focused IO tests and verify GREEN**

Run:

```powershell
cmake --build build --config Debug --target boundary_mesh_cgns_zone_merge_test boundary_mesh_cgns_surface_reader_test
ctest --test-dir build -C Debug -R "boundary_mesh_cgns_(zone_merge|surface_reader)_test" --output-on-failure
```

Expected: both tests PASS, including existing explicit mismatch diagnostics.

- [ ] **Step 4: Update maintenance documentation**

Replace the README statement that cross-Zone vertices require explicit connections with: explicit connectivity is validated and applied first; missing connections are supplemented only for exactly coincident cross-Zone coordinates; no tolerance welding or same-Zone welding occurs.

- [ ] **Step 5: Commit implementation**

```powershell
git add src/io/cgns_surface_reader.cpp tests/helpers/cgns_fixture.hpp tests/helpers/cgns_fixture.cpp tests/unit/io/cgns_zone_merge_test.cpp README.md
git commit -m "feat: weld exact cross-zone CGNS vertices"
```

---

### Task 3: Real-case and full regression verification

**Files:**
- No production file changes expected.
- Test inputs: `C:/Users/zpern/Desktop/todo/jiuyuan-quailty/internal/test_case/curve/curve2.cgns`
- Test inputs: `C:/Users/zpern/Desktop/todo/jiuyuan-quailty/internal/test_case/plane/plane2.cgns`

**Interfaces:**
- Consumes: Release `boundary_mesh_cli.exe` and matching `.bc.txt` maps.
- Produces: evidence separating coordinate-welding success from remaining orientation or mesh-quality failures.

- [ ] **Step 1: Prepare matching boundary maps without changing source CGNS files**

Use temporary case directories and copy each `.cgns` plus its current `curve.bc.txt`/`plane.bc.txt` renamed to the matching `curve2.bc.txt`/`plane2.bc.txt` basename.

- [ ] **Step 2: Run both real cases**

Run Release CLI with:

```powershell
--first-height 0.01 --growth-ratio 1.2 --layer-count 3
```

Write outputs beneath a temporary directory, not into the supplied case directories. Record exit codes and stage messages.

- [ ] **Step 3: Run complete verification**

Run:

```powershell
git diff --check
cmake --build build --config Debug --parallel
ctest --test-dir build -C Debug --output-on-failure
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
git status --short
```

Expected: both builds succeed and both test suites report 100% pass. Real-case failures, if any, must identify a reason other than missing exact-coordinate cross-Zone welding.
