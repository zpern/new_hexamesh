# Incremental Collision Index Design

## Problem

Boundary-layer generation repeatedly converts the complete exposed boundary to
triangles and rebuilds a global `CollisionIndex`. The same immutable or
unchanged geometry is rebuilt during regular obstacle checks, transition
collision checks, resolver iterations, and external-patch distance trials.

For the 59,381-face reference case, layers 1 through 9 take approximately
24--32 seconds. On layer 10, 43,308 terminal faces become external patches at
once. Two complete transition scans take 87.3 and 97.3 seconds, making
`candidate-rejections` take 191.7 seconds and the layer take 212.4 seconds.

The design must eliminate repeated global-index construction while preserving
the current `CollisionTriangle` contact semantics and transition decisions.

## Design goals

- Maintain the committed exposed-boundary collision index incrementally across
  layers.
- Build the current candidate index at most once per resolver iteration.
- Validate an external-patch distance trial using only the changed patch.
- Preserve `hasIllegalTriangleContact`, sliding-contact permissions, rollback
  ownership, and deterministic results.
- Keep the original surface, committed history, and uncommitted candidates in
  separate indexes with explicit lifetimes.
- Retain a rebuild path for index degradation and root-bound expansion.

## Non-goals

- Reusing the legacy `BLFront`, `MBLNode`, `OctreeAgent`, or global triangle
  arrays from `blmesh`.
- Changing exact triangle-contact rules.
- Replacing the existing one-shot regular-layer self-collision index in the
  first implementation phase.
- Introducing concurrent index mutation.

## Reference implementation

The legacy `blmesh` implementation supplies the algorithmic model:

- `BLMesh::insertAndRmTriInOctree()` incrementally removes the old front and
  inserts new top and side triangles.
- `Octree::find_intersected_triangles()` scans changed leaves instead of
  reconstructing the complete tree.
- Deleted entries remain inactive until compaction.
- `Octree::rebuildIfOverloaded()` performs an exceptional full rebuild when a
  leaf becomes overloaded.

The new implementation adopts these lifecycle policies but operates directly
on `CollisionTriangle` and uses the current project's exact-contact predicate.

## Index lifetimes

The collision system has three distinct indexes:

| Index | Lifetime | Mutation |
|---|---|---|
| Original surface | Complete generation | Read-only after initial build |
| Historical exposed boundary | Complete generation | Incrementally updated after a layer commits |
| Provisional candidate boundary | One resolver iteration | Bulk-built once, then updated by primitive group |

`SlidingIntersectionIndex` remains independent and read-only during growth.
Uncommitted candidate geometry must never be inserted into the historical
index.

## Stable identities and primitive groups

```cpp
using CollisionPrimitiveId = std::uint64_t;
using CollisionGroupId = std::uint64_t;

struct StoredCollisionPrimitive
{
    CollisionTriangle triangle;
    Aabb bounds;
    CollisionGroupId group_id{};
    bool active{};
};
```

Primitive IDs are monotonically allocated and are not reused. A group owns all
triangles that must be inserted, removed, or replaced together. Examples are a
triangulated boundary quad, a transition owner, and one external patch.

Spatial primitive identity remains separate from the business ownership stored
in `CollisionTriangle`. No collision rule may depend on an unstable position in
`ExposedBoundaryTracker::faces()`.

## Incremental index API

```cpp
class IncrementalCollisionIndex
{
public:
    static Result<IncrementalCollisionIndex, SpatialError> build(
        std::vector<CollisionPrimitiveGroup> groups,
        IncrementalCollisionIndexOptions options = {});

    Result<void, SpatialError> insertGroup(
        CollisionPrimitiveGroup group);
    Result<void, SpatialError> eraseGroup(CollisionGroupId group);

    std::vector<CollisionPrimitiveId> queryCandidates(
        const Aabb &bounds,
        std::optional<CollisionGroupId> ignored_group = {}) const;

    std::vector<CollisionPrimitiveId> queryIllegalContacts(
        const CollisionTriangle &triangle,
        std::optional<CollisionGroupId> ignored_group = {}) const;

    const CollisionTriangle &primitive(CollisionPrimitiveId id) const;
    Result<bool, SpatialError> rebuildIfDegraded();
};
```

Group insertion is validated completely before the index is mutated. Group
replacement is implemented as validation against a view that ignores the old
group, followed by erase and insert. A failed trial leaves the index unchanged.

## Octree representation

```cpp
struct OctreeNode
{
    Aabb bounds;
    std::array<NodeId, 8> children;
    std::vector<CollisionPrimitiveId> primitives;
    std::size_t inactive_count{};
    bool leaf{};
    bool dirty{};
};
```

An inserted primitive is stored in every intersected leaf. Queries therefore
deduplicate primitive IDs before exact testing. Erasure marks the store entry
inactive and increments tombstone counts; queries skip inactive entries.

Initial tuning values are:

- maximum depth: 12;
- target leaf capacity: 64;
- rebuild leaf capacity: 1024;
- rebuild inactive ratio: 30 percent.

These values are configuration of the implementation, not behavioral
requirements. Tests and real-case profiling determine final values.

The root AABB includes the original surface plus the maximum expected growth
distance and tolerance. An insertion outside the root expands the root and
causes a rebuild. Root-expansion rebuilds are recorded and should normally be
zero.

## Exposed-boundary integration

`ExposedBoundaryTracker` owns the committed historical index and maps every
`BoundaryFaceKey` to its collision group:

```cpp
class ExposedBoundaryTracker
{
public:
    Result<void, SpatialError> apply(const ExposedBoundaryUpdate &update);
    const IncrementalCollisionIndex &collisionIndex() const noexcept;

private:
    BoundaryFaceMap faces_;
    IncrementalCollisionIndex collision_index_;
    std::map<BoundaryFaceKey, CollisionGroupId> collision_groups_;
};
```

`apply()` changes from `void` to `Result`. It first validates and triangulates
every insertion, builds a complete mutation plan, then applies index and face
map changes. Invalid input cannot leave the tracker and index out of sync.

Historical top-contact checks use the metadata already carried by stored
`CollisionTriangle` objects (`boundary_points`, `boundary_vertex_keys`, and
`boundary_vertex_count`). They do not look up a boundary face through a mutable
vector index.

## Regular obstacle checking

`LayerCollisionChecker::filterAgainstObstacles()` receives or accesses the
already-built historical index. It no longer calls
`ExposedBoundaryTracker::collisionTriangles()` followed by
`CollisionIndex::build()`.

Each candidate is tested against:

1. the immutable original-surface index;
2. the persistent historical index;
3. the immutable sliding-surface index when enabled.

The existing regular-layer self-collision implementation remains a one-shot
candidate build during the first phase. It is profiled again after historical
and transition rebuilds have been removed.

## Unified transition inspection

`findCollidingOwners()` and `findRollbackFaces()` are replaced internally by a
single inspection:

```cpp
struct TransitionCollisionReport
{
    std::vector<LayerBoundaryOwner> colliding_owners;
    std::vector<SurfaceFaceId> rollback_faces;
};

Result<TransitionCollisionReport, TransitionBoundaryError>
inspect(const TransitionBoundaryView &boundary) const;
```

One provisional boundary is assembled and scanned once per resolver iteration.
The result contains both external-patch owners and rollback faces. Ordering is
normalized before returning so behavior remains deterministic.

## Candidate self-collision

At the beginning of a resolver iteration, all exposed provisional triangles are
bulk-built into one candidate index. Candidate pairs are generated only within
overlapping leaves. Within a leaf, primitive AABBs are sorted along the widest
axis and interval bounds terminate pair enumeration early.

Pair IDs are canonicalized and deduplicated because a primitive can occupy
several leaves. Exact testing continues to call
`hasIllegalTriangleContact(left, right)`.

## Local external-patch trials

Each external patch owns one candidate group. The initial complete inspection
identifies which patch groups collide. A safe patch is never rebuilt or tested
again during that iteration.

For a colliding patch distance trial:

1. generate only the trial patch vertices and triangles;
2. query the original, historical, prior-transition, sliding, and candidate
   indexes;
3. ignore the candidate patch's existing group when querying candidate
   geometry;
4. if the trial is unsafe, discard it without mutating the index;
5. if safe, erase the old group and insert the new group;
6. continue bracketing and binary search using local trials.

Consequently, a model with 43,308 external patches and no colliding external
owners performs one bulk candidate build and one complete inspection. It does
not reconstruct all patches for each face or repeat the global scan merely to
obtain rollback faces.

## Rebuild and compaction policy

A full rebuild is exceptional and occurs when any of these conditions holds:

- an insertion lies outside the root AABB;
- maximum active leaf load exceeds the configured threshold;
- inactive entries exceed the configured ratio;
- explicit validation detects inconsistent node membership.

Rebuild collects active primitives from the store, recreates the tree, clears
tombstones, and preserves primitive and group IDs. Rebuild does not change
collision ownership or result ordering.

## Diagnostics

```cpp
struct CollisionIndexDiagnostics
{
    std::uint64_t inserts{};
    std::uint64_t erases{};
    std::uint64_t queries{};
    std::uint64_t broad_phase_candidates{};
    std::uint64_t exact_tests{};
    std::uint64_t rebuilds{};
    std::uint64_t root_expansions{};
    std::size_t active_primitives{};
    std::size_t inactive_primitives{};
    std::size_t maximum_leaf_load{};
};
```

Layer-level output reports aggregate diagnostics rather than per-face timing.
These counters distinguish tree degradation, weak broad-phase filtering, and
excessive local patch trials.

## Error handling and determinism

- Non-finite, degenerate, or invalid triangles are rejected before mutation.
- Unknown group erasure returns an explicit error so tracker/index divergence
  cannot be hidden.
- Query results are sorted by stable primitive ID before exact testing.
- Owner and rollback results are sorted and deduplicated before returning.
- An index mutation failure cannot partially update `ExposedBoundaryTracker`.
- Debug validation can compare store membership against tree membership after
  every mutation batch.

## Implementation phases

1. Implement and test the incremental primitive store and Octree without
   production integration.
2. Integrate the persistent historical index with
   `ExposedBoundaryTracker` and regular obstacle checks.
3. Make transition checking consume the cached historical and
   prior-transition indexes, and combine owner/rollback inspection.
4. Bulk-build a candidate transition index and implement local external-patch
   group trials.
5. Profile the real model and decide separately whether regular-layer
   self-collision should adopt dirty-leaf scanning.

Each phase must preserve a working build and pass existing tests before the
next phase begins.

## Verification

### Unit tests

- Insert, erase, group erase, ignore-group query, rebuild, and root expansion.
- Stable IDs remain valid across compaction and rebuild.
- Invalid group insertion is atomic.
- New incremental query results match a freshly built `CollisionIndex` over
  active primitives.
- Random mutation sequences compare the incremental index against the existing
  immutable index after every operation.
- Candidate self-pair results contain no duplicates and match brute force.

### Boundary tests

After every `ExposedBoundaryTracker::apply()`, compare the persistent index
against a fresh index constructed from `collisionTriangles()`. Cover triangles,
quads, shared-face cancellation, removal and reinsertion, and multiple boundary
faces sharing a source face.

### Transition tests

Existing fixtures must retain identical rollback face sets, colliding owner
sets, terminal decisions, topology, and cell counts. External-patch distances
must be identical or within the existing numerical tolerance.

### Real-case acceptance

For the 59,381-wall-face, 10-layer case:

- generated cell counts and topology match the baseline;
- no new illegal intersection or volume/skewness regression appears;
- no historical global build occurs after initialization;
- each transition iteration performs at most one complete inspection;
- safe external patches perform no local trial rebuild;
- layer-10 `candidate-rejections` decreases from 191.7 seconds to below
  20 seconds;
- total layer-10 time decreases from 212.4 seconds to below 45 seconds.

If correctness passes but these performance targets fail, diagnostics must be
used to identify broad-phase candidate volume, exact-test volume, or tree
degradation before changing algorithms.
