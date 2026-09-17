# Batch Self-Collision Detection Design

## Goal

Reduce the per-layer self-collision cost without changing which growth
candidates are stopped. Reuse the useful broad-phase ideas from `blmesh`, but
keep the implementation expressed in the current `CollisionTriangle` model.

## Scope

This change covers collisions among the provisional cells of one newly
generated layer. It does not persist the self-collision index across layers,
replace the existing exact contact predicate, or import the legacy
`Octree`, `OctreeAgent`, or `MBLNode` classes.

The optimized detector must produce the same stopped owner set as the current
`CollisionIndex::queryIllegalContacts()` loop.

## Alternatives Considered

### Keep per-triangle BVH queries and add early owner filtering

This is the smallest change, but each pair can still be visited in both
directions and every query still allocates a result vector. It cannot apply a
leaf-local sweep efficiently.

### Persist a mutable self-collision tree across layers

This can reuse tree topology, but nearly all candidate coordinates change at
each layer. Update and removal bookkeeping would add complexity before the
dominant pair-enumeration cost has been removed.

### Batch pair enumeration in a per-layer spatial index

This is the selected design. Build the broad phase once per layer, enumerate
unordered primitive pairs once, filter cheap legal pairs before exact contact,
and return illegal owner pairs directly.

## Architecture

Add a spatial component dedicated to unordered self-contact enumeration over a
`std::vector<CollisionTriangle>`. Its public operation returns the owner IDs
participating in illegal contacts and optionally fills diagnostic counters.
`LayerCollisionChecker::filterSelfCollisions()` remains responsible for
turning those owner IDs into the `stopped` mask.

The component owns only per-call data. It caches each triangle AABB during
construction and builds a leaf-based spatial partition for the current layer.
Within each leaf it sorts primitive references along the leaf's longest axis
and uses interval sweep-and-prune before checking the other two AABB axes.

Because a triangle may overlap more than one leaf, primitive pairs are
canonicalized as `(min_id, max_id)` and globally deduplicated before exact
testing. Pair identity uses primitive indices, not owner IDs, because different
triangles belonging to the same two owners may encode different contacts.

## Pair Processing Order

For each unique unordered primitive pair:

1. Skip the pair when both triangles have the same `owner_id`.
2. Reject it when their cached AABBs do not overlap on all three axes.
3. Apply only topology exclusions already proven legal by current collision
   semantics. Same-owner exclusion is mandatory; shared-edge exclusion is
   introduced only when parity tests demonstrate that the existing predicate
   always treats it as legal.
4. Call the existing `hasIllegalTriangleContact()` predicate unchanged.
5. Record both owner IDs when the contact is illegal.

The detector may stop exact testing additional primitive pairs once both owner
IDs are already known to be stopped, provided this cannot hide diagnostics
needed by tests or profiling.

## Diagnostics

Expose counters that do not affect decisions:

- triangle count;
- leaf count and maximum leaf load;
- raw leaf-local pair count;
- unique primitive pair count;
- same-owner skip count;
- AABB-rejected pair count;
- topology-rejected pair count;
- exact-contact test count;
- illegal owner-pair count.

Time measurements stay at the caller/stage level so the spatial component
remains deterministic and easy to test.

## Correctness and Error Handling

Empty and single-triangle inputs succeed with no illegal owners. Invalid or
non-finite triangle bounds return the existing spatial error type instead of
silently dropping primitives. Degenerate triangles retain the behavior of the
existing exact contact predicate.

Owner output is deterministic: IDs are unique and sorted. Diagnostics are
deterministic for a fixed input.

## Testing

Unit tests first establish parity with the existing implementation for:

- empty and separated inputs;
- two intersecting triangles with different owners;
- intersecting triangles with the same owner;
- legal shared-edge and shared-point contacts;
- duplicate leaf membership producing only one exact test per primitive pair;
- multiple triangles mapping to the same colliding owners;
- deterministic owner output and diagnostics.

An integration test compares the stopped candidate set from the old and new
paths on representative layer geometry. After correctness tests pass, run the
Release 2dot5 case with 20 layers, first height `0.1`, and growth ratio `1.2`.
Compare total time, self-collision time, per-layer cell counts, and the new
diagnostic counters against the recorded 471.9-second baseline whose
self-collision stage took 212.5 seconds.

## Success Criteria

- All collision and layer-growth tests pass.
- The 2dot5 run produces identical per-layer cell counts and completes all 20
  layers.
- Self-collision exact tests never process an unordered primitive pair twice.
- The Release 2dot5 self-collision time improves measurably over 212.5 seconds.
- Existing user-owned uncommitted CLI, timing, HDF5, editor, and configuration
  changes remain untouched.
