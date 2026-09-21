# Incremental External Patch Search Design

## Goal

Remove repeated full `buildProvisionalTransition()` calls from external-patch distance search. A distance probe must rebuild and recheck only the external patches whose distance scale changed, while producing the same retained faces, resolved topology, collision decisions, and final cells as the current resolver.

## Current Cost

`LayerTransitionResolver` first builds a complete provisional transition and performs a global collision scan. For colliding external patches it then halves and bisects their distance scale. Every probe currently calls the same full `build()` callback, rebuilding transition geometry for roughly 215,000 retained faces even when only tens of external patches changed.

The Benchmark layer-20 measurements show `candidate-rejections=155.7s`. The two logged outer provisional builds and scans account for only about 26 seconds. The remaining time is dominated by unlogged full provisional builds inside external distance search.

## Safety Property

Reducing external distance produces a volume contained by the previous external cell, but the new boundary triangles are not literal subsets of the previous boundary triangles. A static surface that was inside the old volume without touching its boundary can intersect the new boundary during contraction. Therefore the implementation may check previous collision partners first as an early exit, but it must still query every new patch triangle against the complete unchanged collision environment.

## Architecture

### Stable provisional cache

The first build in a resolver iteration remains a complete `ProvisionalLayerTransition`. Partition its data into:

- immutable geometry and topology for regular candidates and non-search external patches;
- one replaceable record per searched external patch, keyed by `SurfaceFaceId`;
- the collision groups corresponding to those records.

The immutable part remains unchanged throughout the halving and bisection probes for that resolver iteration.

### Local external patch builder

Extract the existing external-patch construction logic from `buildProvisionalTransition()` into a focused function. It consumes the source face, current/candidate geometry, terminal hexa data, and one distance scale, and returns:

- the patch's `OwnedBoundaryTriangle` values;
- its `ResolvedTransitionTopology` entry;
- any forced rollback IDs;
- any boundary diagonal requirements owned by the patch.

The full provisional builder calls the same function, so full and local construction cannot diverge.

### Transactional patch replacement

For one distance probe:

1. Build all changed patch records into temporary values.
2. Validate every record before changing cached state.
3. Erase each patch's old collision group from a working incremental index.
4. Insert the new triangles under the same stable group ID.
5. Query only the new triangles against the full working index while ignoring their own group.
6. Publish new records and the working index only when every replacement succeeds.

Unchanged patches, regular candidates, historical boundaries, prior transition boundaries, the original surface, and sliding surfaces are never rebuilt during the probe.

### Collision checking

The initial global scan still identifies which external patch owners require search. During probes:

- previous collision partners may be tested first;
- a surviving collision ends that patch's probe early;
- otherwise all new patch triangles query the persistent BVH;
- obstacle checks against original, historical, prior-transition, and sliding indexes remain enabled;
- contacts between simultaneously changed patches are evaluated after all changed groups have been replaced, not against stale geometry.

This preserves exact `hasIllegalTriangleContact()` semantics and prevents false results caused by checking a new patch against another patch's old version.

## Resolver Flow

For each outer resolver iteration:

1. Apply corner suppression and build one full provisional transition.
2. Run one global inspection.
3. Create an `ExternalPatchSearchState` for colliding external owners.
4. Create the immutable collision environment and stable group registry once.
5. For each halving or bisection round, rebuild all active searched patches as one batch and transactionally replace their groups.
6. Update low/high brackets from local collision results.
7. Materialize the final provisional result by combining immutable data with the final patch records; do not perform another full build merely to apply final distance scales.
8. Continue the existing rollback loop. A rollback changes retained topology, so it starts a new outer iteration and is allowed one new full build.

## Diagnostics

Report per resolver iteration:

- full provisional build time and count;
- external cache construction time;
- number of searched patches;
- number of local probe rounds;
- local patch build time;
- incremental index replacement time;
- exact collision-query time and count;
- whether a new outer rollback iteration forced a new full build.

The acceptance target is that distance search performs zero full provisional builds after the initial build of an outer resolver iteration.

## Error Handling

All new public operations return `Result`. Missing source faces, duplicate patch IDs, missing collision groups, invalid generated geometry, and index replacement failures leave the cached provisional and collision index unchanged. A local patch builder error is returned through the existing `LayerTransitionError` variants.

## Testing

Unit tests must prove:

- local external-patch output equals the corresponding portion of a full build for several distance scales;
- a probe changes only requested patch records;
- two simultaneously changed patches are checked using both new geometries;
- a newly encountered collision not present in the previous partner set is still detected;
- failed replacement leaves the cache unchanged;
- halving and bisection produce the same final distance scales, rollback IDs, and retained faces as the current full-rebuild reference algorithm;
- a resolver diagnostic counter records one full build and multiple local builds during search.

Release validation reruns 2dot5 and anisotropic parity cases, then Benchmark with 20 layers, first height `0.1`, growth ratio `1.2`, and maximum skewness `1.0`. Layer 20 must produce the same 215,050 cells as the accepted run, with a materially lower `candidate-rejections` time than 155.7 seconds.

## Scope

This change does not alter corner suppression, transition templates, exact triangle-contact classification, rollback policy, stepper behavior, or automatic surface orientation. It only changes how external-patch distance probes rebuild geometry and update their collision environment.
