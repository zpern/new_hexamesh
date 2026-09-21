# Provisional Transition Build Context Design

## Goal

Eliminate repeated full-front lookup-table construction during external-patch
distance probes while preserving exactly the same provisional transition
geometry, topology decisions, rollback behavior, and final cell counts.

The accepted Benchmark remains 20 layers, first height `0.1`, growth ratio
`1.2`, maximum skewness `1.0`, and 215,050 cells on layer 20.

## Current Cost

`buildProvisionalExternalPatches()` emits only selected external patches, but
every call still scans the complete current and candidate fronts to rebuild:

- source-face ID to current-face index;
- current edge to incident source-face IDs;
- source vertex and branch to current-vertex index;
- source vertex and branch to candidate-vertex index.

Benchmark layer 19 searches six patches over 30 probe rounds and performs 32
local builds. Those local builds consume 17.192 seconds even though each probe
changes only a few generated patch points.

## Architecture

Introduce a public, immutable `ProvisionalTransitionBuildContext` owned by the
caller for the duration of one current/candidate front pair. The context stores
references to the two fronts and the four lookup tables currently recreated by
`buildProvisionalTransitionImpl()`.

Construction is explicit and fallible only if a structural invariant must be
validated. The preferred interface is:

```cpp
class ProvisionalTransitionBuildContext
{
public:
    ProvisionalTransitionBuildContext(
        const GrowthFront &current,
        const GrowthFront &candidate);

    const GrowthFront &current() const;
    const GrowthFront &candidate() const;

    // Lookup access is exposed only to the builder implementation.
};
```

The full builder and external-only builder receive the context:

```cpp
ProvisionalLayerTransitionResult buildProvisionalTransition(
    const ProvisionalTransitionBuildContext &context,
    const std::vector<SurfaceFaceId> &retained,
    const LayerFaceSets &face_sets,
    const std::function<std::optional<HexaPoints>(SurfaceFaceId)> &
        terminal_hexa_points = {},
    const ExternalPatchControls &external_controls = {},
    const std::vector<SurfaceFaceId> &terminal_candidate_faces = {});

ProvisionalLayerTransitionResult buildProvisionalExternalPatches(
    const ProvisionalTransitionBuildContext &context,
    const std::vector<SurfaceFaceId> &retained,
    const LayerFaceSets &face_sets,
    const std::vector<SurfaceFaceId> &selected_external_faces,
    const std::function<std::optional<HexaPoints>(SurfaceFaceId)> &
        terminal_hexa_points = {},
    const ExternalPatchControls &external_controls = {},
    const std::vector<SurfaceFaceId> &terminal_candidate_faces = {});
```

Existing front-taking overloads remain temporarily as compatibility wrappers.
They construct a context once and delegate to the context-taking overload.
This keeps synthetic tests and callers source-compatible while production uses
the reusable path.

## Lifetime and Invalidation

The context borrows `GrowthFront` objects and must not outlive them. The fronts
must not be mutated while the context exists.

The context is valid across:

- changes to retained high-face IDs;
- changes to `LayerFaceSets` after rollback and corner suppression;
- changes to external distance scales;
- changes to keep-hexa decisions.

These values affect filtering and geometry decisions but do not change the
front storage or adjacency lookup tables.

The context must be rebuilt when either current or candidate front is replaced,
including advancing to a new boundary-layer step. Production constructs it in
the candidate-rejection callback, where both captured fronts remain alive for
the entire resolver call.

## Builder Behavior

`buildProvisionalTransitionImpl()` stops constructing global maps. It reads
the corresponding tables from the supplied context and keeps all existing
transition-template branches unchanged.

The external-only builder continues to:

- omit regular candidate faces;
- skip unselected terminal candidates;
- skip unselected transition-low faces;
- return only the selected patches' triangles, topology, and forced rollbacks.

Selection membership uses a small hash set or sorted lookup created from the
selected IDs. Candidate source-face lookup uses the cached face index instead
of scanning `candidate.source_face_ids`.

No final template decision or generated patch geometry is cached. Those values
depend on retained faces, face sets, terminal-hexa availability, keep-hexa
controls, and distance scale, so each probe recomputes them from immutable
lookup data.

## Production Data Flow

For each candidate-rejection resolver invocation:

1. Construct one `ProvisionalTransitionBuildContext` from `effective_current`
   and `candidate.next_front`.
2. Capture it by reference in both `build_provisional` and
   `build_external_patches` callbacks.
3. Full resolver iterations and all local distance probes reuse the same
   context.
4. Destroy the context after `LayerTransitionResolver::resolve()` returns.

Rollback iterations reuse the context because they change retained/face-set
state, not the two front objects.

## Correctness and Error Handling

The optimization must not change:

- owned boundary triangles or their order where order is observable;
- resolved topology and terminal decisions;
- diagonal requirements;
- forced rollback IDs;
- collision-owner metadata;
- final retained faces or generated cell counts.

Compatibility wrappers and reusable-context calls must return the same error
variants. Context construction performs no partial publication and contains no
mutable probe state, so a failed local build cannot contaminate the next probe.

## Tests

1. Compare the compatibility wrapper and reusable-context full build for the
   existing mixed regular/external fixture.
2. Reuse one context for external patch scales `0.25`, `0.125`, and `0.0625`
   and compare every result with the wrapper path.
3. Change retained IDs and face sets while reusing the context and verify no
   stale topology or owner remains.
4. Add construction diagnostics or an injected counter proving production
   creates one context for one resolver call rather than once per probe.
5. Run transition, regular-layer, and collision tests.
6. Run the Release Benchmark and require layer 20 to add 215,050 cells. Report
   local-build time against the current 8.885 seconds on layer 20 and 17.192
   seconds on layer 19.

## Out of Scope

- Compiled per-patch template descriptors.
- Incremental exposed-boundary assembly across rollback iterations.
- Reusing the initial global collision scan for the external search index.
- Removing the final full collision verification.
- Changing the fixed 12-step bisection policy.

Those optimizations can build on this context after parity is established.
