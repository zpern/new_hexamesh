# CLI output and per-face debug log design

## Goal

Make normal CLI output focus on boundary classification and layer-generation progress, while moving detailed per-Wall-face termination diagnostics into an opt-in text file.

## Startup output

After the CGNS surface is read successfully, group input face tags by boundary kind. Print each non-empty region set once, with duplicate region IDs removed and IDs sorted ascending:

```text
symmetry_regions=3,5
internal_regions=7
wall_regions=1,2
far_regions=4
wall_faces=12800
```

Do not print a region line when that boundary kind has no faces. Print `wall_faces` as the total number of input faces tagged `Wall`; the line is always present, including when the count is zero.

## Generation progress

When `--multi-normal true` is selected, print this line immediately before multi-normal transition generation begins:

```text
Generating multi-normal boundary layer
```

Keep the existing regular-layer messages unchanged, including `generate N boundarylayer` and the matching completion line.

## Removed normal output

Remove the final CLI summary containing input sizes, generated cell counts, generation parameters, farfield face count, and aggregate `stop_*` counts. These values are not printed in normal or debug mode.

## Debug option and file

Add `--debuglog true|false`, accepting the existing Boolean spellings `true`, `false`, `1`, and `0`. Its default is `false`.

When false, no debug text file is created. When true, write `<output-prefix>_debug.txt` after successful generation. Include every Wall source face exactly once, sorted by the zero-based CGNS/source `source_face_id`. Each line has this stable format:

```text
0 VertexLayerLimit accepted_layers=5 stop_layer=6
1 Collision accepted_layers=2 stop_layer=3
2 None accepted_layers=0 stop_layer=0
```

Use the enum names `None`, `VertexLayerLimit`, `DegenerateCandidate`, `ReversedCandidate`, `LocallyInvertedCandidate`, `SkewnessExceeded`, `Collision`, `SlidingProjectionFailure`, `NeighborLayerConstraint`, and `IsotropicHeightReached`. `None` records remain in the file so its line count can be compared directly with `wall_faces`.

Create the debug file in the same output-directory setup used by the VTK files. A debug-file open or write failure is a command error and is reported on stderr.

## Testing

Extend the CLI integration test to verify:

- non-empty region sets, ascending/deduplicated IDs, and Wall face count;
- absent boundary kinds produce no line;
- the English multi-normal progress message appears only when enabled;
- the former final summary keys are absent;
- default debug mode creates no text file;
- enabled debug mode creates one sorted, zero-based record per Wall source face with reason, accepted layer count, and stop layer;
- invalid `--debuglog` values are rejected.

