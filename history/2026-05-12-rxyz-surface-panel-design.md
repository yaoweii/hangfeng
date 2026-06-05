# RXYZ Surface Panel Design

## Goal

Build a local generator that converts a planar `rxyz` control-point array into:

- a triangulated, closed manifold surface panel mesh where each edge belongs to exactly two triangles
- an `OBJ` export of that mesh
- companion `JSON` files containing the source controls and generated mesh data
- an `HTML` demo that visualizes the controls, sampled path, and final mesh

## Input Semantics

The input is a JSON array of control points. Each control point is stored as:

```json
[r, x, y, z]
```

Rules:

1. `rxyz[0][0]` is a special metadata value equal to the total number of control points.
2. All control points are treated as coplanar in a single working plane.
3. For `i` from `1` to `n - 2`, `rxyz[i][0]` defines the segment from point `i` to point `i + 1`:
   - `r = 0`: connect the points with a straight line segment
   - `r > 0`: connect the points with a planar circular arc of radius `r`
4. The last point does not define an outgoing segment. Its `r` value is kept as padding and is ignored by geometry generation.

## Geometry Interpretation

The control array defines a centerline path, not the surface directly.

Generation proceeds in two stages:

1. Resolve each pair of consecutive control points into either a straight segment or a circular arc segment.
2. Sweep a circular cross-section along the sampled centerline to produce a closed tube-like surface panel.

The sweep radius is a separate generation parameter and is not derived from the `rxyz` segment radius.

## Arc Construction

For an arc segment, the generator uses:

- start point `P0`
- end point `P1`
- radius `r`
- the global working plane normal

The chord length must satisfy:

```text
|P1 - P0| <= 2r
```

Otherwise the segment is invalid and generation stops with a descriptive error.

Given a valid radius, there are two possible circle centers in the plane. The generator will choose one deterministic side so repeated runs are stable. The chosen circle center defines:

- the sweep angle
- arc direction
- sampled positions along the segment

The implementation should prefer the minor arc so the path stays predictable.

## Centerline Sampling

The generator samples the resolved path into an ordered polyline.

Sampling rules:

- line segments are subdivided according to segment length and a target step size
- arc segments are subdivided according to arc length and a target angular or linear step size
- both segment types include shared endpoints exactly once in the final sampled centerline

The mesh generator consumes only this sampled centerline.

## Surface Panel Mesh

The surface panel is a closed swept tube.

At each sampled centerline point:

1. build a local frame using tangent plus a stable reference up vector projected into the working plane
2. generate one circular cross-section ring with a fixed number of samples
3. connect consecutive rings into triangle pairs

To enforce manifold topology:

- each quad between adjacent rings is triangulated into two triangles
- the start cap is triangulated as a fan
- the end cap is triangulated as a fan
- duplicate seam vertices are avoided; ring indexing wraps logically

This produces a watertight triangle mesh in which every undirected edge is adjacent to exactly two faces.

## Output Files

The project writes the following files:

### `data/panel-input.json`

Contains:

- the source `rxyz` array
- generator parameters such as tube radius, ring segments, and sampling density

### `data/panel-mesh.json`

Contains:

- the original `rxyz` array
- resolved segment metadata
- sampled centerline positions
- mesh vertices
- mesh triangle indices
- summary statistics such as segment count, sample count, vertex count, and triangle count

### `data/panel.obj`

Contains:

- mesh vertices as `v`
- triangle faces as `f`

The OBJ is intended for exchange with external modeling tools.

### `demo/index.html`

Contains a local visualization page.

### `demo/app.js`

Contains:

- data loading
- mesh construction for rendering
- camera and controls
- view toggles and stats display

### `demo/style.css`

Contains the demo styling.

## Demo Behavior

The demo should:

1. load `panel-mesh.json`
2. render the control points
3. render the sampled centerline
4. render the final triangle mesh
5. provide orbit-like camera interaction
6. support toggling between solid and wireframe views
7. show basic mesh statistics

Because direct local loading of OBJ can be awkward in browsers, the demo uses the JSON mesh data for rendering. The OBJ remains an export artifact.

## Default Parameters

Initial defaults:

- `tubeRadius`: separate numeric parameter for the swept surface thickness
- `ringSegments`: `16` or `24`, with `16` as the likely default
- `targetStep`: adaptive by segment length with a minimum floor to avoid degenerate sparse sampling

These defaults should be easy to change in one place.

## Validation

The generator must validate:

1. `rxyz[0][0]` matches the actual control point count
2. there are at least two geometry points after the metadata point
3. all points lie in the same plane within a small tolerance
4. each arc radius is valid for its chord length
5. sampling produces enough rings to build a mesh
6. the final face list uses only valid vertex indices

## Implementation Structure

Recommended files:

- `scripts/generate_panel.js`: generation entry point
- `src/panel/geometry.js`: vector math, plane checks, arc resolution, centerline sampling
- `src/panel/mesh.js`: tube sweep and OBJ serialization
- `src/panel/io.js`: JSON and file output helpers
- `tests/panel/generator.test.js`: path resolution, validation, and mesh topology tests
- `demo/index.html`
- `demo/app.js`
- `demo/style.css`

## Testing Requirements

Automated tests should prove:

1. straight segments sample correctly
2. arc segments sample correctly for a valid radius
3. invalid radii are rejected
4. metadata count mismatches are rejected
5. generated meshes contain only triangles
6. every undirected mesh edge is used by exactly two faces
7. OBJ export is structurally valid

## Out of Scope

This design does not include:

- arbitrary non-coplanar arc solving
- variable sweep radius along the tube
- self-intersection detection
- import of external CAD formats beyond OBJ
