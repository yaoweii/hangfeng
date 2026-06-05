# RXYZ Surface Panel Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a local generator that converts planar `rxyz` control data into a closed triangulated tube mesh, writes `JSON` and `OBJ` artifacts, and serves an `HTML` demo that visualizes the generated result.

**Architecture:** Implement the generator as small Node.js modules: geometry parsing and arc sampling, manifold tube mesh construction, and output serialization. Drive everything from a single script that writes `data/` artifacts, then keep the browser demo static by reading the generated mesh JSON directly.

**Tech Stack:** Node.js standard library, browser Canvas/WebGL without external package dependencies, plain HTML/CSS/JavaScript, Node test runner

---

## File Structure

- Create: `package.json`
- Create: `scripts/generate_panel.js`
- Create: `src/panel/geometry.js`
- Create: `src/panel/mesh.js`
- Create: `src/panel/io.js`
- Create: `tests/panel/generator.test.js`
- Create: `data/panel-input.json`
- Create: `demo/index.html`
- Create: `demo/app.js`
- Create: `demo/style.css`

Responsibilities:

- `package.json`: declare scripts for tests and artifact generation
- `src/panel/geometry.js`: validate `rxyz`, resolve straight/arc segments, sample the centerline
- `src/panel/mesh.js`: build a closed triangle tube and serialize OBJ
- `src/panel/io.js`: write JSON and OBJ artifacts
- `scripts/generate_panel.js`: orchestrate generation using default sample data and parameters
- `tests/panel/generator.test.js`: regression and topology tests
- `demo/*`: render generated data locally with interactive controls

### Task 1: Scaffold the Node project and failing validation test

**Files:**
- Create: `package.json`
- Create: `tests/panel/generator.test.js`

- [ ] **Step 1: Write the failing test**

```js
import test from 'node:test';
import assert from 'node:assert/strict';
import { validateRxyz } from '../../src/panel/geometry.js';

test('validateRxyz rejects a metadata count mismatch', () => {
  assert.throws(
    () => validateRxyz([
      [5, 0, 0, 0],
      [0, 0, 0, 0],
      [0, 10, 0, 0],
    ]),
    /count/i,
  );
});
```

- [ ] **Step 2: Run test to verify it fails**

Run: `node --test tests/panel/generator.test.js`
Expected: FAIL with a module-not-found or missing-export error for `../../src/panel/geometry.js`

- [ ] **Step 3: Write minimal project scaffolding**

```json
{
  "name": "rxyz-surface-panel",
  "version": "1.0.0",
  "type": "module",
  "private": true,
  "scripts": {
    "test": "node --test tests/panel/generator.test.js",
    "generate": "node scripts/generate_panel.js"
  }
}
```

```js
export function validateRxyz(rxyz) {
  const declaredCount = Number(rxyz?.[0]?.[0]);
  if (!Number.isInteger(declaredCount) || declaredCount !== rxyz.length) {
    throw new Error('rxyz metadata count does not match actual control point count');
  }
  return { count: declaredCount };
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `node --test tests/panel/generator.test.js`
Expected: PASS with `1` test passing

- [ ] **Step 5: Record workspace state**

Run: `Test-Path .git`
Expected: `False`

Because this workspace is not a git repository, do not add commit steps during execution. Preserve the generated file list in the final handoff instead.

### Task 2: Add failing tests for planar validation and straight-segment sampling

**Files:**
- Modify: `tests/panel/generator.test.js`
- Modify: `src/panel/geometry.js`

- [ ] **Step 1: Write the failing tests**

```js
import { samplePath, validateRxyz } from '../../src/panel/geometry.js';

test('validateRxyz rejects non-coplanar control points', () => {
  assert.throws(
    () => validateRxyz([
      [4, 0, 0, 0],
      [0, 0, 0, 0],
      [0, 10, 0, 0],
      [0, 20, 0, 1],
    ]),
    /coplanar/i,
  );
});

test('samplePath returns a straight polyline for a straight segment', () => {
  const result = samplePath({
    rxyz: [
      [3, 0, 0, 0],
      [0, 0, 0, 0],
      [0, 10, 0, 0],
    ],
    targetStep: 2.5,
  });

  assert.equal(result.points[0][0], 0);
  assert.equal(result.points.at(-1)[0], 10);
  assert.ok(result.points.length >= 5);
});
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `node --test tests/panel/generator.test.js`
Expected: FAIL because `samplePath` is not exported or because non-coplanar data is not rejected yet

- [ ] **Step 3: Write minimal implementation**

```js
const EPSILON = 1e-6;

function subtract(a, b) {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function cross(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

function length(v) {
  return Math.hypot(v[0], v[1], v[2]);
}

function normalize(v) {
  const len = length(v);
  return len <= EPSILON ? [0, 0, 0] : [v[0] / len, v[1] / len, v[2] / len];
}

function getGeometryPoints(rxyz) {
  return rxyz.slice(1).map(([, x, y, z]) => [x, y, z]);
}

export function validateRxyz(rxyz) {
  const declaredCount = Number(rxyz?.[0]?.[0]);
  if (!Number.isInteger(declaredCount) || declaredCount !== rxyz.length) {
    throw new Error('rxyz metadata count does not match actual control point count');
  }

  const points = getGeometryPoints(rxyz);
  if (points.length < 2) {
    throw new Error('rxyz must contain at least two geometry points');
  }

  const origin = points[0];
  const basisA = subtract(points[1], origin);
  let basisB = null;
  for (let i = 2; i < points.length; i += 1) {
    const candidate = subtract(points[i], origin);
    if (length(cross(basisA, candidate)) > EPSILON) {
      basisB = candidate;
      break;
    }
  }

  const normal = basisB ? normalize(cross(basisA, basisB)) : [0, 0, 1];
  for (const point of points) {
    const distance = Math.abs(dot(subtract(point, origin), normal));
    if (distance > 1e-5) {
      throw new Error('rxyz control points must be coplanar');
    }
  }

  return { count: declaredCount, points, origin, normal };
}

export function samplePath({ rxyz, targetStep }) {
  const { points } = validateRxyz(rxyz);
  const start = points[0];
  const end = points[1];
  const dx = end[0] - start[0];
  const dy = end[1] - start[1];
  const dz = end[2] - start[2];
  const segmentLength = Math.hypot(dx, dy, dz);
  const steps = Math.max(1, Math.ceil(segmentLength / targetStep));
  const sampled = [];

  for (let i = 0; i <= steps; i += 1) {
    const t = i / steps;
    sampled.push([
      start[0] + dx * t,
      start[1] + dy * t,
      start[2] + dz * t,
    ]);
  }

  return {
    points: sampled,
    segments: [{ type: 'line', startIndex: 0, endIndex: sampled.length - 1 }],
  };
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `node --test tests/panel/generator.test.js`
Expected: PASS with `3` tests passing

- [ ] **Step 5: Record progress**

Run: `Get-ChildItem -Recurse tests,src | Select-Object FullName`
Expected: shows `src/panel/geometry.js` and `tests/panel/generator.test.js`

### Task 3: Add a failing test for arc sampling and implement deterministic minor-arc resolution

**Files:**
- Modify: `tests/panel/generator.test.js`
- Modify: `src/panel/geometry.js`

- [ ] **Step 1: Write the failing test**

```js
test('samplePath resolves an arc segment when radius is non-zero', () => {
  const result = samplePath({
    rxyz: [
      [3, 0, 0, 0],
      [3, 0, 0, 0],
      [0, 3, 3, 0],
    ],
    targetStep: 0.5,
  });

  assert.equal(result.segments[0].type, 'arc');
  assert.deepEqual(result.points[0], [0, 0, 0]);
  assert.deepEqual(result.points.at(-1), [3, 3, 0]);
  assert.ok(result.points.length > 3);
});
```

- [ ] **Step 2: Run tests to verify it fails**

Run: `node --test tests/panel/generator.test.js`
Expected: FAIL because the segment is still treated as a straight line

- [ ] **Step 3: Write minimal implementation**

```js
function add(a, b) {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function scale(v, scalar) {
  return [v[0] * scalar, v[1] * scalar, v[2] * scalar];
}

function distance(a, b) {
  return length(subtract(a, b));
}

function rotateAroundNormal(vector, normal, angle) {
  const cos = Math.cos(angle);
  const sin = Math.sin(angle);
  const crossTerm = cross(normal, vector);
  const dotTerm = dot(normal, vector);
  return [
    vector[0] * cos + crossTerm[0] * sin + normal[0] * dotTerm * (1 - cos),
    vector[1] * cos + crossTerm[1] * sin + normal[1] * dotTerm * (1 - cos),
    vector[2] * cos + crossTerm[2] * sin + normal[2] * dotTerm * (1 - cos),
  ];
}

function resolveArcSegment(start, end, radius, normal, targetStep) {
  const chord = subtract(end, start);
  const chordLength = length(chord);
  if (chordLength / 2 > radius + EPSILON) {
    throw new Error('arc radius is too small for the segment chord');
  }

  const midpoint = scale(add(start, end), 0.5);
  const chordDir = normalize(chord);
  const sideDir = normalize(cross(normal, chordDir));
  const offset = Math.sqrt(Math.max(0, radius * radius - (chordLength * chordLength) / 4));
  const center = add(midpoint, scale(sideDir, offset));
  const startVector = subtract(start, center);
  const endVector = subtract(end, center);
  const sweep = Math.atan2(dot(normal, cross(startVector, endVector)), dot(startVector, endVector));
  const minorSweep = sweep >= 0 ? sweep : (Math.PI * 2 + sweep);
  const arcLength = radius * minorSweep;
  const steps = Math.max(2, Math.ceil(arcLength / targetStep));
  const points = [];

  for (let i = 0; i <= steps; i += 1) {
    const angle = (minorSweep * i) / steps;
    points.push(add(center, rotateAroundNormal(startVector, normal, angle)));
  }

  return { center, radius, points, sweep: minorSweep };
}
```

Update `samplePath` so it:

- reads each segment radius from `rxyz[i][0]`
- uses `resolveArcSegment` when `radius > 0`
- deduplicates shared endpoints when concatenating segment samples
- records `type: 'arc'` with radius and sweep metadata

- [ ] **Step 4: Run tests to verify they pass**

Run: `node --test tests/panel/generator.test.js`
Expected: PASS with `4` tests passing

- [ ] **Step 5: Record progress**

Run: `node --input-type=module -e "import { samplePath } from './src/panel/geometry.js'; console.log(samplePath({ rxyz:[[3,0,0,0],[3,0,0,0],[0,3,3,0]], targetStep:0.5 }).segments[0].type)"`
Expected: prints `arc`

### Task 4: Add a failing test for invalid arc radii and full multi-segment sampling

**Files:**
- Modify: `tests/panel/generator.test.js`
- Modify: `src/panel/geometry.js`

- [ ] **Step 1: Write the failing tests**

```js
test('samplePath rejects an arc whose radius is smaller than half the chord', () => {
  assert.throws(
    () => samplePath({
      rxyz: [
        [3, 0, 0, 0],
        [1, 0, 0, 0],
        [0, 4, 0, 0],
      ],
      targetStep: 0.5,
    }),
    /radius is too small/i,
  );
});

test('samplePath concatenates line and arc segments into one centerline', () => {
  const result = samplePath({
    rxyz: [
      [4, 0, 0, 0],
      [0, 0, 0, 0],
      [2, 4, 0, 0],
      [0, 4, 4, 0],
    ],
    targetStep: 0.5,
  });

  assert.equal(result.segments.length, 2);
  assert.equal(result.segments[0].type, 'line');
  assert.equal(result.segments[1].type, 'arc');
  assert.deepEqual(result.points[0], [0, 0, 0]);
  assert.deepEqual(result.points.at(-1), [4, 4, 0]);
});
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `node --test tests/panel/generator.test.js`
Expected: FAIL because only the first segment is sampled or the invalid radius is not rejected consistently

- [ ] **Step 3: Write minimal implementation**

```js
export function samplePath({ rxyz, targetStep }) {
  const validated = validateRxyz(rxyz);
  const geometryRows = rxyz.slice(1);
  const points = validated.points;
  const sampled = [];
  const segments = [];
  let sampleIndex = 0;

  for (let i = 0; i < points.length - 1; i += 1) {
    const start = points[i];
    const end = points[i + 1];
    const segmentRadius = Number(geometryRows[i][0]);
    const segmentResult = segmentRadius > 0
      ? { type: 'arc', ...resolveArcSegment(start, end, segmentRadius, validated.normal, targetStep) }
      : { type: 'line', points: sampleLineSegment(start, end, targetStep) };

    const segmentPoints = i === 0 ? segmentResult.points : segmentResult.points.slice(1);
    sampled.push(...segmentPoints);
    segments.push({
      type: segmentResult.type,
      radius: segmentRadius,
      startIndex: sampleIndex,
      endIndex: sampleIndex + segmentPoints.length - 1,
    });
    sampleIndex = sampled.length;
  }

  return { points: sampled, segments, normal: validated.normal };
}
```

Also factor the straight-line logic into:

```js
function sampleLineSegment(start, end, targetStep) {
  const delta = subtract(end, start);
  const segmentLength = length(delta);
  const steps = Math.max(1, Math.ceil(segmentLength / targetStep));
  const sampled = [];
  for (let i = 0; i <= steps; i += 1) {
    const t = i / steps;
    sampled.push(add(start, scale(delta, t)));
  }
  return sampled;
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `node --test tests/panel/generator.test.js`
Expected: PASS with `6` tests passing

- [ ] **Step 5: Record progress**

Run: `node --input-type=module -e "import { samplePath } from './src/panel/geometry.js'; console.log(samplePath({ rxyz:[[4,0,0,0],[0,0,0,0],[2,4,0,0],[0,4,4,0]], targetStep:0.5 }).segments.length)"`
Expected: prints `2`

### Task 5: Add a failing mesh topology test and implement closed triangle tube generation

**Files:**
- Modify: `tests/panel/generator.test.js`
- Create: `src/panel/mesh.js`

- [ ] **Step 1: Write the failing test**

```js
import { buildTubeMesh, countUndirectedEdgeUsage } from '../../src/panel/mesh.js';

test('buildTubeMesh creates only triangles and every edge belongs to two faces', () => {
  const path = samplePath({
    rxyz: [
      [3, 0, 0, 0],
      [0, 0, 0, 0],
      [0, 10, 0, 0],
    ],
    targetStep: 2,
  });

  const mesh = buildTubeMesh({
    points: path.points,
    normal: path.normal,
    tubeRadius: 0.5,
    ringSegments: 8,
  });

  assert.ok(mesh.faces.every((face) => face.length === 3));
  const edgeUsage = countUndirectedEdgeUsage(mesh.faces);
  assert.ok([...edgeUsage.values()].every((count) => count === 2));
});
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `node --test tests/panel/generator.test.js`
Expected: FAIL with a module-not-found error for `../../src/panel/mesh.js`

- [ ] **Step 3: Write minimal implementation**

```js
function normalize(v) {
  const len = Math.hypot(v[0], v[1], v[2]);
  return len === 0 ? [0, 0, 0] : [v[0] / len, v[1] / len, v[2] / len];
}

function subtract(a, b) {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

function cross(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function add(a, b) {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

function scale(v, scalar) {
  return [v[0] * scalar, v[1] * scalar, v[2] * scalar];
}

function buildFrame(points, index, normal) {
  const prev = points[Math.max(0, index - 1)];
  const next = points[Math.min(points.length - 1, index + 1)];
  const tangent = normalize(subtract(next, prev));
  const side = normalize(cross(normal, tangent));
  const binormal = normalize(cross(tangent, side));
  return { side, binormal };
}

export function buildTubeMesh({ points, normal, tubeRadius, ringSegments }) {
  const vertices = [];
  const faces = [];
  const ringStarts = [];

  for (let i = 0; i < points.length; i += 1) {
    ringStarts.push(vertices.length);
    const frame = buildFrame(points, i, normal);
    for (let j = 0; j < ringSegments; j += 1) {
      const angle = (Math.PI * 2 * j) / ringSegments;
      const radial = add(
        scale(frame.side, Math.cos(angle) * tubeRadius),
        scale(frame.binormal, Math.sin(angle) * tubeRadius),
      );
      vertices.push(add(points[i], radial));
    }
  }

  const startCenterIndex = vertices.length;
  vertices.push(points[0]);
  const endCenterIndex = vertices.length;
  vertices.push(points.at(-1));

  for (let i = 0; i < ringStarts.length - 1; i += 1) {
    const ringA = ringStarts[i];
    const ringB = ringStarts[i + 1];
    for (let j = 0; j < ringSegments; j += 1) {
      const next = (j + 1) % ringSegments;
      faces.push([ringA + j, ringB + j, ringB + next]);
      faces.push([ringA + j, ringB + next, ringA + next]);
    }
  }

  for (let j = 0; j < ringSegments; j += 1) {
    const next = (j + 1) % ringSegments;
    faces.push([startCenterIndex, ringStarts[0] + next, ringStarts[0] + j]);
    const lastRing = ringStarts.at(-1);
    faces.push([endCenterIndex, lastRing + j, lastRing + next]);
  }

  return { vertices, faces };
}

export function countUndirectedEdgeUsage(faces) {
  const usage = new Map();
  for (const face of faces) {
    for (let i = 0; i < face.length; i += 1) {
      const a = face[i];
      const b = face[(i + 1) % face.length];
      const key = a < b ? `${a}:${b}` : `${b}:${a}`;
      usage.set(key, (usage.get(key) ?? 0) + 1);
    }
  }
  return usage;
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `node --test tests/panel/generator.test.js`
Expected: PASS with `7` tests passing

- [ ] **Step 5: Record progress**

Run: `node --input-type=module -e "import { buildTubeMesh } from './src/panel/mesh.js'; console.log(buildTubeMesh({ points:[[0,0,0],[5,0,0]], normal:[0,0,1], tubeRadius:0.5, ringSegments:8 }).faces.length)"`
Expected: prints a positive integer

### Task 6: Add a failing OBJ serialization test and implement output helpers

**Files:**
- Modify: `tests/panel/generator.test.js`
- Create: `src/panel/io.js`
- Modify: `src/panel/mesh.js`

- [ ] **Step 1: Write the failing test**

```js
import { toObj } from '../../src/panel/mesh.js';

test('toObj writes vertices and triangle faces in OBJ format', () => {
  const obj = toObj({
    vertices: [[0, 0, 0], [1, 0, 0], [0, 1, 0]],
    faces: [[0, 1, 2]],
  });

  assert.match(obj, /^v 0 0 0/m);
  assert.match(obj, /^f 1 2 3/m);
});
```

- [ ] **Step 2: Run tests to verify it fails**

Run: `node --test tests/panel/generator.test.js`
Expected: FAIL because `toObj` is not exported yet

- [ ] **Step 3: Write minimal implementation**

```js
import { mkdir, writeFile } from 'node:fs/promises';
import path from 'node:path';

export function toObj({ vertices, faces }) {
  const lines = [];
  for (const [x, y, z] of vertices) {
    lines.push(`v ${x} ${y} ${z}`);
  }
  for (const [a, b, c] of faces) {
    lines.push(`f ${a + 1} ${b + 1} ${c + 1}`);
  }
  return `${lines.join('\n')}\n`;
}

export async function ensureDir(dirPath) {
  await mkdir(dirPath, { recursive: true });
}

export async function writeJson(filePath, value) {
  await ensureDir(path.dirname(filePath));
  await writeFile(filePath, `${JSON.stringify(value, null, 2)}\n`, 'utf8');
}

export async function writeText(filePath, text) {
  await ensureDir(path.dirname(filePath));
  await writeFile(filePath, text, 'utf8');
}
```

Export `toObj` from `src/panel/mesh.js` or move the function there so the test import remains valid.

- [ ] **Step 4: Run tests to verify they pass**

Run: `node --test tests/panel/generator.test.js`
Expected: PASS with `8` tests passing

- [ ] **Step 5: Record progress**

Run: `node --input-type=module -e "import { toObj } from './src/panel/mesh.js'; console.log(toObj({ vertices:[[0,0,0],[1,0,0],[0,1,0]], faces:[[0,1,2]] }).trim())"`
Expected: prints one `v` block and one `f 1 2 3` line

### Task 7: Add an end-to-end failing test and implement the generator entry script plus default input data

**Files:**
- Modify: `tests/panel/generator.test.js`
- Create: `scripts/generate_panel.js`
- Create: `data/panel-input.json`
- Modify: `src/panel/geometry.js`
- Modify: `src/panel/mesh.js`
- Modify: `src/panel/io.js`

- [ ] **Step 1: Write the failing test**

```js
import { access, mkdtemp, readFile } from 'node:fs/promises';
import os from 'node:os';
import path from 'node:path';
import { generatePanelArtifacts } from '../../scripts/generate_panel.js';

test('generatePanelArtifacts writes JSON and OBJ outputs', async () => {
  const outputDir = await mkdtemp(path.join(os.tmpdir(), 'panel-'));
  await generatePanelArtifacts({
    input: {
      rxyz: [
        [4, 0, 0, 0],
        [0, 0, 0, 0],
        [3, 4, 0, 0],
        [0, 8, 4, 0],
      ],
      tubeRadius: 0.5,
      ringSegments: 12,
      targetStep: 0.5,
    },
    rootDir: outputDir,
  });

  await access(path.join(outputDir, 'data', 'panel-input.json'));
  await access(path.join(outputDir, 'data', 'panel-mesh.json'));
  await access(path.join(outputDir, 'data', 'panel.obj'));
  const meshJson = JSON.parse(await readFile(path.join(outputDir, 'data', 'panel-mesh.json'), 'utf8'));
  assert.ok(meshJson.vertices.length > 0);
  assert.ok(meshJson.faces.length > 0);
});
```

- [ ] **Step 2: Run tests to verify it fails**

Run: `node --test tests/panel/generator.test.js`
Expected: FAIL because `generatePanelArtifacts` does not exist yet

- [ ] **Step 3: Write minimal implementation**

```js
import path from 'node:path';
import { samplePath } from '../src/panel/geometry.js';
import { buildTubeMesh, toObj } from '../src/panel/mesh.js';
import { writeJson, writeText } from '../src/panel/io.js';

const defaultInput = {
  rxyz: [
    [5, 0, 0, 0],
    [0, 0, 0, 0],
    [3, 4, 0, 0],
    [0, 8, 4, 0],
    [0, 12, 4, 0],
  ],
  tubeRadius: 0.45,
  ringSegments: 16,
  targetStep: 0.4,
};

export async function generatePanelArtifacts({ input = defaultInput, rootDir = process.cwd() } = {}) {
  const pathResult = samplePath({
    rxyz: input.rxyz,
    targetStep: input.targetStep,
  });
  const mesh = buildTubeMesh({
    points: pathResult.points,
    normal: pathResult.normal,
    tubeRadius: input.tubeRadius,
    ringSegments: input.ringSegments,
  });

  const meshPayload = {
    rxyz: input.rxyz,
    segments: pathResult.segments,
    sampledPath: pathResult.points,
    normal: pathResult.normal,
    vertices: mesh.vertices,
    faces: mesh.faces,
    stats: {
      segmentCount: pathResult.segments.length,
      sampleCount: pathResult.points.length,
      vertexCount: mesh.vertices.length,
      triangleCount: mesh.faces.length,
    },
  };

  await writeJson(path.join(rootDir, 'data', 'panel-input.json'), input);
  await writeJson(path.join(rootDir, 'data', 'panel-mesh.json'), meshPayload);
  await writeText(path.join(rootDir, 'data', 'panel.obj'), toObj(mesh));

  return { input, mesh: meshPayload };
}

if (import.meta.url === `file://${process.argv[1]}`) {
  generatePanelArtifacts().catch((error) => {
    console.error(error);
    process.exitCode = 1;
  });
}
```

Create `data/panel-input.json` with the same `defaultInput` object so the workspace contains a readable seed file.

- [ ] **Step 4: Run tests to verify they pass**

Run: `node --test tests/panel/generator.test.js`
Expected: PASS with `9` tests passing

- [ ] **Step 5: Record progress**

Run: `node scripts/generate_panel.js`
Expected: exit `0` and write `data/panel-input.json`, `data/panel-mesh.json`, and `data/panel.obj`

### Task 8: Add a failing demo smoke test and implement the HTML demo

**Files:**
- Modify: `tests/panel/generator.test.js`
- Create: `demo/index.html`
- Create: `demo/app.js`
- Create: `demo/style.css`

- [ ] **Step 1: Write the failing smoke test**

```js
test('demo files exist and reference the generated mesh payload', async () => {
  const html = await readFile(new URL('../../demo/index.html', import.meta.url), 'utf8');
  const app = await readFile(new URL('../../demo/app.js', import.meta.url), 'utf8');
  assert.match(html, /canvas/i);
  assert.match(app, /panel-mesh\.json/);
  assert.match(app, /wireframe/i);
});
```

- [ ] **Step 2: Run tests to verify it fails**

Run: `node --test tests/panel/generator.test.js`
Expected: FAIL because the demo files do not exist yet

- [ ] **Step 3: Write minimal implementation**

Create `demo/index.html` with:

```html
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>RXYZ Surface Panel Demo</title>
    <link rel="stylesheet" href="./style.css">
  </head>
  <body>
    <aside class="hud">
      <h1>RXYZ Surface Panel</h1>
      <div id="stats"></div>
      <label><input id="toggle-wireframe" type="checkbox"> wireframe</label>
      <label><input id="toggle-path" type="checkbox" checked> path</label>
      <label><input id="toggle-controls" type="checkbox" checked> control points</label>
    </aside>
    <canvas id="viewport"></canvas>
    <script type="module" src="./app.js"></script>
  </body>
</html>
```

Create `demo/app.js` that:

- fetches `../data/panel-mesh.json`
- projects 3D points into a simple orbit camera view
- draws triangle fills, optional wireframe, sampled path, and control points on a `<canvas>`
- updates stats from `mesh.stats`
- supports drag-to-rotate and wheel-to-zoom

Create `demo/style.css` that:

- uses a full-screen canvas layout
- keeps a readable overlay for stats and toggles
- avoids external assets or frameworks

- [ ] **Step 4: Run tests to verify they pass**

Run: `node --test tests/panel/generator.test.js`
Expected: PASS with `10` tests passing

- [ ] **Step 5: Record progress**

Run: `Get-ChildItem demo | Select-Object Name`
Expected: shows `index.html`, `app.js`, and `style.css`

### Task 9: Run full verification and produce final handoff artifacts

**Files:**
- Verify: `tests/panel/generator.test.js`
- Verify: `data/panel-input.json`
- Verify: `data/panel-mesh.json`
- Verify: `data/panel.obj`
- Verify: `demo/index.html`
- Verify: `demo/app.js`
- Verify: `demo/style.css`

- [ ] **Step 1: Regenerate artifacts from the checked-in default data**

Run: `node scripts/generate_panel.js`
Expected: exit `0` and refresh the three files under `data/`

- [ ] **Step 2: Run the complete automated test suite**

Run: `node --test tests/panel/generator.test.js`
Expected: PASS with `10` tests passing and `0` failures

- [ ] **Step 3: Verify output files exist**

Run: `Get-ChildItem data,demo | Select-Object FullName,Length`
Expected: lists all expected JSON, OBJ, HTML, JS, and CSS files with non-zero lengths

- [ ] **Step 4: Verify OBJ structure**

Run: `Select-String -Path 'data/panel.obj' -Pattern '^v |^f '`
Expected: returns both vertex lines and face lines

- [ ] **Step 5: Final handoff**

Report:

- generated file paths
- mesh statistics from `data/panel-mesh.json`
- that the workspace is not a git repository, so no commit was created
