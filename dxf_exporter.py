from __future__ import annotations

import json
import math
import shutil
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable


Point3 = tuple[float, float, float]
Triangle = tuple[Point3, Point3, Point3]
DEFAULT_SMOOTH_ANGLE_DEGREES = 50.0


@dataclass
class BlockMesh:
    name: str
    triangles: list[Triangle]
    mesh_polylines: list[list[Point3]] = field(default_factory=list)


def _pair_stream(lines: list[str], start: int) -> Iterable[tuple[str, str]]:
    index = start
    while index + 1 < len(lines):
        code = lines[index].strip()
        if code == "0":
            break
        yield code, lines[index + 1].strip()
        index += 2


def _read_point(data: dict[str, str], codes: tuple[str, str, str]) -> Point3:
    return (float(data[codes[0]]), float(data[codes[1]]), float(data[codes[2]]))


def parse_dxf_blocks(lines: list[str]) -> dict[str, BlockMesh]:
    blocks: dict[str, BlockMesh] = {}
    current_name: str | None = None
    index = 0

    while index + 1 < len(lines):
        if lines[index].strip() != "0":
            index += 1
            continue

        entity_type = lines[index + 1].strip()
        if entity_type == "BLOCK":
            current_name = None
            for code, value in _pair_stream(lines, index + 2):
                if code == "2":
                    current_name = value
                    blocks.setdefault(current_name, BlockMesh(name=current_name, triangles=[]))
                    break
        elif entity_type == "ENDBLK":
            current_name = None
        elif current_name and entity_type == "3DFACE":
            data = {code: value for code, value in _pair_stream(lines, index + 2)}
            p1 = _read_point(data, ("10", "20", "30"))
            p2 = _read_point(data, ("11", "21", "31"))
            p3 = _read_point(data, ("12", "22", "32"))
            blocks[current_name].triangles.append((p1, p2, p3))
        elif current_name and entity_type == "POLYLINE":
            polyline: list[Point3] = []
            cursor = index + 2
            while cursor + 1 < len(lines):
                if lines[cursor].strip() == "0" and lines[cursor + 1].strip() == "SEQEND":
                    break
                if lines[cursor].strip() == "0" and lines[cursor + 1].strip() == "VERTEX":
                    data = {code: value for code, value in _pair_stream(lines, cursor + 2)}
                    if {"10", "20", "30"} <= set(data):
                        polyline.append(_read_point(data, ("10", "20", "30")))
                cursor += 1
            if polyline:
                blocks[current_name].mesh_polylines.append(polyline)

        index += 1

    return blocks


def _canonical_edge(a: Point3, b: Point3) -> tuple[Point3, Point3]:
    return (a, b) if a <= b else (b, a)


def _triangle_normal(triangle: Triangle) -> Point3 | None:
    a, b, c = triangle
    return _unit(_cross(_vector_sub(b, a), _vector_sub(c, a)))


def _trace_loops(boundary_graph: dict[Point3, set[Point3]]) -> list[list[Point3]]:
    visited_edges: set[tuple[Point3, Point3]] = set()
    loops: list[list[Point3]] = []

    for start in boundary_graph:
        for neighbor in sorted(boundary_graph[start]):
            edge = _canonical_edge(start, neighbor)
            if edge in visited_edges:
                continue

            loop = [start]
            current = start
            previous: Point3 | None = None

            while True:
                candidates = sorted(boundary_graph[current])
                next_point = None
                for candidate in candidates:
                    if candidate != previous and _canonical_edge(current, candidate) not in visited_edges:
                        next_point = candidate
                        break
                if next_point is None:
                    break

                visited_edges.add(_canonical_edge(current, next_point))
                previous, current = current, next_point
                if current == start:
                    break
                loop.append(current)

            if len(loop) >= 3 and current == start:
                loops.append(loop)

    loops.sort(key=len, reverse=True)
    return loops


def _normalize_loop_key(loop: list[Point3]) -> tuple[Point3, ...]:
    return tuple(sorted(loop))


def _loop_perimeter(loop: list[Point3]) -> float:
    if len(loop) < 2:
        return 0.0
    perimeter = 0.0
    for index, point in enumerate(loop):
        next_point = loop[(index + 1) % len(loop)]
        perimeter += _norm(_vector_sub(next_point, point))
    return perimeter


def _merge_boundary_loops(*loop_groups: list[list[Point3]]) -> list[list[Point3]]:
    merged: list[list[Point3]] = []
    seen_keys: set[tuple[Point3, ...]] = set()

    for loops in loop_groups:
        for loop in loops:
            if len(loop) < 3:
                continue
            loop_key = _normalize_loop_key(loop)
            if loop_key in seen_keys:
                continue
            seen_keys.add(loop_key)
            merged.append(loop)

    merged.sort(key=lambda loop: (len(loop), _loop_perimeter(loop)), reverse=True)
    return merged


def _smooth_region_boundary_loops(
    triangles: list[Triangle],
    smooth_angle_degrees: float = DEFAULT_SMOOTH_ANGLE_DEGREES,
    max_regions: int = 2,
) -> list[list[Point3]]:
    if not triangles:
        return []

    normals = [_triangle_normal(triangle) for triangle in triangles]
    edge_to_triangles: dict[tuple[Point3, Point3], list[int]] = defaultdict(list)
    for triangle_index, (a, b, c) in enumerate(triangles):
        for start, end in ((a, b), (b, c), (c, a)):
            edge_to_triangles[_canonical_edge(start, end)].append(triangle_index)

    smooth_cosine = math.cos(math.radians(smooth_angle_degrees))
    adjacency: list[set[int]] = [set() for _ in triangles]
    for triangle_indices in edge_to_triangles.values():
        if len(triangle_indices) != 2:
            continue
        first_index, second_index = triangle_indices
        first_normal = normals[first_index]
        second_normal = normals[second_index]
        if not first_normal or not second_normal:
            continue
        if abs(_dot(first_normal, second_normal)) < smooth_cosine:
            continue
        adjacency[first_index].add(second_index)
        adjacency[second_index].add(first_index)

    components: list[list[int]] = []
    seen: set[int] = set()
    for triangle_index in range(len(triangles)):
        if triangle_index in seen:
            continue
        stack = [triangle_index]
        seen.add(triangle_index)
        component: list[int] = []
        while stack:
            current = stack.pop()
            component.append(current)
            for neighbor in adjacency[current]:
                if neighbor not in seen:
                    seen.add(neighbor)
                    stack.append(neighbor)
        components.append(component)

    components.sort(key=len, reverse=True)
    component_index: dict[int, int] = {}
    for index, component in enumerate(components):
        for triangle_index in component:
            component_index[triangle_index] = index

    region_loops: list[list[Point3]] = []
    seen_loop_keys: set[tuple[Point3, ...]] = set()
    for region_index, component in enumerate(components[:max_regions]):
        if len(component) < 4:
            continue
        boundary_graph: dict[Point3, set[Point3]] = defaultdict(set)
        for edge, triangle_indices in edge_to_triangles.items():
            if len(triangle_indices) != 2:
                continue
            first_region = component_index[triangle_indices[0]]
            second_region = component_index[triangle_indices[1]]
            if (first_region == region_index) == (second_region == region_index):
                continue
            start, end = edge
            boundary_graph[start].add(end)
            boundary_graph[end].add(start)

        for loop in _trace_loops(boundary_graph):
            if len(loop) < 3:
                continue
            normalized_key = tuple(sorted(loop))
            if normalized_key in seen_loop_keys:
                continue
            seen_loop_keys.add(normalized_key)
            region_loops.append(loop)

    region_loops.sort(key=len, reverse=True)
    return region_loops[:2]


def reconstruct_boundary_loops(triangles: list[Triangle]) -> list[list[Point3]]:
    edge_counts: Counter[tuple[Point3, Point3]] = Counter()
    boundary_graph: dict[Point3, set[Point3]] = defaultdict(set)

    for a, b, c in triangles:
        for start, end in ((a, b), (b, c), (c, a)):
            edge_counts[_canonical_edge(start, end)] += 1

    for edge, count in edge_counts.items():
        if count != 1:
            continue
        a, b = edge
        boundary_graph[a].add(b)
        boundary_graph[b].add(a)

    topology_loops = _trace_loops(boundary_graph)
    feature_loops = _smooth_region_boundary_loops(triangles)
    return _merge_boundary_loops(topology_loops, feature_loops)


def build_indexed_mesh(triangles: list[Triangle]) -> tuple[list[Point3], list[tuple[int, int, int]]]:
    vertices: list[Point3] = []
    vertex_index: dict[Point3, int] = {}
    indexed_triangles: list[tuple[int, int, int]] = []

    for triangle in triangles:
        face_indices = []
        for point in triangle:
            if point not in vertex_index:
                vertex_index[point] = len(vertices)
                vertices.append(point)
            face_indices.append(vertex_index[point])
        indexed_triangles.append(tuple(face_indices))  # type: ignore[arg-type]

    return vertices, indexed_triangles


def build_obj_text(mesh: BlockMesh) -> str:
    vertices, triangles = build_indexed_mesh(mesh.triangles)
    lines = [f"o {mesh.name}"]
    for x, y, z in vertices:
        lines.append(f"v {x:.6f} {y:.6f} {z:.6f}")
    for a, b, c in triangles:
        lines.append(f"f {a + 1} {b + 1} {c + 1}")
    return "\n".join(lines) + "\n"


def _vector_sub(a: Point3, b: Point3) -> Point3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _cross(a: Point3, b: Point3) -> Point3:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _dot(a: Point3, b: Point3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(vector: Point3) -> float:
    return math.sqrt(_dot(vector, vector))


def _unit(vector: Point3) -> Point3 | None:
    length = _norm(vector)
    if length <= 1e-9:
        return None
    return (vector[0] / length, vector[1] / length, vector[2] / length)


def _fit_plane(points: list[Point3]) -> tuple[Point3 | None, Point3 | None, float]:
    unique_points = list(dict.fromkeys(points))
    if len(unique_points) < 3:
        return None, None, 0.0

    origin = unique_points[0]
    normal = None
    for first in unique_points[1:]:
        for second in unique_points[2:]:
            cross = _cross(_vector_sub(first, origin), _vector_sub(second, origin))
            normal = _unit(cross)
            if normal:
                break
        if normal:
            break

    if not normal:
        return origin, None, 0.0

    max_distance = max(abs(_dot(_vector_sub(point, origin), normal)) for point in unique_points)
    return origin, normal, max_distance


def _loop_plane_offset(loop: list[Point3], normal: Point3) -> float:
    return sum(_dot(point, normal) for point in loop) / len(loop)


def _triangles_are_coplanar(triangles: list[Triangle], alignment_threshold: float = 0.999) -> bool:
    reference_normal = None
    for triangle in triangles:
        normal = _triangle_normal(triangle)
        if not normal:
            continue
        if reference_normal is None:
            reference_normal = normal
            continue
        if abs(_dot(reference_normal, normal)) < alignment_threshold:
            return False
    return reference_normal is not None


def classify_panel(triangles: list[Triangle], boundary_loops: list[list[Point3]]) -> str:
    if _triangles_are_coplanar(triangles):
        return "plane"

    if len(boundary_loops) < 2:
        return "surface"

    reference_normal = None
    plane_offsets: list[float] = []
    for loop in boundary_loops:
        origin, normal, residual = _fit_plane(loop)
        if not origin or not normal or residual > 1e-3:
            return "surface"
        if reference_normal is None:
            reference_normal = normal
        alignment = _dot(reference_normal, normal)
        if abs(alignment) < 0.999:
            return "surface"
        if alignment < 0.0:
            normal = (-normal[0], -normal[1], -normal[2])
        offset = _loop_plane_offset(loop, normal)
        matched_offset = None
        for existing_offset in plane_offsets:
            if abs(existing_offset - offset) <= 1e-3:
                matched_offset = existing_offset
                break
        if matched_offset is None:
            plane_offsets.append(offset)
            if len(plane_offsets) > 2:
                return "surface"

    if len(plane_offsets) != 2:
        return "surface"
    if abs(plane_offsets[0] - plane_offsets[1]) <= 1e-6:
        return "surface"
    return "plane"


def compute_bbox(points: list[Point3]) -> dict[str, float]:
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    zs = [point[2] for point in points]
    return {
        "min_x": min(xs),
        "max_x": max(xs),
        "min_y": min(ys),
        "max_y": max(ys),
        "min_z": min(zs),
        "max_z": max(zs),
    }


def _round_scalar(value: float, digits: int = 6) -> float:
    return round(float(value), digits)


def _round_point(point: Point3) -> list[float]:
    return [round(point[0], 6), round(point[1], 6), round(point[2], 6)]


def _round_point_2d(point: tuple[float, float]) -> list[float]:
    return [_round_scalar(point[0]), _round_scalar(point[1])]


def _points_close(a: Point3, b: Point3, epsilon: float = 1.0e-6) -> bool:
    return _norm(_vector_sub(a, b)) <= epsilon


def _dedupe_loop_points(points: list[Point3]) -> list[Point3]:
    if not points:
        return []

    cleaned = [points[0]]
    for point in points[1:]:
        if _points_close(point, cleaned[-1]):
            continue
        cleaned.append(point)

    if len(cleaned) > 1 and _points_close(cleaned[0], cleaned[-1]):
        cleaned.pop()

    return cleaned


def build_sampled_rxyz_curve(points: list[Point3]) -> list[list[float]]:
    cleaned = _dedupe_loop_points(points)
    if not cleaned:
        return []

    geometry = cleaned + [cleaned[0]]
    curve = [[0.0, *_round_point(point)] for point in geometry]
    header = [float(len(curve) + 1), 0.0, 0.0, 0.0]
    return [header, *curve]


def _solve_linear_system_3x3(matrix: list[list[float]], vector: list[float]) -> list[float] | None:
    augmented = [row[:] + [rhs] for row, rhs in zip(matrix, vector)]
    size = 3

    for pivot_index in range(size):
        pivot_row = max(range(pivot_index, size), key=lambda index: abs(augmented[index][pivot_index]))
        pivot_value = augmented[pivot_row][pivot_index]
        if abs(pivot_value) <= 1.0e-12:
            return None
        if pivot_row != pivot_index:
            augmented[pivot_index], augmented[pivot_row] = augmented[pivot_row], augmented[pivot_index]

        factor = augmented[pivot_index][pivot_index]
        for column_index in range(pivot_index, size + 1):
            augmented[pivot_index][column_index] /= factor

        for row_index in range(size):
            if row_index == pivot_index:
                continue
            elimination = augmented[row_index][pivot_index]
            if abs(elimination) <= 1.0e-12:
                continue
            for column_index in range(pivot_index, size + 1):
                augmented[row_index][column_index] -= elimination * augmented[pivot_index][column_index]

    return [augmented[index][size] for index in range(size)]


def _fit_circle_2d(points: list[tuple[float, float]]) -> tuple[tuple[float, float], float] | None:
    if len(points) < 3:
        return None

    sum_x = sum(point[0] for point in points)
    sum_y = sum(point[1] for point in points)
    sum_xx = sum(point[0] * point[0] for point in points)
    sum_yy = sum(point[1] * point[1] for point in points)
    sum_xy = sum(point[0] * point[1] for point in points)
    sum_z = sum(point[0] * point[0] + point[1] * point[1] for point in points)
    sum_xz = sum(point[0] * (point[0] * point[0] + point[1] * point[1]) for point in points)
    sum_yz = sum(point[1] * (point[0] * point[0] + point[1] * point[1]) for point in points)
    count = float(len(points))

    coefficients = _solve_linear_system_3x3(
        [
            [sum_xx, sum_xy, sum_x],
            [sum_xy, sum_yy, sum_y],
            [sum_x, sum_y, count],
        ],
        [-sum_xz, -sum_yz, -sum_z],
    )
    if not coefficients:
        return None

    a, b, c = coefficients
    center = (-0.5 * a, -0.5 * b)
    radius_sq = center[0] * center[0] + center[1] * center[1] - c
    if radius_sq <= 1.0e-12:
        return None

    return center, math.sqrt(radius_sq)


def _point_line_distance_2d(point: tuple[float, float],
                            start: tuple[float, float],
                            end: tuple[float, float]) -> float:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 1.0e-12:
        return math.hypot(point[0] - start[0], point[1] - start[1])

    numerator = abs(dy * point[0] - dx * point[1] + end[0] * start[1] - end[1] * start[0])
    return numerator / math.sqrt(length_sq)


def _circumradius_2d(a: tuple[float, float],
                     b: tuple[float, float],
                     c: tuple[float, float]) -> float | None:
    ab = math.hypot(b[0] - a[0], b[1] - a[1])
    bc = math.hypot(c[0] - b[0], c[1] - b[1])
    ca = math.hypot(a[0] - c[0], a[1] - c[1])
    area = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) * 0.5
    if area <= 1.0e-12:
        return None
    return ab * bc * ca / (4.0 * area)


def _choose_basis_from_plane(points: list[Point3],
                             origin: Point3,
                             normal: Point3) -> tuple[Point3, Point3]:
    reference = None
    for point in points:
        direction = _vector_sub(point, origin)
        candidate = _unit(direction)
        if candidate:
            alignment = abs(_dot(candidate, normal))
            if alignment < 0.95:
                reference = candidate
                break

    if reference is None:
        reference = (1.0, 0.0, 0.0) if abs(normal[0]) < 0.9 else (0.0, 1.0, 0.0)

    u = _unit(_cross(reference, normal))
    if not u:
        u = _unit(_cross((0.0, 0.0, 1.0), normal)) or (1.0, 0.0, 0.0)
    v = _unit(_cross(normal, u)) or (0.0, 1.0, 0.0)
    return u, v


def _canonicalize_normal(normal: Point3) -> Point3:
    dominant_axis = max(range(3), key=lambda index: abs(normal[index]))
    return normal if normal[dominant_axis] >= 0.0 else (-normal[0], -normal[1], -normal[2])


def _project_point_to_plane_uv(point: Point3,
                               origin: Point3,
                               u_axis: Point3,
                               v_axis: Point3) -> tuple[float, float]:
    offset = _vector_sub(point, origin)
    return (_dot(offset, u_axis), _dot(offset, v_axis))


def _unwrap_arc_angles(points: list[tuple[float, float]],
                       center: tuple[float, float]) -> tuple[list[float], float] | None:
    if not points:
        return None

    raw_angles = [math.atan2(point[1] - center[1], point[0] - center[0]) for point in points]
    unwrapped = [raw_angles[0]]
    direction = 0

    for angle in raw_angles[1:]:
        delta = angle - raw_angles[len(unwrapped) - 1]
        while delta <= -math.pi:
            delta += 2.0 * math.pi
        while delta > math.pi:
            delta -= 2.0 * math.pi
        if abs(delta) <= 1.0e-9:
            unwrapped.append(unwrapped[-1])
            continue
        delta_sign = 1 if delta > 0.0 else -1
        if direction == 0:
            direction = delta_sign
        elif delta_sign != direction:
            return None
        unwrapped.append(unwrapped[-1] + delta)

    return unwrapped, unwrapped[-1] - unwrapped[0]


def _fit_line_segment(points_uv: list[tuple[float, float]]) -> dict[str, object]:
    start_uv = points_uv[0]
    end_uv = points_uv[-1]
    errors = [_point_line_distance_2d(point, start_uv, end_uv) for point in points_uv]
    return {
        "kind": "line",
        "start_uv": start_uv,
        "end_uv": end_uv,
        "point_errors": errors,
        "max_error": max(errors) if errors else 0.0,
        "mean_error": sum(errors) / len(errors) if errors else 0.0,
    }


def _fit_arc_segment(points_uv: list[tuple[float, float]]) -> dict[str, object] | None:
    fitted_circle = _fit_circle_2d(points_uv)
    if not fitted_circle:
        return None

    center_uv, radius = fitted_circle
    if radius <= 1.0e-9:
        return None

    unwrapped_angles = _unwrap_arc_angles(points_uv, center_uv)
    if not unwrapped_angles:
        return None
    angles, total_angle = unwrapped_angles
    angle_span = abs(total_angle)
    if angle_span <= math.radians(5.0) or angle_span > math.pi + 1.0e-6:
        return None
    span_degrees = max(0.0, math.degrees(angle_span) - 0.5)
    minimum_point_count = max(3, math.ceil(span_degrees / 22.5) + 1)
    if len(points_uv) < minimum_point_count:
        return None

    point_errors = [
        abs(math.hypot(point[0] - center_uv[0], point[1] - center_uv[1]) - radius)
        for point in points_uv
    ]
    local_radii = [
        local_radius
        for local_radius in (
            _circumradius_2d(points_uv[index], points_uv[index + 1], points_uv[index + 2])
            for index in range(len(points_uv) - 2)
        )
        if local_radius is not None
    ]
    if local_radii:
        relative_spread = (max(local_radii) - min(local_radii)) / radius
        if relative_spread > 0.05:
            return None

    chord = math.hypot(points_uv[-1][0] - points_uv[0][0], points_uv[-1][1] - points_uv[0][1])
    if chord <= 1.0e-9 or chord > 2.0 * radius + 1.0e-6:
        return None

    signed_radius = radius if total_angle < 0.0 else -radius
    mid_angle = 0.5 * (angles[0] + angles[-1])
    return {
        "kind": "arc",
        "start_uv": points_uv[0],
        "end_uv": points_uv[-1],
        "center_uv": center_uv,
        "mid_uv": (center_uv[0] + radius * math.cos(mid_angle), center_uv[1] + radius * math.sin(mid_angle)),
        "radius": radius,
        "signed_radius": signed_radius,
        "clockwise": total_angle < 0.0,
        "angle_span": angle_span,
        "point_errors": point_errors,
        "max_error": max(point_errors) if point_errors else 0.0,
        "mean_error": sum(point_errors) / len(point_errors) if point_errors else 0.0,
    }


def _bbox_diagonal_2d(points: list[tuple[float, float]]) -> float:
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return math.hypot(max(xs) - min(xs), max(ys) - min(ys))


def _select_best_line_segment(points_uv: list[tuple[float, float]],
                              start_index: int,
                              max_index: int,
                              tolerance: float) -> dict[str, object]:
    best = {
        "end_index": start_index + 1,
        **_fit_line_segment(points_uv[start_index:start_index + 2]),
    }

    for end_index in range(start_index + 2, max_index + 1):
        candidate = _fit_line_segment(points_uv[start_index:end_index + 1])
        if candidate["max_error"] <= tolerance:
            best = {"end_index": end_index, **candidate}

    return best


def _select_best_arc_segment(points_uv: list[tuple[float, float]],
                             start_index: int,
                             max_index: int,
                             tolerance: float) -> dict[str, object] | None:
    best: dict[str, object] | None = None

    for end_index in range(start_index + 2, max_index + 1):
        candidate = _fit_arc_segment(points_uv[start_index:end_index + 1])
        if not candidate:
            continue
        if candidate["max_error"] > tolerance:
            continue
        if best is None or end_index > best["end_index"]:
            best = {"end_index": end_index, **candidate}

    return best


def _segment_errors_for_metrics(segment: dict[str, object],
                                seen_any: bool,
                                closes_loop: bool) -> list[float]:
    errors = list(segment["point_errors"])
    if seen_any and errors:
        errors = errors[1:]
    if closes_loop and errors:
        errors = errors[:-1]
    return errors


def _judge_fit_quality(max_error: float, tolerance: float) -> str:
    if max_error <= tolerance:
        return "excellent"
    if max_error <= tolerance * 2.0:
        return "good"
    if max_error <= tolerance * 4.0:
        return "fair"
    return "poor"


def _build_fitted_rxyz_curve(segments: list[dict[str, object]]) -> list[list[float]]:
    if not segments:
        return []

    curve = []
    for segment in segments:
        start_x, start_y, start_z = segment["start_xyz"]
        signed_radius = segment.get("signed_radius", 0.0) if segment["kind"] == "arc" else 0.0
        curve.append([
            _round_scalar(signed_radius),
            _round_scalar(start_x),
            _round_scalar(start_y),
            _round_scalar(start_z),
        ])

    end_x, end_y, end_z = segments[-1]["end_xyz"]
    curve.append([0.0, _round_scalar(end_x), _round_scalar(end_y), _round_scalar(end_z)])
    header = [float(len(curve) + 1), 0.0, 0.0, 0.0]
    return [header, *curve]


def fit_plane_boundary_loop(points: list[Point3], loop_index: int = 0) -> dict[str, object]:
    cleaned = _dedupe_loop_points(points)
    if len(cleaned) < 2:
        return {
            "loop_index": loop_index,
            "sampled_points_xyz": [_round_point(point) for point in cleaned],
            "sampled_points_uv": [],
            "sampled_points_rxyz": build_sampled_rxyz_curve(cleaned),
            "fitted_points_rxyz": build_sampled_rxyz_curve(cleaned),
            "fitted_segments": [],
            "metrics": {
                "source_point_count": len(cleaned),
                "fitted_segment_count": 0,
                "fitted_control_point_count": 0,
                "compression_ratio": 1.0,
                "mean_error": 0.0,
                "max_error": 0.0,
                "tolerance": 0.0,
                "status": "excellent",
            },
        }

    origin, normal, residual = _fit_plane(cleaned)
    if not origin or not normal:
        origin = cleaned[0]
        normal = (0.0, 0.0, 1.0)
    else:
        normal = _canonicalize_normal(normal)

    u_axis, v_axis = _choose_basis_from_plane(cleaned, origin, normal)
    points_uv = [_project_point_to_plane_uv(point, origin, u_axis, v_axis) for point in cleaned]
    diagonal = _bbox_diagonal_2d(points_uv)
    tolerance = max(1.0e-3, diagonal * 1.0e-4, residual * 10.0)

    loop_xyz = cleaned + [cleaned[0]]
    loop_uv = points_uv + [points_uv[0]]
    original_count = len(cleaned)
    segments: list[dict[str, object]] = []
    all_errors: list[float] = []
    start_index = 0

    while start_index < original_count:
        line_segment = _select_best_line_segment(loop_uv, start_index, original_count, tolerance)
        arc_segment = _select_best_arc_segment(loop_uv, start_index, original_count, tolerance)

        selected = line_segment
        if arc_segment is not None:
            line_span = line_segment["end_index"] - start_index
            arc_span = arc_segment["end_index"] - start_index
            if arc_span > line_span and arc_segment["angle_span"] >= math.radians(8.0):
                selected = arc_segment

        end_index = selected["end_index"]
        selected = {
            **selected,
            "start_xyz": loop_xyz[start_index],
            "end_xyz": loop_xyz[end_index],
        }
        segments.append(selected)
        all_errors.extend(
            _segment_errors_for_metrics(
                selected,
                seen_any=len(segments) > 1,
                closes_loop=end_index == original_count,
            )
        )
        start_index = end_index

    fitted_points_rxyz = _build_fitted_rxyz_curve(segments)
    source_point_count = len(cleaned)
    fitted_control_point_count = max(len(fitted_points_rxyz) - 2, 0)
    max_error = max(all_errors) if all_errors else 0.0
    mean_error = sum(all_errors) / len(all_errors) if all_errors else 0.0

    return {
        "loop_index": loop_index,
        "sampled_points_xyz": [_round_point(point) for point in cleaned],
        "sampled_points_uv": [_round_point_2d(point) for point in points_uv],
        "sampled_points_rxyz": build_sampled_rxyz_curve(cleaned),
        "fitted_points_rxyz": fitted_points_rxyz,
        "fitted_segments": [
            {
                "kind": segment["kind"],
                "start_uv": _round_point_2d(segment["start_uv"]),
                "end_uv": _round_point_2d(segment["end_uv"]),
                "max_error": _round_scalar(segment["max_error"]),
                **(
                    {
                        "center_uv": _round_point_2d(segment["center_uv"]),
                        "mid_uv": _round_point_2d(segment["mid_uv"]),
                        "radius": _round_scalar(segment["radius"]),
                        "signed_radius": _round_scalar(segment["signed_radius"]),
                        "clockwise": segment["clockwise"],
                    }
                    if segment["kind"] == "arc"
                    else {}
                ),
            }
            for segment in segments
        ],
        "metrics": {
            "source_point_count": source_point_count,
            "fitted_segment_count": len(segments),
            "fitted_control_point_count": fitted_control_point_count,
            "compression_ratio": _round_scalar(
                fitted_control_point_count / source_point_count if source_point_count else 1.0
            ),
            "mean_error": _round_scalar(mean_error),
            "max_error": _round_scalar(max_error),
            "tolerance": _round_scalar(tolerance),
            "status": _judge_fit_quality(max_error, tolerance),
        },
    }


def _build_plane_fit_payload(name: str,
                             boundary_loops: list[list[Point3]]) -> dict[str, object]:
    basis_origin, basis_normal, _ = _fit_plane(boundary_loops[0]) if boundary_loops else (None, None, 0.0)
    if not basis_origin or not basis_normal:
        basis_origin = boundary_loops[0][0] if boundary_loops and boundary_loops[0] else (0.0, 0.0, 0.0)
        basis_normal = (0.0, 0.0, 1.0)
    else:
        basis_normal = _canonicalize_normal(basis_normal)
    u_axis, v_axis = _choose_basis_from_plane(boundary_loops[0], basis_origin, basis_normal) if boundary_loops else (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
    )

    loops = [fit_plane_boundary_loop(loop, index) for index, loop in enumerate(boundary_loops)]
    return {
        "name": name,
        "category": "plane",
        "plane_basis": {
            "origin": _round_point(basis_origin),
            "u": _round_point(u_axis),
            "v": _round_point(v_axis),
            "normal": _round_point(basis_normal),
        },
        "loops": loops,
    }


def generate_fit_ui_html(panels: list[dict[str, object]]) -> str:
    data_json = json.dumps(panels, ensure_ascii=False)
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>平面板边界拟合对比</title>
  <style>
    :root {{
      --bg: #f1ece3;
      --paper: #fffaf2;
      --ink: #191512;
      --muted: #6b6256;
      --line: #d0c5b6;
      --raw: #4e6fa8;
      --fit: #bf3f2f;
      --good: #246a47;
      --warn: #a56a18;
      --bad: #9f2f2f;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(255,255,255,0.85), rgba(255,255,255,0) 36%),
        linear-gradient(180deg, #f7f1e7 0%, var(--bg) 100%);
      font-family: "Segoe UI", "PingFang SC", "Microsoft YaHei", sans-serif;
    }}
    .hero {{
      padding: 32px 28px 16px;
      border-bottom: 1px solid rgba(84, 67, 45, 0.12);
    }}
    .hero h1 {{
      margin: 0;
      font-size: clamp(30px, 5vw, 52px);
      letter-spacing: -0.04em;
    }}
    .hero p {{
      margin: 10px 0 0;
      max-width: 760px;
      line-height: 1.7;
      color: var(--muted);
    }}
    .app {{
      display: grid;
      grid-template-columns: 340px minmax(0, 1fr);
      min-height: calc(100vh - 160px);
    }}
    .sidebar {{
      padding: 24px;
      border-right: 1px solid rgba(84, 67, 45, 0.12);
      background: rgba(255, 250, 242, 0.7);
      backdrop-filter: blur(10px);
    }}
    .sidebar label,
    .meta h2,
    .legend-title {{
      display: block;
      margin-bottom: 8px;
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: 0.12em;
      color: var(--muted);
    }}
    .sidebar select {{
      width: 100%;
      margin-bottom: 16px;
      padding: 12px 14px;
      border: 1px solid rgba(84, 67, 45, 0.15);
      border-radius: 14px;
      background: rgba(255,255,255,0.85);
      color: var(--ink);
      font: inherit;
    }}
    .legend {{
      display: grid;
      gap: 10px;
      margin: 22px 0;
    }}
    .legend-item {{
      display: flex;
      align-items: center;
      gap: 10px;
      color: var(--muted);
      font-size: 14px;
    }}
    .swatch {{
      width: 28px;
      height: 3px;
      border-radius: 999px;
    }}
    .meta {{
      margin-top: 24px;
      display: grid;
      gap: 10px;
    }}
    .arc-list {{
      margin-top: 20px;
      padding-top: 18px;
      border-top: 1px solid rgba(84, 67, 45, 0.12);
    }}
    .arc-list ul {{
      list-style: none;
      margin: 10px 0 0;
      padding: 0;
      display: grid;
      gap: 8px;
    }}
    .arc-list li {{
      display: flex;
      justify-content: space-between;
      gap: 12px;
      padding: 10px 12px;
      border-radius: 12px;
      background: rgba(255,255,255,0.72);
      border: 1px solid rgba(84, 67, 45, 0.08);
      color: var(--muted);
      font-size: 13px;
    }}
    .arc-list strong {{
      color: var(--ink);
    }}
    .status {{
      display: inline-flex;
      align-items: center;
      justify-content: center;
      min-width: 112px;
      padding: 8px 14px;
      border-radius: 999px;
      font-weight: 600;
      font-size: 14px;
      letter-spacing: 0.02em;
      color: white;
      background: var(--good);
    }}
    .status.good {{ background: var(--good); }}
    .status.fair {{ background: var(--warn); }}
    .status.poor {{ background: var(--bad); }}
    .metric-grid {{
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
      margin-top: 12px;
    }}
    .metric {{
      padding: 14px;
      border-radius: 16px;
      background: rgba(255,255,255,0.72);
      border: 1px solid rgba(84, 67, 45, 0.08);
    }}
    .metric strong {{
      display: block;
      margin-top: 4px;
      font-size: 18px;
    }}
    .stage {{
      padding: 24px;
      display: grid;
      grid-template-rows: auto 1fr;
      gap: 18px;
    }}
    .stage-head {{
      display: flex;
      align-items: end;
      justify-content: space-between;
      gap: 16px;
    }}
    .stage-head h2 {{
      margin: 0;
      font-size: clamp(24px, 3vw, 34px);
      letter-spacing: -0.03em;
    }}
    .stage-head p {{
      margin: 6px 0 0;
      color: var(--muted);
    }}
    .viewer {{
      position: relative;
      min-height: 560px;
      border-radius: 28px;
      overflow: hidden;
      border: 1px solid rgba(84, 67, 45, 0.12);
      background:
        linear-gradient(180deg, rgba(255,255,255,0.86), rgba(245,239,230,0.88)),
        repeating-linear-gradient(
          0deg,
          transparent 0,
          transparent 31px,
          rgba(51, 39, 23, 0.04) 32px
        ),
        repeating-linear-gradient(
          90deg,
          transparent 0,
          transparent 31px,
          rgba(51, 39, 23, 0.04) 32px
        );
      box-shadow: 0 22px 60px rgba(54, 34, 8, 0.08);
    }}
    #fitCanvas {{
      width: 100%;
      height: 100%;
      display: block;
    }}
    .canvas-note {{
      position: absolute;
      left: 18px;
      bottom: 16px;
      padding: 8px 12px;
      border-radius: 999px;
      font-size: 12px;
      color: var(--muted);
      background: rgba(255,255,255,0.84);
      border: 1px solid rgba(84, 67, 45, 0.08);
      backdrop-filter: blur(8px);
    }}
    @media (max-width: 980px) {{
      .app {{ grid-template-columns: 1fr; }}
      .sidebar {{ border-right: 0; border-bottom: 1px solid rgba(84, 67, 45, 0.12); }}
      .viewer {{ min-height: 420px; }}
    }}
  </style>
</head>
<body>
  <section class="hero">
    <h1>平面板边界拟合对比</h1>
    <p>页面展示局部平面投影下的原始边界采样线与拟合边界线。蓝线是原始采样，红线是压缩后的线段/短圆弧；右侧指标给出压缩率、最大误差和效果判断。</p>
  </section>
  <main class="app">
    <aside class="sidebar">
      <label for="panelSelect">平面板</label>
      <select id="panelSelect"></select>
      <label for="loopSelect">边界环</label>
      <select id="loopSelect"></select>
      <div class="legend">
        <span class="legend-title">图例</span>
        <div class="legend-item"><span class="swatch" style="background: var(--raw);"></span>原始边界</div>
        <div class="legend-item"><span class="swatch" style="background: var(--fit);"></span>拟合边界</div>
        <div class="legend-item"><span class="swatch" style="background: var(--fit); height: 18px; width: 18px; border-radius: 50%;"></span>圆弧段编号</div>
      </div>
      <section class="meta">
        <h2>效果判断</h2>
        <span id="statusBadge" class="status good">excellent</span>
        <div class="metric-grid">
          <div class="metric">原始点数<strong id="sourcePointCount">0</strong></div>
          <div class="metric">拟合段数<strong id="segmentCount">0</strong></div>
          <div class="metric">压缩比<strong id="compressionRatio">0</strong></div>
          <div class="metric">最大误差<strong id="maxError">0</strong></div>
          <div class="metric">平均误差<strong id="meanError">0</strong></div>
          <div class="metric">容差<strong id="toleranceValue">0</strong></div>
        </div>
      </section>
      <section class="arc-list">
        <h2>圆弧段编号</h2>
        <ul id="arcSegmentList"></ul>
      </section>
    </aside>
    <section class="stage">
      <div class="stage-head">
        <div>
          <h2 id="panelTitle">未找到平面板数据</h2>
          <p id="panelSubtitle">视图使用板件局部 2D 投影，以便直接判断拟合误差。</p>
        </div>
      </div>
      <div class="viewer">
        <canvas id="fitCanvas"></canvas>
        <div class="canvas-note">蓝色为原始边界，红色为拟合边界</div>
      </div>
    </section>
  </main>
  <script>
    const PANELS = {data_json};
    const state = {{
      panelIndex: 0,
      loopIndex: 0,
    }};

    const panelSelect = document.getElementById('panelSelect');
    const loopSelect = document.getElementById('loopSelect');
    const panelTitle = document.getElementById('panelTitle');
    const panelSubtitle = document.getElementById('panelSubtitle');
    const statusBadge = document.getElementById('statusBadge');
    const sourcePointCount = document.getElementById('sourcePointCount');
    const segmentCount = document.getElementById('segmentCount');
    const compressionRatio = document.getElementById('compressionRatio');
    const maxError = document.getElementById('maxError');
    const meanError = document.getElementById('meanError');
    const toleranceValue = document.getElementById('toleranceValue');
    const arcSegmentList = document.getElementById('arcSegmentList');
    const canvas = document.getElementById('fitCanvas');
    const ctx = canvas.getContext('2d');

    function resizeCanvas() {{
      const rect = canvas.getBoundingClientRect();
      const dpr = window.devicePixelRatio || 1;
      canvas.width = Math.max(1, Math.round(rect.width * dpr));
      canvas.height = Math.max(1, Math.round(rect.height * dpr));
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      render();
    }}

    function populatePanelSelect() {{
      panelSelect.innerHTML = '';
      PANELS.forEach((panel, index) => {{
        const option = document.createElement('option');
        option.value = String(index);
        option.textContent = panel.name;
        panelSelect.appendChild(option);
      }});
      panelSelect.value = String(state.panelIndex);
    }}

    function populateLoopSelect() {{
      loopSelect.innerHTML = '';
      const panel = PANELS[state.panelIndex];
      panel.loops.forEach((loop, index) => {{
        const option = document.createElement('option');
        option.value = String(index);
        option.textContent = 'Loop ' + index;
        loopSelect.appendChild(option);
      }});
      loopSelect.value = String(Math.min(state.loopIndex, panel.loops.length - 1));
    }}

    function toCanvasTransform(points) {{
      const width = canvas.getBoundingClientRect().width;
      const height = canvas.getBoundingClientRect().height;
      const xs = points.map((point) => point[0]);
      const ys = points.map((point) => point[1]);
      const minX = Math.min(...xs);
      const maxX = Math.max(...xs);
      const minY = Math.min(...ys);
      const maxY = Math.max(...ys);
      const spanX = Math.max(maxX - minX, 1e-6);
      const spanY = Math.max(maxY - minY, 1e-6);
      const scale = Math.min((width - 80) / spanX, (height - 80) / spanY);
      const offsetX = (width - spanX * scale) * 0.5;
      const offsetY = (height - spanY * scale) * 0.5;
      return (point) => ([
        offsetX + (point[0] - minX) * scale,
        height - (offsetY + (point[1] - minY) * scale),
      ]);
    }}

    function sampleArc(segment) {{
      const start = segment.start_uv;
      const end = segment.end_uv;
      const center = segment.center_uv;
      let startAngle = Math.atan2(start[1] - center[1], start[0] - center[0]);
      let endAngle = Math.atan2(end[1] - center[1], end[0] - center[0]);
      if (segment.clockwise) {{
        while (endAngle > startAngle) {{
          endAngle -= Math.PI * 2;
        }}
      }} else {{
        while (endAngle < startAngle) {{
          endAngle += Math.PI * 2;
        }}
      }}
      const stepCount = Math.max(24, Math.ceil(Math.abs(endAngle - startAngle) / (Math.PI / 36)));
      const points = [];
      for (let i = 0; i <= stepCount; i += 1) {{
        const t = i / stepCount;
        const angle = startAngle + (endAngle - startAngle) * t;
        points.push([
          center[0] + segment.radius * Math.cos(angle),
          center[1] + segment.radius * Math.sin(angle),
        ]);
      }}
      return points;
    }}

    function collectExtents(loop) {{
      const points = [...loop.sampled_points_uv];
      loop.fitted_segments.forEach((segment) => {{
        points.push(segment.start_uv, segment.end_uv);
        if (segment.kind === 'arc') {{
          points.push(...sampleArc(segment));
        }}
      }});
      return points;
    }}

    function drawPolyline(points, project, color, lineWidth, dashed = false) {{
      if (!points.length) {{
        return;
      }}
      ctx.beginPath();
      const [startX, startY] = project(points[0]);
      ctx.moveTo(startX, startY);
      for (const point of points.slice(1)) {{
        const [x, y] = project(point);
        ctx.lineTo(x, y);
      }}
      ctx.closePath();
      ctx.setLineDash(dashed ? [6, 8] : []);
      ctx.lineWidth = lineWidth;
      ctx.strokeStyle = color;
      ctx.stroke();
      ctx.setLineDash([]);
    }}

    function drawFitted(loop, project) {{
      ctx.beginPath();
      loop.fitted_segments.forEach((segment, index) => {{
        const samples = segment.kind === 'arc'
          ? sampleArc(segment)
          : [segment.start_uv, segment.end_uv];
        samples.forEach((point, sampleIndex) => {{
          const [x, y] = project(point);
          if (index === 0 && sampleIndex === 0) {{
            ctx.moveTo(x, y);
          }} else {{
            ctx.lineTo(x, y);
          }}
        }});
      }});
      ctx.closePath();
      ctx.lineWidth = 3;
      ctx.strokeStyle = getComputedStyle(document.documentElement).getPropertyValue('--fit').trim();
      ctx.stroke();
    }}

    function drawArcCallouts(loop, project) {{
      const fill = getComputedStyle(document.documentElement).getPropertyValue('--paper').trim() || '#fffaf2';
      const stroke = getComputedStyle(document.documentElement).getPropertyValue('--fit').trim();
      const text = getComputedStyle(document.documentElement).getPropertyValue('--ink').trim() || '#191512';
      const muted = getComputedStyle(document.documentElement).getPropertyValue('--muted').trim() || '#6b6256';
      ctx.font = '600 12px "Segoe UI", "PingFang SC", "Microsoft YaHei", sans-serif';
      ctx.textBaseline = 'middle';
      loop.fitted_segments.forEach((seg, index) => {{
        if (seg.kind !== 'arc') {{
          return;
        }}
        const anchorUv = seg.mid_uv || [
          (seg.start_uv[0] + seg.end_uv[0]) * 0.5,
          (seg.start_uv[1] + seg.end_uv[1]) * 0.5,
        ];
        const startProjected = project(seg.start_uv);
        const endProjected = project(seg.end_uv);
        const calloutAnchor = project(anchorUv); // callout-anchor
        const tangentX = endProjected[0] - startProjected[0];
        const tangentY = endProjected[1] - startProjected[1];
        const tangentLength = Math.hypot(tangentX, tangentY) || 1;
        let normalX = -tangentY / tangentLength;
        let normalY = tangentX / tangentLength;
        const midX = (startProjected[0] + endProjected[0]) * 0.5;
        const midY = (startProjected[1] + endProjected[1]) * 0.5;
        if ((calloutAnchor[0] - midX) * normalX + (calloutAnchor[1] - midY) * normalY < 0) {{
          normalX *= -1;
          normalY *= -1;
        }}
        const calloutElbow = [calloutAnchor[0] + normalX * 18, calloutAnchor[1] + normalY * 18]; // callout-elbow
        const calloutLabel = [
          calloutElbow[0] + (normalX >= 0 ? 18 : -18),
          calloutElbow[1] + normalY * 4,
        ];
        const textWidth = Math.max(22, ctx.measureText(String(index)).width + 12);
        const boxWidth = textWidth;
        const boxHeight = 22;
        const boxCenterX = calloutLabel[0] + (normalX >= 0 ? boxWidth * 0.5 : -boxWidth * 0.5);
        const boxCenterY = calloutLabel[1];
        const boxLeft = boxCenterX - boxWidth * 0.5;
        const boxTop = boxCenterY - boxHeight * 0.5;

        ctx.beginPath();
        ctx.strokeStyle = muted;
        ctx.lineWidth = 1.5;
        ctx.moveTo(calloutAnchor[0], calloutAnchor[1]);
        ctx.lineTo(calloutElbow[0], calloutElbow[1]);
        ctx.lineTo(calloutLabel[0], calloutLabel[1]);
        ctx.stroke();

        ctx.beginPath();
        ctx.fillStyle = fill;
        ctx.strokeStyle = stroke;
        ctx.lineWidth = 1.5;
        ctx.rect(boxLeft, boxTop, boxWidth, boxHeight);
        ctx.fill();
        ctx.stroke();

        ctx.beginPath();
        ctx.fillStyle = stroke;
        ctx.arc(calloutAnchor[0], calloutAnchor[1], 3, 0, Math.PI * 2);
        ctx.fill();

        ctx.fillStyle = text;
        ctx.textAlign = 'center';
        ctx.fillText(String(index), boxCenterX, boxCenterY);
      }});
    }}

    function updateArcSegmentList(loop) {{
      arcSegmentList.innerHTML = '';
      const arcSegments = loop.fitted_segments
        .map((seg, index) => ({{ seg, index }}))
        .filter((entry) => entry.seg.kind === 'arc');
      if (!arcSegments.length) {{
        const item = document.createElement('li');
        item.textContent = '当前边界环没有圆弧段';
        arcSegmentList.appendChild(item);
        return;
      }}
      arcSegments.forEach((entry) => {{
        const item = document.createElement('li');
        item.innerHTML = '<strong>idx ' + entry.index + '</strong><span>r=' + entry.seg.radius.toFixed(3) + '</span>';
        arcSegmentList.appendChild(item);
      }});
    }}

    function updateMeta(panel, loop) {{
      panelTitle.textContent = panel.name;
      panelSubtitle.textContent = 'Loop ' + loop.loop_index + ' · 原始点 ' + loop.metrics.source_point_count + ' · 拟合段 ' + loop.metrics.fitted_segment_count;
      sourcePointCount.textContent = String(loop.metrics.source_point_count);
      segmentCount.textContent = String(loop.metrics.fitted_segment_count);
      compressionRatio.textContent = loop.metrics.compression_ratio.toFixed(3);
      maxError.textContent = loop.metrics.max_error.toFixed(6);
      meanError.textContent = loop.metrics.mean_error.toFixed(6);
      toleranceValue.textContent = loop.metrics.tolerance.toFixed(6);
      statusBadge.textContent = loop.metrics.status;
      statusBadge.className = 'status ' + (loop.metrics.status === 'excellent' || loop.metrics.status === 'good' ? 'good' : loop.metrics.status === 'fair' ? 'fair' : 'poor');
      updateArcSegmentList(loop);
    }}

    function render() {{
      const panel = PANELS[state.panelIndex];
      if (!panel || !panel.loops.length) {{
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        return;
      }}
      const loop = panel.loops[state.loopIndex];
      updateMeta(panel, loop);

      const extents = collectExtents(loop);
      const project = toCanvasTransform(extents);
      const width = canvas.getBoundingClientRect().width;
      const height = canvas.getBoundingClientRect().height;
      ctx.clearRect(0, 0, width, height);

      drawPolyline(
        loop.sampled_points_uv,
        project,
        getComputedStyle(document.documentElement).getPropertyValue('--raw').trim(),
        1.5,
        true,
      );
      drawFitted(loop, project);
      drawArcCallouts(loop, project);
    }}

    panelSelect.addEventListener('change', (event) => {{
      state.panelIndex = Number(event.target.value);
      state.loopIndex = 0;
      populateLoopSelect();
      render();
    }});

    loopSelect.addEventListener('change', (event) => {{
      state.loopIndex = Number(event.target.value);
      render();
    }});

    populatePanelSelect();
    populateLoopSelect();
    window.addEventListener('resize', resizeCanvas);
    resizeCanvas();
  </script>
</body>
</html>
"""


def _serialize_panel(name: str, category: str, vertices: list[Point3], triangles: list[tuple[int, int, int]], boundary_loops: list[list[Point3]], mesh_polylines: list[list[Point3]]) -> dict:
    return {
        "name": name,
        "category": category,
        "vertex_count": len(vertices),
        "triangle_count": len(triangles),
        "boundary_loop_count": len(boundary_loops),
        "mesh_polyline_count": len(mesh_polylines),
        "bbox": compute_bbox(vertices),
        "vertices": [_round_point(point) for point in vertices],
        "triangles": [list(triangle) for triangle in triangles],
        "boundary_loops": [[_round_point(point) for point in loop] for loop in boundary_loops],
        "mesh_polylines": [[_round_point(point) for point in polyline] for polyline in mesh_polylines],
    }


def generate_demo_html(panels: list[dict]) -> str:
    data_json = json.dumps(panels, ensure_ascii=False)
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>DXF 板件 Demo</title>
  <style>
    :root {{
      --bg: #f2efe8;
      --ink: #1f1d1a;
      --muted: #6b645b;
      --panel: #fffdfa;
      --line: #b52020;
      --mesh: rgba(61, 103, 160, 0.22);
      --frame: #d8d1c4;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      font-family: "Segoe UI", "PingFang SC", "Microsoft YaHei", sans-serif;
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(255,255,255,0.9), rgba(255,255,255,0) 35%),
        linear-gradient(180deg, #f6f1e7 0%, var(--bg) 100%);
    }}
    header {{
      padding: 28px 32px 12px;
    }}
    h1 {{
      margin: 0 0 8px;
      font-size: 28px;
    }}
    .summary {{
      color: var(--muted);
      line-height: 1.6;
      max-width: 1120px;
    }}
    .app {{
      padding: 0 24px 36px;
    }}
    .viewer {{
      background: var(--panel);
      border: 1px solid var(--frame);
      border-radius: 20px;
      overflow: hidden;
      box-shadow: 0 10px 25px rgba(42, 33, 17, 0.08);
    }}
    .viewer-wrap {{
      background:
        linear-gradient(135deg, rgba(70, 114, 173, 0.08), rgba(181, 32, 32, 0.06)),
        repeating-linear-gradient(
          0deg,
          transparent 0,
          transparent 23px,
          rgba(95, 85, 71, 0.05) 24px
        ),
        repeating-linear-gradient(
          90deg,
          transparent 0,
          transparent 23px,
          rgba(95, 85, 71, 0.05) 24px
        );
      padding: 14px;
      min-height: 420px;
      height: min(72vh, 760px);
      border-bottom: 1px solid var(--frame);
    }}
    canvas {{
      width: 100%;
      height: 100%;
      display: block;
      background: rgba(255,255,255,0.72);
      border-radius: 14px;
      cursor: grab;
    }}
    canvas.dragging {{ cursor: grabbing; }}
    .viewer-status {{
      padding: 10px 24px 0;
      color: var(--muted);
      font-size: 13px;
    }}
    .content {{
      padding: 16px 18px 18px;
    }}
    .controls {{
      display: grid;
      gap: 14px;
      padding: 18px;
      border-bottom: 1px solid var(--frame);
    }}
    .controls-row {{
      display: grid;
      grid-template-columns: 1fr auto auto;
      gap: 10px;
      align-items: center;
    }}
    .view-row {{
      grid-template-columns: auto auto auto auto 1fr;
    }}
    select,
    button,
    input[type="range"] {{
      font: inherit;
    }}
    select {{
      width: 100%;
      padding: 10px 12px;
      border: 1px solid var(--frame);
      border-radius: 12px;
      background: #fff;
      color: var(--ink);
    }}
    button {{
      border: 1px solid var(--frame);
      background: #fff;
      color: var(--ink);
      border-radius: 12px;
      padding: 10px 14px;
      cursor: pointer;
    }}
    button:hover {{
      background: #f7f3eb;
    }}
    .toggles {{
      display: flex;
      flex-wrap: wrap;
      gap: 14px;
      color: var(--muted);
      font-size: 14px;
    }}
    .toggles label {{
      display: inline-flex;
      align-items: center;
      gap: 8px;
    }}
    .sliders {{
      display: grid;
      gap: 12px;
    }}
    .sliders label {{
      display: grid;
      grid-template-columns: 56px 1fr 48px;
      gap: 10px;
      align-items: center;
      color: var(--muted);
      font-size: 14px;
    }}
    .sliders output {{
      text-align: right;
      color: var(--ink);
      font-variant-numeric: tabular-nums;
    }}
    .zoom-readout {{
      justify-self: end;
      color: var(--muted);
      font-size: 13px;
      font-variant-numeric: tabular-nums;
    }}
    .title-row {{
      display: flex;
      justify-content: space-between;
      gap: 12px;
      align-items: baseline;
      margin-bottom: 10px;
    }}
    .title-row h2 {{
      margin: 0;
      font-size: 18px;
      word-break: break-all;
    }}
    .tag {{
      font-size: 12px;
      padding: 4px 8px;
      border-radius: 999px;
      background: #ece7dc;
      color: #5d544a;
      white-space: nowrap;
    }}
    .meta {{
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 8px 14px;
      font-size: 13px;
      color: var(--muted);
    }}
    .legend {{
      display: flex;
      gap: 16px;
      padding: 0 24px 18px;
      color: var(--muted);
      font-size: 13px;
    }}
    .legend span::before {{
      content: "";
      display: inline-block;
      width: 12px;
      height: 12px;
      border-radius: 3px;
      margin-right: 8px;
      vertical-align: -1px;
    }}
    .legend .mesh::before {{ background: rgba(61, 103, 160, 0.5); }}
    .legend .edge::before {{ background: #b52020; }}
    @media (max-width: 960px) {{
      .controls-row {{
        grid-template-columns: 1fr;
      }}
    }}
  </style>
</head>
<body>
  <header>
    <h1>DXF 板件导出预览</h1>
    <div class="summary">
      当前页面展示从 <code>FB03C.dxf</code> 解析出的全部板件。蓝色半透明面表示三角网格，
      红色线表示根据三角网格重建出的边界环，用于人工核对边界提取是否合理。
    </div>
  </header>
  <div class="legend">
    <span class="mesh">三角网格</span>
    <span class="edge">重建边界线</span>
  </div>
  <main class="app">
    <section class="viewer">
      <div class="controls">
        <div class="controls-row">
          <select id="panelSelect"></select>
          <button id="prevPanel" type="button">上一块</button>
          <button id="nextPanel" type="button">下一块</button>
        </div>
        <div class="controls-row view-row">
          <button id="zoomOut" type="button">缩小</button>
          <button id="zoomIn" type="button">放大</button>
          <button id="resetView" type="button">重置视图</button>
          <output class="zoom-readout" id="zoomValue">100%</output>
        </div>
        <div class="toggles">
          <label><input id="showMesh" type="checkbox" checked>显示网格</label>
          <label><input id="showBoundary" type="checkbox" checked>显示边界</label>
          <label>边界模式
            <select id="boundaryMode">
              <option value="raw">原始边界</option>
              <option value="visible">可见边界</option>
            </select>
          </label>
        </div>
        <div class="sliders">
          <label>Yaw <input id="yawSlider" type="range" min="-180" max="180" value="-42"><output id="yawValue">-42°</output></label>
          <label>Pitch <input id="pitchSlider" type="range" min="-80" max="80" value="32"><output id="pitchValue">32°</output></label>
        </div>
      </div>
      <div class="viewer-wrap">
        <canvas id="viewerCanvas"></canvas>
      </div>
      <div class="viewer-status" id="viewerStatus"></div>
      <div class="content">
        <div class="title-row">
          <h2 id="panelTitle"></h2>
          <span class="tag" id="panelCategory"></span>
        </div>
        <div class="meta" id="panelMeta"></div>
      </div>
    </section>
  </main>
  <script>
    const PANELS = {data_json};
    const state = {{
      panelIndex: 0,
      yaw: -42 * Math.PI / 180,
      pitch: 32 * Math.PI / 180,
      zoom: 1,
      panX: 0,
      panY: 0,
      showMesh: true,
      showBoundary: true,
      boundaryMode: 'visible',
    }};

    function rotatePoint(point, yaw, pitch) {{
      const cy = Math.cos(yaw), sy = Math.sin(yaw);
      const cp = Math.cos(pitch), sp = Math.sin(pitch);
      const x1 = point[0] * cy - point[2] * sy;
      const z1 = point[0] * sy + point[2] * cy;
      const y2 = point[1] * cp - z1 * sp;
      const z2 = point[1] * sp + z1 * cp;
      return [x1, y2, z2];
    }}

    function transformPositions(positions) {{
      const transformed = new Float32Array(positions.length);
      for (let i = 0; i < positions.length; i += 3) {{
        const rotated = rotatePoint(
          [positions[i], positions[i + 1], positions[i + 2]],
          state.yaw,
          state.pitch
        );
        transformed[i] = rotated[0] * 0.92;
        transformed[i + 1] = rotated[1] * 0.92;
        transformed[i + 2] = rotated[2] * 0.92;
      }}
      return transformed;
    }}

    function projectToCanvas(positions, width, height) {{
      const projected = new Float32Array(positions.length / 3 * 2);
      const xScale = width * 0.46 * state.zoom;
      const yScale = height * 0.46 * state.zoom;
      const halfWidth = width / 2;
      const halfHeight = height / 2;
      for (let i = 0, j = 0; i < positions.length; i += 3, j += 2) {{
        projected[j] = halfWidth + state.panX + positions[i] * xScale;
        projected[j + 1] = halfHeight + state.panY - positions[i + 1] * yScale;
      }}
      return projected;
    }}

    function clampZoom(value) {{
      return Math.max(0.2, Math.min(12, value));
    }}

    function setZoom(nextZoom, anchorX = viewerCanvas.width / 2, anchorY = viewerCanvas.height / 2) {{
      const previousZoom = state.zoom;
      const clampedZoom = clampZoom(nextZoom);
      if (Math.abs(clampedZoom - previousZoom) < 1e-9) {{
        return;
      }}
      const halfWidth = viewerCanvas.width / 2;
      const halfHeight = viewerCanvas.height / 2;
      const scale = clampedZoom / previousZoom;
      state.panX = anchorX - halfWidth - (anchorX - halfWidth - state.panX) * scale;
      state.panY = anchorY - halfHeight - (anchorY - halfHeight - state.panY) * scale;
      state.zoom = clampedZoom;
    }}

    function resetView() {{
      state.yaw = -42 * Math.PI / 180;
      state.pitch = 32 * Math.PI / 180;
      state.zoom = 1;
      state.panX = 0;
      state.panY = 0;
    }}

    function pointInTriangle(px, py, ax, ay, bx, by, cx, cy) {{
      const denominator = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy);
      if (Math.abs(denominator) < 1e-9) {{
        return false;
      }}
      const alpha = ((by - cy) * (px - cx) + (cx - bx) * (py - cy)) / denominator;
      const beta = ((cy - ay) * (px - cx) + (ax - cx) * (py - cy)) / denominator;
      const gamma = 1 - alpha - beta;
      return alpha >= -1e-6 && beta >= -1e-6 && gamma >= -1e-6;
    }}

    function segmentVisibility(midX, midY, segmentDepth, triangles, projectedVertices, transformedPositions) {{
      for (const triangle of triangles) {{
        const ax = projectedVertices[triangle.a * 2];
        const ay = projectedVertices[triangle.a * 2 + 1];
        const bx = projectedVertices[triangle.b * 2];
        const by = projectedVertices[triangle.b * 2 + 1];
        const cx = projectedVertices[triangle.c * 2];
        const cy = projectedVertices[triangle.c * 2 + 1];
        if (!pointInTriangle(midX, midY, ax, ay, bx, by, cx, cy)) {{
          continue;
        }}
        const denominator = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy);
        if (Math.abs(denominator) < 1e-9) {{
          continue;
        }}
        const alpha = ((by - cy) * (midX - cx) + (cx - bx) * (midY - cy)) / denominator;
        const beta = ((cy - ay) * (midX - cx) + (ax - cx) * (midY - cy)) / denominator;
        const gamma = 1 - alpha - beta;
        const triangleDepth =
          alpha * transformedPositions[triangle.a * 3 + 2] +
          beta * transformedPositions[triangle.b * 3 + 2] +
          gamma * transformedPositions[triangle.c * 3 + 2];
        if (triangleDepth > segmentDepth + 0.01) {{
          return false;
        }}
      }}
      return true;
    }}

    function drawVisibleBoundarySegment(
      ctx,
      startX,
      startY,
      startDepth,
      endX,
      endY,
      endDepth,
      triangles,
      projectedVertices,
      transformedPositions
    ) {{
      const sampleCount = 12;
      let lastVisiblePoint = null;
      for (let sampleIndex = 0; sampleIndex <= sampleCount; sampleIndex += 1) {{
        const t = sampleIndex / sampleCount;
        const x = startX + (endX - startX) * t;
        const y = startY + (endY - startY) * t;
        const depth = startDepth + (endDepth - startDepth) * t;
        const visible = segmentVisibility(x, y, depth, triangles, projectedVertices, transformedPositions);
        if (!visible) {{
          lastVisiblePoint = null;
          continue;
        }}
        if (lastVisiblePoint) {{
          ctx.moveTo(lastVisiblePoint[0], lastVisiblePoint[1]);
          ctx.lineTo(x, y);
        }}
        lastVisiblePoint = [x, y];
      }}
    }}

    function buildWireframeSegments(panel) {{
      const edges = new Set();
      const positions = [];
      panel.triangles.forEach((triangle) => {{
        for (let index = 0; index < 3; index += 1) {{
          const a = triangle[index];
          const b = triangle[(index + 1) % 3];
          const key = a < b ? a + ':' + b : b + ':' + a;
          if (edges.has(key)) continue;
          edges.add(key);
          const va = panel.vertices[a];
          const vb = panel.vertices[b];
          positions.push(va[0], va[1], va[2], vb[0], vb[1], vb[2]);
        }}
      }});
      return new Float32Array(positions);
    }}

    function normalizePanel(panel) {{
      const bbox = panel.bbox;
      const center = [
        (bbox.min_x + bbox.max_x) / 2,
        (bbox.min_y + bbox.max_y) / 2,
        (bbox.min_z + bbox.max_z) / 2,
      ];
      const extent = Math.max(
        bbox.max_x - bbox.min_x,
        bbox.max_y - bbox.min_y,
        bbox.max_z - bbox.min_z,
        1
      );
      const scale = 1.7 / extent;
      const positions = new Float32Array(panel.vertices.length * 3);
      panel.vertices.forEach((vertex, index) => {{
        positions[index * 3] = (vertex[0] - center[0]) * scale;
        positions[index * 3 + 1] = (vertex[1] - center[1]) * scale;
        positions[index * 3 + 2] = (vertex[2] - center[2]) * scale;
      }});

      const triangleIndices = new Uint16Array(panel.triangles.length * 3);
      panel.triangles.forEach((triangle, index) => {{
        triangleIndices[index * 3] = triangle[0];
        triangleIndices[index * 3 + 1] = triangle[1];
        triangleIndices[index * 3 + 2] = triangle[2];
      }});

      const linePositions = [];
      panel.boundary_loops.forEach((loop) => {{
        if (loop.length < 2) return;
        for (let i = 0; i < loop.length; i += 1) {{
          const start = loop[i];
          const end = loop[(i + 1) % loop.length];
          linePositions.push(
            (start[0] - center[0]) * scale,
            (start[1] - center[1]) * scale,
            (start[2] - center[2]) * scale,
            (end[0] - center[0]) * scale,
            (end[1] - center[1]) * scale,
            (end[2] - center[2]) * scale
          );
        }}
      }});

      const wireframeSource = buildWireframeSegments(panel);
      const wireframePositions = new Float32Array(wireframeSource.length);
      for (let i = 0; i < wireframeSource.length; i += 3) {{
        wireframePositions[i] = (wireframeSource[i] - center[0]) * scale;
        wireframePositions[i + 1] = (wireframeSource[i + 1] - center[1]) * scale;
        wireframePositions[i + 2] = (wireframeSource[i + 2] - center[2]) * scale;
      }}

      return {{
        positions,
        triangleIndices,
        linePositions: new Float32Array(linePositions),
        wireframePositions,
      }};
    }}

    const panelCache = PANELS.map(normalizePanel);
    const viewerCanvas = document.getElementById('viewerCanvas');
    const viewerStatus = document.getElementById('viewerStatus');
    const ctx2d = viewerCanvas.getContext('2d');
    viewerStatus.textContent = '渲染模式: Canvas 2D 线框投影';

    function resizeCanvas() {{
      const ratio = window.devicePixelRatio || 1;
      const rect = viewerCanvas.getBoundingClientRect();
      const cssWidth = Math.max(1, rect.width || viewerCanvas.clientWidth || 960);
      const cssHeight = Math.max(1, rect.height || viewerCanvas.clientHeight || Math.round(cssWidth * 0.625));
      const width = Math.max(1, Math.floor(cssWidth * ratio));
      const height = Math.max(1, Math.floor(cssHeight * ratio));
      if (viewerCanvas.width !== width || viewerCanvas.height !== height) {{
        viewerCanvas.width = width;
        viewerCanvas.height = height;
      }}
    }}

    function renderPanel(panelIndex) {{
      const panel = PANELS[panelIndex];
      resizeCanvas();
      const panelData = panelCache[panelIndex];
      const transformedPositions = transformPositions(panelData.positions);
      const transformedLinePositions = transformPositions(panelData.linePositions);
      const transformedWireframePositions = transformPositions(panelData.wireframePositions);
      const projectedVertices = projectToCanvas(transformedPositions, viewerCanvas.width, viewerCanvas.height);
      const projectedWireframe = projectToCanvas(transformedWireframePositions, viewerCanvas.width, viewerCanvas.height);
      const projectedBoundary = projectToCanvas(transformedLinePositions, viewerCanvas.width, viewerCanvas.height);
      const triangles = [];
      for (let i = 0; i < panelData.triangleIndices.length; i += 3) {{
        const a = panelData.triangleIndices[i];
        const b = panelData.triangleIndices[i + 1];
        const c = panelData.triangleIndices[i + 2];
        triangles.push({{
          a,
          b,
          c,
          depth:
            transformedPositions[a * 3 + 2] +
            transformedPositions[b * 3 + 2] +
            transformedPositions[c * 3 + 2],
        }});
      }}
      triangles.sort((left, right) => left.depth - right.depth);
      ctx2d.clearRect(0, 0, viewerCanvas.width, viewerCanvas.height);
      ctx2d.fillStyle = 'rgba(250, 248, 243, 1)';
      ctx2d.fillRect(0, 0, viewerCanvas.width, viewerCanvas.height);
      if (state.showMesh) {{
        ctx2d.fillStyle = 'rgba(79, 128, 191, 0.18)';
        for (const triangle of triangles) {{
          const ax = projectedVertices[triangle.a * 2];
          const ay = projectedVertices[triangle.a * 2 + 1];
          const bx = projectedVertices[triangle.b * 2];
          const by = projectedVertices[triangle.b * 2 + 1];
          const cx = projectedVertices[triangle.c * 2];
          const cy = projectedVertices[triangle.c * 2 + 1];
          ctx2d.beginPath();
          ctx2d.moveTo(ax, ay);
          ctx2d.lineTo(bx, by);
          ctx2d.lineTo(cx, cy);
          ctx2d.closePath();
          ctx2d.fill();
        }}

        ctx2d.strokeStyle = 'rgba(61, 103, 160, 0.55)';
        ctx2d.lineWidth = Math.max(1, (window.devicePixelRatio || 1) * 0.8);
        ctx2d.beginPath();
        for (let i = 0; i < projectedWireframe.length; i += 4) {{
          ctx2d.moveTo(projectedWireframe[i], projectedWireframe[i + 1]);
          ctx2d.lineTo(projectedWireframe[i + 2], projectedWireframe[i + 3]);
        }}
        ctx2d.stroke();
      }}
      if (state.showBoundary && projectedBoundary.length) {{
        ctx2d.strokeStyle = 'rgba(181, 32, 32, 1)';
        ctx2d.lineWidth = Math.max(2, (window.devicePixelRatio || 1) * 1.6);
        ctx2d.beginPath();
        for (let i = 0, j = 0; i < projectedBoundary.length; i += 4, j += 6) {{
          if (panel.category === 'plane' || state.boundaryMode === 'raw') {{
            ctx2d.moveTo(projectedBoundary[i], projectedBoundary[i + 1]);
            ctx2d.lineTo(projectedBoundary[i + 2], projectedBoundary[i + 3]);
            continue;
          }}
          drawVisibleBoundarySegment(
            ctx2d,
            projectedBoundary[i],
            projectedBoundary[i + 1],
            transformedLinePositions[j + 2],
            projectedBoundary[i + 2],
            projectedBoundary[i + 3],
            transformedLinePositions[j + 5],
            triangles,
            projectedVertices,
            transformedPositions
          );
        }}
        ctx2d.stroke();
      }}
    }}

    const panelSelect = document.getElementById('panelSelect');
    const panelTitle = document.getElementById('panelTitle');
    const panelCategory = document.getElementById('panelCategory');
    const panelMeta = document.getElementById('panelMeta');
    const yawSlider = document.getElementById('yawSlider');
    const pitchSlider = document.getElementById('pitchSlider');
    const yawValue = document.getElementById('yawValue');
    const pitchValue = document.getElementById('pitchValue');
    const zoomValue = document.getElementById('zoomValue');
    const showMesh = document.getElementById('showMesh');
    const showBoundary = document.getElementById('showBoundary');
    const boundaryMode = document.getElementById('boundaryMode');

    function panelAt(index) {{
      return PANELS[(index + PANELS.length) % PANELS.length];
    }}

    function updateMeta(panel) {{
      panelTitle.textContent = panel.name;
      panelCategory.textContent = panel.category === 'plane' ? '平面板' : '曲面板';
      panelMeta.innerHTML =
        '<div>顶点数: ' + panel.vertex_count + '</div>' +
        '<div>三角面数: ' + panel.triangle_count + '</div>' +
        '<div>边界环数: ' + panel.boundary_loop_count + '</div>' +
        '<div>Mesh Polyline 数: ' + panel.mesh_polyline_count + '</div>';
    }}

    function syncControls() {{
      yawSlider.value = Math.round(state.yaw * 180 / Math.PI);
      pitchSlider.value = Math.round(state.pitch * 180 / Math.PI);
      yawValue.textContent = yawSlider.value + '°';
      pitchValue.textContent = pitchSlider.value + '°';
      zoomValue.textContent = Math.round(state.zoom * 100) + '%';
      showMesh.checked = state.showMesh;
      showBoundary.checked = state.showBoundary;
      boundaryMode.value = state.boundaryMode;
      panelSelect.value = String(state.panelIndex);
      viewerStatus.textContent =
        '渲染模式: Canvas 2D 线框投影 | 缩放: ' + Math.round(state.zoom * 100) + '% | 平移: ' +
        Math.round(state.panX) + ', ' + Math.round(state.panY);
    }}

    function renderCurrentPanel() {{
      const panel = panelAt(state.panelIndex);
      updateMeta(panel);
      syncControls();
      renderPanel(state.panelIndex);
    }}

    function setPanel(index) {{
      state.panelIndex = (index + PANELS.length) % PANELS.length;
      renderCurrentPanel();
    }}

    PANELS.forEach((panel, index) => {{
      const option = document.createElement('option');
      option.value = String(index);
      option.textContent = panel.name + ' (' + (panel.category === 'plane' ? '平面板' : '曲面板') + ')';
      panelSelect.appendChild(option);
    }});

    panelSelect.addEventListener('change', (event) => setPanel(Number(event.target.value)));
    document.getElementById('prevPanel').addEventListener('click', () => setPanel(state.panelIndex - 1));
    document.getElementById('nextPanel').addEventListener('click', () => setPanel(state.panelIndex + 1));
    document.getElementById('zoomIn').addEventListener('click', () => {{
      setZoom(state.zoom * 1.2);
      renderCurrentPanel();
    }});
    document.getElementById('zoomOut').addEventListener('click', () => {{
      setZoom(state.zoom / 1.2);
      renderCurrentPanel();
    }});
    document.getElementById('resetView').addEventListener('click', () => {{
      resetView();
      renderCurrentPanel();
    }});

    yawSlider.addEventListener('input', () => {{
      state.yaw = Number(yawSlider.value) * Math.PI / 180;
      renderCurrentPanel();
    }});
    pitchSlider.addEventListener('input', () => {{
      state.pitch = Number(pitchSlider.value) * Math.PI / 180;
      renderCurrentPanel();
    }});
    showMesh.addEventListener('change', () => {{
      state.showMesh = showMesh.checked;
      renderCurrentPanel();
    }});
    showBoundary.addEventListener('change', () => {{
      state.showBoundary = showBoundary.checked;
      renderCurrentPanel();
    }});
    boundaryMode.addEventListener('change', () => {{
      state.boundaryMode = boundaryMode.value;
      renderCurrentPanel();
    }});

    let dragging = false;
    let dragMode = 'rotate';
    let lastX = 0;
    let lastY = 0;
    viewerCanvas.addEventListener('contextmenu', (event) => event.preventDefault());
    viewerCanvas.addEventListener('wheel', (event) => {{
      event.preventDefault();
      const rect = viewerCanvas.getBoundingClientRect();
      const ratioX = viewerCanvas.width / Math.max(rect.width, 1);
      const ratioY = viewerCanvas.height / Math.max(rect.height, 1);
      const anchorX = (event.clientX - rect.left) * ratioX;
      const anchorY = (event.clientY - rect.top) * ratioY;
      const factor = event.deltaY < 0 ? 1.12 : 1 / 1.12;
      setZoom(state.zoom * factor, anchorX, anchorY);
      renderCurrentPanel();
    }}, {{ passive: false }});
    viewerCanvas.addEventListener('pointerdown', (event) => {{
      dragging = true;
      dragMode = (event.button === 1 || event.button === 2 || (event.button === 0 && event.shiftKey)) ? 'pan' : 'rotate';
      lastX = event.clientX;
      lastY = event.clientY;
      viewerCanvas.classList.add('dragging');
      viewerCanvas.setPointerCapture(event.pointerId);
    }});
    viewerCanvas.addEventListener('pointermove', (event) => {{
      if (!dragging) return;
      const dx = event.clientX - lastX;
      const dy = event.clientY - lastY;
      lastX = event.clientX;
      lastY = event.clientY;
      if (dragMode === 'pan') {{
        const rect = viewerCanvas.getBoundingClientRect();
        const ratioX = viewerCanvas.width / Math.max(rect.width, 1);
        const ratioY = viewerCanvas.height / Math.max(rect.height, 1);
        state.panX += dx * ratioX;
        state.panY += dy * ratioY;
      }} else {{
        state.yaw += dx * 0.01;
        state.pitch = Math.max(-1.38, Math.min(1.38, state.pitch + dy * 0.01));
      }}
      renderCurrentPanel();
    }});
    function stopDragging(event) {{
      dragging = false;
      dragMode = 'rotate';
      viewerCanvas.classList.remove('dragging');
      if (event && viewerCanvas.hasPointerCapture(event.pointerId)) {{
        viewerCanvas.releasePointerCapture(event.pointerId);
      }}
    }}
    viewerCanvas.addEventListener('pointerup', stopDragging);
    viewerCanvas.addEventListener('pointerleave', stopDragging);
    viewerCanvas.addEventListener('pointercancel', stopDragging);
    window.addEventListener('resize', renderCurrentPanel);

    renderCurrentPanel();
  </script>
</body>
</html>
"""


def export_dxf_models(dxf_path: Path,
                      model_root: Path,
                      demo_root: Path,
                      ui_path: Path | None = None) -> dict:
    blocks = parse_dxf_blocks(dxf_path.read_text(encoding="utf-8", errors="ignore").splitlines())
    if ui_path is None:
        ui_path = dxf_path.parent / "ui.html"

    if model_root.exists():
        shutil.rmtree(model_root)
    if demo_root.exists():
        shutil.rmtree(demo_root)

    model_root.mkdir(parents=True, exist_ok=True)
    demo_root.mkdir(parents=True, exist_ok=True)
    plane_root = model_root / "plane_panels"
    surface_root = model_root / "surface_panels"
    plane_root.mkdir(parents=True, exist_ok=True)
    surface_root.mkdir(parents=True, exist_ok=True)

    panel_summaries: list[dict] = []
    demo_panels: list[dict] = []
    plane_fit_panels: list[dict] = []

    for name, mesh in sorted(blocks.items()):
        vertices, triangles = build_indexed_mesh(mesh.triangles)
        boundary_loops = reconstruct_boundary_loops(mesh.triangles)
        category = classify_panel(mesh.triangles, boundary_loops)
        parent_root = plane_root if category == "plane" else surface_root
        panel_root = parent_root / name
        panel_root.mkdir(parents=True, exist_ok=True)

        obj_path = panel_root / f"{name}.obj"
        boundary_path = panel_root / f"{name}.boundary.xyz.json"
        panel_path = panel_root / f"{name}.panel.xyz.json"
        sampled_rxyz_path = panel_root / f"{name}.boundary.sampled.rxyz.json"
        fitted_rxyz_path = panel_root / f"{name}.boundary.fitted.rxyz.json"

        obj_path.write_text(build_obj_text(mesh), encoding="utf-8")
        boundary_path.write_text(
            json.dumps(
                {
                    "name": name,
                    "category": category,
                    "boundary_loops": [[_round_point(point) for point in loop] for loop in boundary_loops],
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )

        panel_payload = _serialize_panel(name, category, vertices, triangles, boundary_loops, mesh.mesh_polylines)
        panel_path.write_text(json.dumps(panel_payload, ensure_ascii=False, indent=2), encoding="utf-8")
        if category == "plane":
            sampled_payload = {
                "name": name,
                "category": "plane",
                "loops": [
                    {
                        "loop_index": loop_index,
                        "points_rxyz": build_sampled_rxyz_curve(loop),
                    }
                    for loop_index, loop in enumerate(boundary_loops)
                ],
            }
            fitted_payload = _build_plane_fit_payload(name, boundary_loops)
            sampled_rxyz_path.write_text(
                json.dumps(sampled_payload, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            fitted_rxyz_path.write_text(
                json.dumps(fitted_payload, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            plane_fit_panels.append(fitted_payload)

        panel_summaries.append(
            {
                "name": name,
                "category": category,
                "directory": panel_root.as_posix(),
                "obj": obj_path.as_posix(),
                "panel_xyz_json": panel_path.as_posix(),
                "boundary_xyz_json": boundary_path.as_posix(),
                "vertex_count": len(vertices),
                "triangle_count": len(triangles),
                "boundary_loop_count": len(boundary_loops),
                **(
                    {
                        "sampled_rxyz_json": sampled_rxyz_path.as_posix(),
                        "fitted_rxyz_json": fitted_rxyz_path.as_posix(),
                    }
                    if category == "plane"
                    else {}
                ),
            }
        )
        demo_panels.append(panel_payload)

    metadata = {
        "source_dxf": dxf_path.as_posix(),
        "panel_count": len(panel_summaries),
        "panels": panel_summaries,
    }

    (model_root / "metadata.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    (demo_root / "index.html").write_text(generate_demo_html(demo_panels), encoding="utf-8")
    ui_path.write_text(generate_fit_ui_html(plane_fit_panels), encoding="utf-8")
    return metadata


def main() -> None:
    workspace = Path.cwd()
    export_dxf_models(workspace / "FB03C.dxf", workspace / "model", workspace / "demo", workspace / "ui.html")


if __name__ == "__main__":
    main()
