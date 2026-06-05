import json
import math
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RESULT_JSON = ROOT / "result" / "weld_results.json"
OUTPUT_HTML = ROOT / "result" / "local_gap_demo.html"
OUTPUT_INSPECTOR_HTML = ROOT / "result" / "gap_inspector.html"
OUTPUT_SECTION_HTML = ROOT / "result" / "gap_section_viewer.html"
TARGET_PLANE = "FB03C-166F-PLATE3"
TARGET_SURFACE = "FB03C-PL251-CPLATE"
TOLERANCE_MM = 1.0
X_EXAGGERATION = 12.0


def sub(a, b):
    return [a[i] - b[i] for i in range(3)]


def add(a, b):
    return [a[i] + b[i] for i in range(3)]


def mul(a, scalar):
    return [a[i] * scalar for i in range(3)]


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def norm_sq(a):
    return dot(a, a)


def norm(a):
    return math.sqrt(norm_sq(a))


def unit(a):
    length = norm(a)
    if length <= 1.0e-12:
        return [0.0, 0.0, 0.0]
    return [a[0] / length, a[1] / length, a[2] / length]


def lerp(a, b, t):
    return [a[i] + (b[i] - a[i]) * t for i in range(3)]


@dataclass
class Triangle:
    a: list
    b: list
    c: list


def closest_point_triangle(point, triangle):
    a, b, c = triangle.a, triangle.b, triangle.c
    ab = sub(b, a)
    ac = sub(c, a)
    ap = sub(point, a)

    d1 = dot(ab, ap)
    d2 = dot(ac, ap)
    if d1 <= 0.0 and d2 <= 0.0:
        return a, "va"

    bp = sub(point, b)
    d3 = dot(ab, bp)
    d4 = dot(ac, bp)
    if d3 >= 0.0 and d4 <= d3:
        return b, "vb"

    vc = d1 * d4 - d3 * d2
    if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
        v = d1 / (d1 - d3)
        return add(a, mul(ab, v)), "eab"

    cp = sub(point, c)
    d5 = dot(ab, cp)
    d6 = dot(ac, cp)
    if d6 >= 0.0 and d5 <= d6:
        return c, "vc"

    vb = d5 * d2 - d1 * d6
    if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
        w = d2 / (d2 - d6)
        return add(a, mul(ac, w)), "eac"

    va = d3 * d6 - d5 * d4
    if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
        bc = sub(c, b)
        w = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        return add(b, mul(bc, w)), "ebc"

    denom = 1.0 / (va + vb + vc)
    v = vb * denom
    w = vc * denom
    return add(a, add(mul(ab, v), mul(ac, w))), "face"


def nearest_triangle_info(point, triangles):
    best = None
    for index, triangle in enumerate(triangles):
        closest, region = closest_point_triangle(point, triangle)
        distance_sq = norm_sq(sub(point, closest))
        if best is None or distance_sq < best["distance_sq"]:
            normal_vec = cross(sub(triangle.b, triangle.a), sub(triangle.c, triangle.a))
            normal_unit = unit(normal_vec)
            offset = sub(point, closest)
            best = {
                "triangle": index,
                "closest": closest,
                "region": region,
                "distance_sq": distance_sq,
                "distance": math.sqrt(distance_sq),
                "normal": normal_unit,
                "signed_distance": dot(offset, normal_unit),
                "delta_vector": offset,
            }
    return best


def sample_segment_min_distance(start, end, triangles, steps=160):
    best = None
    for step in range(steps + 1):
        t = step / steps
        point = lerp(start, end, t)
        info = nearest_triangle_info(point, triangles)
        if best is None or info["distance"] < best["distance"]:
            best = {"t": t, "point": point, **info}
    return best


def find_focus_segment_range(loop, triangles, tolerance):
    hit_segments = []
    for index in range(len(loop) - 1):
        best = sample_segment_min_distance(loop[index], loop[index + 1], triangles, steps=120)
        if best["distance"] <= tolerance:
            hit_segments.append(index)
    if not hit_segments:
        raise RuntimeError("No local hit segment found for focus range.")
    start = max(min(hit_segments) - 1, 0)
    end = min(max(hit_segments) + 1, len(loop) - 2)
    return start, end


def build_chart_samples(loop_a, loop_b, triangles, start_segment, end_segment, samples_per_segment=24):
    arc_length = 0.0
    chart = []
    samples_a = []
    samples_b = []
    for segment_index in range(start_segment, end_segment + 1):
        seg_start_a = loop_a[segment_index]
        seg_end_a = loop_a[segment_index + 1]
        seg_start_b = loop_b[segment_index]
        seg_end_b = loop_b[segment_index + 1]
        seg_length = norm(sub(seg_end_a, seg_start_a))
        segment_steps = samples_per_segment if segment_index < end_segment else samples_per_segment + 1
        for step in range(segment_steps):
            t = step / samples_per_segment
            point_a = lerp(seg_start_a, seg_end_a, t)
            point_b = lerp(seg_start_b, seg_end_b, t)
            info_a = nearest_triangle_info(point_a, triangles)
            info_b = nearest_triangle_info(point_b, triangles)
            current_arc = arc_length + seg_length * t
            chart.append(
                {
                    "s": current_arc,
                    "lower_dist": info_a["distance"],
                    "upper_dist": info_b["distance"],
                    "normal_x": info_a["normal"][0],
                }
            )
            samples_a.append({"s": current_arc, "point": point_a, **info_a})
            samples_b.append({"s": current_arc, "point": point_b, **info_b})
        arc_length += seg_length
    return chart, samples_a, samples_b


def point_label(point):
    return f"({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})"


def collect_strip_points(loop, start_segment, end_segment):
    return [loop[index] for index in range(start_segment, end_segment + 2)]


def collect_sample_window(samples, center_index, radius):
    start = max(0, center_index - radius)
    end = min(len(samples), center_index + radius + 1)
    return [sample["point"] for sample in samples[start:end]]


def triangle_centroid(triangle):
    return [
        (triangle.a[0] + triangle.b[0] + triangle.c[0]) / 3.0,
        (triangle.a[1] + triangle.b[1] + triangle.c[1]) / 3.0,
        (triangle.a[2] + triangle.b[2] + triangle.c[2]) / 3.0,
    ]


def collect_local_surface_triangles(triangles, guide_points, margin_yz=55.0, margin_x=18.0):
    ys = [point[1] for point in guide_points]
    zs = [point[2] for point in guide_points]
    xs = [point[0] for point in guide_points]
    min_y = min(ys) - margin_yz
    max_y = max(ys) + margin_yz
    min_z = min(zs) - margin_yz
    max_z = max(zs) + margin_yz
    min_x = min(xs) - margin_x
    max_x = max(xs) + margin_x

    selected = []
    for triangle in triangles:
        xs_tri = [triangle.a[0], triangle.b[0], triangle.c[0]]
        ys_tri = [triangle.a[1], triangle.b[1], triangle.c[1]]
        zs_tri = [triangle.a[2], triangle.b[2], triangle.c[2]]
        if (
            max(xs_tri) >= min_x
            and min(xs_tri) <= max_x
            and max(ys_tri) >= min_y
            and min(ys_tri) <= max_y
            and max(zs_tri) >= min_z
            and min(zs_tri) <= max_z
        ):
            selected.append([triangle.a, triangle.b, triangle.c])
    return selected


def collect_focus_surface_triangles(triangles, center_point, radius_yz=32.0, radius_x=12.0, limit=90):
    selected = []
    for triangle in triangles:
        closest, _ = closest_point_triangle(center_point, triangle)
        dx = abs(closest[0] - center_point[0])
        dy = closest[1] - center_point[1]
        dz = closest[2] - center_point[2]
        yz_distance = math.sqrt(dy * dy + dz * dz)
        if dx <= radius_x and yz_distance <= radius_yz:
            selected.append((yz_distance, dx, [triangle.a, triangle.b, triangle.c]))
    selected.sort(key=lambda item: (item[0], item[1]))
    return [triangle for _, _, triangle in selected[:limit]]


def build_anchor(label, lower_sample, upper_sample):
    return {
        "label": label,
        "s": lower_sample["s"],
        "lower_point": lower_sample["point"],
        "upper_point": upper_sample["point"],
        "lower_closest": lower_sample["closest"],
        "upper_closest": upper_sample["closest"],
        "lower_dist": lower_sample["distance"],
        "upper_dist": upper_sample["distance"],
        "lower_delta_vector": lower_sample["delta_vector"],
        "upper_delta_vector": upper_sample["delta_vector"],
    }


def pick_spread_indices(length, count):
    if length <= 0:
        return []
    if length <= count:
        return list(range(length))
    return sorted({round(index * (length - 1) / (count - 1)) for index in range(count)})


def build_html(payload):
    json_payload = json.dumps(payload, ensure_ascii=False)
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>局部最小距离证明图</title>
  <style>
    :root {{
      --bg: #f6f1e7;
      --paper: #fffaf3;
      --ink: #17212b;
      --muted: #677381;
      --line: rgba(23, 33, 43, 0.1);
      --shadow: 0 24px 44px rgba(17, 24, 39, 0.08);
      --plane-low: #0f766e;
      --plane-high: #c2410c;
      --surface: #1f2937;
      --accent: #b91c1c;
      --mono: "IBM Plex Mono", "Consolas", monospace;
      --sans: "Segoe UI", "PingFang SC", "Microsoft YaHei", sans-serif;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      color: var(--ink);
      font-family: var(--sans);
      background:
        radial-gradient(circle at top left, rgba(15, 118, 110, 0.12), transparent 24rem),
        radial-gradient(circle at top right, rgba(194, 65, 12, 0.10), transparent 28rem),
        linear-gradient(180deg, #f9f4eb 0%, var(--bg) 100%);
    }}
    .page {{
      max-width: 1480px;
      margin: 0 auto;
      padding: 28px 22px 44px;
    }}
    .hero {{
      display: grid;
      grid-template-columns: 1.15fr 0.85fr;
      gap: 22px;
      margin-bottom: 22px;
    }}
    .card {{
      background: color-mix(in srgb, var(--paper) 92%, white 8%);
      border: 1px solid rgba(23, 33, 43, 0.08);
      box-shadow: var(--shadow);
      border-radius: 24px;
      overflow: hidden;
    }}
    .hero-main {{
      padding: 28px 30px 30px;
      position: relative;
    }}
    .hero-main::after {{
      content: "";
      position: absolute;
      right: -30px;
      top: -34px;
      width: 180px;
      height: 180px;
      border-radius: 50%;
      background: radial-gradient(circle, rgba(15, 118, 110, 0.16), transparent 70%);
      pointer-events: none;
    }}
    .eyebrow {{
      font-family: var(--mono);
      font-size: 12px;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      color: var(--plane-low);
      margin-bottom: 12px;
    }}
    h1 {{
      margin: 0 0 14px;
      font-size: clamp(34px, 5vw, 62px);
      line-height: 0.95;
      letter-spacing: -0.045em;
    }}
    .lead {{
      margin: 0 0 18px;
      max-width: 42rem;
      color: var(--muted);
      font-size: 16px;
      line-height: 1.68;
    }}
    .formula {{
      display: inline-flex;
      align-items: center;
      gap: 10px;
      border-radius: 999px;
      padding: 12px 16px;
      background: rgba(15, 118, 110, 0.08);
      font-family: var(--mono);
      font-size: 14px;
      flex-wrap: wrap;
    }}
    .hero-side {{
      padding: 24px;
      display: grid;
      gap: 14px;
      align-content: start;
    }}
    .stat-grid {{
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
    }}
    .stat {{
      padding: 16px;
      border-radius: 18px;
      background: rgba(255, 255, 255, 0.72);
      border: 1px solid rgba(23, 33, 43, 0.08);
    }}
    .stat label {{
      display: block;
      color: var(--muted);
      font-size: 12px;
      letter-spacing: 0.05em;
      text-transform: uppercase;
      margin-bottom: 8px;
    }}
    .stat strong {{
      display: block;
      font-size: 30px;
      line-height: 1;
      letter-spacing: -0.04em;
    }}
    .stat small {{
      display: block;
      margin-top: 8px;
      color: var(--muted);
      font-size: 12px;
    }}
    .hero-note {{
      padding: 14px 16px;
      border-radius: 18px;
      background: rgba(255, 255, 255, 0.68);
      border: 1px solid rgba(23, 33, 43, 0.08);
      line-height: 1.65;
      color: var(--ink);
      font-size: 14px;
    }}
    .alert-strong {{
      margin-top: 12px;
      padding: 14px 16px;
      border-radius: 18px;
      background: rgba(185, 28, 28, 0.10);
      border: 1px solid rgba(185, 28, 28, 0.24);
      color: var(--accent);
      font-family: var(--mono);
      font-size: 14px;
      line-height: 1.6;
    }}
    .main-grid {{
      display: grid;
      grid-template-columns: 1.28fr 0.72fr;
      gap: 22px;
      margin-bottom: 22px;
    }}
    .panel {{
      padding: 20px 22px 22px;
    }}
    .panel h2 {{
      margin: 0 0 6px;
      font-size: 25px;
      letter-spacing: -0.03em;
    }}
    .panel p {{
      margin: 0 0 14px;
      color: var(--muted);
      font-size: 14px;
      line-height: 1.62;
    }}
    .subtle {{
      color: var(--muted);
      font-family: var(--mono);
      font-size: 12px;
    }}
    .legend {{
      display: flex;
      flex-wrap: wrap;
      gap: 14px;
      margin-bottom: 12px;
      font-size: 13px;
      color: var(--muted);
    }}
    .legend-item {{
      display: inline-flex;
      align-items: center;
      gap: 8px;
    }}
    .swatch {{
      width: 14px;
      height: 14px;
      border-radius: 999px;
      display: inline-block;
    }}
    .viz-wrap {{
      border-radius: 18px;
      border: 1px solid rgba(23, 33, 43, 0.08);
      background: linear-gradient(180deg, rgba(255,255,255,0.82), rgba(248,243,234,0.92));
      padding: 12px;
    }}
    .viewer-toolbar {{
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      margin-bottom: 12px;
    }}
    .viewer-toolbar button {{
      border: 1px solid rgba(23, 33, 43, 0.12);
      background: rgba(255,255,255,0.82);
      color: var(--ink);
      border-radius: 999px;
      padding: 8px 12px;
      font-family: var(--mono);
      font-size: 12px;
      cursor: pointer;
    }}
    .viewer-toolbar button.primary {{
      background: rgba(185, 28, 28, 0.10);
      border-color: rgba(185, 28, 28, 0.18);
      color: var(--accent);
      font-weight: 700;
    }}
    .proof-grid {{
      display: grid;
      grid-template-columns: 1.16fr 0.84fr;
      gap: 22px;
      margin-bottom: 22px;
    }}
    .proof-side {{
      display: grid;
      gap: 14px;
      align-content: start;
    }}
    .proof-metric {{
      padding: 16px;
      border-radius: 18px;
      background: rgba(255,255,255,0.76);
      border: 1px solid rgba(23, 33, 43, 0.08);
    }}
    .proof-metric strong {{
      display: block;
      font-family: var(--mono);
      font-size: 28px;
      line-height: 1.05;
      letter-spacing: -0.04em;
      color: var(--accent);
    }}
    .proof-metric small {{
      display: block;
      margin-top: 8px;
      color: var(--muted);
      line-height: 1.6;
    }}
    svg {{
      width: 100%;
      height: auto;
      display: block;
    }}
    .right-column {{
      display: grid;
      gap: 18px;
    }}
    .anchor-list {{
      display: grid;
      gap: 12px;
    }}
    .anchor-card {{
      padding: 15px 16px;
      border-radius: 18px;
      background: rgba(255, 255, 255, 0.72);
      border: 1px solid rgba(23, 33, 43, 0.08);
    }}
    .anchor-head {{
      display: flex;
      align-items: baseline;
      justify-content: space-between;
      gap: 12px;
      margin-bottom: 10px;
    }}
    .anchor-head strong {{
      font-size: 18px;
      letter-spacing: -0.02em;
    }}
    .anchor-head span {{
      color: var(--muted);
      font-family: var(--mono);
      font-size: 12px;
    }}
    .distance-row {{
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      margin-bottom: 10px;
    }}
    .distance-box {{
      padding: 10px 11px;
      border-radius: 14px;
      border: 1px solid rgba(23, 33, 43, 0.08);
      background: rgba(255,255,255,0.8);
    }}
    .distance-box label {{
      display: block;
      color: var(--muted);
      font-size: 11px;
      text-transform: uppercase;
      letter-spacing: 0.06em;
      margin-bottom: 6px;
    }}
    .distance-box strong {{
      display: block;
      font-family: var(--mono);
      font-size: 18px;
    }}
    .distance-box strong.over {{
      color: var(--accent);
    }}
    .distance-box strong.under {{
      color: var(--plane-low);
    }}
    .status-chip {{
      display: inline-flex;
      align-items: center;
      gap: 6px;
      padding: 5px 9px;
      border-radius: 999px;
      font-size: 11px;
      font-family: var(--mono);
      margin-top: 8px;
    }}
    .status-chip.over {{
      background: rgba(185, 28, 28, 0.12);
      color: var(--accent);
    }}
    .status-chip.under {{
      background: rgba(15, 118, 110, 0.12);
      color: var(--plane-low);
    }}
    .vector {{
      color: var(--muted);
      font-size: 12px;
      font-family: var(--mono);
      line-height: 1.55;
    }}
    .bottom-grid {{
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 22px;
    }}
    .table {{
      width: 100%;
      border-collapse: collapse;
      font-size: 13px;
    }}
    .table th, .table td {{
      text-align: left;
      padding: 11px 10px;
      border-bottom: 1px solid rgba(23, 33, 43, 0.08);
      vertical-align: top;
    }}
    .table th {{
      color: var(--muted);
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: 0.05em;
    }}
    code {{
      font-family: var(--mono);
      font-size: 0.95em;
      background: rgba(23, 33, 43, 0.06);
      padding: 0.12rem 0.35rem;
      border-radius: 0.35rem;
    }}
    .note-list {{
      display: grid;
      gap: 10px;
    }}
    .note {{
      padding: 14px 16px;
      border-radius: 16px;
      background: rgba(255, 255, 255, 0.72);
      border: 1px solid rgba(23, 33, 43, 0.08);
      line-height: 1.65;
      font-size: 14px;
    }}
    .footer {{
      margin-top: 12px;
      color: var(--muted);
      font-size: 12px;
      font-family: var(--mono);
    }}
    @media (max-width: 1120px) {{
      .hero, .proof-grid, .main-grid, .bottom-grid {{
        grid-template-columns: 1fr;
      }}
    }}
    @media (max-width: 760px) {{
      .page {{
        padding: 16px 12px 24px;
      }}
      .hero-main, .hero-side, .panel {{
        border-radius: 18px;
      }}
      .hero-main {{
        padding: 22px 20px;
      }}
      .stat-grid {{
        grid-template-columns: 1fr 1fr;
      }}
    }}
  </style>
</head>
<body>
  <div class="page">
    <section class="hero">
      <div class="card hero-main">
        <div class="eyebrow">Local Min-Distance Proof</div>
        <h1>第二条长焊缝不是漏算，而是局部最小距离本来就大于 <span style="color: var(--accent)">1.0mm</span></h1>
        <p class="lead">
          这页只针对 <code>{payload["plane_name"]}</code> × <code>{payload["surface_name"]}</code>。
          主图直接画出两张板的局部关系，以及边界点到曲面最近点的最小距离连线。
          绿色面上的连线很短，橙色平行面上的连线在中段稳定变长，所以第二条完整焊缝不会成立。
        </p>
        <div class="formula">
          <span>14.000mm × |n<sub>x</sub>| {payload["midpoint"]["normal_x_abs"]:.4f}</span>
          <span>=</span>
          <strong>{payload["midpoint"]["predicted_gap_mm"]:.3f}mm</strong>
          <span>≈</span>
          <strong>实测差值 {payload["midpoint"]["gap_delta_mm"]:.3f}mm</strong>
        </div>
      </div>
      <aside class="card hero-side">
        <div class="stat-grid">
          <div class="stat">
            <label>阈值</label>
            <strong>{payload["tolerance_mm"]:.1f} mm</strong>
            <small>距离小于等于这个值才计入焊缝</small>
          </div>
          <div class="stat">
            <label>中段差值</label>
            <strong>{payload["midpoint"]["gap_delta_mm"]:.3f} mm</strong>
            <small>两张平行面对应点的距离差</small>
          </div>
          <div class="stat">
            <label>低面中段</label>
            <strong>{payload["midpoint"]["lower_dist_mm"]:.3f} mm</strong>
            <small>x=289675，形成完整长焊缝</small>
          </div>
          <div class="stat">
            <label>高面中段</label>
            <strong>{payload["midpoint"]["upper_dist_mm"]:.3f} mm</strong>
            <small>x=289689，稳定超出 1.0mm</small>
          </div>
        </div>
        <div class="hero-note">
          <strong>视图说明</strong><br>
          主图把 <code>X</code> 方向的 14mm 板厚做了 <code>{payload["x_exaggeration"]:.0f}×</code> 视觉放大，
          这样最小距离连线不会被大尺度的 <code>YZ</code> 轮廓淹没。
        </div>
        <div class="alert-strong">
          关键证明语句：局部中段在 <code>x=289689</code> 面上的精确最小距离是
          <strong>{payload["midpoint"]["upper_dist_mm"]:.3f}mm &gt; 1.0mm</strong>，
          因此该处不应形成焊缝。
        </div>
      </aside>
    </section>

    <section class="proof-grid">
      <div class="card panel">
        <h2>中段极简证明视图</h2>
        <p>这里只保留中段附近的局部 mesh、两条对应边界线和两根最小距离连线。红色粗连线就是 <code>x=289689</code> 这一侧在中段的精确最小距离，直接标出 <code>{payload["midpoint"]["upper_dist_mm"]:.3f}mm &gt; {payload["tolerance_mm"]:.1f}mm</code>。</p>
        <div class="legend">
          <span class="legend-item"><span class="swatch" style="background: rgba(31,41,55,0.82)"></span> 中段局部 surface mesh</span>
          <span class="legend-item"><span class="swatch" style="background: var(--plane-low)"></span> 对照边界 x=289675</span>
          <span class="legend-item"><span class="swatch" style="background: var(--accent)"></span> 被质疑的中段最小距离</span>
        </div>
        <div class="viewer-toolbar">
          <button id="proofFocusBtn" class="primary" type="button">只看中段</button>
          <button id="proofZoomInBtn" type="button">放大</button>
          <button id="proofZoomOutBtn" type="button">缩小</button>
          <button id="proofResetBtn" type="button">重置视图</button>
        </div>
        <div class="viz-wrap">
          <svg id="midpointProofView" viewBox="0 0 980 620" aria-label="中段极简证明视图"></svg>
        </div>
        <div class="footer">鼠标滚轮缩放，按住拖拽平移。默认就会锁定到中段那一根 <code>{payload["midpoint"]["upper_dist_mm"]:.3f}mm</code> 的最小距离连线。</div>
      </div>
      <aside class="proof-side">
        <div class="card panel">
          <h2>一句话结论</h2>
          <p>这张图只回答一个问题: 为什么第二条完整焊缝没有出现。</p>
          <div class="proof-metric">
            <strong>{payload["midpoint"]["upper_dist_mm"]:.3f}mm &gt; {payload["tolerance_mm"]:.1f}mm</strong>
            <small><code>x=289689</code> 这条边界线在中段到另一块板子的精确最小距离已经超过阈值，所以这里不会形成连续焊缝。</small>
          </div>
          <div class="proof-metric">
            <strong>{payload["midpoint"]["lower_dist_mm"]:.3f}mm</strong>
            <small>对照的 <code>x=289675</code> 这一侧在同一位置几乎贴住曲面，所以它能形成完整长焊缝。</small>
          </div>
        </div>
      </aside>
    </section>

    <section class="main-grid">
      <div class="card panel">
        <h2>局部 3D 关系图</h2>
        <p>深灰色是另一块板子的局部 mesh。橙色线是当前要检查的边界线，红色粗线是这条边界线上 <code>min distance &gt; 1.0mm</code> 的连续区段。红色虚线是这些超阈值点到另一块板子的精确最小距离连线。</p>
        <div class="legend">
          <span class="legend-item"><span class="swatch" style="background: var(--surface)"></span> 另一块板子的局部 mesh</span>
          <span class="legend-item"><span class="swatch" style="background: var(--plane-high)"></span> 被检查的边界线 x=289689</span>
          <span class="legend-item"><span class="swatch" style="background: var(--plane-low)"></span> 对照边界线 x=289675</span>
          <span class="legend-item"><span class="swatch" style="background: var(--accent)"></span> 边界线上 min distance &gt; 1.0mm 的红段</span>
        </div>
        <div class="viewer-toolbar">
          <button id="focusMidBtn" class="primary" type="button">聚焦中段最小距离</button>
          <button id="fitAllBtn" type="button">适配全部</button>
          <button id="zoomInBtn" type="button">放大</button>
          <button id="zoomOutBtn" type="button">缩小</button>
          <button id="resetPanBtn" type="button">重置视图</button>
        </div>
        <div class="viz-wrap">
          <svg id="geometryView" viewBox="0 0 960 680" aria-label="局部三维关系图"></svg>
        </div>
        <div class="footer">鼠标滚轮缩放，按住拖拽平移。默认视图会直接聚焦到中段那根 <code>{payload["midpoint"]["upper_dist_mm"]:.3f}mm &gt; 1.0mm</code> 的最小距离连线。</div>
      </div>

      <div class="right-column">
        <div class="card panel">
          <h2>YZ 投影为何容易误判</h2>
        <p>左图如果还不够直观，这里单独画出 <code>YZ</code> 投影。红色粗线依旧是同一条边界线上 <code>min distance &gt; 1.0mm</code> 的那一段，说明问题不是点状偶发，而是连续区段超阈值。</p>
          <div class="viz-wrap">
            <svg id="yzView" viewBox="0 0 520 300" aria-label="YZ 投影视图"></svg>
          </div>
        </div>

        <div class="card panel">
          <h2>三个关键位置</h2>
        <p>每张卡片都取自红色超阈值区段内部，给出边界点、最近曲面点，以及精确最小距离向量。</p>
          <div class="anchor-list" id="anchorList"></div>
        </div>
      </div>
    </section>

    <section class="bottom-grid">
      <div class="card panel">
        <h2>精确坐标对照</h2>
        <p>这里列出三个代表位置的坐标与距离。它们足够用于给别人解释“为什么一侧有完整长焊缝，另一侧只有两端短焊缝”。</p>
        <table class="table" id="anchorTable"></table>
      </div>

      <div class="card panel">
        <h2>证据摘要</h2>
        <p>这三条结论可以直接作为说明用语。</p>
        <div class="note-list">
          <div class="note"><strong>证据 1</strong><br><code>x=289675</code> 中段距离约 <code>{payload["midpoint"]["lower_dist_mm"]:.3f}mm</code>，明显落在阈值内，所以形成完整长焊缝。</div>
          <div class="note"><strong>证据 2</strong><br><code>x=289689</code> 对应中段距离约 <code>{payload["midpoint"]["upper_dist_mm"]:.3f}mm</code>，稳定超出 <code>1.0mm</code>，所以中段不应形成焊缝。</div>
          <div class="note"><strong>证据 3</strong><br>局部法向的 <code>|n_x| ≈ {payload["midpoint"]["normal_x_abs"]:.4f}</code>，板厚 <code>14mm</code> 在法向上投影成约 <code>{payload["midpoint"]["predicted_gap_mm"]:.3f}mm</code>，与实际差值 <code>{payload["midpoint"]["gap_delta_mm"]:.3f}mm</code> 一致。</div>
          <div class="note"><strong>补充</strong><br>平面板两张面的边界点列是严格一一对应的，只差一个 14mm 的 X 偏移；问题不在平面板边界数据，而在曲面局部法向与板厚方向的投影关系。</div>
        </div>
        <div class="footer">Generated from <code>result/weld_results.json</code> by <code>demo/generate_local_gap_demo.py</code></div>
      </div>
    </section>
  </div>

  <script>
    const DATA = {json_payload};

    function createSvg(tag, attrs = {{}}) {{
      const node = document.createElementNS('http://www.w3.org/2000/svg', tag);
      for (const [key, value] of Object.entries(attrs)) {{
        node.setAttribute(key, value);
      }}
      return node;
    }}

    function statusText(value) {{
      return value > DATA.tolerance_mm
        ? `${{value.toFixed(3)}}mm > ${{DATA.tolerance_mm.toFixed(1)}}mm`
        : `${{value.toFixed(3)}}mm < ${{DATA.tolerance_mm.toFixed(1)}}mm`;
    }}

    function statusClass(value) {{
      return value > DATA.tolerance_mm ? 'over' : 'under';
    }}

    function computeBounds(points) {{
      const xs = points.map(p => p[0]);
      const ys = points.map(p => p[1]);
      return {{
        minX: Math.min(...xs),
        maxX: Math.max(...xs),
        minY: Math.min(...ys),
        maxY: Math.max(...ys)
      }};
    }}

    function fitProjector(points, width, height, padding) {{
      const bounds = computeBounds(points);
      const spanX = Math.max(1e-6, bounds.maxX - bounds.minX);
      const spanY = Math.max(1e-6, bounds.maxY - bounds.minY);
      const scale = Math.min((width - padding * 2) / spanX, (height - padding * 2) / spanY);
      return point => ([
        padding + (point[0] - bounds.minX) * scale,
        height - padding - (point[1] - bounds.minY) * scale
      ]);
    }}

    function projectLocal(point) {{
      const c = DATA.local_center;
      const dx = (point[0] - c[0]) * DATA.x_exaggeration;
      const dy = point[1] - c[1];
      const dz = point[2] - c[2];
      return [
        dy + dx * 0.72,
        -dz + dx * 0.34,
        dx * 0.22 - dy * 0.06 + dz * 0.12
      ];
    }}

    function polylinePath(points, projector) {{
      return points.map((point, index) => {{
        const p = projector(point);
        return `${{index ? 'L' : 'M'}} ${{p[0].toFixed(2)}} ${{p[1].toFixed(2)}}`;
      }}).join(' ');
    }}

    let geometryViewApi = null;
    let midpointProofApi = null;

    function attachZoomPan(svg, viewport, width, height, initialBounds, fitPadding, minScale = 0.35, maxScale = 22) {{
      let viewState = {{ scale: 1, tx: 0, ty: 0 }};
      let dragState = null;

      function applyView() {{
        viewport.setAttribute('transform', `translate(${{viewState.tx}} ${{viewState.ty}}) scale(${{viewState.scale}})`);
      }}

      function fitBounds(bounds, padding = fitPadding) {{
        const spanX = Math.max(1, bounds.maxX - bounds.minX);
        const spanY = Math.max(1, bounds.maxY - bounds.minY);
        const scale = Math.min((width - padding * 2) / spanX, (height - padding * 2) / spanY);
        const cx = (bounds.minX + bounds.maxX) * 0.5;
        const cy = (bounds.minY + bounds.maxY) * 0.5;
        viewState.scale = scale;
        viewState.tx = width * 0.5 - cx * scale;
        viewState.ty = height * 0.5 - cy * scale;
        applyView();
      }}

      function zoomAt(cx, cy, factor) {{
        const nextScale = Math.max(minScale, Math.min(maxScale, viewState.scale * factor));
        const worldX = (cx - viewState.tx) / viewState.scale;
        const worldY = (cy - viewState.ty) / viewState.scale;
        viewState.scale = nextScale;
        viewState.tx = cx - worldX * nextScale;
        viewState.ty = cy - worldY * nextScale;
        applyView();
      }}

      svg.addEventListener('wheel', event => {{
        event.preventDefault();
        const rect = svg.getBoundingClientRect();
        const cx = event.clientX - rect.left;
        const cy = event.clientY - rect.top;
        zoomAt(cx, cy, event.deltaY < 0 ? 1.15 : 1 / 1.15);
      }}, {{ passive: false }});

      svg.addEventListener('pointerdown', event => {{
        dragState = {{
          x: event.clientX,
          y: event.clientY,
          tx: viewState.tx,
          ty: viewState.ty
        }};
        svg.setPointerCapture(event.pointerId);
      }});

      svg.addEventListener('pointermove', event => {{
        if (!dragState) return;
        viewState.tx = dragState.tx + (event.clientX - dragState.x);
        viewState.ty = dragState.ty + (event.clientY - dragState.y);
        applyView();
      }});

      function endDrag(event) {{
        if (dragState) {{
          dragState = null;
          if (event.pointerId !== undefined && svg.hasPointerCapture(event.pointerId)) {{
            svg.releasePointerCapture(event.pointerId);
          }}
        }}
      }}

      svg.addEventListener('pointerup', endDrag);
      svg.addEventListener('pointerleave', endDrag);
      svg.addEventListener('pointercancel', endDrag);

      fitBounds(initialBounds, fitPadding);
      return {{
        fit(bounds = initialBounds, padding = fitPadding) {{ fitBounds(bounds, padding); }},
        zoomIn() {{ zoomAt(width * 0.5, height * 0.5, 1.2); }},
        zoomOut() {{ zoomAt(width * 0.5, height * 0.5, 1 / 1.2); }},
      }};
    }}

    function renderMidpointProofView() {{
      const svg = document.getElementById('midpointProofView');
      const width = 980;
      const height = 620;
      svg.innerHTML = '';
      const viewport = createSvg('g', {{ id: 'proofViewport' }});
      svg.appendChild(viewport);

      const focusTriangles = DATA.focus_surface_triangles.map(tri => tri.map(projectLocal));
      const lowerSegment = DATA.focus_lower_segment.map(projectLocal);
      const upperSegment = DATA.focus_upper_segment.map(projectLocal);
      const anchor = DATA.anchors.find(item => item.label === '局部中段') || DATA.anchors[0];
      const lowerPoint = projectLocal(anchor.lower_point);
      const upperPoint = projectLocal(anchor.upper_point);
      const lowerClosest = projectLocal(anchor.lower_closest);
      const upperClosest = projectLocal(anchor.upper_closest);

      const allPoints = [];
      for (const tri of focusTriangles) allPoints.push(...tri);
      allPoints.push(...lowerSegment, ...upperSegment, lowerPoint, upperPoint, lowerClosest, upperClosest);
      const projector = fitProjector(allPoints.map(point => [point[0], point[1]]), width, height, 24);

      const sortedTriangles = focusTriangles
        .map(tri => ({{
          points: tri,
          depth: (tri[0][2] + tri[1][2] + tri[2][2]) / 3
        }}))
        .sort((a, b) => a.depth - b.depth);

      for (const tri of sortedTriangles) {{
        const points = tri.points.map(point => projector([point[0], point[1]]));
        const path = points.map((point, index) => `${{index ? 'L' : 'M'}} ${{point[0].toFixed(2)}} ${{point[1].toFixed(2)}}`).join(' ') + ' Z';
        viewport.appendChild(createSvg('path', {{
          d: path,
          fill: 'rgba(31,41,55,0.24)',
          stroke: 'rgba(17,24,39,0.48)',
          'stroke-width': '1.6'
        }}));
      }}

      viewport.appendChild(createSvg('path', {{
        d: polylinePath(DATA.focus_lower_segment, point => projector(projectLocal(point))),
        fill: 'none',
        stroke: 'var(--plane-low)',
        'stroke-width': '6',
        'stroke-linecap': 'round',
        'stroke-linejoin': 'round'
      }}));

      viewport.appendChild(createSvg('path', {{
        d: polylinePath(DATA.focus_upper_segment, point => projector(projectLocal(point))),
        fill: 'none',
        stroke: 'rgba(194,65,12,0.55)',
        'stroke-width': '5',
        'stroke-linecap': 'round',
        'stroke-linejoin': 'round'
      }}));

      const pLowerPoint = projector([lowerPoint[0], lowerPoint[1]]);
      const pUpperPoint = projector([upperPoint[0], upperPoint[1]]);
      const pLowerClosest = projector([lowerClosest[0], lowerClosest[1]]);
      const pUpperClosest = projector([upperClosest[0], upperClosest[1]]);

      viewport.appendChild(createSvg('line', {{
        x1: pLowerPoint[0], y1: pLowerPoint[1], x2: pLowerClosest[0], y2: pLowerClosest[1],
        stroke: 'var(--plane-low)', 'stroke-width': '3.0'
      }}));
      viewport.appendChild(createSvg('line', {{
        x1: pUpperPoint[0], y1: pUpperPoint[1], x2: pUpperClosest[0], y2: pUpperClosest[1],
        stroke: 'var(--accent)', 'stroke-width': '6.5'
      }}));

      for (const marker of [
        [pLowerPoint, 'var(--plane-low)', 6],
        [pUpperPoint, 'var(--accent)', 7],
        [pLowerClosest, 'var(--surface)', 6],
        [pUpperClosest, 'var(--surface)', 7]
      ]) {{
        viewport.appendChild(createSvg('circle', {{
          cx: marker[0][0], cy: marker[0][1], r: marker[2],
          fill: marker[1], stroke: '#fff', 'stroke-width': '2.4'
        }}));
      }}

      viewport.appendChild(createSvg('circle', {{
        cx: pUpperPoint[0], cy: pUpperPoint[1], r: 16,
        fill: 'none', stroke: 'rgba(185,28,28,0.24)', 'stroke-width': '8'
      }}));
      viewport.appendChild(createSvg('circle', {{
        cx: pUpperClosest[0], cy: pUpperClosest[1], r: 16,
        fill: 'none', stroke: 'rgba(185,28,28,0.18)', 'stroke-width': '8'
      }}));

      const labelBox = createSvg('rect', {{
        x: Math.min(pUpperPoint[0], pUpperClosest[0]) - 20,
        y: Math.min(pUpperPoint[1], pUpperClosest[1]) - 74,
        width: 250,
        height: 52,
        rx: 14,
        fill: 'rgba(185,28,28,0.10)',
        stroke: 'rgba(185,28,28,0.24)'
      }});
      viewport.appendChild(labelBox);

      const label1 = createSvg('text', {{
        x: Math.min(pUpperPoint[0], pUpperClosest[0]) - 4,
        y: Math.min(pUpperPoint[1], pUpperClosest[1]) - 48,
        'font-size': '12',
        fill: 'var(--accent)',
        'font-family': 'var(--mono)'
      }});
      label1.textContent = 'midpoint exact min distance';
      viewport.appendChild(label1);

      const label2 = createSvg('text', {{
        x: Math.min(pUpperPoint[0], pUpperClosest[0]) - 4,
        y: Math.min(pUpperPoint[1], pUpperClosest[1]) - 28,
        'font-size': '24',
        fill: 'var(--accent)',
        'font-family': 'var(--mono)',
        'font-weight': '700'
      }});
      label2.textContent = `${{DATA.midpoint.upper_dist_mm.toFixed(3)}}mm > ${{DATA.tolerance_mm.toFixed(1)}}mm`;
      viewport.appendChild(label2);

      const greenLabel = createSvg('text', {{
        x: ((pLowerPoint[0] + pLowerClosest[0]) * 0.5) + 12,
        y: ((pLowerPoint[1] + pLowerClosest[1]) * 0.5) + 18,
        'font-size': '12',
        fill: 'var(--plane-low)',
        'font-family': 'var(--mono)'
      }});
      greenLabel.textContent = `${{DATA.midpoint.lower_dist_mm.toFixed(3)}}mm`;
      viewport.appendChild(greenLabel);

      const note = createSvg('text', {{
        x: 26, y: 28, 'font-size': '12', fill: 'var(--muted)', 'font-family': 'var(--mono)'
      }});
      note.textContent = 'Only midpoint local mesh + corresponding boundary lines + exact shortest connectors';
      viewport.appendChild(note);

      const boundsPoints = [...focusTriangles.flat(), ...lowerSegment, ...upperSegment, lowerPoint, upperPoint, lowerClosest, upperClosest]
        .map(point => projector([point[0], point[1]]));
      const xs = boundsPoints.map(point => point[0]);
      const ys = boundsPoints.map(point => point[1]);
      const focusBounds = {{
        minX: Math.min(...xs) - 24,
        maxX: Math.max(...xs) + 24,
        minY: Math.min(...ys) - 34,
        maxY: Math.max(...ys) + 34
      }};

      const controls = attachZoomPan(svg, viewport, width, height, focusBounds, 52, 0.8, 30);
      midpointProofApi = {{
        focus() {{ controls.fit(focusBounds, 52); }},
        zoomIn() {{ controls.zoomIn(); }},
        zoomOut() {{ controls.zoomOut(); }},
        reset() {{ controls.fit(focusBounds, 52); }}
      }};
    }}

    function renderGeometryView() {{
      const svg = document.getElementById('geometryView');
      const width = 960;
      const height = 680;
      svg.innerHTML = '';
      const viewport = createSvg('g', {{ id: 'geometryViewport' }});
      svg.appendChild(viewport);
      const surfaceTriangles = DATA.surface_patch_triangles.map(tri => tri.map(projectLocal));
      const focusSurfaceTriangles = DATA.focus_surface_triangles.map(tri => tri.map(projectLocal));
      const lowerStrip = DATA.lower_strip.map(projectLocal);
      const upperStrip = DATA.upper_strip.map(projectLocal);
      const overUpper = DATA.upper_over_strip.map(projectLocal);
      const lowerSurfaceCurve = DATA.surface_curve.map(projectLocal);
      const anchorProjected = DATA.anchors.map(anchor => ({{
        label: anchor.label,
        lowerPoint: projectLocal(anchor.lower_point),
        upperPoint: projectLocal(anchor.upper_point),
        lowerClosest: projectLocal(anchor.lower_closest),
        upperClosest: projectLocal(anchor.upper_closest),
        lowerDist: anchor.lower_dist,
        upperDist: anchor.upper_dist
      }}));
      const allProjected = [];
      for (const tri of surfaceTriangles) allProjected.push(...tri);
      for (const tri of focusSurfaceTriangles) allProjected.push(...tri);
      allProjected.push(...lowerStrip, ...upperStrip, ...lowerSurfaceCurve);
      for (const anchor of anchorProjected) {{
        allProjected.push(anchor.lowerPoint, anchor.upperPoint, anchor.lowerClosest, anchor.upperClosest);
      }}
      const projector = fitProjector(allProjected.map(p => [p[0], p[1]]), width, height, 34);

      const banner = createSvg('rect', {{
        x: 28, y: 22, width: 410, height: 58, rx: 16,
        fill: 'rgba(185,28,28,0.10)', stroke: 'rgba(185,28,28,0.22)'
      }});
      viewport.appendChild(banner);
      const bannerText1 = createSvg('text', {{
        x: 46, y: 46, 'font-size': '13', fill: 'var(--accent)', 'font-family': 'var(--mono)'
      }});
      bannerText1.textContent = 'Midpoint proof';
      viewport.appendChild(bannerText1);
      const bannerText2 = createSvg('text', {{
        x: 46, y: 66, 'font-size': '18', fill: 'var(--accent)', 'font-family': 'var(--mono)', 'font-weight': '700'
      }});
      bannerText2.textContent = `${{DATA.midpoint.upper_dist_mm.toFixed(3)}}mm > ${{DATA.tolerance_mm.toFixed(1)}}mm`;
      viewport.appendChild(bannerText2);

      const sortedTriangles = surfaceTriangles
        .map(tri => ({{
          points: tri,
          depth: (tri[0][2] + tri[1][2] + tri[2][2]) / 3
        }}))
        .sort((a, b) => a.depth - b.depth);

      for (const tri of sortedTriangles) {{
        const points = tri.points.map(point => projector([point[0], point[1]]));
        const path = points.map((point, index) => `${{index ? 'L' : 'M'}} ${{point[0].toFixed(2)}} ${{point[1].toFixed(2)}}`).join(' ') + ' Z';
        viewport.appendChild(createSvg('path', {{
          d: path,
          fill: 'rgba(31,41,55,0.045)',
          stroke: 'rgba(31,41,55,0.06)',
          'stroke-width': '0.7'
        }}));
      }}

      const sortedFocusTriangles = focusSurfaceTriangles
        .map(tri => ({{
          points: tri,
          depth: (tri[0][2] + tri[1][2] + tri[2][2]) / 3
        }}))
        .sort((a, b) => a.depth - b.depth);

      for (const tri of sortedFocusTriangles) {{
        const points = tri.points.map(point => projector([point[0], point[1]]));
        const path = points.map((point, index) => `${{index ? 'L' : 'M'}} ${{point[0].toFixed(2)}} ${{point[1].toFixed(2)}}`).join(' ') + ' Z';
        viewport.appendChild(createSvg('path', {{
          d: path,
          fill: 'rgba(31,41,55,0.20)',
          stroke: 'rgba(17,24,39,0.42)',
          'stroke-width': '1.5'
        }}));
      }}

      for (let i = 0; i < lowerStrip.length - 1; i++) {{
        const quad = [lowerStrip[i], upperStrip[i], upperStrip[i + 1], lowerStrip[i + 1]].map(point => projector([point[0], point[1]]));
        const path = quad.map((point, index) => `${{index ? 'L' : 'M'}} ${{point[0].toFixed(2)}} ${{point[1].toFixed(2)}}`).join(' ') + ' Z';
        viewport.appendChild(createSvg('path', {{
          d: path,
          fill: 'rgba(194,65,12,0.035)',
          stroke: 'rgba(194,65,12,0.05)',
          'stroke-width': '0.8'
        }}));
      }}

      viewport.appendChild(createSvg('path', {{
        d: polylinePath(DATA.surface_curve, point => projector(projectLocal(point))),
        fill: 'none',
        stroke: 'rgba(31,41,55,0.92)',
        'stroke-width': '4',
        'stroke-linecap': 'round',
        'stroke-linejoin': 'round'
      }}));

      viewport.appendChild(createSvg('path', {{
        d: polylinePath(DATA.lower_strip, point => projector(projectLocal(point))),
        fill: 'none',
        stroke: 'var(--plane-low)',
        'stroke-width': '5',
        'stroke-linecap': 'round',
        'stroke-linejoin': 'round'
      }}));

      viewport.appendChild(createSvg('path', {{
        d: polylinePath(DATA.upper_strip, point => projector(projectLocal(point))),
        fill: 'none',
        stroke: 'rgba(194,65,12,0.45)',
        'stroke-width': '4',
        'stroke-linecap': 'round',
        'stroke-linejoin': 'round'
      }}));

      viewport.appendChild(createSvg('path', {{
        d: polylinePath(DATA.upper_over_strip, point => projector(projectLocal(point))),
        fill: 'none',
        stroke: 'var(--accent)',
        'stroke-width': '8',
        'stroke-linecap': 'round',
        'stroke-linejoin': 'round'
      }}));

      const focusAnchor = anchorProjected.find(anchor => anchor.label === '局部中段') || anchorProjected[0];
      let focusBounds = null;

      for (const anchor of anchorProjected) {{
        const lowerPoint = projector([anchor.lowerPoint[0], anchor.lowerPoint[1]]);
        const upperPoint = projector([anchor.upperPoint[0], anchor.upperPoint[1]]);
        const lowerClosest = projector([anchor.lowerClosest[0], anchor.lowerClosest[1]]);
        const upperClosest = projector([anchor.upperClosest[0], anchor.upperClosest[1]]);

        viewport.appendChild(createSvg('line', {{
          x1: lowerPoint[0], y1: lowerPoint[1], x2: lowerClosest[0], y2: lowerClosest[1],
          stroke: 'var(--plane-low)', 'stroke-width': '2.4', 'stroke-dasharray': '7 6'
        }}));
        viewport.appendChild(createSvg('line', {{
          x1: upperPoint[0], y1: upperPoint[1], x2: upperClosest[0], y2: upperClosest[1],
          stroke: anchor.upperDist > DATA.tolerance_mm ? 'var(--accent)' : 'var(--plane-high)',
          'stroke-width': anchor.label === '局部中段' ? '5.0' : (anchor.upperDist > DATA.tolerance_mm ? '3.2' : '2.6'),
          'stroke-dasharray': '7 6'
        }}));

        for (const marker of [
          [lowerPoint, 'var(--plane-low)'],
          [upperPoint, anchor.upperDist > DATA.tolerance_mm ? 'var(--accent)' : 'var(--plane-high)'],
          [lowerClosest, 'var(--surface)'],
          [upperClosest, 'var(--surface)']
        ]) {{
          viewport.appendChild(createSvg('circle', {{
            cx: marker[0][0], cy: marker[0][1], r: 5.3,
            fill: marker[1], stroke: '#fff', 'stroke-width': '2'
          }}));
        }}

        const upperMidX = (upperPoint[0] + upperClosest[0]) * 0.5 + 8;
        const upperMidY = (upperPoint[1] + upperClosest[1]) * 0.5 - 8;
        const text = createSvg('text', {{
          x: upperMidX,
          y: upperMidY,
          'font-size': anchor.label === '局部中段' ? '16' : '12',
          fill: anchor.upperDist > DATA.tolerance_mm ? 'var(--accent)' : 'var(--plane-high)',
          'font-family': 'var(--mono)',
          'font-weight': anchor.label === '局部中段' ? '700' : '400'
        }});
        text.textContent = `${{anchor.label}}: ${{statusText(anchor.upperDist)}}`;
        viewport.appendChild(text);

        const lowerText = createSvg('text', {{
          x: (lowerPoint[0] + lowerClosest[0]) * 0.5 + 8,
          y: (lowerPoint[1] + lowerClosest[1]) * 0.5 + 16,
          'font-size': '12',
          fill: 'var(--plane-low)',
          'font-family': 'var(--mono)'
        }});
        lowerText.textContent = statusText(anchor.lowerDist);
        viewport.appendChild(lowerText);

        if (anchor === focusAnchor) {{
          const focusProjected = [];
          for (const tri of focusSurfaceTriangles) focusProjected.push(...tri.map(point => projector([point[0], point[1]])));
          focusProjected.push(upperPoint, upperClosest, lowerPoint, lowerClosest);
          const focusXs = focusProjected.map(point => point[0]);
          const focusYs = focusProjected.map(point => point[1]);
          focusBounds = {{
            minX: Math.min(...focusXs) - 32,
            maxX: Math.max(...focusXs) + 32,
            minY: Math.min(...focusYs) - 32,
            maxY: Math.max(...focusYs) + 32
          }};
          viewport.appendChild(createSvg('circle', {{
            cx: upperPoint[0], cy: upperPoint[1], r: 11,
            fill: 'none', stroke: 'rgba(185,28,28,0.28)', 'stroke-width': '6'
          }}));
          viewport.appendChild(createSvg('circle', {{
            cx: upperClosest[0], cy: upperClosest[1], r: 11,
            fill: 'none', stroke: 'rgba(185,28,28,0.22)', 'stroke-width': '6'
          }}));
        }}
      }}

      const tag = createSvg('text', {{
        x: 42, y: 36, 'font-size': '12', fill: 'var(--muted)', 'font-family': 'var(--mono)'
      }});
      tag.textContent = `X exaggerated ${{DATA.x_exaggeration.toFixed(0)}}x`;
      viewport.appendChild(tag);

      const note = createSvg('text', {{
        x: 42, y: 96, 'font-size': '13', fill: 'var(--accent)', 'font-family': 'var(--mono)', 'font-weight': '700'
      }});
      note.textContent = 'Red boundary segment = exact min distance > 1.0mm';
      viewport.appendChild(note);

      const meshNote = createSvg('text', {{
        x: 42, y: 116, 'font-size': '12', fill: 'rgba(31,41,55,0.78)', 'font-family': 'var(--mono)'
      }});
      meshNote.textContent = 'Dark mesh patch = midpoint local surface triangles';
      viewport.appendChild(meshNote);

      let viewState = {{ scale: 1, tx: 0, ty: 0 }};
      let dragState = null;

      function applyView() {{
        viewport.setAttribute('transform', `translate(${{viewState.tx}} ${{viewState.ty}}) scale(${{viewState.scale}})`);
      }}

      function fitBounds(bounds, padding = 80) {{
        const spanX = Math.max(1, bounds.maxX - bounds.minX);
        const spanY = Math.max(1, bounds.maxY - bounds.minY);
        const scale = Math.min((width - padding * 2) / spanX, (height - padding * 2) / spanY);
        const cx = (bounds.minX + bounds.maxX) * 0.5;
        const cy = (bounds.minY + bounds.maxY) * 0.5;
        viewState.scale = scale;
        viewState.tx = width * 0.5 - cx * scale;
        viewState.ty = height * 0.5 - cy * scale;
        applyView();
      }}

      function zoomAt(cx, cy, factor) {{
        const nextScale = Math.max(0.35, Math.min(18, viewState.scale * factor));
        const worldX = (cx - viewState.tx) / viewState.scale;
        const worldY = (cy - viewState.ty) / viewState.scale;
        viewState.scale = nextScale;
        viewState.tx = cx - worldX * nextScale;
        viewState.ty = cy - worldY * nextScale;
        applyView();
      }}

      svg.addEventListener('wheel', event => {{
        event.preventDefault();
        const rect = svg.getBoundingClientRect();
        const cx = event.clientX - rect.left;
        const cy = event.clientY - rect.top;
        zoomAt(cx, cy, event.deltaY < 0 ? 1.15 : 1 / 1.15);
      }}, {{ passive: false }});

      svg.addEventListener('pointerdown', event => {{
        dragState = {{
          x: event.clientX,
          y: event.clientY,
          tx: viewState.tx,
          ty: viewState.ty
        }};
        svg.setPointerCapture(event.pointerId);
      }});

      svg.addEventListener('pointermove', event => {{
        if (!dragState) return;
        viewState.tx = dragState.tx + (event.clientX - dragState.x);
        viewState.ty = dragState.ty + (event.clientY - dragState.y);
        applyView();
      }});

      function endDrag(event) {{
        if (dragState) {{
          dragState = null;
          if (event.pointerId !== undefined && svg.hasPointerCapture(event.pointerId)) {{
            svg.releasePointerCapture(event.pointerId);
          }}
        }}
      }}

      svg.addEventListener('pointerup', endDrag);
      svg.addEventListener('pointerleave', endDrag);
      svg.addEventListener('pointercancel', endDrag);

      const allBounds = {{
        minX: 28,
        maxX: width - 28,
        minY: 22,
        maxY: height - 22
      }};
      fitBounds(focusBounds || allBounds, 110);

      geometryViewApi = {{
        fitAll() {{ fitBounds(allBounds, 70); }},
        focusMid() {{ fitBounds(focusBounds || allBounds, 110); }},
        zoomIn() {{ zoomAt(width * 0.5, height * 0.5, 1.2); }},
        zoomOut() {{ zoomAt(width * 0.5, height * 0.5, 1 / 1.2); }},
        reset() {{ fitBounds(focusBounds || allBounds, 110); }}
      }};
    }}

    function renderYZView() {{
      const svg = document.getElementById('yzView');
      const width = 520;
      const height = 300;
      const points = [];
      for (const group of [DATA.lower_strip, DATA.upper_strip, DATA.surface_curve]) {{
        for (const point of group) points.push([point[1], point[2]]);
      }}
      const projector = fitProjector(points, width, height, 26);
      const pathFor = polyline => polyline.map((point, index) => {{
        const p = projector([point[1], point[2]]);
        return `${{index ? 'L' : 'M'}} ${{p[0].toFixed(2)}} ${{p[1].toFixed(2)}}`;
      }}).join(' ');

      svg.appendChild(createSvg('path', {{
        d: pathFor(DATA.surface_curve),
        fill: 'none',
        stroke: 'rgba(31,41,55,0.88)',
        'stroke-width': '4'
      }}));
      svg.appendChild(createSvg('path', {{
        d: pathFor(DATA.lower_strip),
        fill: 'none',
        stroke: 'var(--plane-low)',
        'stroke-width': '4'
      }}));
      svg.appendChild(createSvg('path', {{
        d: pathFor(DATA.upper_strip),
        fill: 'none',
        stroke: 'rgba(194,65,12,0.45)',
        'stroke-width': '3',
        opacity: '0.88'
      }}));
      svg.appendChild(createSvg('path', {{
        d: pathFor(DATA.upper_over_strip),
        fill: 'none',
        stroke: 'var(--accent)',
        'stroke-width': '6'
      }}));

      for (const anchor of DATA.anchors) {{
        const lowerPoint = projector([anchor.lower_point[1], anchor.lower_point[2]]);
        const upperPoint = projector([anchor.upper_point[1], anchor.upper_point[2]]);
        svg.appendChild(createSvg('circle', {{
          cx: lowerPoint[0], cy: lowerPoint[1], r: 4.5, fill: 'var(--plane-low)', stroke: '#fff', 'stroke-width': '1.8'
        }}));
        svg.appendChild(createSvg('circle', {{
          cx: upperPoint[0], cy: upperPoint[1], r: 4.5, fill: 'var(--plane-high)', stroke: '#fff', 'stroke-width': '1.8'
        }}));
      }}

      const note = createSvg('text', {{
        x: 26, y: 26, 'font-size': '12', fill: 'var(--muted)', 'font-family': 'var(--mono)'
      }});
      note.textContent = 'YZ view: the red part is still the >1.0mm segment on the same boundary line.';
      svg.appendChild(note);
    }}

    function renderAnchorCards() {{
      const root = document.getElementById('anchorList');
      for (const anchor of DATA.anchors) {{
        const lowerCls = statusClass(anchor.lower_dist);
        const upperCls = statusClass(anchor.upper_dist);
        const card = document.createElement('div');
        card.className = 'anchor-card';
        card.innerHTML = `
          <div class="anchor-head">
            <strong>${{anchor.label}}</strong>
            <span>s=${{anchor.s.toFixed(1)}}mm</span>
          </div>
          <div class="distance-row">
            <div class="distance-box">
              <label>x=289675</label>
              <strong class="${{lowerCls}}">${{statusText(anchor.lower_dist)}}</strong>
              <span class="status-chip ${{lowerCls}}">${{lowerCls === 'over' ? '超阈值' : '阈值内'}}</span>
            </div>
            <div class="distance-box">
              <label>x=289689</label>
              <strong class="${{upperCls}}">${{statusText(anchor.upper_dist)}}</strong>
              <span class="status-chip ${{upperCls}}">${{upperCls === 'over' ? '超阈值' : '阈值内'}}</span>
            </div>
          </div>
          <div class="vector">
            lower Δ = (${{anchor.lower_delta_vector[0].toFixed(3)}}, ${{anchor.lower_delta_vector[1].toFixed(3)}}, ${{anchor.lower_delta_vector[2].toFixed(3)}})<br>
            upper Δ = (${{anchor.upper_delta_vector[0].toFixed(3)}}, ${{anchor.upper_delta_vector[1].toFixed(3)}}, ${{anchor.upper_delta_vector[2].toFixed(3)}})
          </div>`;
        root.appendChild(card);
      }}
    }}

    function renderAnchorTable() {{
      const table = document.getElementById('anchorTable');
      const thead = document.createElement('thead');
      thead.innerHTML = `
        <tr>
          <th>位置</th>
          <th>x=289675 边界点</th>
          <th>最近曲面点</th>
          <th>距离</th>
          <th>x=289689 边界点</th>
          <th>最近曲面点</th>
          <th>距离</th>
        </tr>`;
      table.appendChild(thead);
      const tbody = document.createElement('tbody');
      for (const anchor of DATA.anchors) {{
        const lowerText = statusText(anchor.lower_dist);
        const upperText = statusText(anchor.upper_dist);
        const lowerColor = anchor.lower_dist > DATA.tolerance_mm ? 'var(--accent)' : 'var(--plane-low)';
        const upperColor = anchor.upper_dist > DATA.tolerance_mm ? 'var(--accent)' : 'var(--plane-low)';
        const row = document.createElement('tr');
        row.innerHTML = `
          <td><strong>${{anchor.label}}</strong><br><span class="subtle">s=${{anchor.s.toFixed(1)}}mm</span></td>
          <td><code>${{anchor.lower_point_label}}</code></td>
          <td><code>${{anchor.lower_closest_label}}</code></td>
          <td><code style="color:${{lowerColor}}">${{lowerText}}</code></td>
          <td><code>${{anchor.upper_point_label}}</code></td>
          <td><code>${{anchor.upper_closest_label}}</code></td>
          <td><code style="color:${{upperColor}}">${{upperText}}</code></td>`;
        tbody.appendChild(row);
      }}
      table.appendChild(tbody);
    }}

    renderMidpointProofView();
    renderGeometryView();
    renderYZView();
    renderAnchorCards();
    renderAnchorTable();

    document.getElementById('proofFocusBtn').addEventListener('click', () => midpointProofApi && midpointProofApi.focus());
    document.getElementById('proofZoomInBtn').addEventListener('click', () => midpointProofApi && midpointProofApi.zoomIn());
    document.getElementById('proofZoomOutBtn').addEventListener('click', () => midpointProofApi && midpointProofApi.zoomOut());
    document.getElementById('proofResetBtn').addEventListener('click', () => midpointProofApi && midpointProofApi.reset());
    document.getElementById('focusMidBtn').addEventListener('click', () => geometryViewApi && geometryViewApi.focusMid());
    document.getElementById('fitAllBtn').addEventListener('click', () => geometryViewApi && geometryViewApi.fitAll());
    document.getElementById('zoomInBtn').addEventListener('click', () => geometryViewApi && geometryViewApi.zoomIn());
    document.getElementById('zoomOutBtn').addEventListener('click', () => geometryViewApi && geometryViewApi.zoomOut());
    document.getElementById('resetPanBtn').addEventListener('click', () => geometryViewApi && geometryViewApi.reset());
  </script>
</body>
</html>
"""


def build_gap_inspector_html(payload):
    json_payload = json.dumps(payload, ensure_ascii=False)
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Gap Inspector</title>
  <style>
    :root {{
      --bg: #f2efe8;
      --panel: rgba(255,255,255,0.82);
      --line: rgba(20, 26, 33, 0.10);
      --ink: #16202a;
      --muted: #66727f;
      --plane-low: #0f766e;
      --plane-high: #c2410c;
      --surface: #1f2937;
      --accent: #b91c1c;
      --mono: "IBM Plex Mono", "Consolas", monospace;
      --sans: "Segoe UI", "PingFang SC", "Microsoft YaHei", sans-serif;
    }}
    * {{ box-sizing: border-box; }}
    html, body {{ margin: 0; height: 100%; }}
    body {{
      font-family: var(--sans);
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(15,118,110,0.10), transparent 24rem),
        radial-gradient(circle at top right, rgba(194,65,12,0.10), transparent 26rem),
        linear-gradient(180deg, #f7f3ec 0%, var(--bg) 100%);
    }}
    .app {{
      height: 100%;
      display: grid;
      grid-template-rows: auto 1fr;
      gap: 14px;
      padding: 16px;
    }}
    .topbar {{
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 14px;
      align-items: start;
    }}
    .titlebox, .hud {{
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 18px;
      padding: 14px 16px;
      backdrop-filter: blur(10px);
    }}
    .eyebrow {{
      font-family: var(--mono);
      font-size: 12px;
      color: var(--muted);
      letter-spacing: 0.06em;
      text-transform: uppercase;
      margin-bottom: 8px;
    }}
    h1 {{
      margin: 0 0 8px;
      font-size: 28px;
      line-height: 1.05;
      letter-spacing: -0.04em;
    }}
    .subtitle {{
      margin: 0;
      color: var(--muted);
      line-height: 1.6;
      font-size: 14px;
    }}
    .toolbar {{
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      margin-top: 12px;
    }}
    .toolbar button {{
      border: 1px solid rgba(22, 32, 42, 0.12);
      background: rgba(255,255,255,0.9);
      color: var(--ink);
      border-radius: 999px;
      padding: 8px 12px;
      font-family: var(--mono);
      font-size: 12px;
      cursor: pointer;
    }}
    .toolbar button.primary {{
      color: var(--accent);
      border-color: rgba(185,28,28,0.22);
      background: rgba(185,28,28,0.10);
      font-weight: 700;
    }}
    .hud {{
      min-width: 320px;
      display: grid;
      gap: 10px;
      align-content: start;
    }}
    .metric {{
      padding: 10px 12px;
      border-radius: 14px;
      background: rgba(255,255,255,0.76);
      border: 1px solid rgba(22,32,42,0.08);
    }}
    .metric label {{
      display: block;
      color: var(--muted);
      font-size: 11px;
      text-transform: uppercase;
      letter-spacing: 0.06em;
      margin-bottom: 6px;
    }}
    .metric strong {{
      display: block;
      font-family: var(--mono);
      font-size: 22px;
      line-height: 1.05;
      letter-spacing: -0.03em;
    }}
    .metric strong.over {{ color: var(--accent); }}
    .metric strong.under {{ color: var(--plane-low); }}
    .viewer {{
      min-height: 0;
      background: rgba(255,255,255,0.68);
      border: 1px solid var(--line);
      border-radius: 24px;
      overflow: hidden;
      box-shadow: 0 20px 40px rgba(20, 26, 33, 0.08);
      position: relative;
    }}
    svg {{
      width: 100%;
      height: 100%;
      display: block;
      touch-action: none;
    }}
    .legend {{
      position: absolute;
      left: 16px;
      bottom: 16px;
      z-index: 2;
      background: rgba(255,255,255,0.82);
      border: 1px solid rgba(22,32,42,0.08);
      border-radius: 16px;
      padding: 10px 12px;
      display: grid;
      gap: 8px;
      font-size: 12px;
      color: var(--muted);
      backdrop-filter: blur(10px);
    }}
    .legend-item {{
      display: inline-flex;
      align-items: center;
      gap: 8px;
    }}
    .swatch {{
      width: 14px;
      height: 14px;
      border-radius: 999px;
      display: inline-block;
    }}
    @media (max-width: 980px) {{
      .topbar {{
        grid-template-columns: 1fr;
      }}
      .hud {{
        min-width: 0;
      }}
    }}
  </style>
</head>
<body>
  <div class="app">
    <div class="topbar">
      <div class="titlebox">
        <div class="eyebrow">Direct Gap Viewer</div>
        <h1>两块板局部缝隙查看器</h1>
        <p class="subtitle">这个页面只做一件事：直接显示这两块板子的局部几何，并允许你高倍缩放查看缝隙。默认拖拽旋转，<code>Shift + 拖拽</code> 平移，滚轮缩放。SVG 是矢量的，缩放上限也放得很高。</p>
        <div class="toolbar">
          <button id="fitGapBtn" class="primary" type="button">聚焦缝隙</button>
          <button id="fitPlateBtn" type="button">查看整块局部</button>
          <button id="zoomInBtn" type="button">放大</button>
          <button id="zoomOutBtn" type="button">缩小</button>
          <button id="resetBtn" type="button">重置视角</button>
        </div>
      </div>
      <div class="hud">
        <div class="metric">
          <label>被质疑的最小距离</label>
          <strong class="over">{payload["midpoint"]["upper_dist_mm"]:.3f}mm &gt; {payload["tolerance_mm"]:.1f}mm</strong>
        </div>
        <div class="metric">
          <label>对照侧最小距离</label>
          <strong class="under">{payload["midpoint"]["lower_dist_mm"]:.3f}mm</strong>
        </div>
        <div class="metric">
          <label>当前对象</label>
          <strong style="font-size:14px">{payload["plane_name"]}<br>{payload["surface_name"]}</strong>
        </div>
      </div>
    </div>
    <div class="viewer">
      <svg id="viewer" viewBox="0 0 1600 980" aria-label="两块板局部缝隙查看器"></svg>
      <div class="legend">
        <span class="legend-item"><span class="swatch" style="background: rgba(31,41,55,0.78)"></span> 曲面板局部 mesh</span>
        <span class="legend-item"><span class="swatch" style="background: rgba(194,65,12,0.20)"></span> 平面板局部带状区域</span>
        <span class="legend-item"><span class="swatch" style="background: var(--accent)"></span> 中段缝隙连线</span>
        <span class="legend-item"><span class="swatch" style="background: var(--plane-low)"></span> 对照侧连线</span>
      </div>
    </div>
  </div>

  <script>
    const DATA = {json_payload};

    function createSvg(tag, attrs = {{}}) {{
      const node = document.createElementNS('http://www.w3.org/2000/svg', tag);
      for (const [key, value] of Object.entries(attrs)) {{
        node.setAttribute(key, value);
      }}
      return node;
    }}

    function computeBounds(points) {{
      const xs = points.map(p => p[0]);
      const ys = points.map(p => p[1]);
      return {{
        minX: Math.min(...xs),
        maxX: Math.max(...xs),
        minY: Math.min(...ys),
        maxY: Math.max(...ys)
      }};
    }}

    function fitProjector(points, width, height, padding) {{
      const bounds = computeBounds(points);
      const spanX = Math.max(1e-6, bounds.maxX - bounds.minX);
      const spanY = Math.max(1e-6, bounds.maxY - bounds.minY);
      const scale = Math.min((width - padding * 2) / spanX, (height - padding * 2) / spanY);
      return point => ([
        padding + (point[0] - bounds.minX) * scale,
        height - padding - (point[1] - bounds.minY) * scale
      ]);
    }}

    function toLocal(point) {{
      const c = DATA.local_center;
      return [
        (point[0] - c[0]) * DATA.x_exaggeration,
        point[1] - c[1],
        point[2] - c[2]
      ];
    }}

    function rotatePoint(point, yaw, pitch) {{
      const cosYaw = Math.cos(yaw);
      const sinYaw = Math.sin(yaw);
      const cosPitch = Math.cos(pitch);
      const sinPitch = Math.sin(pitch);

      const x1 = point[0] * cosYaw + point[1] * sinYaw;
      const y1 = -point[0] * sinYaw + point[1] * cosYaw;
      const z1 = point[2];

      const y2 = y1 * cosPitch - z1 * sinPitch;
      const z2 = y1 * sinPitch + z1 * cosPitch;
      return [x1, y2, z2];
    }}

    function polylinePathFrom2D(points) {{
      return points.map((point, index) => `${{index ? 'L' : 'M'}} ${{point[0].toFixed(2)}} ${{point[1].toFixed(2)}}`).join(' ');
    }}

    const svg = document.getElementById('viewer');
    const width = 1600;
    const height = 980;

    const SCENE = {{
      surfaceTriangles: DATA.focus_surface_triangles.map(tri => tri.map(toLocal)),
      lowerSegment: DATA.focus_lower_segment.map(toLocal),
      upperSegment: DATA.focus_upper_segment.map(toLocal),
      anchor: (() => {{
        const anchor = DATA.anchors.find(item => item.label === '局部中段') || DATA.anchors[0];
        return {{
          lowerPoint: toLocal(anchor.lower_point),
          upperPoint: toLocal(anchor.upper_point),
          lowerClosest: toLocal(anchor.lower_closest),
          upperClosest: toLocal(anchor.upper_closest)
        }};
      }})()
    }};

    const view = {{
      yaw: -1.02,
      pitch: 0.62,
      zoom: 1,
      panX: 0,
      panY: 0,
      fitMode: 'gap'
    }};

    function getRotatedScene() {{
      const rotate = point => rotatePoint(point, view.yaw, view.pitch);
      return {{
        surfaceTriangles: SCENE.surfaceTriangles.map(tri => tri.map(rotate)),
        lowerSegment: SCENE.lowerSegment.map(rotate),
        upperSegment: SCENE.upperSegment.map(rotate),
        anchor: {{
          lowerPoint: rotate(SCENE.anchor.lowerPoint),
          upperPoint: rotate(SCENE.anchor.upperPoint),
          lowerClosest: rotate(SCENE.anchor.lowerClosest),
          upperClosest: rotate(SCENE.anchor.upperClosest)
        }}
      }};
    }}

    function fitInfoFor(points, padding) {{
      const bounds = computeBounds(points.map(point => [point[0], point[1]]));
      const spanX = Math.max(1e-6, bounds.maxX - bounds.minX);
      const spanY = Math.max(1e-6, bounds.maxY - bounds.minY);
      const scale = Math.min((width - padding * 2) / spanX, (height - padding * 2) / spanY);
      const centerX = (bounds.minX + bounds.maxX) * 0.5;
      const centerY = (bounds.minY + bounds.maxY) * 0.5;
      return {{ bounds, scale, centerX, centerY }};
    }}

    function screenProject(point, fit) {{
      const cx = width * 0.5;
      const cy = height * 0.5;
      const baseX = cx + (point[0] - fit.centerX) * fit.scale;
      const baseY = cy - (point[1] - fit.centerY) * fit.scale;
      return [
        cx + (baseX - cx) * view.zoom + view.panX,
        cy + (baseY - cy) * view.zoom + view.panY
      ];
    }}

    function render() {{
      svg.innerHTML = '';
      const viewport = createSvg('g', {{ id: 'viewport' }});
      svg.appendChild(viewport);

      const rotated = getRotatedScene();
      const allPoints = [
        ...rotated.surfaceTriangles.flat(),
        ...rotated.lowerSegment,
        ...rotated.upperSegment,
        rotated.anchor.lowerPoint,
        rotated.anchor.upperPoint,
        rotated.anchor.lowerClosest,
        rotated.anchor.upperClosest
      ];
      const gapPoints = [
        rotated.anchor.lowerPoint,
        rotated.anchor.upperPoint,
        rotated.anchor.lowerClosest,
        rotated.anchor.upperClosest
      ];
      const fit = fitInfoFor(view.fitMode === 'gap' ? gapPoints : allPoints, view.fitMode === 'gap' ? 90 : 58);

      const sortedTriangles = rotated.surfaceTriangles
        .map(tri => ({{
          points: tri,
          depth: (tri[0][2] + tri[1][2] + tri[2][2]) / 3
        }}))
        .sort((a, b) => a.depth - b.depth);

      const lower2d = rotated.lowerSegment.map(point => screenProject(point, fit));
      const upper2d = rotated.upperSegment.map(point => screenProject(point, fit));
      const bandPoints = [...lower2d, ...[...upper2d].reverse()];
      const bandPath = bandPoints.map((point, index) => `${{index ? 'L' : 'M'}} ${{point[0].toFixed(2)}} ${{point[1].toFixed(2)}}`).join(' ') + ' Z';
      viewport.appendChild(createSvg('path', {{
        d: bandPath,
        fill: 'rgba(194,65,12,0.12)',
        stroke: 'rgba(194,65,12,0.16)',
        'stroke-width': '1.2'
      }}));

      for (const tri of sortedTriangles) {{
        const points = tri.points.map(point => screenProject(point, fit));
        const path = points.map((point, index) => `${{index ? 'L' : 'M'}} ${{point[0].toFixed(2)}} ${{point[1].toFixed(2)}}`).join(' ') + ' Z';
        viewport.appendChild(createSvg('path', {{
          d: path,
          fill: 'rgba(31,41,55,0.28)',
          stroke: 'rgba(17,24,39,0.52)',
          'stroke-width': '1.2'
        }}));
      }}

      viewport.appendChild(createSvg('path', {{
        d: polylinePathFrom2D(lower2d),
        fill: 'none',
        stroke: 'var(--plane-low)',
        'stroke-width': '5.5',
        'stroke-linecap': 'round',
        'stroke-linejoin': 'round'
      }}));

      viewport.appendChild(createSvg('path', {{
        d: polylinePathFrom2D(upper2d),
        fill: 'none',
        stroke: 'rgba(194,65,12,0.92)',
        'stroke-width': '5.5',
        'stroke-linecap': 'round',
        'stroke-linejoin': 'round'
      }}));

      const pLowerPoint = screenProject(rotated.anchor.lowerPoint, fit);
      const pUpperPoint = screenProject(rotated.anchor.upperPoint, fit);
      const pLowerClosest = screenProject(rotated.anchor.lowerClosest, fit);
      const pUpperClosest = screenProject(rotated.anchor.upperClosest, fit);

      viewport.appendChild(createSvg('line', {{
        x1: pLowerPoint[0], y1: pLowerPoint[1], x2: pLowerClosest[0], y2: pLowerClosest[1],
        stroke: 'var(--plane-low)', 'stroke-width': '3'
      }}));
      viewport.appendChild(createSvg('line', {{
        x1: pUpperPoint[0], y1: pUpperPoint[1], x2: pUpperClosest[0], y2: pUpperClosest[1],
        stroke: 'var(--accent)', 'stroke-width': '7'
      }}));

      for (const marker of [
        [pLowerPoint, 'var(--plane-low)', 6],
        [pUpperPoint, 'var(--accent)', 7],
        [pLowerClosest, 'var(--surface)', 6],
        [pUpperClosest, 'var(--surface)', 7]
      ]) {{
        viewport.appendChild(createSvg('circle', {{
          cx: marker[0][0], cy: marker[0][1], r: marker[2],
          fill: marker[1], stroke: '#fff', 'stroke-width': '2.2'
        }}));
      }}

      const labelOriginX = Math.min(pUpperPoint[0], pUpperClosest[0]) + 10;
      const labelOriginY = Math.min(pUpperPoint[1], pUpperClosest[1]) - 34;
      viewport.appendChild(createSvg('rect', {{
        x: labelOriginX - 12,
        y: labelOriginY - 30,
        width: 280,
        height: 56,
        rx: 16,
        fill: 'rgba(185,28,28,0.10)',
        stroke: 'rgba(185,28,28,0.24)'
      }}));

      const gapLabel = createSvg('text', {{
        x: labelOriginX,
        y: labelOriginY,
        'font-size': '26',
        fill: 'var(--accent)',
        'font-family': 'var(--mono)',
        'font-weight': '700'
      }});
      gapLabel.textContent = `${{DATA.midpoint.upper_dist_mm.toFixed(3)}}mm > ${{DATA.tolerance_mm.toFixed(1)}}mm`;
      viewport.appendChild(gapLabel);

      const tip = createSvg('text', {{
        x: 32,
        y: 34,
        'font-size': '12',
        fill: 'var(--muted)',
        'font-family': 'var(--mono)'
      }});
      tip.textContent = `Drag rotate | Shift+Drag pan | yaw=${{view.yaw.toFixed(2)}} pitch=${{view.pitch.toFixed(2)}} zoom=${{view.zoom.toFixed(2)}}x`;
      viewport.appendChild(tip);
    }}

    let dragState = null;

    svg.addEventListener('wheel', event => {{
      event.preventDefault();
      const factor = event.deltaY < 0 ? 1.18 : 1 / 1.18;
      view.zoom = Math.max(0.2, Math.min(2500, view.zoom * factor));
      render();
    }}, {{ passive: false }});

    svg.addEventListener('pointerdown', event => {{
      dragState = {{
        x: event.clientX,
        y: event.clientY,
        yaw: view.yaw,
        pitch: view.pitch,
        panX: view.panX,
        panY: view.panY,
        panMode: event.shiftKey
      }};
      svg.setPointerCapture(event.pointerId);
    }});

    svg.addEventListener('pointermove', event => {{
      if (!dragState) return;
      const dx = event.clientX - dragState.x;
      const dy = event.clientY - dragState.y;
      if (dragState.panMode) {{
        view.panX = dragState.panX + dx;
        view.panY = dragState.panY + dy;
      }} else {{
        view.yaw = dragState.yaw + dx * 0.008;
        view.pitch = Math.max(-1.45, Math.min(1.45, dragState.pitch - dy * 0.008));
      }}
      render();
    }});

    function endDrag(event) {{
      if (!dragState) return;
      dragState = null;
      if (event.pointerId !== undefined && svg.hasPointerCapture(event.pointerId)) {{
        svg.releasePointerCapture(event.pointerId);
      }}
    }}

    svg.addEventListener('pointerup', endDrag);
    svg.addEventListener('pointerleave', endDrag);
    svg.addEventListener('pointercancel', endDrag);

    document.getElementById('fitGapBtn').addEventListener('click', () => {{
      view.fitMode = 'gap';
      view.zoom = 1;
      view.panX = 0;
      view.panY = 0;
      render();
    }});

    document.getElementById('fitPlateBtn').addEventListener('click', () => {{
      view.fitMode = 'plate';
      view.zoom = 1;
      view.panX = 0;
      view.panY = 0;
      render();
    }});

    document.getElementById('zoomInBtn').addEventListener('click', () => {{
      view.zoom = Math.min(2500, view.zoom * 1.2);
      render();
    }});

    document.getElementById('zoomOutBtn').addEventListener('click', () => {{
      view.zoom = Math.max(0.2, view.zoom / 1.2);
      render();
    }});

    document.getElementById('resetBtn').addEventListener('click', () => {{
      view.yaw = -1.02;
      view.pitch = 0.62;
      view.zoom = 1;
      view.panX = 0;
      view.panY = 0;
      view.fitMode = 'gap';
      render();
    }});

    render();
  </script>
</body>
</html>
"""


def build_gap_section_html(payload):
    json_payload = json.dumps(payload, ensure_ascii=False)
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Gap Section Viewer</title>
  <style>
    :root {{
      --bg: #f4f0e7;
      --panel: rgba(255,255,255,0.84);
      --line: rgba(22, 32, 42, 0.10);
      --ink: #16202a;
      --muted: #64707d;
      --plane: #c2410c;
      --plane-fill: rgba(194,65,12,0.14);
      --surface: #1f2937;
      --surface-soft: rgba(31,41,55,0.22);
      --accent: #b91c1c;
      --ok: #0f766e;
      --mono: "IBM Plex Mono", "Consolas", monospace;
      --sans: "Segoe UI", "PingFang SC", "Microsoft YaHei", sans-serif;
    }}
    * {{ box-sizing: border-box; }}
    html, body {{ margin: 0; height: 100%; }}
    body {{
      font-family: var(--sans);
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(194,65,12,0.08), transparent 24rem),
        radial-gradient(circle at top right, rgba(15,118,110,0.08), transparent 24rem),
        linear-gradient(180deg, #f7f4ee 0%, var(--bg) 100%);
    }}
    .app {{
      height: 100%;
      display: grid;
      grid-template-rows: auto 1fr;
      gap: 14px;
      padding: 16px;
    }}
    .topbar {{
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 14px;
      align-items: start;
    }}
    .titlebox, .hud {{
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 18px;
      padding: 14px 16px;
      backdrop-filter: blur(10px);
    }}
    .eyebrow {{
      font-family: var(--mono);
      font-size: 12px;
      color: var(--muted);
      letter-spacing: 0.06em;
      text-transform: uppercase;
      margin-bottom: 8px;
    }}
    h1 {{
      margin: 0 0 8px;
      font-size: 28px;
      line-height: 1.05;
      letter-spacing: -0.04em;
    }}
    .subtitle {{
      margin: 0;
      color: var(--muted);
      line-height: 1.6;
      font-size: 14px;
    }}
    .toolbar {{
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      margin-top: 12px;
    }}
    .toolbar button {{
      border: 1px solid rgba(22, 32, 42, 0.12);
      background: rgba(255,255,255,0.9);
      color: var(--ink);
      border-radius: 999px;
      padding: 8px 12px;
      font-family: var(--mono);
      font-size: 12px;
      cursor: pointer;
    }}
    .toolbar button.primary {{
      color: var(--accent);
      border-color: rgba(185,28,28,0.22);
      background: rgba(185,28,28,0.10);
      font-weight: 700;
    }}
    .hud {{
      min-width: 340px;
      display: grid;
      gap: 10px;
      align-content: start;
    }}
    .metric {{
      padding: 10px 12px;
      border-radius: 14px;
      background: rgba(255,255,255,0.78);
      border: 1px solid rgba(22,32,42,0.08);
    }}
    .metric label {{
      display: block;
      color: var(--muted);
      font-size: 11px;
      text-transform: uppercase;
      letter-spacing: 0.06em;
      margin-bottom: 6px;
    }}
    .metric strong {{
      display: block;
      font-family: var(--mono);
      font-size: 22px;
      line-height: 1.05;
      letter-spacing: -0.03em;
    }}
    .metric strong.over {{ color: var(--accent); }}
    .metric strong.under {{ color: var(--ok); }}
    .viewer {{
      min-height: 0;
      background: rgba(255,255,255,0.72);
      border: 1px solid var(--line);
      border-radius: 24px;
      overflow: hidden;
      position: relative;
      box-shadow: 0 20px 40px rgba(20, 26, 33, 0.08);
    }}
    svg {{
      width: 100%;
      height: 100%;
      display: block;
      touch-action: none;
    }}
    .legend {{
      position: absolute;
      left: 16px;
      bottom: 16px;
      z-index: 2;
      background: rgba(255,255,255,0.84);
      border: 1px solid rgba(22,32,42,0.08);
      border-radius: 16px;
      padding: 10px 12px;
      display: grid;
      gap: 8px;
      font-size: 12px;
      color: var(--muted);
      backdrop-filter: blur(10px);
    }}
    .legend-item {{
      display: inline-flex;
      align-items: center;
      gap: 8px;
    }}
    .swatch {{
      width: 14px;
      height: 14px;
      border-radius: 999px;
      display: inline-block;
    }}
    @media (max-width: 980px) {{
      .topbar {{
        grid-template-columns: 1fr;
      }}
      .hud {{
        min-width: 0;
      }}
    }}
  </style>
</head>
<body>
  <div class="app">
    <div class="topbar">
      <div class="titlebox">
        <div class="eyebrow">Section Gap Viewer</div>
        <h1>中段剖切缝隙查看器</h1>
        <p class="subtitle">这个页面不再做普通 3D 投影，而是沿中段做局部剖切。这样能直接看到平面板截面、曲面板截线，以及最小距离线是否落在板外。滚轮缩放，拖拽平移。</p>
        <div class="toolbar">
          <button id="fitGapBtn" class="primary" type="button">聚焦缝隙</button>
          <button id="fitAllBtn" type="button">查看整个剖面</button>
          <button id="zoomInBtn" type="button">放大</button>
          <button id="zoomOutBtn" type="button">缩小</button>
          <button id="resetBtn" type="button">重置</button>
        </div>
      </div>
      <div class="hud">
        <div class="metric">
          <label>上侧最小距离</label>
          <strong class="over">{payload["midpoint"]["upper_dist_mm"]:.3f}mm &gt; {payload["tolerance_mm"]:.1f}mm</strong>
        </div>
        <div class="metric">
          <label>下侧对照</label>
          <strong class="under">{payload["midpoint"]["lower_dist_mm"]:.3f}mm</strong>
        </div>
        <div class="metric">
          <label>查看对象</label>
          <strong style="font-size:14px">{payload["plane_name"]}<br>{payload["surface_name"]}</strong>
        </div>
      </div>
    </div>
    <div class="viewer">
      <svg id="viewer" viewBox="0 0 1500 920" aria-label="中段剖切缝隙查看器"></svg>
      <div class="legend">
        <span class="legend-item"><span class="swatch" style="background: var(--plane-fill)"></span> 平面板剖切实体</span>
        <span class="legend-item"><span class="swatch" style="background: rgba(31,41,55,0.70)"></span> 曲面板剖切线</span>
        <span class="legend-item"><span class="swatch" style="background: var(--accent)"></span> 上侧最小距离</span>
        <span class="legend-item"><span class="swatch" style="background: var(--ok)"></span> 下侧对照距离</span>
      </div>
    </div>
  </div>

  <script>
    const DATA = {json_payload};

    function createSvg(tag, attrs = {{}}) {{
      const node = document.createElementNS('http://www.w3.org/2000/svg', tag);
      for (const [key, value] of Object.entries(attrs)) {{
        node.setAttribute(key, value);
      }}
      return node;
    }}

    function add(a, b) {{ return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }}
    function sub(a, b) {{ return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }}
    function mul(a, s) {{ return [a[0] * s, a[1] * s, a[2] * s]; }}
    function dot(a, b) {{ return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }}
    function cross(a, b) {{
      return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
      ];
    }}
    function norm(a) {{ return Math.hypot(a[0], a[1], a[2]); }}
    function unit(a) {{
      const n = norm(a);
      return n <= 1e-9 ? [0, 0, 0] : [a[0] / n, a[1] / n, a[2] / n];
    }}

    function computeBounds(points) {{
      const xs = points.map(p => p[0]);
      const ys = points.map(p => p[1]);
      return {{
        minX: Math.min(...xs),
        maxX: Math.max(...xs),
        minY: Math.min(...ys),
        maxY: Math.max(...ys)
      }};
    }}

    function fitInfo(bounds, width, height, padding, zoom, panX, panY) {{
      const spanX = Math.max(1e-6, bounds.maxX - bounds.minX);
      const spanY = Math.max(1e-6, bounds.maxY - bounds.minY);
      const base = Math.min((width - padding * 2) / spanX, (height - padding * 2) / spanY);
      const centerX = (bounds.minX + bounds.maxX) * 0.5;
      const centerY = (bounds.minY + bounds.maxY) * 0.5;
      return {{ base, centerX, centerY, zoom, panX, panY }};
    }}

    function project2D(point, fit, width, height) {{
      const cx = width * 0.5;
      const cy = height * 0.5;
      return [
        cx + (point[0] - fit.centerX) * fit.base * fit.zoom + fit.panX,
        cy - (point[1] - fit.centerY) * fit.base * fit.zoom + fit.panY
      ];
    }}

    function uniquePoints(points, eps = 1e-6) {{
      const out = [];
      for (const point of points) {{
        let exists = false;
        for (const item of out) {{
          if (Math.hypot(point[0] - item[0], point[1] - item[1], point[2] - item[2]) <= eps) {{
            exists = true;
            break;
          }}
        }}
        if (!exists) out.push(point);
      }}
      return out;
    }}

    function intersectTriangleWithPlane(triangle, origin, normal) {{
      const eps = 1e-7;
      const distances = triangle.map(point => dot(sub(point, origin), normal));
      const hits = [];
      const edges = [[0,1], [1,2], [2,0]];
      for (const [i, j] of edges) {{
        const a = triangle[i];
        const b = triangle[j];
        const da = distances[i];
        const db = distances[j];
        if (Math.abs(da) <= eps) hits.push(a);
        if (Math.abs(da) > eps && Math.abs(db) > eps && da * db < 0) {{
          const t = da / (da - db);
          hits.push(add(a, mul(sub(b, a), t)));
        }}
      }}
      const unique = uniquePoints(hits);
      return unique.length >= 2 ? [unique[0], unique[1]] : null;
    }}

    const upperSegment = DATA.focus_upper_segment;
    const midIndex = Math.floor(upperSegment.length / 2);
    const anchor = DATA.anchors.find(item => item.label === '局部中段') || DATA.anchors[0];
    const upperPoint = anchor.upper_point;
    const lowerPoint = anchor.lower_point;
    const upperClosest = anchor.upper_closest;
    const lowerClosest = anchor.lower_closest;

    const tangent = unit(sub(
      upperSegment[Math.min(upperSegment.length - 1, midIndex + 1)],
      upperSegment[Math.max(0, midIndex - 1)]
    ));
    const ex = unit(sub(lowerPoint, upperPoint));
    let ev = sub(upperClosest, upperPoint);
    ev = sub(ev, mul(tangent, dot(ev, tangent)));
    ev = sub(ev, mul(ex, dot(ev, ex)));
    if (norm(ev) <= 1e-8) {{
      ev = cross(tangent, ex);
    }}
    ev = unit(ev);
    if (dot(ev, sub(upperClosest, upperPoint)) < 0) {{
      ev = mul(ev, -1);
    }}

    function toSection(point) {{
      const d = sub(point, upperPoint);
      return [dot(d, ex), dot(d, ev)];
    }}

    const thickness = Math.hypot(
      lowerPoint[0] - upperPoint[0],
      lowerPoint[1] - upperPoint[1],
      lowerPoint[2] - upperPoint[2]
    );

    const surfaceSegments = [];
    for (const tri of DATA.focus_surface_triangles) {{
      const seg = intersectTriangleWithPlane(tri, upperPoint, tangent);
      if (seg) {{
        surfaceSegments.push(seg.map(toSection));
      }}
    }}

    const upper2 = toSection(upperPoint);
    const lower2 = toSection(lowerPoint);
    const upperClosest2 = toSection(upperClosest);
    const lowerClosest2 = toSection(lowerClosest);
    const plateDepth = Math.max(28, Math.min(90, Math.abs(upperClosest2[1]) * 2.2 + 12));
    const platePolygon = [
      upper2,
      lower2,
      [lower2[0], -plateDepth],
      [upper2[0], -plateDepth]
    ];

    const allSectionPoints = [...surfaceSegments.flat(), ...platePolygon, upperClosest2, lowerClosest2];
    const boundsAll = computeBounds(allSectionPoints);
    const boundsGap = computeBounds([upper2, lower2, upperClosest2, lowerClosest2]);
    const width = 1500;
    const height = 920;

    const view = {{
      mode: 'gap',
      zoom: 1,
      panX: 0,
      panY: 0
    }};

    function currentBounds() {{
      return view.mode === 'gap' ? {{
        minX: boundsGap.minX - 6,
        maxX: boundsGap.maxX + 6,
        minY: Math.min(boundsGap.minY, -plateDepth) - 6,
        maxY: Math.max(boundsGap.maxY, 8) + 6
      }} : {{
        minX: boundsAll.minX - 6,
        maxX: boundsAll.maxX + 6,
        minY: boundsAll.minY - 6,
        maxY: boundsAll.maxY + 6
      }};
    }}

    function render() {{
      const svg = document.getElementById('viewer');
      svg.innerHTML = '';
      const viewport = createSvg('g');
      svg.appendChild(viewport);

      const fit = fitInfo(currentBounds(), width, height, view.mode === 'gap' ? 70 : 46, view.zoom, view.panX, view.panY);
      const project = point => project2D(point, fit, width, height);

      const platePath = platePolygon.map((point, index) => {{
        const p = project(point);
        return `${{index ? 'L' : 'M'}} ${{p[0].toFixed(2)}} ${{p[1].toFixed(2)}}`;
      }}).join(' ') + ' Z';
      viewport.appendChild(createSvg('path', {{
        d: platePath,
        fill: 'var(--plane-fill)',
        stroke: 'rgba(194,65,12,0.22)',
        'stroke-width': '1.4'
      }}));

      viewport.appendChild(createSvg('line', {{
        x1: project(upper2)[0], y1: project(upper2)[1],
        x2: project(lower2)[0], y2: project(lower2)[1],
        stroke: 'var(--plane)',
        'stroke-width': '4.6'
      }}));

      for (const seg of surfaceSegments) {{
        const a = project(seg[0]);
        const b = project(seg[1]);
        viewport.appendChild(createSvg('line', {{
          x1: a[0], y1: a[1], x2: b[0], y2: b[1],
          stroke: 'rgba(31,41,55,0.78)',
          'stroke-width': '3.6',
          'stroke-linecap': 'round'
        }}));
      }}

      const pu = project(upper2);
      const pl = project(lower2);
      const puc = project(upperClosest2);
      const plc = project(lowerClosest2);

      viewport.appendChild(createSvg('line', {{
        x1: pu[0], y1: pu[1], x2: puc[0], y2: puc[1],
        stroke: 'var(--accent)', 'stroke-width': '6.4'
      }}));
      viewport.appendChild(createSvg('line', {{
        x1: pl[0], y1: pl[1], x2: plc[0], y2: plc[1],
        stroke: 'var(--ok)', 'stroke-width': '3.2'
      }}));

      for (const marker of [
        [pu, 'var(--accent)', 6.6],
        [puc, 'var(--surface)', 6.6],
        [pl, 'var(--ok)', 5.6],
        [plc, 'var(--surface)', 5.6]
      ]) {{
        viewport.appendChild(createSvg('circle', {{
          cx: marker[0][0], cy: marker[0][1], r: marker[2],
          fill: marker[1], stroke: '#fff', 'stroke-width': '2'
        }}));
      }}

      const labelX = Math.min(pu[0], puc[0]) + 12;
      const labelY = Math.min(pu[1], puc[1]) - 18;
      viewport.appendChild(createSvg('rect', {{
        x: labelX - 12,
        y: labelY - 30,
        width: 272,
        height: 56,
        rx: 16,
        fill: 'rgba(185,28,28,0.10)',
        stroke: 'rgba(185,28,28,0.24)'
      }}));
      const gapLabel = createSvg('text', {{
        x: labelX,
        y: labelY,
        'font-size': '26',
        fill: 'var(--accent)',
        'font-family': 'var(--mono)',
        'font-weight': '700'
      }});
      gapLabel.textContent = `${{DATA.midpoint.upper_dist_mm.toFixed(3)}}mm > ${{DATA.tolerance_mm.toFixed(1)}}mm`;
      viewport.appendChild(gapLabel);

      const help = createSvg('text', {{
        x: 32,
        y: 34,
        'font-size': '12',
        fill: 'var(--muted)',
        'font-family': 'var(--mono)'
      }});
      help.textContent = `Section plane normal = midpoint tangent | zoom=${{view.zoom.toFixed(2)}}x`;
      viewport.appendChild(help);
    }}

    let dragState = null;
    const svg = document.getElementById('viewer');
    svg.addEventListener('wheel', event => {{
      event.preventDefault();
      const factor = event.deltaY < 0 ? 1.18 : 1 / 1.18;
      view.zoom = Math.max(0.2, Math.min(2000, view.zoom * factor));
      render();
    }}, {{ passive: false }});

    svg.addEventListener('pointerdown', event => {{
      dragState = {{ x: event.clientX, y: event.clientY, panX: view.panX, panY: view.panY }};
      svg.setPointerCapture(event.pointerId);
    }});

    svg.addEventListener('pointermove', event => {{
      if (!dragState) return;
      view.panX = dragState.panX + (event.clientX - dragState.x);
      view.panY = dragState.panY + (event.clientY - dragState.y);
      render();
    }});

    function endDrag(event) {{
      if (!dragState) return;
      dragState = null;
      if (event.pointerId !== undefined && svg.hasPointerCapture(event.pointerId)) {{
        svg.releasePointerCapture(event.pointerId);
      }}
    }}

    svg.addEventListener('pointerup', endDrag);
    svg.addEventListener('pointerleave', endDrag);
    svg.addEventListener('pointercancel', endDrag);

    function resetView(mode) {{
      view.mode = mode;
      view.zoom = 1;
      view.panX = 0;
      view.panY = 0;
      render();
    }}

    document.getElementById('fitGapBtn').addEventListener('click', () => resetView('gap'));
    document.getElementById('fitAllBtn').addEventListener('click', () => resetView('all'));
    document.getElementById('zoomInBtn').addEventListener('click', () => {{
      view.zoom = Math.min(2000, view.zoom * 1.2);
      render();
    }});
    document.getElementById('zoomOutBtn').addEventListener('click', () => {{
      view.zoom = Math.max(0.2, view.zoom / 1.2);
      render();
    }});
    document.getElementById('resetBtn').addEventListener('click', () => resetView('gap'));

    render();
  </script>
</body>
</html>
"""


def main():
    data = json.loads(RESULT_JSON.read_text(encoding="utf-8"))
    combo = next(
        item
        for item in data["combinations"]
        if item["plane_panel"]["name"] == TARGET_PLANE
        and item["surface_panel"]["name"] == TARGET_SURFACE
    )

    triangles = [
        Triangle(combo["surface_vertices"][i], combo["surface_vertices"][j], combo["surface_vertices"][k])
        for i, j, k in combo["surface_triangles"]
    ]

    lower_loop = combo["plane_boundary_loops"][4]
    upper_loop = combo["plane_boundary_loops"][0]
    start_segment, end_segment = find_focus_segment_range(lower_loop, triangles, TOLERANCE_MM)
    chart_samples, lower_samples, upper_samples = build_chart_samples(
        lower_loop, upper_loop, triangles, start_segment, end_segment
    )

    lower_strip = collect_strip_points(lower_loop, start_segment, end_segment)
    upper_strip = collect_strip_points(upper_loop, start_segment, end_segment)

    midpoint_index = len(chart_samples) // 2
    midpoint_lower = lower_samples[midpoint_index]
    midpoint_upper = upper_samples[midpoint_index]
    focus_lower_segment = collect_sample_window(lower_samples, midpoint_index, 12)
    focus_upper_segment = collect_sample_window(upper_samples, midpoint_index, 12)

    over_indices = [index for index, sample in enumerate(upper_samples) if sample["distance"] > TOLERANCE_MM]
    if not over_indices:
        raise RuntimeError("No >1mm interval found on the inspected boundary.")
    over_start = over_indices[0]
    over_end = over_indices[-1]
    anchor_indices = pick_spread_indices(len(over_indices), 3)
    anchor_indices = [over_indices[index] for index in anchor_indices]
    anchor_labels = ["局部起点", "局部中段", "局部终点"]
    anchors = []
    for label, index in zip(anchor_labels, anchor_indices):
        lower_anchor = lower_samples[index]
        upper_anchor = upper_samples[index]
        anchor = build_anchor(label, lower_anchor, upper_anchor)
        anchor["lower_point_label"] = point_label(anchor["lower_point"])
        anchor["upper_point_label"] = point_label(anchor["upper_point"])
        anchor["lower_closest_label"] = point_label(anchor["lower_closest"])
        anchor["upper_closest_label"] = point_label(anchor["upper_closest"])
        anchors.append(anchor)

    surface_curve_stride = max(1, len(lower_samples) // 16)
    surface_curve = [sample["closest"] for sample in lower_samples[::surface_curve_stride]]
    if surface_curve[-1] != lower_samples[-1]["closest"]:
        surface_curve.append(lower_samples[-1]["closest"])

    guide_points = []
    guide_points.extend(lower_strip)
    guide_points.extend(upper_strip)
    guide_points.extend(surface_curve)
    for anchor in anchors:
        guide_points.extend(
            [anchor["lower_point"], anchor["upper_point"], anchor["lower_closest"], anchor["upper_closest"]]
        )
    surface_patch_triangles = collect_local_surface_triangles(triangles, guide_points)
    focus_surface_triangles = collect_focus_surface_triangles(triangles, midpoint_upper["closest"])

    local_center = [
        (
            midpoint_lower["point"][axis]
            + midpoint_upper["point"][axis]
            + midpoint_lower["closest"][axis]
            + midpoint_upper["closest"][axis]
        )
        / 4.0
        for axis in range(3)
    ]

    payload = {
        "plane_name": TARGET_PLANE,
        "surface_name": TARGET_SURFACE,
        "tolerance_mm": TOLERANCE_MM,
        "x_exaggeration": X_EXAGGERATION,
        "local_center": local_center,
        "lower_strip": lower_strip,
        "upper_strip": upper_strip,
        "focus_lower_segment": focus_lower_segment,
        "focus_upper_segment": focus_upper_segment,
        "upper_over_strip": [sample["point"] for sample in upper_samples[over_start:over_end + 1]],
        "surface_curve": surface_curve,
        "surface_patch_triangles": surface_patch_triangles,
        "focus_surface_triangles": focus_surface_triangles,
        "anchors": anchors,
        "midpoint": {
            "lower_dist_mm": midpoint_lower["distance"],
            "upper_dist_mm": midpoint_upper["distance"],
            "gap_delta_mm": midpoint_upper["distance"] - midpoint_lower["distance"],
            "predicted_gap_mm": 14.0 * abs(midpoint_lower["normal"][0]),
            "normal_x_abs": abs(midpoint_lower["normal"][0]),
        },
    }

    OUTPUT_HTML.write_text(build_html(payload), encoding="utf-8")
    OUTPUT_INSPECTOR_HTML.write_text(build_gap_inspector_html(payload), encoding="utf-8")
    OUTPUT_SECTION_HTML.write_text(build_gap_section_html(payload), encoding="utf-8")
    print(f"Wrote local gap demo to: {OUTPUT_HTML}")
    print(f"Wrote gap inspector to: {OUTPUT_INSPECTOR_HTML}")
    print(f"Wrote gap section viewer to: {OUTPUT_SECTION_HTML}")


if __name__ == "__main__":
    main()
