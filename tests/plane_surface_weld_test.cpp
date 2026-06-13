// =============================================================================
// plane_surface_weld_test.cpp — 平面板与曲面板焊缝计算测试
// =============================================================================
//
// 测试覆盖：
//   - 合成数据：曲面板边界靠近平面面的焊缝检测
//   - 合成数据：平面板边界在曲面板内部的焊缝检测
//   - 真实模型：平面板 × 曲面板全组合焊缝计算，仅导出存在焊缝的组合
//
// 真实模型宏控制：
//   HANFENG_PLANE_SURFACE_REAL_MODEL_TESTS=1 时，只调用平面板/曲面板全量真实模型测试
//   HANFENG_SURFACE_SURFACE_REAL_MODEL_TESTS=1 时，只调用曲面板/曲面板全量真实模型测试
// =============================================================================

#include <cassert>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "hanfeng/hanfeng.hpp"
#include "hanfeng/plane_surface_hanfeng_util.hpp"
#include "test_common.hpp"

namespace {

	// =========================================================================
	// 合成测试辅助函数
	// =========================================================================

	hanfeng::SurfacePatch make_rect_patch(const hanfeng::SPAposition& a,
		const hanfeng::SPAposition& b,
		const hanfeng::SPAposition& c,
		const hanfeng::SPAposition& d,
		bool with_boundary = true) {
		hanfeng::SurfacePatch patch;
		patch.vertices = { a, b, c, d };
		patch.triangles = {
			hanfeng::Triangle{0U, 1U, 2U},
			hanfeng::Triangle{0U, 2U, 3U},
		};
		if (with_boundary) {
			patch.boundaries.outer_loops.push_back({ a, b, c, d });
		}
		return patch;
	}

	hanfeng::SurfacePanel make_box_surface_panel(double min_x,
		double max_x,
		double min_y,
		double max_y,
		double min_z,
		double max_z,
		bool with_boundaries = true) {
		hanfeng::SurfacePanel panel;
		panel.surfaces.push_back(make_rect_patch(
			hanfeng::SPAposition(min_x, min_y, min_z),
			hanfeng::SPAposition(min_x, min_y, max_z),
			hanfeng::SPAposition(min_x, max_y, max_z),
			hanfeng::SPAposition(min_x, max_y, min_z),
			with_boundaries));
		panel.surfaces.push_back(make_rect_patch(
			hanfeng::SPAposition(max_x, min_y, min_z),
			hanfeng::SPAposition(max_x, max_y, min_z),
			hanfeng::SPAposition(max_x, max_y, max_z),
			hanfeng::SPAposition(max_x, min_y, max_z),
			with_boundaries));
		panel.surfaces.push_back(make_rect_patch(
			hanfeng::SPAposition(min_x, min_y, min_z),
			hanfeng::SPAposition(max_x, min_y, min_z),
			hanfeng::SPAposition(max_x, min_y, max_z),
			hanfeng::SPAposition(min_x, min_y, max_z),
			with_boundaries));
		panel.surfaces.push_back(make_rect_patch(
			hanfeng::SPAposition(min_x, max_y, min_z),
			hanfeng::SPAposition(min_x, max_y, max_z),
			hanfeng::SPAposition(max_x, max_y, max_z),
			hanfeng::SPAposition(max_x, max_y, min_z),
			with_boundaries));
		panel.surfaces.push_back(make_rect_patch(
			hanfeng::SPAposition(min_x, min_y, min_z),
			hanfeng::SPAposition(min_x, max_y, min_z),
			hanfeng::SPAposition(max_x, max_y, min_z),
			hanfeng::SPAposition(max_x, min_y, min_z),
			with_boundaries));
		panel.surfaces.push_back(make_rect_patch(
			hanfeng::SPAposition(min_x, min_y, max_z),
			hanfeng::SPAposition(max_x, min_y, max_z),
			hanfeng::SPAposition(max_x, max_y, max_z),
			hanfeng::SPAposition(min_x, max_y, max_z),
			with_boundaries));
		return panel;
	}

	hanfeng::SurfacePanel make_surface_panel_with_boundary_on_plane_face() {
		hanfeng::SurfacePatch patch;
		patch.vertices = {
			hanfeng::SPAposition(2.0, 2.0, 0.0),
			hanfeng::SPAposition(8.0, 2.0, 0.0),
			hanfeng::SPAposition(8.0, 2.0, 5.0),
			hanfeng::SPAposition(2.0, 2.0, 5.0),
		};
		patch.triangles = {
			hanfeng::Triangle{0U, 1U, 2U},
			hanfeng::Triangle{0U, 2U, 3U},
		};
		patch.boundaries.outer_loops.push_back(
			{ patch.vertices[0], patch.vertices[1], patch.vertices[2], patch.vertices[3] });

		hanfeng::SurfacePanel panel;
		panel.surfaces.push_back(std::move(patch));
		return panel;
	}

	hanfeng::PlanePanel make_plane_panel_for_face_hit() {
		hanfeng::PlanePanel panel;
		panel.face_a.boundaries.outer_loops.push_back(
			{ hanfeng::SPAposition(0.0, 0.0, 0.0),
			 hanfeng::SPAposition(10.0, 0.0, 0.0),
			 hanfeng::SPAposition(10.0, 10.0, 0.0),
			 hanfeng::SPAposition(0.0, 10.0, 0.0) });
		panel.face_b.boundaries.outer_loops.push_back(
			{ hanfeng::SPAposition(0.0, 0.0, 1.0),
			 hanfeng::SPAposition(10.0, 0.0, 1.0),
			 hanfeng::SPAposition(10.0, 10.0, 1.0),
			 hanfeng::SPAposition(0.0, 10.0, 1.0) });
		return panel;
	}

	hanfeng::PlanePanel make_yz_plane_panel(double face_a_x,
		double face_b_x,
		double min_y,
		double max_y,
		double min_z,
		double max_z) {
		hanfeng::PlanePanel panel;
		panel.face_a.boundaries.outer_loops.push_back(
			{ hanfeng::SPAposition(face_a_x, min_y, min_z),
			 hanfeng::SPAposition(face_a_x, max_y, min_z),
			 hanfeng::SPAposition(face_a_x, max_y, max_z),
			 hanfeng::SPAposition(face_a_x, min_y, max_z) });
		panel.face_b.boundaries.outer_loops.push_back(
			{ hanfeng::SPAposition(face_b_x, min_y, min_z),
			 hanfeng::SPAposition(face_b_x, max_y, min_z),
			 hanfeng::SPAposition(face_b_x, max_y, max_z),
			 hanfeng::SPAposition(face_b_x, min_y, max_z) });
		return panel;
	}

	hanfeng::SurfacePanel make_surface_box_panel() {
		return make_box_surface_panel(0.0, 1.0, 0.0, 10.0, 0.0, 10.0);
	}

	hanfeng::SurfacePanel make_surface_box_panel_without_boundaries() {
		return make_box_surface_panel(0.0, 1.0, 0.0, 10.0, 0.0, 10.0, false);
	}

	hanfeng::PlanePanel make_plane_panel_coplanar_with_surface_face() {
		return make_yz_plane_panel(0.0, -1.0, 2.0, 8.0, 2.0, 3.0);
	}

	hanfeng::PlanePanel make_plane_panel_with_boundary_inside_surface_solid() {
		return make_yz_plane_panel(0.5, 1.5, 2.0, 8.0, 2.0, 3.0);
	}

	hanfeng::SurfacePanel make_surface_panel_with_exact_band_hit() {
		return make_box_surface_panel(0.0, 0.2, -0.1, 0.1, 0.0, 1.0);
	}

	hanfeng::PlanePanel make_plane_panel_with_short_exact_hit_band() {
		return make_yz_plane_panel(-1.05, -0.05, -0.2, 0.2, 0.5, 0.7);
	}

	hanfeng::PlanePanel make_plane_panel_near_surface_exterior_side() {
		return make_yz_plane_panel(-2.0, -1.0, 2.0, 8.0, 2.0, 3.0);
	}

	bool polyline_matches_segment(const hanfeng::WeldPolyline& polyline,
		const hanfeng::SPAposition& start,
		const hanfeng::SPAposition& end,
		double epsilon = 1.0e-6) {
		for (std::size_t index = 1; index < polyline.points.size(); ++index) {
			const hanfeng::SPAposition& first = polyline.points[index - 1U];
			const hanfeng::SPAposition& last = polyline.points[index];
			const bool same_direction =
				nearly_equal(first.x(), start.x(), epsilon) &&
				nearly_equal(first.y(), start.y(), epsilon) &&
				nearly_equal(first.z(), start.z(), epsilon) &&
				nearly_equal(last.x(), end.x(), epsilon) &&
				nearly_equal(last.y(), end.y(), epsilon) &&
				nearly_equal(last.z(), end.z(), epsilon);
			const bool reverse_direction =
				nearly_equal(first.x(), end.x(), epsilon) &&
				nearly_equal(first.y(), end.y(), epsilon) &&
				nearly_equal(first.z(), end.z(), epsilon) &&
				nearly_equal(last.x(), start.x(), epsilon) &&
				nearly_equal(last.y(), start.y(), epsilon) &&
				nearly_equal(last.z(), start.z(), epsilon);
			if (same_direction || reverse_direction) {
				return true;
			}
		}
		return false;
	}

	bool polyline_matches_rectangle_on_x_face(const hanfeng::WeldPolyline& polyline,
		double x,
		double min_y,
		double max_y,
		double min_z,
		double max_z,
		double epsilon = 1.0e-6) {
		if (polyline.points.size() < 4U) {
			return false;
		}

		double actual_min_y = std::numeric_limits<double>::infinity();
		double actual_max_y = -std::numeric_limits<double>::infinity();
		double actual_min_z = std::numeric_limits<double>::infinity();
		double actual_max_z = -std::numeric_limits<double>::infinity();
		for (const hanfeng::SPAposition& point : polyline.points) {
			if (!nearly_equal(point.x(), x, epsilon)) {
				return false;
			}
			actual_min_y = std::min(actual_min_y, point.y());
			actual_max_y = std::max(actual_max_y, point.y());
			actual_min_z = std::min(actual_min_z, point.z());
			actual_max_z = std::max(actual_max_z, point.z());
		}
		return nearly_equal(actual_min_y, min_y, epsilon) &&
			nearly_equal(actual_max_y, max_y, epsilon) &&
			nearly_equal(actual_min_z, min_z, epsilon) &&
			nearly_equal(actual_max_z, max_z, epsilon);
	}

	// =========================================================================
	// JSON 写入辅助
	// =========================================================================

	void write_json_array(std::ofstream& out, const hanfeng::SPAposition& p) {
		out << "[" << p.x() << ", " << p.y() << ", " << p.z() << "]";
	}

	void write_json_triangle(std::ofstream& out, const hanfeng::Triangle& tri) {
		out << "[" << tri[0] << ", " << tri[1] << ", " << tri[2] << "]";
	}

	void write_json_polyline3(std::ofstream& out, const hanfeng::Polyline3& pts, const std::string& indent) {
		out << "[\n";
		for (std::size_t i = 0; i < pts.size(); ++i) {
			out << indent << "  ";
			write_json_array(out, pts[i]);
			if (i + 1 < pts.size()) out << ",";
			out << "\n";
		}
		out << indent << "]";
	}

	void write_json_boundary_loops(std::ofstream& out,
		const std::vector<hanfeng::Polyline3>& loops,
		const std::string& indent) {
		out << "[\n";
		for (std::size_t li = 0; li < loops.size(); ++li) {
			out << indent << "  ";
			write_json_polyline3(out, loops[li], indent + "  ");
			if (li + 1 < loops.size()) out << ",";
			out << "\n";
		}
		out << indent << "]";
	}

	void write_plane_panel_geometry(std::ofstream& out, const hanfeng::PlanePanel& panel,
		const std::string& indent) {
		// 边界环（face_a 外环 + 内环 + face_b 外环 + 内环）
		out << indent << "\"boundary_loops\": [\n";
		bool first = true;
		auto write_loops = [&](const std::vector<hanfeng::Polyline3>& loops) {
			for (const auto& loop : loops) {
				if (!first) out << ",\n";
				first = false;
				out << indent << "  ";
				write_json_polyline3(out, loop, indent + "  ");
			}
			};
		write_loops(panel.face_a.boundaries.outer_loops);
		write_loops(panel.face_a.boundaries.inner_loops);
		write_loops(panel.face_b.boundaries.outer_loops);
		write_loops(panel.face_b.boundaries.inner_loops);
		out << "\n" << indent << "]";
	}

	void write_surface_panel_geometry(std::ofstream& out, const hanfeng::SurfacePanel& panel,
		const std::string& indent) {
		// 收集所有 patch 的顶点和三角形
		std::vector<hanfeng::SPAposition> all_vertices;
		std::vector<hanfeng::Triangle> all_triangles;
		std::vector<hanfeng::Polyline3> all_boundary_loops;

		for (const auto& patch : panel.surfaces) {
			std::size_t offset = all_vertices.size();
			for (const auto& v : patch.vertices) {
				all_vertices.push_back(v);
			}
			for (const auto& tri : patch.triangles) {
				all_triangles.push_back({ offset + tri[0], offset + tri[1], offset + tri[2] });
			}
			for (const auto& loop : patch.boundaries.outer_loops) {
				all_boundary_loops.push_back(loop);
			}
			for (const auto& loop : patch.boundaries.inner_loops) {
				all_boundary_loops.push_back(loop);
			}
		}

		// 顶点
		out << indent << "\"vertices\": [\n";
		for (std::size_t i = 0; i < all_vertices.size(); ++i) {
			out << indent << "  ";
			write_json_array(out, all_vertices[i]);
			if (i + 1 < all_vertices.size()) out << ",";
			out << "\n";
		}
		out << indent << "],\n";

		// 三角形
		out << indent << "\"triangles\": [\n";
		for (std::size_t i = 0; i < all_triangles.size(); ++i) {
			out << indent << "  ";
			write_json_triangle(out, all_triangles[i]);
			if (i + 1 < all_triangles.size()) out << ",";
			out << "\n";
		}
		out << indent << "],\n";

		// 边界环
		out << indent << "\"boundary_loops\": ";
		write_json_boundary_loops(out, all_boundary_loops, indent);
	}

	// =========================================================================
	// metadata.json 的最小化解析
	// =========================================================================

	struct PanelEntry {
		std::string name;
		std::string category;
		std::string directory;
	};

	struct LoadedPlaneEntry {
		PanelEntry entry;
		hanfeng::PlanePanel panel;
		hanfeng::SurfacePanel mesh;
		double load_panel_ms = 0.0;
		double load_mesh_ms = 0.0;
	};

	struct LoadedSurfaceEntry {
		PanelEntry entry;
		hanfeng::SurfacePanel panel;
		double load_panel_ms = 0.0;
	};

	std::string extract_json_string(const std::string& json, const std::string& key) {
		std::string search = "\"" + key + "\"";
		auto pos = json.find(search);
		if (pos == std::string::npos) return "";
		pos = json.find(':', pos + search.size());
		if (pos == std::string::npos) return "";
		pos = json.find('"', pos + 1);
		if (pos == std::string::npos) return "";
		auto end = json.find('"', pos + 1);
		return json.substr(pos + 1, end - pos - 1);
	}

	std::vector<PanelEntry> parse_metadata(const std::filesystem::path& metadata_path) {
		std::ifstream file(metadata_path);
		if (!file.is_open()) {
			throw std::runtime_error("Cannot open metadata.json: " + metadata_path.string());
		}
		std::string json((std::istreambuf_iterator<char>(file)),
			std::istreambuf_iterator<char>());
		file.close();

		std::vector<PanelEntry> entries;
		// 查找 "panels": [ ... ] 中的每个 { ... }
		auto panels_pos = json.find("\"panels\"");
		if (panels_pos == std::string::npos) return entries;
		auto bracket_pos = json.find('[', panels_pos);
		if (bracket_pos == std::string::npos) return entries;

		std::size_t i = bracket_pos + 1;
		while (i < json.size()) {
			auto obj_start = json.find('{', i);
			if (obj_start == std::string::npos) break;
			auto obj_end = json.find('}', obj_start);
			if (obj_end == std::string::npos) break;

			std::string obj = json.substr(obj_start, obj_end - obj_start + 1);
			PanelEntry entry;
			entry.name = extract_json_string(obj, "name");
			entry.category = extract_json_string(obj, "category");
			entry.directory = extract_json_string(obj, "directory");
			if (!entry.name.empty()) {
				entries.push_back(std::move(entry));
			}
			i = obj_end + 1;
		}
		return entries;
	}

	// =========================================================================
	// HTML 生成
	// =========================================================================

	void write_weld_viewer_html(const std::filesystem::path& html_path,
		const std::string& json_data) {
		std::ofstream out(html_path);
		if (!out.is_open()) {
			std::cerr << "Failed to create HTML: " << html_path << "\n";
			return;
		}

		out << R"html(<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>焊缝计算结果可视化</title>
<style>
:root {
  --bg: #f2efe8;
  --ink: #1f1d1a;
  --muted: #6b645b;
  --panel: #fffdfa;
  --line: #b52020;
  --mesh: rgba(61, 103, 160, 0.22);
  --frame: #d8d1c4;
}
* { box-sizing: border-box; }
body {
  margin: 0;
  font-family: "Segoe UI", "PingFang SC", "Microsoft YaHei", sans-serif;
  color: var(--ink);
  background: linear-gradient(180deg, #f6f1e7 0%, var(--bg) 100%);
}
header { padding: 28px 32px 12px; }
h1 { margin: 0 0 8px; font-size: 28px; }
.summary { color: var(--muted); line-height: 1.6; max-width: 1120px; }
.app { padding: 0 24px 36px; }
.viewer {
  background: var(--panel);
  border: 1px solid var(--frame);
  border-radius: 20px;
  overflow: hidden;
  box-shadow: 0 10px 25px rgba(42, 33, 17, 0.08);
}
.viewer-wrap {
  background:
    repeating-linear-gradient(0deg, transparent 0, transparent 23px, rgba(95,85,71,0.05) 24px),
    repeating-linear-gradient(90deg, transparent 0, transparent 23px, rgba(95,85,71,0.05) 24px);
  padding: 14px;
  min-height: 420px;
  height: min(72vh, 760px);
  border-bottom: 1px solid var(--frame);
}
canvas { width: 100%; height: 100%; display: block; background: rgba(255,255,255,0.72); border-radius: 14px; cursor: grab; }
canvas.dragging { cursor: grabbing; }
.viewer-status { padding: 10px 24px 0; color: var(--muted); font-size: 13px; }
.controls { display: grid; gap: 14px; padding: 18px; border-bottom: 1px solid var(--frame); }
.controls-row { display: grid; grid-template-columns: 1fr auto auto; gap: 10px; align-items: center; }
.view-row { grid-template-columns: auto auto auto auto 1fr; }
select, button, input[type="range"] { font: inherit; }
select { width: 100%; padding: 10px 12px; border: 1px solid var(--frame); border-radius: 12px; background: #fff; }
button { border: 1px solid var(--frame); background: #fff; border-radius: 12px; padding: 10px 14px; cursor: pointer; }
button:hover { background: #f7f3eb; }
.toggles { display: flex; flex-wrap: wrap; gap: 14px; color: var(--muted); font-size: 14px; }
.toggles label { display: inline-flex; align-items: center; gap: 8px; }
.sliders { display: grid; gap: 12px; }
.sliders label { display: grid; grid-template-columns: 56px 1fr 48px; gap: 10px; align-items: center; color: var(--muted); font-size: 14px; }
.sliders output { text-align: right; font-variant-numeric: tabular-nums; }
.zoom-readout { justify-self: end; color: var(--muted); font-size: 13px; font-variant-numeric: tabular-nums; }
.content { padding: 16px 18px 18px; }
.title-row { display: flex; justify-content: space-between; gap: 12px; align-items: baseline; margin-bottom: 10px; }
.title-row h2 { margin: 0; font-size: 18px; word-break: break-all; }
.tag { font-size: 12px; padding: 4px 8px; border-radius: 999px; background: #ece7dc; color: #5d544a; white-space: nowrap; }
.meta { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 8px 14px; font-size: 13px; color: var(--muted); }
.legend { display: flex; flex-wrap: wrap; gap: 16px; padding: 0 24px 18px; color: var(--muted); font-size: 13px; }
.legend span::before {
  content: ""; display: inline-block; width: 12px; height: 12px; border-radius: 3px;
  margin-right: 8px; vertical-align: -1px;
}
.legend .l-plane::before { background: rgba(50,140,70,0.45); }
.legend .l-surface::before { background: rgba(61,103,160,0.55); }
.legend .l-boundary::before { background: var(--line); }
.legend .l-weld::before { background: #FFD700; }
.info-panel { padding: 16px 24px; border-top: 1px solid var(--frame); }
.info-panel h3 { margin: 0 0 8px; font-size: 15px; }
.info-panel .meta { font-size: 13px; }
</style>
</head>
<body>
<header>
  <h1>焊缝计算结果可视化</h1>
  <p class="summary">12 组平面板 × 曲面板焊缝检测结果。选择组合查看平面板（蓝色线框）、曲面板（蓝色线框）、边界环（红色）和焊缝折线（鲜艳色标注段号）。</p>
</header>
<div class="app">
  <div class="viewer">
    <div class="controls">
      <div class="controls-row">
        <select id="comboSelect"></select>
        <button id="prevCombo">&larr;</button>
        <button id="nextCombo">&rarr;</button>
      </div>
      <div class="controls-row view-row">
        <button id="zoomOut">&minus;</button>
        <button id="zoomIn">&plus;</button>
        <button id="resetView">重置视角</button>
        <div class="zoom-readout" id="zoomValue">100%</div>
        <div></div>
      </div>
      <div class="toggles">
        <label><input type="checkbox" id="showMesh" checked> 网格</label>
        <label><input type="checkbox" id="showBoundary" checked> 边界环</label>
        <label><input type="checkbox" id="showWeld" checked> 焊缝</label>
      </div>
      <div class="sliders">
        <label>偏航 <input type="range" id="yawSlider" min="-180" max="180" value="-42"> <output id="yawValue">-42&deg;</output></label>
        <label>俯仰 <input type="range" id="pitchSlider" min="-90" max="90" value="32"> <output id="pitchValue">32&deg;</output></label>
      </div>
    </div>
    <div class="viewer-wrap">
      <canvas id="viewerCanvas"></canvas>
    </div>
    <div class="viewer-status" id="viewerStatus">加载中...</div>
    <div class="content">
      <div class="title-row">
        <h2 id="comboTitle">--</h2>
        <span class="tag" id="comboTag">--</span>
      </div>
      <div class="meta" id="comboMeta"></div>
    </div>
    <div class="info-panel">
      <h3>焊缝详情</h3>
      <div class="meta" id="weldMeta"></div>
    </div>
  </div>
  <div class="legend">
    <span class="l-plane">平面板网格</span>
    <span class="l-surface">曲面板网格</span>
    <span class="l-boundary">边界环</span>
    <span class="l-weld">焊缝折线</span>
  </div>
</div>

<script>
var WELD_DATA = )html" << json_data << R"html(;
</script>
<script>
const WELD_COLORS = [
  '#FFD700', '#00FF00', '#FF6600', '#FF00FF', '#00FFFF',
  '#FF4444', '#44FF44', '#FFAA00', '#AA00FF', '#00AAFF',
  '#FF0088', '#88FF00', '#0088FF', '#FF8800', '#8800FF',
  '#00FF88', '#FF0044', '#44FFAA', '#AAFF00', '#0044FF'
];

let DATA = null;
let state = {
  comboIndex: 0,
  yaw: -42 * Math.PI / 180,
  pitch: 32 * Math.PI / 180,
  zoom: 1,
  panX: 0,
  panY: 0,
  showMesh: true,
  showBoundary: true,
  showWeld: true,
};

function rotatePoint(point, yaw, pitch) {
  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const x1 = point[0] * cy - point[2] * sy;
  const z1 = point[0] * sy + point[2] * cy;
  const y2 = point[1] * cp - z1 * sp;
  const z2 = point[1] * sp + z1 * cp;
  return [x1, y2, z2];
}

function transformPositions(positions) {
  const transformed = new Float32Array(positions.length);
  for (let i = 0; i < positions.length; i += 3) {
    const rotated = rotatePoint([positions[i], positions[i+1], positions[i+2]], state.yaw, state.pitch);
    transformed[i] = rotated[0] * 0.92;
    transformed[i+1] = rotated[1] * 0.92;
    transformed[i+2] = rotated[2] * 0.92;
  }
  return transformed;
}

function projectToCanvas(positions, width, height) {
  const projected = new Float32Array(positions.length / 3 * 2);
  const xScale = width * 0.46 * state.zoom;
  const yScale = height * 0.46 * state.zoom;
  const halfW = width / 2, halfH = height / 2;
  for (let i = 0, j = 0; i < positions.length; i += 3, j += 2) {
    projected[j] = halfW + state.panX + positions[i] * xScale;
    projected[j+1] = halfH + state.panY - positions[i+1] * yScale;
  }
  return projected;
}

function clampZoom(v) { return Math.max(0.2, Math.min(12, v)); }

function setZoom(next, ax, ay) {
  const canvas = document.getElementById('viewerCanvas');
  ax = ax || canvas.width / 2;
  ay = ay || canvas.height / 2;
  const prev = state.zoom;
  const nextZ = clampZoom(next);
  if (Math.abs(nextZ - prev) < 1e-9) return;
  const hw = canvas.width / 2, hh = canvas.height / 2;
  const s = nextZ / prev;
  state.panX = ax - hw - (ax - hw - state.panX) * s;
  state.panY = ay - hh - (ay - hh - state.panY) * s;
  state.zoom = nextZ;
}

function resetView() {
  state.yaw = -42 * Math.PI / 180;
  state.pitch = 32 * Math.PI / 180;
  state.zoom = 1;
  state.panX = 0;
  state.panY = 0;
}

function computeBBox(points) {
  let minX = Infinity, minY = Infinity, minZ = Infinity;
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
  for (const p of points) {
    if (p[0] < minX) minX = p[0]; if (p[0] > maxX) maxX = p[0];
    if (p[1] < minY) minY = p[1]; if (p[1] > maxY) maxY = p[1];
    if (p[2] < minZ) minZ = p[2]; if (p[2] > maxZ) maxZ = p[2];
  }
  return { minX, minY, minZ, maxX, maxY, maxZ };
}

function normalizeVertices(vertices, bbox) {
  const cx = (bbox.minX + bbox.maxX) / 2;
  const cy = (bbox.minY + bbox.maxY) / 2;
  const cz = (bbox.minZ + bbox.maxZ) / 2;
  const extent = Math.max(bbox.maxX - bbox.minX, bbox.maxY - bbox.minY, bbox.maxZ - bbox.minZ, 1);
  const scale = 1.7 / extent;
  const positions = new Float32Array(vertices.length * 3);
  for (let i = 0; i < vertices.length; i++) {
    positions[i*3]   = (vertices[i][0] - cx) * scale;
    positions[i*3+1] = (vertices[i][1] - cy) * scale;
    positions[i*3+2] = (vertices[i][2] - cz) * scale;
  }
  return { positions, center: [cx, cy, cz], scale };
}

function buildWireframe(vertices, triangles) {
  const edges = new Set();
  const pts = [];
  for (const tri of triangles) {
    for (let k = 0; k < 3; k++) {
      const a = tri[k], b = tri[(k+1)%3];
      const key = a < b ? a+':'+b : b+':'+a;
      if (edges.has(key)) continue;
      edges.add(key);
      pts.push(vertices[a][0], vertices[a][1], vertices[a][2],
               vertices[b][0], vertices[b][1], vertices[b][2]);
    }
  }
  return new Float32Array(pts);
}

function normalizeLinePositions(points, center, scale) {
  const pts = [];
  for (const p of points) {
    pts.push((p[0]-center[0])*scale, (p[1]-center[1])*scale, (p[2]-center[2])*scale);
  }
  return new Float32Array(pts);
}

function resizeCanvas() {
  const canvas = document.getElementById('viewerCanvas');
  const ratio = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  const cssW = Math.max(1, rect.width || canvas.clientWidth || 960);
  const cssH = Math.max(1, rect.height || canvas.clientHeight || Math.round(cssW * 0.625));
  const w = Math.max(1, Math.floor(cssW * ratio));
  const h = Math.max(1, Math.floor(cssH * ratio));
  if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
}

function renderCombo(index) {
  if (!DATA || index >= DATA.combinations.length) return;
  const combo = DATA.combinations[index];
  const canvas = document.getElementById('viewerCanvas');
  const ctx = canvas.getContext('2d');
  resizeCanvas();

  // 收集所有点以计算统一包围盒
  let allPoints = [];
  if (combo.plane_vertices) for (const p of combo.plane_vertices) allPoints.push(p);
  if (combo.plane_boundary_loops) for (const loop of combo.plane_boundary_loops) for (const p of loop) allPoints.push(p);
  if (combo.surface_vertices) for (const p of combo.surface_vertices) allPoints.push(p);
  if (combo.surface_boundary_loops) for (const loop of combo.surface_boundary_loops) for (const p of loop) allPoints.push(p);
  if (combo.weld_result && combo.weld_result.polylines)
    for (const pl of combo.weld_result.polylines) for (const p of pl.points) allPoints.push(p);
  if (allPoints.length === 0) return;

  const bbox = computeBBox(allPoints);
  const norm = normalizeVertices(allPoints, bbox);

  // Helper: normalize mesh vertices + wireframe
  function prepareMeshData(vertices, triangles) {
    const positions = new Float32Array(vertices.length * 3);
    for (let i = 0; i < vertices.length; i++) {
      positions[i*3]   = (vertices[i][0]-norm.center[0])*norm.scale;
      positions[i*3+1] = (vertices[i][1]-norm.center[1])*norm.scale;
      positions[i*3+2] = (vertices[i][2]-norm.center[2])*norm.scale;
    }
    const wireframe = buildWireframe(
      vertices.map(v => [(v[0]-norm.center[0])*norm.scale, (v[1]-norm.center[1])*norm.scale, (v[2]-norm.center[2])*norm.scale]),
      triangles
    );
    return { positions, wireframe };
  }

  // Helper: normalize boundary loop line segments
  function prepareBoundaryData(loops) {
    const rawFlat = [];
    for (const loop of loops) {
      if (loop.length < 2) continue;
      for (let i = 0; i < loop.length; i++) {
        const s = loop[i], e = loop[(i+1)%loop.length];
        rawFlat.push(s[0],s[1],s[2], e[0],e[1],e[2]);
      }
    }
    return normalizeLinePositions(rawFlat, norm.center, norm.scale);
  }

  // 平面板 mesh
  const planeMesh = (combo.plane_vertices && combo.plane_triangles)
    ? prepareMeshData(combo.plane_vertices, combo.plane_triangles) : null;
  const planeBoundary = combo.plane_boundary_loops ? prepareBoundaryData(combo.plane_boundary_loops) : new Float32Array(0);

  // 曲面板 mesh
  const surfMesh = (combo.surface_vertices && combo.surface_triangles)
    ? prepareMeshData(combo.surface_vertices, combo.surface_triangles) : null;
  const surfBoundary = combo.surface_boundary_loops ? prepareBoundaryData(combo.surface_boundary_loops) : new Float32Array(0);

  // Transform & project
  function project(data) { return projectToCanvas(transformPositions(data), canvas.width, canvas.height); }

  const pPlanePos = planeMesh ? project(planeMesh.positions) : new Float32Array(0);
  const pPlaneWire = planeMesh ? project(planeMesh.wireframe) : new Float32Array(0);
  const pPlaneBound = project(planeBoundary);
  const pSurfPos = surfMesh ? project(surfMesh.positions) : new Float32Array(0);
  const pSurfWire = surfMesh ? project(surfMesh.wireframe) : new Float32Array(0);
  const pSurfBound = project(surfBoundary);

  // Clear
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = 'rgba(250,248,243,1)';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  const dpr = window.devicePixelRatio || 1;

  // Draw plane panel mesh (green)
  if (state.showMesh && pPlaneWire.length > 0) {
    ctx.fillStyle = 'rgba(60,160,80,0.15)';
    for (const tri of combo.plane_triangles) {
      ctx.beginPath();
      ctx.moveTo(pPlanePos[tri[0]*2], pPlanePos[tri[0]*2+1]);
      ctx.lineTo(pPlanePos[tri[1]*2], pPlanePos[tri[1]*2+1]);
      ctx.lineTo(pPlanePos[tri[2]*2], pPlanePos[tri[2]*2+1]);
      ctx.closePath();
      ctx.fill();
    }
    ctx.strokeStyle = 'rgba(50,140,70,0.45)';
    ctx.lineWidth = Math.max(1, dpr * 0.8);
    ctx.beginPath();
    for (let i = 0; i < pPlaneWire.length; i += 4) {
      ctx.moveTo(pPlaneWire[i], pPlaneWire[i+1]);
      ctx.lineTo(pPlaneWire[i+2], pPlaneWire[i+3]);
    }
    ctx.stroke();
  }

  // Draw surface panel mesh (blue)
  if (state.showMesh && pSurfWire.length > 0) {
    ctx.fillStyle = 'rgba(79,128,191,0.18)';
    for (const tri of combo.surface_triangles) {
      ctx.beginPath();
      ctx.moveTo(pSurfPos[tri[0]*2], pSurfPos[tri[0]*2+1]);
      ctx.lineTo(pSurfPos[tri[1]*2], pSurfPos[tri[1]*2+1]);
      ctx.lineTo(pSurfPos[tri[2]*2], pSurfPos[tri[2]*2+1]);
      ctx.closePath();
      ctx.fill();
    }
    ctx.strokeStyle = 'rgba(61,103,160,0.55)';
    ctx.lineWidth = Math.max(1, dpr * 0.8);
    ctx.beginPath();
    for (let i = 0; i < pSurfWire.length; i += 4) {
      ctx.moveTo(pSurfWire[i], pSurfWire[i+1]);
      ctx.lineTo(pSurfWire[i+2], pSurfWire[i+3]);
    }
    ctx.stroke();
  }

  // Draw boundary loops
  if (state.showBoundary) {
    if (pPlaneBound.length > 0) {
      ctx.strokeStyle = 'rgba(181,32,32,1)';
      ctx.lineWidth = Math.max(2, dpr * 1.6);
      ctx.beginPath();
      for (let i = 0; i < pPlaneBound.length; i += 4) {
        ctx.moveTo(pPlaneBound[i], pPlaneBound[i+1]);
        ctx.lineTo(pPlaneBound[i+2], pPlaneBound[i+3]);
      }
      ctx.stroke();
    }
    if (pSurfBound.length > 0) {
      ctx.strokeStyle = 'rgba(181,32,32,0.7)';
      ctx.lineWidth = Math.max(2, dpr * 1.2);
      ctx.beginPath();
      for (let i = 0; i < pSurfBound.length; i += 4) {
        ctx.moveTo(pSurfBound[i], pSurfBound[i+1]);
        ctx.lineTo(pSurfBound[i+2], pSurfBound[i+3]);
      }
      ctx.stroke();
    }
  }

  // Draw weld polylines
  if (state.showWeld && combo.weld_result && combo.weld_result.polylines) {
    ctx.lineWidth = Math.max(3, dpr * 2.4);
    ctx.font = 'bold ' + Math.max(12, dpr * 13) + 'px "Segoe UI", sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';

    for (let wi = 0; wi < combo.weld_result.polylines.length; wi++) {
      const polyline = combo.weld_result.polylines[wi];
      if (polyline.points.length < 2) continue;
      const color = WELD_COLORS[wi % WELD_COLORS.length];

      const weldNorm = [];
      for (const p of polyline.points) {
        weldNorm.push(
          (p[0]-norm.center[0])*norm.scale,
          (p[1]-norm.center[1])*norm.scale,
          (p[2]-norm.center[2])*norm.scale
        );
      }
      const pWeld = project(new Float32Array(weldNorm));

      ctx.strokeStyle = color;
      ctx.beginPath();
      ctx.moveTo(pWeld[0], pWeld[1]);
      for (let k = 1; k < polyline.points.length; k++) {
        ctx.lineTo(pWeld[k*2], pWeld[k*2+1]);
      }
      ctx.stroke();

      const midIdx = Math.floor(polyline.points.length / 2);
      const lx = pWeld[midIdx*2];
      const ly = pWeld[midIdx*2+1];

      const label = '#' + wi;
      const metrics = ctx.measureText(label);
      const pad = 4 * dpr;
      ctx.fillStyle = 'rgba(255,255,255,0.85)';
      ctx.fillRect(lx - metrics.width/2 - pad, ly - 8*dpr - pad, metrics.width + 2*pad, 16*dpr + 2*pad);

      ctx.fillStyle = color;
      ctx.fillText(label, lx, ly);
    }
  }

  // Update status
  document.getElementById('viewerStatus').textContent =
    '渲染模式: Canvas 2D 线框投影 | 缩放: ' + Math.round(state.zoom * 100) + '%';
}

function updateMeta(index) {
  if (!DATA || index >= DATA.combinations.length) return;
  const combo = DATA.combinations[index];
  document.getElementById('comboTitle').textContent =
    combo.plane_panel.name + ' × ' + combo.surface_panel.name;
  document.getElementById('comboTag').textContent =
    '组合 ' + (index + 1) + ' / ' + DATA.combinations.length;

  const weldCount = combo.weld_result ? combo.weld_result.polylines.length : 0;
  let totalWeldPts = 0;
  if (combo.weld_result) {
    for (const pl of combo.weld_result.polylines) totalWeldPts += pl.points.length;
  }

  document.getElementById('comboMeta').innerHTML =
    '<div>平面板: ' + combo.plane_panel.name + '</div>' +
    '<div>曲面板: ' + combo.surface_panel.name + '</div>' +
    '<div>焊缝段数: ' + weldCount + '</div>' +
    '<div>焊缝总点数: ' + totalWeldPts + '</div>';

  if (combo.timing_ms) {
    document.getElementById('comboMeta').innerHTML +=
      '<div>加载平面板: ' + combo.timing_ms.load_plane.toFixed(1) + ' ms</div>' +
      '<div>加载曲面板: ' + combo.timing_ms.load_surface.toFixed(1) + ' ms</div>' +
      '<div>计算焊缝: ' + combo.timing_ms.compute_weld.toFixed(1) + ' ms</div>' +
      '<div>总耗时: ' + combo.timing_ms.total.toFixed(1) + ' ms</div>';
  }

  // Weld details
  let weldHtml = '';
  if (combo.weld_result && combo.weld_result.polylines.length > 0) {
    for (let i = 0; i < combo.weld_result.polylines.length; i++) {
      const color = WELD_COLORS[i % WELD_COLORS.length];
      weldHtml += '<div style="color:' + color + '; font-weight:bold;">段 #' + i +
        ': ' + combo.weld_result.polylines[i].points.length + ' 点</div>';
    }
  } else {
    weldHtml = '<div>无焊缝结果</div>';
  }
  document.getElementById('weldMeta').innerHTML = weldHtml;
}

function syncControls() {
  const yawSlider = document.getElementById('yawSlider');
  const pitchSlider = document.getElementById('pitchSlider');
  yawSlider.value = Math.round(state.yaw * 180 / Math.PI);
  pitchSlider.value = Math.round(state.pitch * 180 / Math.PI);
  document.getElementById('yawValue').textContent = yawSlider.value + '°';
  document.getElementById('pitchValue').textContent = pitchSlider.value + '°';
  document.getElementById('zoomValue').textContent = Math.round(state.zoom * 100) + '%';
  document.getElementById('showMesh').checked = state.showMesh;
  document.getElementById('showBoundary').checked = state.showBoundary;
  document.getElementById('showWeld').checked = state.showWeld;
  document.getElementById('comboSelect').value = String(state.comboIndex);
}

function renderCurrent() {
  updateMeta(state.comboIndex);
  syncControls();
  renderCombo(state.comboIndex);
}

function setCombo(index) {
  state.comboIndex = (index + DATA.combinations.length) % DATA.combinations.length;
  renderCurrent();
}

// Init — 使用内联 WELD_DATA
DATA = WELD_DATA;
(function() {
  const sel = document.getElementById('comboSelect');
  DATA.combinations.forEach((c, i) => {
    const opt = document.createElement('option');
    opt.value = String(i);
    opt.textContent = c.plane_panel.name + ' × ' + c.surface_panel.name;
    sel.appendChild(opt);
  });
  renderCurrent();
})();

// Events
document.getElementById('comboSelect').addEventListener('change', e => setCombo(Number(e.target.value)));
document.getElementById('prevCombo').addEventListener('click', () => setCombo(state.comboIndex - 1));
document.getElementById('nextCombo').addEventListener('click', () => setCombo(state.comboIndex + 1));
document.getElementById('zoomIn').addEventListener('click', () => { setZoom(state.zoom * 1.2); renderCurrent(); });
document.getElementById('zoomOut').addEventListener('click', () => { setZoom(state.zoom / 1.2); renderCurrent(); });
document.getElementById('resetView').addEventListener('click', () => { resetView(); renderCurrent(); });
document.getElementById('yawSlider').addEventListener('input', function() {
  state.yaw = Number(this.value) * Math.PI / 180;
  renderCurrent();
});
document.getElementById('pitchSlider').addEventListener('input', function() {
  state.pitch = Number(this.value) * Math.PI / 180;
  renderCurrent();
});
document.getElementById('showMesh').addEventListener('change', function() { state.showMesh = this.checked; renderCurrent(); });
document.getElementById('showBoundary').addEventListener('change', function() { state.showBoundary = this.checked; renderCurrent(); });
document.getElementById('showWeld').addEventListener('change', function() { state.showWeld = this.checked; renderCurrent(); });

// Drag rotation & pan
(function() {
  const canvas = document.getElementById('viewerCanvas');
  let dragging = false, dragButton = 0, lastX = 0, lastY = 0;
  canvas.addEventListener('mousedown', e => {
    dragging = true; dragButton = e.button; lastX = e.clientX; lastY = e.clientY;
    canvas.classList.add('dragging'); e.preventDefault();
  });
  window.addEventListener('mousemove', e => {
    if (!dragging) return;
    const dx = e.clientX - lastX, dy = e.clientY - lastY;
    if (dragButton === 0) {
      state.yaw += dx * 0.005;
      state.pitch += dy * 0.005;
      state.pitch = Math.max(-Math.PI/2, Math.min(Math.PI/2, state.pitch));
    } else {
      state.panX += dx * (window.devicePixelRatio || 1);
      state.panY += dy * (window.devicePixelRatio || 1);
    }
    lastX = e.clientX; lastY = e.clientY;
    renderCurrent();
  });
  window.addEventListener('mouseup', () => { dragging = false; canvas.classList.remove('dragging'); });
  canvas.addEventListener('wheel', e => {
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    const mx = (e.clientX - rect.left) * dpr;
    const my = (e.clientY - rect.top) * dpr;
    setZoom(state.zoom * (e.deltaY > 0 ? 0.9 : 1.1), mx, my);
    renderCurrent();
    e.preventDefault();
  }, { passive: false });
  canvas.addEventListener('contextmenu', e => e.preventDefault());
})();
window.addEventListener('resize', () => { if (DATA) renderCurrent(); });
</script>
</body>
</html>
)html";

		out.close();
	}

	void write_surface_surface_weld_viewer_html(const std::filesystem::path& html_path,
		const std::string& json_data) {
		std::ofstream out(html_path);
		if (!out.is_open()) {
			std::cerr << "Failed to create HTML: " << html_path << "\n";
			return;
		}

		out << R"html(<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>曲面板-曲面板焊缝结果</title>
<style>
* { box-sizing: border-box; }
body {
  margin: 0;
  font-family: "Segoe UI", "Microsoft YaHei", sans-serif;
  color: #202124;
  background: #f5f7f9;
}
header { padding: 24px 28px 10px; }
h1 { margin: 0 0 8px; font-size: 24px; }
.summary { color: #5f6670; line-height: 1.6; }
.layout { display: grid; grid-template-columns: minmax(280px, 360px) 1fr; gap: 16px; padding: 16px 24px 28px; }
.panel {
  background: #fff;
  border: 1px solid #d9dee5;
  border-radius: 8px;
  overflow: hidden;
}
.side { max-height: calc(100vh - 150px); overflow: auto; }
.combo {
  width: 100%;
  display: block;
  text-align: left;
  border: 0;
  border-bottom: 1px solid #edf0f3;
  background: #fff;
  padding: 10px 12px;
  cursor: pointer;
}
.combo:hover, .combo.active { background: #eef5ff; }
.combo strong { display: block; font-size: 13px; word-break: break-all; }
.combo span { display: block; margin-top: 4px; color: #69717c; font-size: 12px; }
.toolbar { display: flex; gap: 8px; align-items: center; padding: 12px; border-bottom: 1px solid #edf0f3; }
button { border: 1px solid #cfd6df; background: #fff; border-radius: 6px; padding: 8px 10px; cursor: pointer; }
button:hover { background: #f2f6fb; }
canvas { width: 100%; height: min(68vh, 720px); display: block; background: #fbfcfd; }
.meta { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 8px 14px; padding: 14px; color: #535b66; font-size: 13px; border-top: 1px solid #edf0f3; }
.welds { padding: 14px; border-top: 1px solid #edf0f3; }
.weld { margin-bottom: 10px; color: #535b66; font-size: 13px; }
.weld b { color: #202124; }
@media (max-width: 900px) { .layout { grid-template-columns: 1fr; } canvas { height: 56vh; } }
</style>
</head>
<body>
<header>
  <h1>曲面板-曲面板焊缝结果</h1>
  <div class="summary" id="summary"></div>
</header>
<div class="layout">
  <div class="panel side" id="comboList"></div>
  <div class="panel">
    <div class="toolbar">
      <button id="prevBtn">上一组</button>
      <button id="nextBtn">下一组</button>
      <button id="resetBtn">重置视图</button>
      <span id="title"></span>
    </div>
    <canvas id="canvas"></canvas>
    <div class="meta" id="meta"></div>
    <div class="welds" id="welds"></div>
  </div>
</div>
<script>
var SURFACE_SURFACE_WELD_DATA = )html" << json_data << R"html(;
</script>
<script>
const DATA = SURFACE_SURFACE_WELD_DATA;
const COLORS = ['#d71920', '#1f77b4', '#2ca02c', '#ff7f0e', '#9467bd', '#17becf', '#bcbd22', '#e377c2'];
let current = 0;
let scale = 1;
let panX = 0;
let panY = 0;

function allWeldPoints(combo) {
  const pts = [];
  if (!combo.weld_result) return pts;
  for (const weld of combo.weld_result.polylines) {
    for (const p of weld.points) pts.push(p);
  }
  return pts;
}

function bounds(points) {
  const b = { minX: Infinity, minY: Infinity, minZ: Infinity, maxX: -Infinity, maxY: -Infinity, maxZ: -Infinity };
  for (const p of points) {
    b.minX = Math.min(b.minX, p[0]); b.maxX = Math.max(b.maxX, p[0]);
    b.minY = Math.min(b.minY, p[1]); b.maxY = Math.max(b.maxY, p[1]);
    b.minZ = Math.min(b.minZ, p[2]); b.maxZ = Math.max(b.maxZ, p[2]);
  }
  return b;
}

function project(p, b, w, h) {
  const cx = (b.minX + b.maxX) / 2;
  const cy = (b.minY + b.maxY) / 2;
  const cz = (b.minZ + b.maxZ) / 2;
  const extent = Math.max(b.maxX - b.minX, b.maxY - b.minY, b.maxZ - b.minZ, 1);
  const x = (p[0] - cx) - 0.42 * (p[2] - cz);
  const y = (p[1] - cy) + 0.28 * (p[2] - cz);
  const s = Math.min(w, h) * 0.74 / extent * scale;
  return [w / 2 + panX + x * s, h / 2 + panY - y * s];
}

function resizeCanvas(canvas) {
  const ratio = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.max(1, Math.floor(rect.width * ratio));
  canvas.height = Math.max(1, Math.floor(rect.height * ratio));
}

function render() {
  const canvas = document.getElementById('canvas');
  resizeCanvas(canvas);
  const ctx = canvas.getContext('2d');
  const combo = DATA.combinations[current];
  const pts = allWeldPoints(combo);
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = '#fbfcfd';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  if (pts.length === 0) return;
  const b = bounds(pts);
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';

  combo.weld_result.polylines.forEach((weld, wi) => {
    if (weld.points.length < 2) return;
    ctx.strokeStyle = COLORS[wi % COLORS.length];
    ctx.lineWidth = Math.max(2, (window.devicePixelRatio || 1) * 2);
    ctx.beginPath();
    weld.points.forEach((p, i) => {
      const pp = project(p, b, canvas.width, canvas.height);
      if (i === 0) ctx.moveTo(pp[0], pp[1]);
      else ctx.lineTo(pp[0], pp[1]);
    });
    ctx.stroke();
  });
}

function updateInfo() {
  const combo = DATA.combinations[current];
  document.getElementById('title').textContent =
    combo.first_surface_panel.name + ' x ' + combo.second_surface_panel.name;
  document.querySelectorAll('.combo').forEach((el, i) => el.classList.toggle('active', i === current));
  document.getElementById('meta').innerHTML =
    '<div>组合序号: ' + (current + 1) + ' / ' + DATA.combinations.length + '</div>' +
    '<div>焊缝段数: ' + combo.weld_result.polyline_count + '</div>' +
    '<div>曲面板1: ' + combo.first_surface_panel.name + '</div>' +
    '<div>曲面板2: ' + combo.second_surface_panel.name + '</div>' +
    '<div>计算耗时: ' + combo.timing_ms.compute_weld.toFixed(2) + ' ms</div>' +
    '<div>拟合耗时: ' + combo.timing_ms.fit_weld.toFixed(2) + ' ms</div>';
  document.getElementById('welds').innerHTML = combo.weld_result.polylines.map((weld, i) =>
    '<div class="weld"><b style="color:' + COLORS[i % COLORS.length] + '">#' + i +
    '</b> points=' + weld.point_count +
    ', reference=[' + weld.reference_point.map(v => v.toFixed(3)).join(', ') +
    '], tangent=[' + weld.tangent_direction.map(v => v.toFixed(4)).join(', ') + ']</div>'
  ).join('');
}

function setCurrent(index) {
  current = (index + DATA.combinations.length) % DATA.combinations.length;
  scale = 1; panX = 0; panY = 0;
  updateInfo();
  render();
}

document.getElementById('summary').textContent =
  '测试组合 ' + DATA.tested_combination_count + ' 组，导出存在焊缝的组合 ' +
  DATA.exported_combination_count + ' 组。';
const list = document.getElementById('comboList');
DATA.combinations.forEach((combo, i) => {
  const btn = document.createElement('button');
  btn.className = 'combo';
  btn.innerHTML = '<strong>' + combo.first_surface_panel.name + ' x ' + combo.second_surface_panel.name +
    '</strong><span>' + combo.weld_result.polyline_count + ' 条焊缝</span>';
  btn.onclick = () => setCurrent(i);
  list.appendChild(btn);
});
document.getElementById('prevBtn').onclick = () => setCurrent(current - 1);
document.getElementById('nextBtn').onclick = () => setCurrent(current + 1);
document.getElementById('resetBtn').onclick = () => setCurrent(current);
document.getElementById('canvas').addEventListener('wheel', e => {
  e.preventDefault();
  scale = Math.max(0.2, Math.min(12, scale * (e.deltaY < 0 ? 1.12 : 0.89)));
  render();
}, { passive: false });
let dragging = false, lastX = 0, lastY = 0;
document.getElementById('canvas').addEventListener('pointerdown', e => { dragging = true; lastX = e.clientX; lastY = e.clientY; });
window.addEventListener('pointerup', () => dragging = false);
window.addEventListener('pointermove', e => {
  if (!dragging) return;
  const ratio = window.devicePixelRatio || 1;
  panX += (e.clientX - lastX) * ratio;
  panY += (e.clientY - lastY) * ratio;
  lastX = e.clientX; lastY = e.clientY;
  render();
});
window.addEventListener('resize', render);
if (DATA.combinations.length > 0) setCurrent(0);
</script>
</body>
</html>
)html";

		out.close();
	}

}  // namespace

// =============================================================================
// 合成数据测试
// =============================================================================

void test_hanfeng_fit_weld_splines_converts_line_and_arc_polylines() {
	hanfeng::WeldCurveResult weld_result{};
	weld_result.polylines.push_back(hanfeng::WeldPolyline{ {
		hanfeng::SPAposition(0.0, 0.0, 0.0),
		hanfeng::SPAposition(5.0, 0.0, 0.0),
		hanfeng::SPAposition(10.0, 0.0, 0.0),
	} });

	hanfeng::Polyline3 arc_points;
	constexpr double pi = 3.14159265358979323846;
	for (int degrees = 0; degrees <= 90; degrees += 10) {
		const double angle = static_cast<double>(degrees) * pi / 180.0;
		arc_points.emplace_back(10.0 * std::cos(angle), 10.0 * std::sin(angle), 0.0);
	}
	weld_result.polylines.push_back(hanfeng::WeldPolyline{ arc_points });

	const hanfeng::WeldSplineResult splines =
		hanfeng::hanfeng_fit_weld_splines(weld_result, 1.0e-3);

	assert(splines.welds.size() == 2U);

	const hanfeng::WeldSpline& line = splines.welds[0];
	assert(line.raw_points.size() == 3U);
	assert(line.points_rxyz.header_matches_point_count());
	assert(line.points_rxyz.geometry_point_count() == 2U);
	assert(line.points_rxyz.points[1].defines_line_to_next());
	assert(nearly_equal(line.reference_point.x(), 0.0));
	assert(nearly_equal(line.reference_point.y(), 0.0));
	assert(nearly_equal(line.tangent_direction.x(), 1.0));
	assert(nearly_equal(line.tangent_direction.y(), 0.0));
	assert(nearly_equal(line.tangent_direction.z(), 0.0));

	const hanfeng::WeldSpline& arc = splines.welds[1];
	assert(arc.raw_points.size() == 10U);
	assert(arc.points_rxyz.header_matches_point_count());
	assert(arc.points_rxyz.geometry_point_count() == 2U);
	assert(arc.points_rxyz.points[1].defines_arc_to_next());
	assert(nearly_equal(std::fabs(arc.points_rxyz.points[1].r), 10.0, 1.0e-3));
	assert(nearly_equal(arc.reference_point.x(), 10.0));
	assert(nearly_equal(arc.reference_point.y(), 0.0));
	assert(nearly_equal(arc.tangent_direction.x(), 0.0, 1.0e-3));
	assert(nearly_equal(arc.tangent_direction.y(), 1.0, 1.0e-3));
	assert(nearly_equal(arc.tangent_direction.z(), 0.0, 1.0e-3));
}

void test_api_get_plane_surface_weld_detects_surface_boundary_on_plane_face() {
	const hanfeng::PlanePanel plane_panel = make_plane_panel_for_face_hit();
	const hanfeng::SurfacePanel surface_panel =
		make_surface_panel_with_boundary_on_plane_face();

	const hanfeng::WeldCurveResult result =
		hanfeng::api_get_plane_surface_weld(plane_panel, surface_panel, 1.0);

	assert(!result.polylines.empty());

	bool found_bottom_edge = false;
	for (const hanfeng::WeldPolyline& polyline : result.polylines) {
		if (polyline_matches_segment(polyline,
			hanfeng::SPAposition(2.0, 2.0, 0.0),
			hanfeng::SPAposition(8.0, 2.0, 0.0))) {
			found_bottom_edge = true;
			break;
		}
	}
	assert(found_bottom_edge);
}

void test_compute_plane_surface_weld_from_geometry_module() {
	const hanfeng::PlanePanel plane_panel = make_plane_panel_for_face_hit();
	const hanfeng::SurfacePanel surface_panel =
		make_surface_panel_with_boundary_on_plane_face();

	const hanfeng::WeldCurveResult result =
		hanfeng::hanfeng_compute_plane_surface_weld(
			plane_panel, surface_panel, 1.0);

	assert(!result.polylines.empty());

	bool found_bottom_edge = false;
	for (const hanfeng::WeldPolyline& polyline : result.polylines) {
		if (polyline_matches_segment(polyline,
			hanfeng::SPAposition(2.0, 2.0, 0.0),
			hanfeng::SPAposition(8.0, 2.0, 0.0))) {
			found_bottom_edge = true;
			break;
		}
	}

	assert(found_bottom_edge);
}

void test_api_get_plane_surface_weld_detects_plane_boundary_coplanar_with_surface_face() {
	const hanfeng::PlanePanel plane_panel =
		make_plane_panel_coplanar_with_surface_face();
	const hanfeng::SurfacePanel surface_panel =
		make_surface_box_panel();

	const hanfeng::WeldCurveResult result =
		hanfeng::api_get_plane_surface_weld(plane_panel, surface_panel, 1.0);

	bool found_plane_loop = false;
	for (const hanfeng::WeldPolyline& polyline : result.polylines) {
		if (polyline_matches_rectangle_on_x_face(polyline, 0.0, 2.0, 8.0, 2.0, 3.0)) {
			found_plane_loop = true;
			break;
		}
	}

	assert(found_plane_loop);
}

void test_api_get_plane_surface_weld_detects_plane_boundary_inside_surface_solid() {
	const hanfeng::PlanePanel plane_panel =
		make_plane_panel_with_boundary_inside_surface_solid();
	const hanfeng::SurfacePanel surface_panel =
		make_surface_box_panel();

	const hanfeng::WeldCurveResult result =
		hanfeng::api_get_plane_surface_weld(plane_panel, surface_panel, 1.0);

	bool found_plane_loop = false;
	for (const hanfeng::WeldPolyline& polyline : result.polylines) {
		if (polyline_matches_rectangle_on_x_face(polyline, 0.5, 2.0, 8.0, 2.0, 3.0)) {
			found_plane_loop = true;
			break;
		}
	}

	assert(found_plane_loop);
}

void test_compute_plane_surface_weld_detects_exact_band_without_sampling_loss() {
	const hanfeng::PlanePanel plane_panel =
		make_plane_panel_with_short_exact_hit_band();
	const hanfeng::SurfacePanel surface_panel =
		make_surface_panel_with_exact_band_hit();

	const hanfeng::WeldCurveResult result =
		hanfeng::hanfeng_compute_plane_surface_weld(
			plane_panel, surface_panel, 0.1);

	const double expected_y = 0.1 + std::sqrt(0.1 * 0.1 - 0.05 * 0.05);

	bool found_exact_band = false;
	for (const hanfeng::WeldPolyline& polyline : result.polylines) {
		if (polyline.points.size() < 2U) {
			continue;
		}

		const hanfeng::SPAposition& first = polyline.points.front();
		const hanfeng::SPAposition& last = polyline.points.back();
		const bool same_forward =
			nearly_equal(first.x(), -0.05) &&
			nearly_equal(first.y(), -expected_y, 1.0e-3) &&
			nearly_equal(first.z(), 0.5) &&
			nearly_equal(last.x(), -0.05) &&
			nearly_equal(last.y(), expected_y, 1.0e-3) &&
			nearly_equal(last.z(), 0.5);
		const bool same_reverse =
			nearly_equal(first.x(), -0.05) &&
			nearly_equal(first.y(), expected_y, 1.0e-3) &&
			nearly_equal(first.z(), 0.5) &&
			nearly_equal(last.x(), -0.05) &&
			nearly_equal(last.y(), -expected_y, 1.0e-3) &&
			nearly_equal(last.z(), 0.5);
		if (same_forward || same_reverse) {
			found_exact_band = true;
			break;
		}
	}

	if (!found_exact_band) {
		throw std::runtime_error(
			"Expected exact plane-boundary hit band was not reconstructed.");
	}
}

void test_api_get_plane_surface_weld_detects_plane_boundary_near_surface_exterior_side() {
	const hanfeng::PlanePanel plane_panel =
		make_plane_panel_near_surface_exterior_side();
	const hanfeng::SurfacePanel surface_panel =
		make_surface_box_panel_without_boundaries();

	const hanfeng::WeldCurveResult result =
		hanfeng::api_get_plane_surface_weld(plane_panel, surface_panel, 1.0);

	bool found_plane_loop = false;
	for (const hanfeng::WeldPolyline& polyline : result.polylines) {
		if (polyline_matches_rectangle_on_x_face(polyline, -1.0, 2.0, 8.0, 2.0, 3.0)) {
			found_plane_loop = true;
			break;
		}
	}

	assert(found_plane_loop);
}

void test_compute_plane_surface_weld_merges_touching_weld_polylines() {
	const hanfeng::PlanePanel plane_panel = make_plane_panel_for_face_hit();

	const hanfeng::SPAposition a(1.0, 2.0, 0.0);
	const hanfeng::SPAposition b(5.0, 2.0, 0.0);
	const hanfeng::SPAposition c(9.0, 2.0, 0.0);
	const hanfeng::SPAposition a_top(1.0, 2.0, 4.0);
	const hanfeng::SPAposition b_top(5.0, 2.0, 4.0);
	const hanfeng::SPAposition c_top(9.0, 2.0, 4.0);

	hanfeng::SurfacePanel surface_panel;
	hanfeng::SurfacePatch first_patch;
	first_patch.boundaries.outer_loops.push_back({ a, b, b_top, a_top });
	surface_panel.surfaces.push_back(first_patch);

	hanfeng::SurfacePatch second_patch;
	second_patch.boundaries.outer_loops.push_back({ b, c, c_top, b_top });
	surface_panel.surfaces.push_back(second_patch);

	const hanfeng::WeldCurveResult result =
		hanfeng::hanfeng_compute_plane_surface_weld(
			plane_panel, surface_panel, 0.0);

	assert(result.polylines.size() == 1U);
	const hanfeng::WeldPolyline& merged = result.polylines.front();
	assert(merged.points.size() == 3U);
	const bool forward =
		merged.points.front() == a &&
		merged.points[1] == b &&
		merged.points.back() == c;
	const bool reversed =
		merged.points.front() == c &&
		merged.points[1] == b &&
		merged.points.back() == a;
	assert(forward || reversed);
}

void test_api_get_surface_surface_weld_detects_boundary_on_surface_face() {
	hanfeng::SurfacePanel first_panel;
	first_panel.surfaces.push_back(make_rect_patch(
		hanfeng::SPAposition(0.0, 0.0, 0.0),
		hanfeng::SPAposition(10.0, 0.0, 0.0),
		hanfeng::SPAposition(10.0, 10.0, 0.0),
		hanfeng::SPAposition(0.0, 10.0, 0.0)));

	hanfeng::SurfacePanel second_panel;
	second_panel.surfaces.push_back(make_rect_patch(
		hanfeng::SPAposition(2.0, 5.0, 0.0),
		hanfeng::SPAposition(8.0, 5.0, 0.0),
		hanfeng::SPAposition(8.0, 5.0, 4.0),
		hanfeng::SPAposition(2.0, 5.0, 4.0)));

	const hanfeng::WeldCurveResult result =
		hanfeng::api_get_surface_surface_weld(first_panel, second_panel, 0.1);

	bool found_shared_edge = false;
	for (const hanfeng::WeldPolyline& polyline : result.polylines) {
		if (polyline_matches_segment(polyline,
			hanfeng::SPAposition(2.0, 5.0, 0.0),
			hanfeng::SPAposition(8.0, 5.0, 0.0))) {
			found_shared_edge = true;
			break;
		}
	}
	assert(found_shared_edge);
}

void test_api_get_surface_surface_weld_from_raw_mesh_matches_panel_api() {
	float first_vertices[4][3] = {
		{ 0.0f, 0.0f, 0.0f },
		{ 10.0f, 0.0f, 0.0f },
		{ 10.0f, 10.0f, 0.0f },
		{ 0.0f, 10.0f, 0.0f }
	};
	int first_triangles[2][3] = {
		{ 0, 1, 2 },
		{ 0, 2, 3 }
	};
	float first_normals[4][3] = {
		{ 0.0f, 0.0f, 1.0f },
		{ 0.0f, 0.0f, 1.0f },
		{ 0.0f, 0.0f, 1.0f },
		{ 0.0f, 0.0f, 1.0f }
	};

	float second_vertices[4][3] = {
		{ 2.0f, 5.0f, 0.0f },
		{ 8.0f, 5.0f, 0.0f },
		{ 8.0f, 5.0f, 4.0f },
		{ 2.0f, 5.0f, 4.0f }
	};
	int second_triangles[2][3] = {
		{ 0, 1, 2 },
		{ 0, 2, 3 }
	};
	float second_normals[4][3] = {
		{ 0.0f, -1.0f, 0.0f },
		{ 0.0f, -1.0f, 0.0f },
		{ 0.0f, -1.0f, 0.0f },
		{ 0.0f, -1.0f, 0.0f }
	};

	const hanfeng::SurfacePanel first_panel =
		hanfeng::api_get_surface_panel(first_vertices, 4, first_triangles, 2,
			first_normals);
	const hanfeng::SurfacePanel second_panel =
		hanfeng::api_get_surface_panel(second_vertices, 4, second_triangles, 2,
			second_normals);

	const hanfeng::WeldCurveResult panel_result =
		hanfeng::api_get_surface_surface_weld(first_panel, second_panel, 0.1);
	const hanfeng::WeldCurveResult raw_result =
		hanfeng::api_get_surface_surface_weld(
			first_vertices, 4, first_triangles, 2, first_normals,
			second_vertices, 4, second_triangles, 2, second_normals,
			0.1);

	assert(raw_result.polylines.size() == panel_result.polylines.size());
	bool found_shared_edge = false;
	for (const hanfeng::WeldPolyline& polyline : raw_result.polylines) {
		if (polyline_matches_segment(polyline,
			hanfeng::SPAposition(2.0, 5.0, 0.0),
			hanfeng::SPAposition(8.0, 5.0, 0.0))) {
			found_shared_edge = true;
			break;
		}
	}
	assert(found_shared_edge);
}

void test_api_get_surface_surface_weld_detects_boundary_inside_surface_solid() {
	hanfeng::SurfacePanel first_panel;
	first_panel.surfaces.push_back(make_rect_patch(
		hanfeng::SPAposition(0.5, 2.0, 2.0),
		hanfeng::SPAposition(0.5, 8.0, 2.0),
		hanfeng::SPAposition(0.5, 8.0, 3.0),
		hanfeng::SPAposition(0.5, 2.0, 3.0)));
	const hanfeng::SurfacePanel second_panel = make_surface_box_panel();

	const hanfeng::WeldCurveResult result =
		hanfeng::api_get_surface_surface_weld(first_panel, second_panel, 0.1);

	bool found_inside_loop = false;
	for (const hanfeng::WeldPolyline& polyline : result.polylines) {
		if (polyline_matches_rectangle_on_x_face(polyline, 0.5, 2.0, 8.0, 2.0, 3.0)) {
			found_inside_loop = true;
			break;
		}
	}
	assert(found_inside_loop);
}

void test_api_get_surface_surface_weld_detects_near_boundary() {
	hanfeng::SurfacePanel first_panel;
	first_panel.surfaces.push_back(make_rect_patch(
		hanfeng::SPAposition(-0.5, 2.0, 2.0),
		hanfeng::SPAposition(-0.5, 8.0, 2.0),
		hanfeng::SPAposition(-0.5, 8.0, 3.0),
		hanfeng::SPAposition(-0.5, 2.0, 3.0)));
	const hanfeng::SurfacePanel second_panel =
		make_box_surface_panel(0.0, 1.0, 0.0, 10.0, 0.0, 10.0, false);

	const hanfeng::WeldCurveResult result =
		hanfeng::api_get_surface_surface_weld(first_panel, second_panel, 1.0);

	bool found_near_loop = false;
	for (const hanfeng::WeldPolyline& polyline : result.polylines) {
		if (polyline_matches_rectangle_on_x_face(polyline, -0.5, 2.0, 8.0, 2.0, 3.0)) {
			found_near_loop = true;
			break;
		}
	}
	assert(found_near_loop);
}

void test_api_get_surface_surface_weld_deduplicates_bidirectional_matches() {
	hanfeng::SurfacePanel first_panel;
	first_panel.surfaces.push_back(make_rect_patch(
		hanfeng::SPAposition(0.0, 0.0, 0.0),
		hanfeng::SPAposition(10.0, 0.0, 0.0),
		hanfeng::SPAposition(10.0, 10.0, 0.0),
		hanfeng::SPAposition(0.0, 10.0, 0.0)));

	hanfeng::SurfacePanel second_panel;
	second_panel.surfaces.push_back(make_rect_patch(
		hanfeng::SPAposition(0.0, 0.0, 0.0),
		hanfeng::SPAposition(10.0, 0.0, 0.0),
		hanfeng::SPAposition(10.0, 10.0, 0.0),
		hanfeng::SPAposition(0.0, 10.0, 0.0)));

	const hanfeng::WeldCurveResult result =
		hanfeng::api_get_surface_surface_weld(first_panel, second_panel, 0.1);

	assert(result.polylines.size() == 1U);
}

void test_api_get_surface_surface_weld_splines_returns_rxyz_and_tangent() {
	hanfeng::SurfacePanel first_panel;
	first_panel.surfaces.push_back(make_rect_patch(
		hanfeng::SPAposition(0.0, 0.0, 0.0),
		hanfeng::SPAposition(10.0, 0.0, 0.0),
		hanfeng::SPAposition(10.0, 10.0, 0.0),
		hanfeng::SPAposition(0.0, 10.0, 0.0)));

	hanfeng::SurfacePanel second_panel;
	second_panel.surfaces.push_back(make_rect_patch(
		hanfeng::SPAposition(2.0, 5.0, 0.0),
		hanfeng::SPAposition(8.0, 5.0, 0.0),
		hanfeng::SPAposition(8.0, 5.0, 4.0),
		hanfeng::SPAposition(2.0, 5.0, 4.0)));

	const hanfeng::WeldSplineResult result =
		hanfeng::api_get_surface_surface_weld_splines(
			first_panel, second_panel, 0.1, 1.0e-3);

	assert(!result.welds.empty());
	const hanfeng::WeldSpline& weld = result.welds.front();
	assert(weld.raw_points.size() >= 2U);
	assert(weld.points_rxyz.header_matches_point_count());
	assert(weld.points_rxyz.geometry_point_count() >= 2U);
	assert(nearly_equal(weld.reference_point.x(), weld.raw_points.front().x()));
	assert(nearly_equal(weld.reference_point.y(), weld.raw_points.front().y()));
	assert(nearly_equal(weld.reference_point.z(), weld.raw_points.front().z()));
	assert(nearly_equal(weld.tangent_direction.vector().len(), 1.0, 1.0e-6));
}

// =============================================================================
// 真实模型焊缝测试：12 组组合，带计时与结果导出
// =============================================================================

void test_weld_with_real_models_and_export() {
	namespace fs = std::filesystem;
	using clock = std::chrono::steady_clock;

	// 基础路径
	const fs::path source_dir(HANFENG_SOURCE_DIR);
	const fs::path metadata_path = source_dir / "model" / "metadata.json";
	const fs::path result_dir = source_dir / "result";

	// 创建 result 目录
	fs::create_directories(result_dir);

	// 解析 metadata.json
	std::vector<PanelEntry> entries = parse_metadata(metadata_path);

	// 按类别分组
	std::vector<PanelEntry> plane_entries;
	std::vector<PanelEntry> surface_entries;
	for (const auto& e : entries) {
		if (e.category == "plane") {
			plane_entries.push_back(e);
		}
		else if (e.category == "surface") {
			surface_entries.push_back(e);
		}
	}

	std::cout << "平面板数量: " << plane_entries.size() << "\n";
	std::cout << "曲面板数量: " << surface_entries.size() << "\n";
	std::cout << "总组合数: " << (plane_entries.size() * surface_entries.size()) << "\n\n";

	std::vector<LoadedPlaneEntry> loaded_planes;
	loaded_planes.reserve(plane_entries.size());
	for (const PanelEntry& plane_entry : plane_entries) {
		LoadedPlaneEntry loaded;
		loaded.entry = plane_entry;

		auto t0 = clock::now();
		try {
			loaded.panel = hanfeng::api_get_plane_panel(plane_entry.directory);
		}
		catch (const std::exception& ex) {
			std::cerr << "ERROR loading plane: " << plane_entry.name
				<< ": " << ex.what() << "\n";
			continue;
		}
		auto t1 = clock::now();

		try {
			loaded.mesh = hanfeng::api_get_surface_panel(plane_entry.directory);
		}
		catch (const std::exception& ex) {
			std::cerr << "WARN loading plane mesh: " << plane_entry.name
				<< ": " << ex.what() << "\n";
		}
		auto t2 = clock::now();

		loaded.load_panel_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
		loaded.load_mesh_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
		loaded_planes.push_back(std::move(loaded));
	}

	std::vector<LoadedSurfaceEntry> loaded_surfaces;
	loaded_surfaces.reserve(surface_entries.size());
	for (const PanelEntry& surface_entry : surface_entries) {
		LoadedSurfaceEntry loaded;
		loaded.entry = surface_entry;

		auto t0 = clock::now();
		try {
			loaded.panel = hanfeng::api_get_surface_panel(surface_entry.directory);
		}
		catch (const std::exception& ex) {
			std::cerr << "ERROR loading surface: " << surface_entry.name
				<< ": " << ex.what() << "\n";
			continue;
		}
		auto t1 = clock::now();

		loaded.load_panel_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
		loaded_surfaces.push_back(std::move(loaded));
	}

	const std::size_t total_combination_count =
		loaded_planes.size() * loaded_surfaces.size();
	std::cout << "可测试平面板数量: " << loaded_planes.size() << "\n";
	std::cout << "可测试曲面板数量: " << loaded_surfaces.size() << "\n";
	std::cout << "可测试组合数: " << total_combination_count << "\n\n";

	// 遍历所有组合
	std::ostringstream json_content;
	json_content << "{\n";
	json_content << "  \"tolerance\": 1.0,\n";
	json_content << "  \"weld_fit_tolerance\": 0.001,\n";
	json_content << "  \"combination_count\": " << total_combination_count << ",\n";
	json_content << "  \"combinations\": [\n";

	bool first_combo = true;
	int combo_idx = 0;
	int exported_combo_idx = 0;

	for (const auto& plane_loaded : loaded_planes) {
		for (const auto& surface_loaded : loaded_surfaces) {
			++combo_idx;
			if (combo_idx == 1 || combo_idx % 100 == 0) {
				std::cout << "[" << combo_idx << "/" << total_combination_count
					<< "] 已测试，当前有焊缝组合: " << exported_combo_idx << "\n";
			}

			const PanelEntry& plane_entry = plane_loaded.entry;
			const PanelEntry& surface_entry = surface_loaded.entry;
			const hanfeng::PlanePanel& plane_panel = plane_loaded.panel;
			const hanfeng::SurfacePanel& surface_panel = surface_loaded.panel;
			const std::string combo_label =
				plane_entry.name + " x " + surface_entry.name;

			auto combo_start = clock::now();
			auto t2 = clock::now();

			// 计算焊缝
			hanfeng::WeldCurveResult weld_result;
			try {
				weld_result = hanfeng::api_get_plane_surface_weld(
					plane_panel, surface_panel, 1.0);
			}
			catch (const std::exception& ex) {
				std::cerr << "  ERROR computing weld: " << ex.what() << "\n";
				continue;
			}
			const hanfeng::WeldProfilingData profiling =
				hanfeng::hanfeng_get_weld_profiling_data();
			auto t3 = clock::now();

			if (weld_result.polylines.empty()) {
				continue;
			}

			const hanfeng::WeldSplineResult weld_splines =
				hanfeng::hanfeng_fit_weld_splines(weld_result, 1.0e-3);
			auto t4 = clock::now();

			auto combo_end = clock::now();
			double ms_load_plane = plane_loaded.load_panel_ms + plane_loaded.load_mesh_ms;
			double ms_load_surface = surface_loaded.load_panel_ms;
			double ms_compute_weld = std::chrono::duration<double, std::milli>(t3 - t2).count();
			double ms_fit_weld = std::chrono::duration<double, std::milli>(t4 - t3).count();
			double ms_total = std::chrono::duration<double, std::milli>(combo_end - combo_start).count();

			++exported_combo_idx;
			std::cout << "[" << combo_idx << "/" << total_combination_count
				<< "] " << combo_label << "\n";
			std::cout << "  命中焊缝组合序号: " << exported_combo_idx << "\n";
			std::cout << "  预加载平面板: " << ms_load_plane << " ms\n";
			std::cout << "  预加载曲面板: " << ms_load_surface << " ms\n";
			std::cout << "  计算焊缝:   " << ms_compute_weld << " ms\n";
			std::cout << "  总耗时:     " << ms_total << " ms\n";
			std::cout << "  焊缝段数:   " << weld_result.polylines.size() << "\n";
			std::cout << "  Profiling(total/build/clip_plane): "
				<< profiling.total_ms << " / "
				<< (profiling.build_plane_geometry_ms + profiling.build_surface_geometry_ms)
				<< " / " << profiling.clip_plane_candidates_ms << " ms\n";
			std::cout << "  rxyz fit:     " << ms_fit_weld << " ms\n";
			std::cout << "  Profiling(topo/inside/near): "
				<< profiling.topo_coplanar_ms << " / "
				<< profiling.inside_ms << " / "
				<< profiling.near_ms << " ms\n";
			std::cout << "  Profiling(loop_count plane/surface): "
				<< profiling.plane_candidate_loop_count << " / "
				<< profiling.surface_candidate_loop_count << "\n";
			std::cout << "  Profiling(segment_count plane/surface): "
				<< profiling.plane_candidate_segment_count << " / "
				<< profiling.surface_candidate_segment_count << "\n";
			std::cout << "  Profiling(point_in_solid/events calls): "
				<< profiling.point_in_solid_call_count << " / "
				<< profiling.segment_solid_event_call_count << "\n\n";

			// 写入 JSON
			if (!first_combo) json_content << ",\n";
			first_combo = false;

			json_content << "    {\n";
			json_content << "      \"plane_panel\": {\"name\": \"" << plane_entry.name
				<< "\", \"path\": \"" << plane_entry.directory << "\"},\n";
			json_content << "      \"surface_panel\": {\"name\": \"" << surface_entry.name
				<< "\", \"path\": \"" << surface_entry.directory << "\"},\n";

			// 焊缝结果
			json_content << "      \"weld_result\": {\n";
			json_content << "        \"polyline_count\": " << weld_result.polylines.size() << ",\n";
			json_content << "        \"polylines\": [\n";
			for (std::size_t wi = 0; wi < weld_result.polylines.size(); ++wi) {
				const auto& pl = weld_result.polylines[wi];
				const auto& spline = weld_splines.welds[wi];
				json_content << "          {\"index\": " << wi
					<< ", \"point_count\": " << pl.points.size() << ",\n";
				json_content << "           \"points\": ";
				std::ofstream mock_out;  // 用于 write_json_polyline3 的接口复用
				// 直接写入 json_content
				json_content << "[\n";
				for (std::size_t pi = 0; pi < pl.points.size(); ++pi) {
					json_content << "            [" << pl.points[pi].x() << ", "
						<< pl.points[pi].y() << ", "
						<< pl.points[pi].z() << "]";
					if (pi + 1 < pl.points.size()) json_content << ",";
					json_content << "\n";
				}
				json_content << "          ],\n";
				json_content << "           \"points_rxyz\": [\n";
				for (std::size_t ri = 0; ri < spline.points_rxyz.points.size(); ++ri) {
					const auto& point = spline.points_rxyz.points[ri];
					json_content << "            [" << point.r << ", "
						<< point.x << ", "
						<< point.y << ", "
						<< point.z << "]";
					if (ri + 1 < spline.points_rxyz.points.size()) json_content << ",";
					json_content << "\n";
				}
				json_content << "          ],\n";
				json_content << "           \"reference_point\": ["
					<< spline.reference_point.x() << ", "
					<< spline.reference_point.y() << ", "
					<< spline.reference_point.z() << "],\n";
				json_content << "           \"tangent_direction\": ["
					<< spline.tangent_direction.x() << ", "
					<< spline.tangent_direction.y() << ", "
					<< spline.tangent_direction.z() << "]}\n";
				if (wi + 1 < weld_result.polylines.size()) json_content << ",";
			}
			json_content << "        ]\n";
			json_content << "      },\n";

			// 平面板几何（mesh + 边界）
			{
				const hanfeng::SurfacePanel& plane_mesh = plane_loaded.mesh;
				std::vector<std::array<double, 3>> pv;
				std::vector<std::array<std::size_t, 3>> ptt;
				for (const auto& patch : plane_mesh.surfaces) {
					std::size_t offset = pv.size();
					for (const auto& v : patch.vertices) pv.push_back({ v.x(), v.y(), v.z() });
					for (const auto& tri : patch.triangles) ptt.push_back({ offset + tri[0], offset + tri[1], offset + tri[2] });
				}
				json_content << "      \"plane_vertices\": [\n";
				for (std::size_t i = 0; i < pv.size(); ++i) {
					json_content << "        [" << pv[i][0] << ", " << pv[i][1] << ", " << pv[i][2] << "]";
					if (i + 1 < pv.size()) json_content << ",";
					json_content << "\n";
				}
				json_content << "      ],\n";
				json_content << "      \"plane_triangles\": [\n";
				for (std::size_t i = 0; i < ptt.size(); ++i) {
					json_content << "        [" << ptt[i][0] << ", " << ptt[i][1] << ", " << ptt[i][2] << "]";
					if (i + 1 < ptt.size()) json_content << ",";
					json_content << "\n";
				}
				json_content << "      ],\n";
				json_content << "      \"plane_boundary_loops\": ";
				bool first_loop = true;
				json_content << "[\n";
				auto write_loops = [&](const std::vector<hanfeng::Polyline3>& loops) {
					for (const auto& loop : loops) {
						if (!first_loop) json_content << ",\n";
						first_loop = false;
						json_content << "        [";
						for (std::size_t pi = 0; pi < loop.size(); ++pi) {
							json_content << "[" << loop[pi].x() << "," << loop[pi].y() << "," << loop[pi].z() << "]";
							if (pi + 1 < loop.size()) json_content << ", ";
						}
						json_content << "]";
					}
					};
				write_loops(plane_panel.face_a.boundaries.outer_loops);
				write_loops(plane_panel.face_a.boundaries.inner_loops);
				write_loops(plane_panel.face_b.boundaries.outer_loops);
				write_loops(plane_panel.face_b.boundaries.inner_loops);
				json_content << "\n      ],\n";
			}

			// 曲面板几何
			{
				// 收集所有 patch 的顶点和三角形
				std::vector<std::array<double, 3>> all_verts;
				std::vector<std::array<std::size_t, 3>> all_tris;
				std::vector<std::vector<std::array<double, 3>>> all_boundary;

				for (const auto& patch : surface_panel.surfaces) {
					std::size_t offset = all_verts.size();
					for (const auto& v : patch.vertices) {
						all_verts.push_back({ v.x(), v.y(), v.z() });
					}
					for (const auto& tri : patch.triangles) {
						all_tris.push_back({ offset + tri[0], offset + tri[1], offset + tri[2] });
					}
					auto collect = [&](const std::vector<hanfeng::Polyline3>& loops) {
						for (const auto& loop : loops) {
							std::vector<std::array<double, 3>> pts;
							for (const auto& p : loop) pts.push_back({ p.x(), p.y(), p.z() });
							all_boundary.push_back(std::move(pts));
						}
						};
					collect(patch.boundaries.outer_loops);
					collect(patch.boundaries.inner_loops);
				}

				// vertices
				json_content << "      \"surface_vertices\": [\n";
				for (std::size_t i = 0; i < all_verts.size(); ++i) {
					json_content << "        [" << all_verts[i][0] << ", " << all_verts[i][1] << ", " << all_verts[i][2] << "]";
					if (i + 1 < all_verts.size()) json_content << ",";
					json_content << "\n";
				}
				json_content << "      ],\n";

				// triangles
				json_content << "      \"surface_triangles\": [\n";
				for (std::size_t i = 0; i < all_tris.size(); ++i) {
					json_content << "        [" << all_tris[i][0] << ", " << all_tris[i][1] << ", " << all_tris[i][2] << "]";
					if (i + 1 < all_tris.size()) json_content << ",";
					json_content << "\n";
				}
				json_content << "      ],\n";

				// boundary loops
				json_content << "      \"surface_boundary_loops\": [\n";
				for (std::size_t li = 0; li < all_boundary.size(); ++li) {
					json_content << "        [";
					for (std::size_t pi = 0; pi < all_boundary[li].size(); ++pi) {
						json_content << "[" << all_boundary[li][pi][0] << ","
							<< all_boundary[li][pi][1] << ","
							<< all_boundary[li][pi][2] << "]";
						if (pi + 1 < all_boundary[li].size()) json_content << ", ";
					}
					json_content << "]";
					if (li + 1 < all_boundary.size()) json_content << ",";
					json_content << "\n";
				}
				json_content << "      ],\n";
			}

			// 计时
			json_content << "      \"timing_ms\": {\n";
			json_content << "        \"load_plane\": " << ms_load_plane << ",\n";
			json_content << "        \"load_surface\": " << ms_load_surface << ",\n";
			json_content << "        \"compute_weld\": " << ms_compute_weld << ",\n";
			json_content << "        \"total\": " << ms_total << "\n";
			json_content << "      },\n";
			json_content << "      \"profiling\": {\n";
			json_content << "        \"build_plane_geometry_ms\": "
				<< profiling.build_plane_geometry_ms << ",\n";
			json_content << "        \"build_surface_geometry_ms\": "
				<< profiling.build_surface_geometry_ms << ",\n";
			json_content << "        \"collect_surface_candidates_ms\": "
				<< profiling.collect_surface_candidates_ms << ",\n";
			json_content << "        \"collect_plane_candidates_ms\": "
				<< profiling.collect_plane_candidates_ms << ",\n";
			json_content << "        \"clip_surface_candidates_ms\": "
				<< profiling.clip_surface_candidates_ms << ",\n";
			json_content << "        \"clip_plane_candidates_ms\": "
				<< profiling.clip_plane_candidates_ms << ",\n";
			json_content << "        \"deduplicate_ms\": "
				<< profiling.deduplicate_ms << ",\n";
			json_content << "        \"total_ms\": "
				<< profiling.total_ms << ",\n";
			json_content << "        \"topo_coplanar_ms\": "
				<< profiling.topo_coplanar_ms << ",\n";
			json_content << "        \"inside_ms\": "
				<< profiling.inside_ms << ",\n";
			json_content << "        \"near_ms\": "
				<< profiling.near_ms << ",\n";
			json_content << "        \"point_in_solid_ms\": "
				<< profiling.point_in_solid_ms << ",\n";
			json_content << "        \"segment_solid_events_ms\": "
				<< profiling.segment_solid_events_ms << ",\n";
			json_content << "        \"surface_triangle_count\": "
				<< profiling.surface_triangle_count << ",\n";
			json_content << "        \"plane_candidate_loop_count\": "
				<< profiling.plane_candidate_loop_count << ",\n";
			json_content << "        \"surface_candidate_loop_count\": "
				<< profiling.surface_candidate_loop_count << ",\n";
			json_content << "        \"plane_candidate_segment_count\": "
				<< profiling.plane_candidate_segment_count << ",\n";
			json_content << "        \"surface_candidate_segment_count\": "
				<< profiling.surface_candidate_segment_count << ",\n";
			json_content << "        \"topo_query_count\": "
				<< profiling.topo_query_count << ",\n";
			json_content << "        \"inside_query_count\": "
				<< profiling.inside_query_count << ",\n";
			json_content << "        \"near_query_count\": "
				<< profiling.near_query_count << ",\n";
			json_content << "        \"point_in_solid_call_count\": "
				<< profiling.point_in_solid_call_count << ",\n";
			json_content << "        \"segment_solid_event_call_count\": "
				<< profiling.segment_solid_event_call_count << ",\n";
			json_content << "        \"topo_triangle_tests\": "
				<< profiling.topo_triangle_tests << ",\n";
			json_content << "        \"near_triangle_tests\": "
				<< profiling.near_triangle_tests << ",\n";
			json_content << "        \"segment_event_triangle_tests\": "
				<< profiling.segment_event_triangle_tests << ",\n";
			json_content << "        \"point_in_solid_boundary_triangle_tests\": "
				<< profiling.point_in_solid_boundary_triangle_tests << ",\n";
			json_content << "        \"point_in_solid_ray_triangle_tests\": "
				<< profiling.point_in_solid_ray_triangle_tests << "\n";
			json_content << "      }\n";

			json_content << "    }";
		}
	}

	json_content << "\n  ],\n";
	json_content << "  \"tested_combination_count\": " << combo_idx << ",\n";
	json_content << "  \"exported_combination_count\": " << exported_combo_idx << ",\n";
	json_content << "  \"export_filter\": \"weld_result.polyline_count > 0\"\n";
	json_content << "}\n";

	// 写入 JS 数据文件（供 HTML 通过 <script> 标签加载，兼容 file:// 协议）
	const fs::path js_path = result_dir / "weld_results.js";
	{
		std::ofstream js_file(js_path);
		if (!js_file.is_open()) {
			std::cerr << "Failed to write: " << js_path << "\n";
		}
		else {
			js_file << "var WELD_DATA = " << json_content.str() << ";\n";
			js_file.close();
			std::cout << "JS 数据已写入: " << js_path << "\n";
		}
	}

	// 写入 JSON 文件（保留，供其他工具使用）
	const fs::path json_path = result_dir / "weld_results.json";
	{
		std::ofstream json_file(json_path);
		if (!json_file.is_open()) {
			std::cerr << "Failed to write: " << json_path << "\n";
		}
		else {
			json_file << json_content.str();
			json_file.close();
			std::cout << "JSON 结果已写入: " << json_path << "\n";
		}
	}

	// 生成 HTML 文件（内联数据，兼容 file:// 协议）
	const fs::path html_path = result_dir / "weld_viewer.html";
	write_weld_viewer_html(html_path, json_content.str());
	std::cout << "HTML 可视化已写入: " << html_path << "\n";

	// 验证结果
	assert(fs::exists(json_path));
	assert(fs::exists(html_path));
	assert(combo_idx > 0);
	assert(exported_combo_idx >= 0);
	std::cout << "\n全部 " << combo_idx << " 组焊缝计算完成。\n";
	std::cout << "HTML/JSON 仅导出 " << exported_combo_idx
		<< " 组存在焊缝的结果。\n";
}

void test_surface_surface_weld_with_real_models() {
	namespace fs = std::filesystem;
	using clock = std::chrono::steady_clock;

	const fs::path source_dir(HANFENG_SOURCE_DIR);
	const fs::path metadata_path = source_dir / "model" / "metadata.json";
	const fs::path result_dir = source_dir / "result";
	fs::create_directories(result_dir);

	std::vector<PanelEntry> entries = parse_metadata(metadata_path);
	std::vector<PanelEntry> surface_entries;
	for (const auto& entry : entries) {
		if (entry.category == "surface") {
			surface_entries.push_back(entry);
		}
	}

	std::cout << "曲面板数量: " << surface_entries.size() << "\n";
	const std::size_t declared_combination_count = surface_entries.size() < 2U
		? 0U
		: surface_entries.size() * (surface_entries.size() - 1U) / 2U;
	std::cout << "曲面板配对组合数: "
		<< declared_combination_count << "\n\n";

	std::vector<LoadedSurfaceEntry> loaded_surfaces;
	loaded_surfaces.reserve(surface_entries.size());
	for (const PanelEntry& surface_entry : surface_entries) {
		LoadedSurfaceEntry loaded;
		loaded.entry = surface_entry;

		auto t0 = clock::now();
		try {
			loaded.panel = hanfeng::api_get_surface_panel(surface_entry.directory);
		}
		catch (const std::exception& ex) {
			std::cerr << "ERROR loading surface: " << surface_entry.name
				<< ": " << ex.what() << "\n";
			continue;
		}
		auto t1 = clock::now();

		loaded.load_panel_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
		loaded_surfaces.push_back(std::move(loaded));
	}

	const std::size_t total_combination_count =
		loaded_surfaces.size() < 2U
			? 0U
			: loaded_surfaces.size() * (loaded_surfaces.size() - 1U) / 2U;
	std::cout << "可测试曲面板数量: " << loaded_surfaces.size() << "\n";
	std::cout << "可测试曲面板配对组合数: " << total_combination_count << "\n\n";

	std::ostringstream json_content;
	json_content << "{\n";
	json_content << "  \"tolerance\": 1.0,\n";
	json_content << "  \"weld_fit_tolerance\": 0.001,\n";
	json_content << "  \"combination_count\": " << total_combination_count << ",\n";
	json_content << "  \"combination_mode\": \"surface_surface_unique_pairs\",\n";
	json_content << "  \"export_filter\": \"weld_result.polyline_count > 0\",\n";
	json_content << "  \"combinations\": [\n";

	bool first_combo = true;
	std::size_t combo_idx = 0;
	std::size_t exported_combo_idx = 0;

	for (std::size_t first_index = 0; first_index < loaded_surfaces.size(); ++first_index) {
		for (std::size_t second_index = first_index + 1U;
			second_index < loaded_surfaces.size(); ++second_index) {
			++combo_idx;
			if (combo_idx == 1U || combo_idx % 100U == 0U) {
				std::cout << "[" << combo_idx << "/" << total_combination_count
					<< "] 已测试，当前有焊缝组合: " << exported_combo_idx << "\n";
			}

			const LoadedSurfaceEntry& first_loaded = loaded_surfaces[first_index];
			const LoadedSurfaceEntry& second_loaded = loaded_surfaces[second_index];
			const std::string combo_label =
				first_loaded.entry.name + " x " + second_loaded.entry.name;

			auto combo_start = clock::now();
			auto t0 = clock::now();
			hanfeng::WeldCurveResult weld_result;
			try {
				weld_result = hanfeng::api_get_surface_surface_weld(
					first_loaded.panel, second_loaded.panel, 1.0);
			}
			catch (const std::exception& ex) {
				std::cerr << "  ERROR computing surface-surface weld: "
					<< combo_label << ": " << ex.what() << "\n";
				continue;
			}
			const hanfeng::WeldProfilingData profiling =
				hanfeng::hanfeng_get_weld_profiling_data();
			auto t1 = clock::now();

			if (weld_result.polylines.empty()) {
				continue;
			}

			const hanfeng::WeldSplineResult weld_splines =
				hanfeng::hanfeng_fit_weld_splines(weld_result, 1.0e-3);
			auto t2 = clock::now();
			assert(weld_splines.welds.size() == weld_result.polylines.size());

			auto combo_end = clock::now();
			const double ms_compute_weld =
				std::chrono::duration<double, std::milli>(t1 - t0).count();
			const double ms_fit_weld =
				std::chrono::duration<double, std::milli>(t2 - t1).count();
			const double ms_total =
				std::chrono::duration<double, std::milli>(combo_end - combo_start).count();

			++exported_combo_idx;
			std::cout << "[" << combo_idx << "/" << total_combination_count
				<< "] " << combo_label << "\n";
			std::cout << "  命中焊缝组合序号: " << exported_combo_idx << "\n";
			std::cout << "  预加载曲面板1: " << first_loaded.load_panel_ms << " ms\n";
			std::cout << "  预加载曲面板2: " << second_loaded.load_panel_ms << " ms\n";
			std::cout << "  计算焊缝:   " << ms_compute_weld << " ms\n";
			std::cout << "  rxyz fit:   " << ms_fit_weld << " ms\n";
			std::cout << "  总耗时:     " << ms_total << " ms\n";
			std::cout << "  焊缝段数:   " << weld_result.polylines.size() << "\n";
			std::cout << "  Profiling(total/build/clip_surface): "
				<< profiling.total_ms << " / "
				<< profiling.build_surface_geometry_ms << " / "
				<< profiling.clip_surface_candidates_ms << " ms\n";
			std::cout << "  Profiling(topo/inside/near): "
				<< profiling.topo_coplanar_ms << " / "
				<< profiling.inside_ms << " / "
				<< profiling.near_ms << " ms\n";
			std::cout << "  Profiling(surface loops/segments): "
				<< profiling.surface_candidate_loop_count << " / "
				<< profiling.surface_candidate_segment_count << "\n\n";

			if (!first_combo) json_content << ",\n";
			first_combo = false;

			json_content << "    {\n";
			json_content << "      \"first_surface_panel\": {\"name\": \""
				<< first_loaded.entry.name << "\", \"path\": \""
				<< first_loaded.entry.directory << "\"},\n";
			json_content << "      \"second_surface_panel\": {\"name\": \""
				<< second_loaded.entry.name << "\", \"path\": \""
				<< second_loaded.entry.directory << "\"},\n";
			// 第一个曲面板几何
				{
					const hanfeng::SurfacePanel& panel = first_loaded.panel;
					std::vector<std::array<double, 3>> all_verts;
					std::vector<std::array<std::size_t, 3>> all_tris;
					std::vector<std::vector<std::array<double, 3>>> all_boundary;

					for (const auto& patch : panel.surfaces) {
						std::size_t offset = all_verts.size();
						for (const auto& v : patch.vertices)
							all_verts.push_back({ v.x(), v.y(), v.z() });
						for (const auto& tri : patch.triangles)
							all_tris.push_back({ offset + tri[0], offset + tri[1], offset + tri[2] });
						auto collect = [&](const std::vector<hanfeng::Polyline3>& loops) {
							for (const auto& loop : loops) {
								std::vector<std::array<double, 3>> pts;
								for (const auto& p : loop) pts.push_back({ p.x(), p.y(), p.z() });
								all_boundary.push_back(std::move(pts));
							}
						};
						collect(patch.boundaries.outer_loops);
						collect(patch.boundaries.inner_loops);
					}

					json_content << "      \"first_vertices\": [\n";
					for (std::size_t i = 0; i < all_verts.size(); ++i) {
						json_content << "        [" << all_verts[i][0] << ", " << all_verts[i][1] << ", " << all_verts[i][2] << "]";
						if (i + 1 < all_verts.size()) json_content << ",";
						json_content << "\n";
					}
					json_content << "      ],\n";
					json_content << "      \"first_triangles\": [\n";
					for (std::size_t i = 0; i < all_tris.size(); ++i) {
						json_content << "        [" << all_tris[i][0] << ", " << all_tris[i][1] << ", " << all_tris[i][2] << "]";
						if (i + 1 < all_tris.size()) json_content << ",";
						json_content << "\n";
					}
					json_content << "      ],\n";
					json_content << "      \"first_boundary_loops\": [\n";
					for (std::size_t li = 0; li < all_boundary.size(); ++li) {
						json_content << "        [";
						for (std::size_t pi = 0; pi < all_boundary[li].size(); ++pi) {
							json_content << "[" << all_boundary[li][pi][0] << ","
								<< all_boundary[li][pi][1] << ","
								<< all_boundary[li][pi][2] << "]";
							if (pi + 1 < all_boundary[li].size()) json_content << ", ";
						}
						json_content << "]";
						if (li + 1 < all_boundary.size()) json_content << ",";
						json_content << "\n";
					}
					json_content << "      ],\n";
				}

				// 第二个曲面板几何
				{
					const hanfeng::SurfacePanel& panel = second_loaded.panel;
					std::vector<std::array<double, 3>> all_verts;
					std::vector<std::array<std::size_t, 3>> all_tris;
					std::vector<std::vector<std::array<double, 3>>> all_boundary;

					for (const auto& patch : panel.surfaces) {
						std::size_t offset = all_verts.size();
						for (const auto& v : patch.vertices)
							all_verts.push_back({ v.x(), v.y(), v.z() });
						for (const auto& tri : patch.triangles)
							all_tris.push_back({ offset + tri[0], offset + tri[1], offset + tri[2] });
						auto collect = [&](const std::vector<hanfeng::Polyline3>& loops) {
							for (const auto& loop : loops) {
								std::vector<std::array<double, 3>> pts;
								for (const auto& p : loop) pts.push_back({ p.x(), p.y(), p.z() });
								all_boundary.push_back(std::move(pts));
							}
						};
						collect(patch.boundaries.outer_loops);
						collect(patch.boundaries.inner_loops);
					}

					json_content << "      \"second_vertices\": [\n";
					for (std::size_t i = 0; i < all_verts.size(); ++i) {
						json_content << "        [" << all_verts[i][0] << ", " << all_verts[i][1] << ", " << all_verts[i][2] << "]";
						if (i + 1 < all_verts.size()) json_content << ",";
						json_content << "\n";
					}
					json_content << "      ],\n";
					json_content << "      \"second_triangles\": [\n";
					for (std::size_t i = 0; i < all_tris.size(); ++i) {
						json_content << "        [" << all_tris[i][0] << ", " << all_tris[i][1] << ", " << all_tris[i][2] << "]";
						if (i + 1 < all_tris.size()) json_content << ",";
						json_content << "\n";
					}
					json_content << "      ],\n";
					json_content << "      \"second_boundary_loops\": [\n";
					for (std::size_t li = 0; li < all_boundary.size(); ++li) {
						json_content << "        [";
						for (std::size_t pi = 0; pi < all_boundary[li].size(); ++pi) {
							json_content << "[" << all_boundary[li][pi][0] << ","
								<< all_boundary[li][pi][1] << ","
								<< all_boundary[li][pi][2] << "]";
							if (pi + 1 < all_boundary[li].size()) json_content << ", ";
						}
						json_content << "]";
						if (li + 1 < all_boundary.size()) json_content << ",";
						json_content << "\n";
					}
					json_content << "      ],\n";
				}

				json_content << "      \"weld_result\": {\n";
			json_content << "        \"polyline_count\": "
				<< weld_result.polylines.size() << ",\n";
			json_content << "        \"polylines\": [\n";
			for (std::size_t wi = 0; wi < weld_result.polylines.size(); ++wi) {
				const auto& polyline = weld_result.polylines[wi];
				const auto& spline = weld_splines.welds[wi];
				json_content << "          {\"index\": " << wi
					<< ", \"point_count\": " << polyline.points.size() << ",\n";
				json_content << "           \"points\": [\n";
				for (std::size_t pi = 0; pi < polyline.points.size(); ++pi) {
					json_content << "            [" << polyline.points[pi].x() << ", "
						<< polyline.points[pi].y() << ", "
						<< polyline.points[pi].z() << "]";
					if (pi + 1U < polyline.points.size()) json_content << ",";
					json_content << "\n";
				}
				json_content << "          ],\n";
				json_content << "           \"points_rxyz\": [\n";
				for (std::size_t ri = 0; ri < spline.points_rxyz.points.size(); ++ri) {
					const auto& point = spline.points_rxyz.points[ri];
					json_content << "            [" << point.r << ", "
						<< point.x << ", "
						<< point.y << ", "
						<< point.z << "]";
					if (ri + 1U < spline.points_rxyz.points.size()) json_content << ",";
					json_content << "\n";
				}
				json_content << "          ],\n";
				json_content << "           \"reference_point\": ["
					<< spline.reference_point.x() << ", "
					<< spline.reference_point.y() << ", "
					<< spline.reference_point.z() << "],\n";
				json_content << "           \"tangent_direction\": ["
					<< spline.tangent_direction.x() << ", "
					<< spline.tangent_direction.y() << ", "
					<< spline.tangent_direction.z() << "]}\n";
				if (wi + 1U < weld_result.polylines.size()) json_content << ",";
			}
			json_content << "        ]\n";
			json_content << "      },\n";
			json_content << "      \"timing_ms\": {\n";
			json_content << "        \"load_first_surface\": "
				<< first_loaded.load_panel_ms << ",\n";
			json_content << "        \"load_second_surface\": "
				<< second_loaded.load_panel_ms << ",\n";
			json_content << "        \"compute_weld\": " << ms_compute_weld << ",\n";
			json_content << "        \"fit_weld\": " << ms_fit_weld << ",\n";
			json_content << "        \"total\": " << ms_total << "\n";
			json_content << "      },\n";
			json_content << "      \"profiling\": {\n";
			json_content << "        \"build_surface_geometry_ms\": "
				<< profiling.build_surface_geometry_ms << ",\n";
			json_content << "        \"collect_surface_candidates_ms\": "
				<< profiling.collect_surface_candidates_ms << ",\n";
			json_content << "        \"clip_surface_candidates_ms\": "
				<< profiling.clip_surface_candidates_ms << ",\n";
			json_content << "        \"deduplicate_ms\": "
				<< profiling.deduplicate_ms << ",\n";
			json_content << "        \"total_ms\": "
				<< profiling.total_ms << ",\n";
			json_content << "        \"topo_coplanar_ms\": "
				<< profiling.topo_coplanar_ms << ",\n";
			json_content << "        \"inside_ms\": "
				<< profiling.inside_ms << ",\n";
			json_content << "        \"near_ms\": "
				<< profiling.near_ms << ",\n";
			json_content << "        \"surface_triangle_count\": "
				<< profiling.surface_triangle_count << ",\n";
			json_content << "        \"surface_candidate_loop_count\": "
				<< profiling.surface_candidate_loop_count << ",\n";
			json_content << "        \"surface_candidate_segment_count\": "
				<< profiling.surface_candidate_segment_count << "\n";
			json_content << "      }\n";
			json_content << "    }";
		}
	}

	json_content << "\n  ],\n";
	json_content << "  \"tested_combination_count\": " << combo_idx << ",\n";
	json_content << "  \"exported_combination_count\": " << exported_combo_idx << "\n";
	json_content << "}\n";

	const fs::path json_path = result_dir / "surface_surface_weld_results.json";
	{
		std::ofstream json_file(json_path);
		if (!json_file.is_open()) {
			std::cerr << "Failed to write: " << json_path << "\n";
		}
		else {
			json_file << json_content.str();
			json_file.close();
			std::cout << "JSON 结果已写入: " << json_path << "\n";
		}
	}

	const fs::path js_path = result_dir / "surface_surface_weld_results.js";
	{
		std::ofstream js_file(js_path);
		if (!js_file.is_open()) {
			std::cerr << "Failed to write: " << js_path << "\n";
		}
		else {
			js_file << "var SURFACE_SURFACE_WELD_DATA = "
				<< json_content.str() << ";\n";
			js_file.close();
			std::cout << "JS 数据已写入: " << js_path << "\n";
		}
	}

	const fs::path html_path = result_dir / "surface_surface_weld_viewer.html";
	write_surface_surface_weld_viewer_html(html_path, json_content.str());
	std::cout << "HTML 可视化已写入: " << html_path << "\n";

	assert(fs::exists(json_path));
	assert(fs::exists(js_path));
	assert(fs::exists(html_path));
	assert(combo_idx == total_combination_count);
	std::cout << "\n全部 " << combo_idx << " 组曲面板配对焊缝计算完成。\n";
	std::cout << "HTML/JSON/JS 仅导出 " << exported_combo_idx
		<< " 组存在焊缝的结果。\n";
}
