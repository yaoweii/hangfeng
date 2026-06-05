// =============================================================================
// hafeng_geometry.hpp - 焊缝几何计算模块
// =============================================================================
//
// 这个文件承载“平面板/曲面板焊缝提取”的纯几何接口。
// `hanfeng_util` 负责模型读取，这里只负责几何计算。
// =============================================================================

#ifndef HANFENG_PLANE_SURFACE_HANFENG_UTIL_HPP
#define HANFENG_PLANE_SURFACE_HANFENG_UTIL_HPP

#include <cstddef>

#include "hanfeng/panel.hpp"
#include "hanfeng/weld.hpp"

namespace hanfeng {

	struct WeldProfilingData {
		double build_plane_geometry_ms = 0.0;
		double build_surface_geometry_ms = 0.0;
		double collect_surface_candidates_ms = 0.0;
		double collect_plane_candidates_ms = 0.0;
		double clip_surface_candidates_ms = 0.0;
		double clip_plane_candidates_ms = 0.0;
		double deduplicate_ms = 0.0;
		double total_ms = 0.0;

		double topo_coplanar_ms = 0.0;
		double inside_ms = 0.0;
		double near_ms = 0.0;
		double point_in_solid_ms = 0.0;
		double segment_solid_events_ms = 0.0;

		std::size_t surface_triangle_count = 0;
		std::size_t plane_candidate_loop_count = 0;
		std::size_t surface_candidate_loop_count = 0;
		std::size_t plane_candidate_segment_count = 0;
		std::size_t surface_candidate_segment_count = 0;
		std::size_t topo_query_count = 0;
		std::size_t inside_query_count = 0;
		std::size_t near_query_count = 0;
		std::size_t point_in_solid_call_count = 0;
		std::size_t segment_solid_event_call_count = 0;
		std::size_t topo_triangle_tests = 0;
		std::size_t near_triangle_tests = 0;
		std::size_t segment_event_triangle_tests = 0;
		std::size_t point_in_solid_boundary_triangle_tests = 0;
		std::size_t point_in_solid_ray_triangle_tests = 0;
	};

	/**
	 * @brief 计算平面板与曲面板之间的焊缝折线结果。
	 *
	 * 当前算法语义：
	 * - 焊缝候选只来自两块板的边界线
	 * - 命中判定基于另一块板的有效表面
	 * - 返回值中的每条 `WeldPolyline` 都是连续的三维折线
	 *
	 * @param plane_panel    平面板
	 * @param surface_panel  曲面板
	 * @param tolerance      命中容差，单位 mm，默认 1.0
	 * @return WeldCurveResult 全部焊缝折线
	 */
	WeldCurveResult hanfeng_compute_plane_surface_weld(
		const PlanePanel& plane_panel,
		const SurfacePanel& surface_panel,
		double tolerance = 1.0);

	void hanfeng_reset_weld_profiling_data();
	WeldProfilingData hanfeng_get_weld_profiling_data();

}  // namespace hanfeng

#endif
