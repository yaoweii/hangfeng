// =============================================================================
// plane_surface_hanfeng_util.hpp — 平面板/曲面板焊缝计算模块
// =============================================================================
//
// 提供平面板与曲面板之间的焊缝折线计算接口及性能分析数据。
// 模型读取由 panel 模块负责，本文件只负责焊缝几何计算。
// =============================================================================

#ifndef HANFENG_PLANE_SURFACE_HANFENG_UTIL_HPP
#define HANFENG_PLANE_SURFACE_HANFENG_UTIL_HPP

#include <cstddef>

#include "hanfeng/panel.hpp"
#include "hanfeng/weld.hpp"

namespace hanfeng {

	/// 焊缝计算性能分析数据（各阶段耗时 + 计数器）
	struct WeldProfilingData {
		// --- 各阶段耗时（毫秒） ---
		double build_plane_geometry_ms = 0.0;        ///< 构建平面板几何耗时
		double build_surface_geometry_ms = 0.0;      ///< 构建曲面板几何耗时
		double collect_surface_candidates_ms = 0.0;  ///< 收集曲面候选环耗时
		double collect_plane_candidates_ms = 0.0;    ///< 收集平面候选环耗时
		double clip_surface_candidates_ms = 0.0;     ///< 裁剪曲面候选环耗时
		double clip_plane_candidates_ms = 0.0;       ///< 裁剪平面候选环耗时
		double deduplicate_ms = 0.0;                  ///< 焊缝折线去重耗时
		double total_ms = 0.0;                        ///< 焊缝计算总耗时

		// --- 子阶段耗时（毫秒） ---
		double topo_coplanar_ms = 0.0;            ///< 拓扑共面检测耗时
		double inside_ms = 0.0;                    ///< 体内判定耗时
		double near_ms = 0.0;                      ///< 近距检测耗时
		double point_in_solid_ms = 0.0;            ///< 点在体内检测总耗时
		double segment_solid_events_ms = 0.0;      ///< 线段-体交点事件检测总耗时

		// --- 计数器 ---
		std::size_t surface_triangle_count = 0;                         ///< 曲面三角形总数
		std::size_t plane_candidate_loop_count = 0;                     ///< 平面候选环数
		std::size_t surface_candidate_loop_count = 0;                   ///< 曲面候选环数
		std::size_t plane_candidate_segment_count = 0;                  ///< 平面候选线段总数
		std::size_t surface_candidate_segment_count = 0;                ///< 曲面候选线段总数
		std::size_t topo_query_count = 0;                                ///< 拓扑共面查询次数
		std::size_t inside_query_count = 0;                              ///< 体内判定查询次数
		std::size_t near_query_count = 0;                                ///< 近距查询次数
		std::size_t point_in_solid_call_count = 0;                       ///< point_in_surface_solid 调用次数
		std::size_t segment_solid_event_call_count = 0;                  ///< segment_surface_solid_events 调用次数
		std::size_t topo_triangle_tests = 0;                             ///< 拓扑共面-三角形测试次数
		std::size_t near_triangle_tests = 0;                             ///< 近距-三角形测试次数
		std::size_t segment_event_triangle_tests = 0;                    ///< 交点事件-三角形测试次数
		std::size_t point_in_solid_boundary_triangle_tests = 0;          ///< 边界检测-三角形测试次数
		std::size_t point_in_solid_ray_triangle_tests = 0;               ///< 射线检测-三角形测试次数
	};

	/// 计算平面板与曲面板之间的焊缝折线结果。
	/// 焊缝候选来自两块板的边界线，命中判定基于另一块板的有效表面。
	/// @param plane_panel    平面板
	/// @param surface_panel  曲面板
	/// @param tolerance      命中容差（mm），默认 1.0
	/// @return 全部焊缝折线
	WeldCurveResult hanfeng_compute_plane_surface_weld(
		const PlanePanel& plane_panel,
		const SurfacePanel& surface_panel,
		double tolerance = 1.0);

	WeldSplineResult hanfeng_fit_weld_splines(
		const WeldCurveResult& weld_result,
		double tolerance = 1.0e-3);

	/// 重置焊缝性能分析数据
	void hanfeng_reset_weld_profiling_data();

	/// 获取当前焊缝性能分析数据的副本
	WeldProfilingData hanfeng_get_weld_profiling_data();

}  // namespace hanfeng

#endif
