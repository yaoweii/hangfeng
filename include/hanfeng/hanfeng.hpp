// =============================================================================
// hanfeng.hpp — 对外焊缝 API 入口
// =============================================================================

#ifndef HANFENG_HANFENG_HPP
#define HANFENG_HANFENG_HPP

#include "hanfeng/weld.hpp"

namespace hanfeng {

	/// 计算平面板与曲面板之间的焊缝（对外接口）
	WeldCurveResult api_get_plane_surface_weld(const PlanePanel& plane_panel,
		const SurfacePanel& surface_panel,
		double tolerance = 1.0);

	WeldSplineResult api_get_plane_surface_weld_splines(const PlanePanel& plane_panel,
		const SurfacePanel& surface_panel,
		double tolerance = 1.0,
		double fit_tolerance = 1.0e-3);

	WeldCurveResult api_get_surface_surface_weld(const SurfacePanel& first_panel,
		const SurfacePanel& second_panel,
		double tolerance = 1.0);

	WeldSplineResult api_get_surface_surface_weld_splines(
		const SurfacePanel& first_panel,
		const SurfacePanel& second_panel,
		double tolerance = 1.0,
		double fit_tolerance = 1.0e-3);

	WeldCurveResult api_get_surface_surface_weld(const float first_vertices[][3],
		int first_vertex_count,
		const int first_triangles[][3],
		int first_triangle_count,
		const float first_normals[][3],
		const float second_vertices[][3],
		int second_vertex_count,
		const int second_triangles[][3],
		int second_triangle_count,
		const float second_normals[][3],
		double tolerance = 1.0);

	WeldSplineResult api_get_surface_surface_weld_splines(
		const float first_vertices[][3],
		int first_vertex_count,
		const int first_triangles[][3],
		int first_triangle_count,
		const float first_normals[][3],
		const float second_vertices[][3],
		int second_vertex_count,
		const int second_triangles[][3],
		int second_triangle_count,
		const float second_normals[][3],
		double tolerance = 1.0,
		double fit_tolerance = 1.0e-3);

}  // namespace hanfeng

#endif
