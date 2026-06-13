// =============================================================================
// hanfeng.cpp — 对外焊缝 API 实现
// =============================================================================

#include "hanfeng/hanfeng.hpp"

#include "hanfeng/plane_surface_hanfeng_util.hpp"

namespace hanfeng {

	WeldCurveResult api_get_plane_surface_weld(const PlanePanel& plane_panel,
		const SurfacePanel& surface_panel,
		double tolerance) {
		return hanfeng_compute_plane_surface_weld(
			plane_panel, surface_panel, tolerance);
	}

	WeldSplineResult api_get_plane_surface_weld_splines(const PlanePanel& plane_panel,
		const SurfacePanel& surface_panel,
		double tolerance,
		double fit_tolerance) {
		return hanfeng_fit_weld_splines(
			api_get_plane_surface_weld(plane_panel, surface_panel, tolerance),
			fit_tolerance);
	}

	WeldCurveResult api_get_surface_surface_weld(const SurfacePanel& first_panel,
		const SurfacePanel& second_panel,
		double tolerance) {
		return hanfeng_compute_surface_surface_weld(
			first_panel, second_panel, tolerance);
	}

	WeldSplineResult api_get_surface_surface_weld_splines(
		const SurfacePanel& first_panel,
		const SurfacePanel& second_panel,
		double tolerance,
		double fit_tolerance) {
		return hanfeng_fit_weld_splines(
			api_get_surface_surface_weld(first_panel, second_panel, tolerance),
			fit_tolerance);
	}

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
		double tolerance) {
		const SurfacePanel first_panel = api_get_surface_panel(
			first_vertices, first_vertex_count, first_triangles,
			first_triangle_count, first_normals);
		const SurfacePanel second_panel = api_get_surface_panel(
			second_vertices, second_vertex_count, second_triangles,
			second_triangle_count, second_normals);
		return api_get_surface_surface_weld(first_panel, second_panel, tolerance);
	}

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
		double tolerance,
		double fit_tolerance) {
		return hanfeng_fit_weld_splines(
			api_get_surface_surface_weld(
				first_vertices, first_vertex_count, first_triangles, first_triangle_count,
				first_normals, second_vertices, second_vertex_count, second_triangles,
				second_triangle_count, second_normals, tolerance),
			fit_tolerance);
	}

}  // namespace hanfeng
