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

}  // namespace hanfeng
