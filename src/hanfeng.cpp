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

}  // namespace hanfeng
