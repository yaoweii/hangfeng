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

}  // namespace hanfeng

#endif
