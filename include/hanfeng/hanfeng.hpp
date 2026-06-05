#ifndef HANFENG_HANFENG_HPP
#define HANFENG_HANFENG_HPP

#include "hanfeng/weld.hpp"

namespace hanfeng {

	WeldCurveResult api_get_plane_surface_weld(const PlanePanel& plane_panel,
		const SurfacePanel& surface_panel,
		double tolerance = 1.0);

}  // namespace hanfeng

#endif
