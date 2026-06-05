#ifndef HANFENG_WELD_HPP
#define HANFENG_WELD_HPP

#include <vector>

#include "hanfeng/panel.hpp"

namespace hanfeng {

	struct WeldPolyline {
		Polyline3 points;
	};

	struct WeldCurveResult {
		std::vector<WeldPolyline> polylines;
	};

}  // namespace hanfeng

#endif
