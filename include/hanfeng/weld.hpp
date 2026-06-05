// =============================================================================
// weld.hpp — 焊缝结果数据结构
// =============================================================================

#ifndef HANFENG_WELD_HPP
#define HANFENG_WELD_HPP

#include <vector>

#include "hanfeng/panel.hpp"

namespace hanfeng {

	/// 一条连续焊缝折线
	struct WeldPolyline {
		Polyline3 points;
	};

	/// 焊缝计算结果：包含多条焊缝折线
	struct WeldCurveResult {
		std::vector<WeldPolyline> polylines;
	};

}  // namespace hanfeng

#endif
