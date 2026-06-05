// =============================================================================
// data_structures.hpp — hanfeng 项目的高层数据结构定义
// =============================================================================
//
// 本文件定义了项目中使用的几何曲线编码、面板数据模型和焊缝结果等核心数据结构。
// 主要包括：
//   - 常量：kCurveTolerance（曲线容差）、kTruncationMarkerMagnitude（截断标记幅值）
//   - 类型别名：Triangle（三角形索引）、Polyline3（三维折线）
//   - 工具函数：is_zero()、is_truncation_marker()
//   - 曲线控制点：RxyPoint、RxyzPoint
//   - 曲线容器：BasicCurve<T>（模板），以及 RxyCurve、RxyzCurve 别名
//   - 面板结构：FaceBoundaries、PlaneFace、PlanePanel、SurfacePatch、SurfacePanel
//   - 焊缝结构：WeldPolyline、WeldCurveResult
//
// 曲线编码规则（rxy / rxyz）：
//   - 每个控制点包含 r、x、y（、z）字段
//   - r == 0：到下一个控制点是直线段
//   - r != 0 且 abs(r) != 2：到下一个控制点是圆弧段
//   - abs(r) == 2：在该点处存在截断标记
//   - 第 0 个点是头元素（header），其 r 字段记录数组总点数
// =============================================================================

#ifndef HANFENG_PANEL_HPP
#define HANFENG_PANEL_HPP

#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <vector>

#include "hanfeng/geometry.hpp"

namespace hanfeng {

	// =============================================================================
	// 常量定义
	// =============================================================================

	/// 曲线判断的容差阈值，用于浮点零值判断和曲线类型识别
	inline constexpr double kCurveTolerance = 1.0e-6;

	/// 截断标记的幅值常量：当 |r| 约等于此值时，表示该点是一个截断标记
	inline constexpr double kTruncationMarkerMagnitude = 2.0;

	// =============================================================================
	// 类型别名
	// =============================================================================

	/// 三角形，用三个 size_t 索引表示，对应顶点数组的下标
	using Triangle = std::array<std::size_t, 3>;

	/// 三维折线（多段线），由一系列三维空间点组成
	using Polyline3 = std::vector<SPAposition>;

	// =============================================================================
	// 工具函数
	// =============================================================================

	/**
	 * @brief 判断一个实数是否可视为 0。
	 *
	 * 当 |value| <= epsilon 时返回 true。
	 *
	 * @param value   待判断的值
	 * @param epsilon 容差，默认为 kCurveTolerance
	 * @return true 表示该值可视为零
	 */
	inline bool check_zero(double value, double epsilon = kCurveTolerance) {
		return std::fabs(value) <= epsilon;
	}

	/**
	 * @brief 判断一个曲线点的 `r` 值是否表示截断标记。
	 *
	 * 当 ||value| - kTruncationMarkerMagnitude| <= epsilon 时返回 true，
	 * 即 |r| 约等于 2.0 时视为截断标记。
	 *
	 * @param value   待判断的 r 值
	 * @param epsilon 容差，默认为 kCurveTolerance
	 * @return true 表示该 r 值是截断标记
	 */
	inline bool check_truncation_marker(double value,
		double epsilon = kCurveTolerance) {
		return std::fabs(std::fabs(value) - kTruncationMarkerMagnitude) <= epsilon;
	}

	// =============================================================================
	// 曲线控制点结构
	// =============================================================================

	/**
	 * @brief `rxy` 编码中的单个控制点（二维曲线）。
	 *
	 * 语义沿用现有项目约定：
	 * - `r == 0` 表示到下一点是直线
	 * - `r != 0 && abs(r) != 2` 表示到下一点是圆弧
	 * - `abs(r) == 2` 表示在该处存在截断
	 */
	struct RxyPoint {
		double r = 0.0;  ///< 曲线类型标记：0=直线, 非0且|r|!=2=圆弧, |r|≈2=截断
		double x = 0.0;  ///< x 坐标
		double y = 0.0;  ///< y 坐标

		/**
		 * @brief 判断从当前点到下一个点是否为直线段。
		 *
		 * 条件：不是截断标记，且 r 约等于 0。
		 */
		bool defines_line_to_next(double epsilon = kCurveTolerance) const {
			return !defines_truncation_to_next(epsilon) && check_zero(r, epsilon);
		}

		/**
		 * @brief 判断从当前点到下一个点是否为圆弧段。
		 *
		 * 条件：不是截断标记，且 r 不等于 0。
		 */
		bool defines_arc_to_next(double epsilon = kCurveTolerance) const {
			return !defines_truncation_to_next(epsilon) && !check_zero(r, epsilon);
		}

		/**
		 * @brief 判断当前点是否为截断标记。
		 *
		 * 条件：|r| 约等于 kTruncationMarkerMagnitude（2.0）。
		 */
		bool defines_truncation_to_next(double epsilon = kCurveTolerance) const {
			return check_truncation_marker(r, epsilon);
		}
	};

	/**
	 * @brief `rxyz` 编码中的单个控制点（三维曲线）。
	 *
	 * 与 RxyPoint 类似，但增加了 z 坐标分量。
	 */
	struct RxyzPoint {
		double r = 0.0;  ///< 曲线类型标记：0=直线, 非0且|r|!=2=圆弧, |r|≈2=截断
		double x = 0.0;  ///< x 坐标
		double y = 0.0;  ///< y 坐标
		double z = 0.0;  ///< z 坐标

		/**
		 * @brief 判断从当前点到下一个点是否为直线段。
		 */
		bool defines_line_to_next(double epsilon = kCurveTolerance) const {
			return !defines_truncation_to_next(epsilon) && check_zero(r, epsilon);
		}

		/**
		 * @brief 判断从当前点到下一个点是否为圆弧段。
		 */
		bool defines_arc_to_next(double epsilon = kCurveTolerance) const {
			return !defines_truncation_to_next(epsilon) && !check_zero(r, epsilon);
		}

		/**
		 * @brief 判断当前点是否为截断标记。
		 */
		bool defines_truncation_to_next(double epsilon = kCurveTolerance) const {
			return check_truncation_marker(r, epsilon);
		}
	};

	// =============================================================================
	// 曲线容器模板
	// =============================================================================

	/**
	 * @brief 通用曲线容器。
	 *
	 * 使用模板参数 PointT 来支持不同的控制点类型（RxyPoint 或 RxyzPoint）。
	 *
	 * 数据布局约定：
	 * - 第 0 个点是头元素（header），其 `r` 字段用于记录数组总点数（含 header 自身）
	 * - 从第 1 个点开始才是实际的几何控制点
	 *
	 * 例如，若 points 有 5 个元素，则：
	 *   - points[0].r ≈ 5.0（声明总点数）
	 *   - points[1..4] 是 4 个几何控制点
	 */
	template <typename PointT>
	struct BasicCurve {
		std::vector<PointT> points;  ///< 控制点数组，第 0 个为 header

		/**
		 * @brief 返回头元素中声明的总点数（含 header）。
		 *
		 * 从 points[0].r 中读取并四舍五入为 size_t。
		 * 若数组为空或值非正，返回 0。
		 */
		std::size_t declared_point_count() const {
			if (points.empty()) {
				return 0U;
			}

			const long long rounded = std::llround(points.front().r);
			return rounded > 0 ? static_cast<std::size_t>(rounded) : 0U;
		}

		/**
		 * @brief 返回实际的几何控制点数量（不含 header）。
		 *
		 * 即 points.size() - 1。若数组为空返回 0。
		 */
		std::size_t geometry_point_count() const {
			return points.empty() ? 0U : points.size() - 1U;
		}

		/**
		 * @brief 检查头元素的 r 值是否与实际数组大小一致。
		 *
		 * 当 |points[0].r - points.size()| <= epsilon 时返回 true。
		 * 用于验证数据完整性。
		 */
		bool header_matches_point_count(double epsilon = kCurveTolerance) const {
			return !points.empty() &&
				std::fabs(points.front().r - static_cast<double>(points.size())) <=
				epsilon;
		}

		/**
		 * @brief 判断曲线是否为空（没有任何控制点，包括 header）。
		 */
		bool empty() const { return points.empty(); }
	};

	/// 二维曲线类型（rxy 编码）
	using RxyCurve = BasicCurve<RxyPoint>;

	/// 三维曲线类型（rxyz 编码）
	using RxyzCurve = BasicCurve<RxyzPoint>;

	// =============================================================================
	// 面板数据结构
	// =============================================================================

	/**
	 * @brief 一个几何面上的边界集合。
	 *
	 * 将一个面的所有边界环分为两类：
	 * - `outer_loops`：外轮廓（定义面的外部形状）
	 * - `inner_loops`：内孔轮廓（定义面内的孔洞）
	 */
	struct FaceBoundaries {
		std::vector<Polyline3> outer_loops;  ///< 外轮廓环列表
		std::vector<Polyline3> inner_loops;  ///< 内孔轮廓环列表
	};

	/**
	 * @brief 平面板单侧主面的边界定义。
	 *
	 * 每个平面板有两面（face_a 和 face_b），每面各自有独立的边界集合。
	 */
	struct PlaneFace {
		FaceBoundaries boundaries;  ///< 该面的外轮廓和内孔轮廓
	};

	/**
	 * @brief 带厚度平面板的数据定义。
	 *
	 * 当前模型显式保留两侧主面：
	 * - `face_a`：通常对应较低的 Z 面
	 * - `face_b`：通常对应较高的 Z 面
	 *
	 * 后续可以根据 face_a 和 face_b 的对应边界补出侧壁三角面。
	 */
	struct PlanePanel {
		PlaneFace face_a;  ///< 第一侧主面
		PlaneFace face_b;  ///< 第二侧主面
	};

	/**
	 * @brief 曲面面片。
	 *
	 * 每个曲面面片既包含三角网格本体（vertices + triangles），
	 * 也包含该面片自己的边界定义（boundaries）。
	 *
	 * vertices 是顶点坐标数组，triangles 是三角形索引数组，
	 * 每个 Triangle 中的三个索引对应 vertices 中的三个顶点。
	 */
	struct SurfacePatch {
		std::vector<SPAposition> vertices;   ///< 三角网格的顶点坐标列表
		std::vector<Triangle> triangles;     ///< 三角形索引列表，每个三角形引用 vertices
		FaceBoundaries boundaries;           ///< 该面片的边界环定义
	};

	/**
	 * @brief 曲面板，由多个曲面面片组成。
	 *
	 * 一个曲面板可以包含多个 SurfacePatch，用于表达复杂的曲面形状。
	 */
	struct SurfacePanel {
		std::vector<SurfacePatch> surfaces;  ///< 曲面面片列表
	};

	// =============================================================================
	// 焊缝数据结构
	// =============================================================================

	/**
	 * @brief 一条连续焊缝折线。
	 *
	 * 约定 `points[i]` 与 `points[i + 1]` 相连，
	 * 表示焊缝的路径。
	 */
	PlanePanel api_get_plane_panel(const std::filesystem::path& model_path);

	PlanePanel api_get_plane_panel(const float rxyPltBndry1[1000][3],
		const float rxyholes[20][1000][3],
		const float cAxis[4][3],
		float thickdis1,
		float thickdis2);

	SurfacePanel api_get_surface_panel(const std::filesystem::path& model_path);

	SurfacePanel api_get_surface(const std::filesystem::path& model_path);

	SurfacePanel api_get_surface_panel(const float vertices[][3],
		int vertex_count,
		const int triangles[][3],
		int triangle_count,
		const float normals[][3]);

}  // namespace hanfeng

#endif
