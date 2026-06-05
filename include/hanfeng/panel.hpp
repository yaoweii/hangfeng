// =============================================================================
// panel.hpp — 面板数据结构与模型读取接口
// =============================================================================
//
// 定义 hanfeng 项目的面板数据模型和对外构造接口：
//   - 常量：kCurveTolerance、kTruncationMarkerMagnitude
//   - 工具函数：check_zero()、check_truncation_marker()
//   - 曲线控制点：RxyPoint、RxyzPoint
//   - 曲线容器：BasicCurve<T>，以及 RxyCurve、RxyzCurve 别名
//   - 面板结构：FaceBoundaries、PlaneFace、PlanePanel、SurfacePatch、SurfacePanel
//   - 对外接口：api_get_plane_panel()、api_get_surface_panel()、api_get_surface()
//
// 曲线编码规则（rxy / rxyz）：
//   - r == 0：直线段；r != 0 且 |r| != 2：圆弧段；|r| ≈ 2：截断标记
//   - 第 0 个点是 header，其 r 字段记录数组总点数
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
	// 常量
	// =============================================================================

	/// 曲线判断容差阈值
	inline constexpr double kCurveTolerance = 1.0e-6;

	/// 截断标记幅值：|r| ≈ 此值时为截断标记
	inline constexpr double kTruncationMarkerMagnitude = 2.0;

	// =============================================================================
	// 类型别名
	// =============================================================================

	using Triangle = std::array<std::size_t, 3>;
	using Polyline3 = std::vector<SPAposition>;

	// =============================================================================
	// 工具函数
	// =============================================================================

	/// |value| <= epsilon 时视为零
	inline bool check_zero(double value, double epsilon = kCurveTolerance) {
		return std::fabs(value) <= epsilon;
	}

	/// ||value| - kTruncationMarkerMagnitude| <= epsilon 时视为截断标记
	inline bool check_truncation_marker(double value,
		double epsilon = kCurveTolerance) {
		return std::fabs(std::fabs(value) - kTruncationMarkerMagnitude) <= epsilon;
	}

	// =============================================================================
	// 曲线控制点
	// =============================================================================

	/// rxy 编码控制点（二维曲线）
	struct RxyPoint {
		double r = 0.0;  ///< 0=直线, 非0且|r|!=2=圆弧, |r|≈2=截断
		double x = 0.0;
		double y = 0.0;

		bool defines_line_to_next(double epsilon = kCurveTolerance) const {
			return !defines_truncation_to_next(epsilon) && check_zero(r, epsilon);
		}

		bool defines_arc_to_next(double epsilon = kCurveTolerance) const {
			return !defines_truncation_to_next(epsilon) && !check_zero(r, epsilon);
		}

		bool defines_truncation_to_next(double epsilon = kCurveTolerance) const {
			return check_truncation_marker(r, epsilon);
		}
	};

	/// rxyz 编码控制点（三维曲线）
	struct RxyzPoint {
		double r = 0.0;
		double x = 0.0;
		double y = 0.0;
		double z = 0.0;

		bool defines_line_to_next(double epsilon = kCurveTolerance) const {
			return !defines_truncation_to_next(epsilon) && check_zero(r, epsilon);
		}

		bool defines_arc_to_next(double epsilon = kCurveTolerance) const {
			return !defines_truncation_to_next(epsilon) && !check_zero(r, epsilon);
		}

		bool defines_truncation_to_next(double epsilon = kCurveTolerance) const {
			return check_truncation_marker(r, epsilon);
		}
	};

	// =============================================================================
	// 曲线容器模板
	// =============================================================================

	/// 通用曲线容器。points[0] 为 header（其 r 字段记录总点数），
	/// points[1..] 为几何控制点。
	template <typename PointT>
	struct BasicCurve {
		std::vector<PointT> points;

		/// header 中声明的总点数（含 header 自身）
		std::size_t declared_point_count() const {
			if (points.empty()) {
				return 0U;
			}

			const long long rounded = std::llround(points.front().r);
			return rounded > 0 ? static_cast<std::size_t>(rounded) : 0U;
		}

		/// 实际几何控制点数量（不含 header）
		std::size_t geometry_point_count() const {
			return points.empty() ? 0U : points.size() - 1U;
		}

		/// header 的 r 值是否与实际数组大小一致
		bool header_matches_point_count(double epsilon = kCurveTolerance) const {
			return !points.empty() &&
				std::fabs(points.front().r - static_cast<double>(points.size())) <=
				epsilon;
		}

		bool empty() const { return points.empty(); }
	};

	using RxyCurve = BasicCurve<RxyPoint>;
	using RxyzCurve = BasicCurve<RxyzPoint>;

	// =============================================================================
	// 面板数据结构
	// =============================================================================

	/// 一个几何面的边界集合：outer_loops 为外轮廓，inner_loops 为内孔
	struct FaceBoundaries {
		std::vector<Polyline3> outer_loops;
		std::vector<Polyline3> inner_loops;
	};

	/// 平面板单侧主面的边界定义
	struct PlaneFace {
		FaceBoundaries boundaries;
	};

	/// 带厚度平面板：face_a（低 Z 侧）、face_b（高 Z 侧）
	struct PlanePanel {
		PlaneFace face_a;
		PlaneFace face_b;
	};

	/// 曲面面片：三角网格（vertices + triangles）+ 边界环
	struct SurfacePatch {
		std::vector<SPAposition> vertices;
		std::vector<Triangle> triangles;
		FaceBoundaries boundaries;
	};

	/// 曲面板，由多个曲面面片组成
	struct SurfacePanel {
		std::vector<SurfacePatch> surfaces;
	};

	// =============================================================================
	// 模型读取接口
	// =============================================================================

	/// 从模型目录/文件解析平面板
	PlanePanel api_get_plane_panel(const std::filesystem::path& model_path);

	/// 从原始 rxy 数组构造平面板
	PlanePanel api_get_plane_panel(const float rxyPltBndry1[1000][3],
		const float rxyholes[20][1000][3],
		const float cAxis[4][3],
		float thickdis1,
		float thickdis2);

	/// 从模型目录/文件解析曲面板
	SurfacePanel api_get_surface_panel(const std::filesystem::path& model_path);

	/// 从原始三角网格数组构造曲面板
	SurfacePanel api_get_surface_panel(const float vertices[][3],
		int vertex_count,
		const int triangles[][3],
		int triangle_count,
		const float normals[][3]);

}  // namespace hanfeng

#endif
