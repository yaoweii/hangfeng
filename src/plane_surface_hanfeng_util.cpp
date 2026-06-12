// =============================================================================
// plane_surface_hanfeng_util.cpp — 平面板/曲面板焊缝计算实现
// =============================================================================
//
// 实现焊缝折线的核心计算逻辑（模型读取无关）：
//   - 平面板有效表面构造、曲面三角面集合构造
//   - BVH 加速结构构建与查询
//   - 线段-三角形精确相交区间计算
//   - 候选边界折线裁剪与焊缝折线去重
// =============================================================================

#include "hanfeng/plane_surface_hanfeng_util.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace hanfeng {

	namespace {

		using ProfilingClock = std::chrono::steady_clock;

		/// 三维三角形（含预计算包围盒，用于加速距离查询）
		struct Triangle3 {
			SPAposition a;
			SPAposition b;
			SPAposition c;
			SPAbox box;
		};

		/// 投影到平面后的二维边界环（含 UV 包围盒用于快速排除）
		struct ProjectedLoop {
			Polyline3 world_points;
			std::vector<std::array<double, 2>> uv_points;
			std::array<double, 2> uv_min{
				std::numeric_limits<double>::infinity(),
				std::numeric_limits<double>::infinity()
			};
			std::array<double, 2> uv_max{
				-std::numeric_limits<double>::infinity(),
				-std::numeric_limits<double>::infinity()
			};
		};

		/// 平面板单侧主面的几何信息（含外法线方向和投影环）
		struct PlaneFaceGeometry {
			SPAposition origin;
			SPAunit_vector normal;
			SPAunit_vector outward_normal;
			SPAunit_vector u_axis;
			SPAunit_vector v_axis;
			std::vector<ProjectedLoop> outer_loops;
			std::vector<ProjectedLoop> inner_loops;
		};

		/// BVH 节点：叶节点持有三角形范围，中间节点持有左右子索引
		struct SurfaceBvhNode {
			SPAbox box;
			std::size_t begin = 0;
			std::size_t end = 0;
			int left = -1;
			int right = -1;

			bool is_leaf() const {
				return left < 0 && right < 0;
			}
		};

		/// 平面板完整几何（两侧主面 + 侧壁三角形 + 侧壁 BVH）
		struct PlanePanelGeometry {
			std::vector<PlaneFaceGeometry> main_faces;
			std::vector<Triangle3> side_triangles;
			std::vector<SurfaceBvhNode> side_bvh_nodes;
		};

		/// 曲面板几何（展开三角形 + BVH 加速结构）
		struct SurfaceGeometry {
			std::vector<Triangle3> triangles;
			std::vector<SurfaceBvhNode> bvh_nodes;
		};

		/// 曲面上最近点及其所在三角形的法向量
		struct SurfaceClosestPoint {
			SPAposition point;
			SPAunit_vector triangle_normal;
		};

		/// 点相对于曲面体的分类：外部 / 内部 / 边界上
		enum class SolidPointClassification {
			Outside,
			Inside,
			OnBoundary
		};

		/// 线段与曲面体的交点事件：Crossing=穿越，Touching=擦过
		struct SolidIntersectionEvent {
			double t = 0.0;

			enum class Type {
				Crossing,
				Touching
			} type = Type::Touching;
		};

		/// 平面板候选边界环（含外法线方向）
		struct PlaneCandidateLoop {
			Polyline3 loop;
			SPAunit_vector outward_normal;
		};

		/// 参数区间 [start, end]，用于线段命中区间的描述
		struct Interval {
			double start = 0.0;
			double end = 0.0;
		};

		/// 参数线性函数 f(t) = constant + slope * t
		struct LinearFunction {
			double constant = 0.0;
			double slope = 0.0;
		};

		/// 参数二次函数 f(t) = constant + linear*t + quadratic*t²
		struct QuadraticFunction {
			double constant = 0.0;
			double linear = 0.0;
			double quadratic = 0.0;
		};

		constexpr double kIntervalEpsilon = 1.0e-9;
		thread_local WeldProfilingData g_weld_profiling_data;

		/// RAII 计时器：构造时记录起点，析构时累加耗时到目标毫秒引用
		struct ScopedDurationAccumulator {
			double& destination_ms;
			ProfilingClock::time_point start = ProfilingClock::now();

			explicit ScopedDurationAccumulator(double& destination)
				: destination_ms(destination) {}

			~ScopedDurationAccumulator() {
				destination_ms += std::chrono::duration<double, std::milli>(
					ProfilingClock::now() - start).count();
			}
		};

		/// 线段上参数 t 处的插值点：t=0 返回 start，t=1 返回 end
		SPAposition interpolate_position(const SPAposition& start,
			const SPAposition& end,
			double t) {
			return SPAposition(start.x() + (end.x() - start.x()) * t,
				start.y() + (end.y() - start.y()) * t,
				start.z() + (end.z() - start.z()) * t);
		}

		/// 将数值裁剪到 [0, 1] 区间
		double clamp01(double value) {
			return std::max(0.0, std::min(1.0, value));
		}

		/// 浮点近似相等判断
		bool nearly_equal(double lhs, double rhs, double epsilon = kIntervalEpsilon) {
			return std::fabs(lhs - rhs) <= epsilon;
		}

		/// 计算候选环的有效线段数（首尾相同时减 1）
		std::size_t candidate_loop_segment_count(const Polyline3& loop) {
			if (loop.size() < 2U) {
				return 0U;
			}
			return (loop.front() == loop.back()) ? (loop.size() - 1U) : loop.size();
		}

		/// 求线性函数在 t 处的值
		double evaluate(const LinearFunction& function, double t) {
			return function.constant + function.slope * t;
		}

		/// 求二次函数在 t 处的值
		double evaluate(const QuadraticFunction& function, double t) {
			return function.constant + function.linear * t +
				function.quadratic * t * t;
		}

		/// 两个线性函数相乘得到二次函数
		QuadraticFunction multiply(const LinearFunction& left,
			const LinearFunction& right) {
			return QuadraticFunction{
				left.constant * right.constant,
				left.constant * right.slope + left.slope * right.constant,
				left.slope * right.slope
			};
		}

		/// 两个线性函数相减
		LinearFunction subtract(const LinearFunction& left,
			const LinearFunction& right) {
			return LinearFunction{
				left.constant - right.constant,
				left.slope - right.slope
			};
		}

		/// 两个二次函数相减
		QuadraticFunction subtract(const QuadraticFunction& left,
			const QuadraticFunction& right) {
			return QuadraticFunction{
				left.constant - right.constant,
				left.linear - right.linear,
				left.quadratic - right.quadratic
			};
		}

		/// 构造二次函数 ||offset + t*direction||² < radius²
		QuadraticFunction squared_norm_less_than(const SPAvector& offset,
			const SPAvector& direction,
			double radius_squared) {
			return QuadraticFunction{
				offset.len_sq() - radius_squared,
				2.0 * (offset % direction),
				direction.len_sq()
			};
		}

		/// 对区间列表排序、合并重叠、过滤极小区间
		std::vector<Interval> normalise_intervals(std::vector<Interval> intervals,
			double epsilon = kIntervalEpsilon) {
			std::vector<Interval> filtered;
			for (Interval interval : intervals) {
				if (interval.end < interval.start) {
					std::swap(interval.start, interval.end);
				}
				if (interval.end - interval.start <= epsilon) {
					continue;
				}
				filtered.push_back(interval);
			}

			if (filtered.empty()) {
				return filtered;
			}

			std::sort(filtered.begin(), filtered.end(),
				[](const Interval& left, const Interval& right) {
					if (!nearly_equal(left.start, right.start)) {
						return left.start < right.start;
					}
					return left.end < right.end;
				});

			std::vector<Interval> merged;
			merged.push_back(filtered.front());
			for (std::size_t index = 1; index < filtered.size(); ++index) {
				Interval& back = merged.back();
				if (filtered[index].start <= back.end + epsilon) {
					back.end = std::max(back.end, filtered[index].end);
				}
				else {
					merged.push_back(filtered[index]);
				}
			}
			return merged;
		}

		/// 两个有序区间集合求交集
		std::vector<Interval> intersect_interval_sets(const std::vector<Interval>& left,
			const std::vector<Interval>& right,
			double epsilon = kIntervalEpsilon) {
			std::vector<Interval> result;
			std::size_t left_index = 0U;
			std::size_t right_index = 0U;
			while (left_index < left.size() && right_index < right.size()) {
				const double start =
					std::max(left[left_index].start, right[right_index].start);
				const double end =
					std::min(left[left_index].end, right[right_index].end);
				if (end - start > epsilon) {
					result.push_back({ start, end });
				}

				if (left[left_index].end < right[right_index].end - epsilon) {
					++left_index;
				}
				else if (right[right_index].end < left[left_index].end - epsilon) {
					++right_index;
				}
				else {
					++left_index;
					++right_index;
				}
			}
			return result;
		}

		/// 求区间集合在 [0,1] 上的补集
		std::vector<Interval> complement_on_unit_interval(
			const std::vector<Interval>& covered,
			double epsilon = kIntervalEpsilon) {
			const std::vector<Interval> normalised = normalise_intervals(covered, epsilon);
			std::vector<Interval> result;
			double cursor = 0.0;
			for (const Interval& interval : normalised) {
				if (interval.start > cursor + epsilon) {
					result.push_back({ cursor, interval.start });
				}
				cursor = std::max(cursor, interval.end);
			}
			if (cursor < 1.0 - epsilon) {
				result.push_back({ cursor, 1.0 });
			}
			return normalise_intervals(std::move(result), epsilon);
		}

		/// 将子线段上的局部区间映射回父线段参数空间
		std::vector<Interval> remap_intervals_to_parent(
			const std::vector<Interval>& local_intervals,
			const Interval& parent,
			double epsilon = kIntervalEpsilon) {
			const double scale = parent.end - parent.start;
			if (scale <= epsilon) {
				return {};
			}

			std::vector<Interval> result;
			result.reserve(local_intervals.size());
			for (const Interval& interval : local_intervals) {
				result.push_back({
					parent.start + interval.start * scale,
					parent.start + interval.end * scale
					});
			}
			return normalise_intervals(std::move(result), epsilon);
		}

		/// 求解线性不等式 f(t) ≤ 0 或 f(t) ≥ 0 的区间解
		std::vector<Interval> solve_linear_inequality(const LinearFunction& function,
			bool less_equal) {
			if (std::fabs(function.slope) <= kIntervalEpsilon) {
				const bool satisfied = less_equal
					? function.constant <= kIntervalEpsilon
					: function.constant >= -kIntervalEpsilon;
				return satisfied ? std::vector<Interval>{ { 0.0, 1.0 } } : std::vector<Interval>{};
			}

			const double root = -function.constant / function.slope;
			if (less_equal) {
				if (function.slope > 0.0) {
					return std::vector<Interval>{ { -std::numeric_limits<double>::infinity(), root } };
				}
				return std::vector<Interval>{ { root, std::numeric_limits<double>::infinity() } };
			}

			if (function.slope > 0.0) {
				return std::vector<Interval>{ { root, std::numeric_limits<double>::infinity() } };
			}
			return std::vector<Interval>{ { -std::numeric_limits<double>::infinity(), root } };
		}

		/// 求解二次不等式 f(t) ≤ 0 或 f(t) ≥ 0 的区间解（含判别式分析）
		std::vector<Interval> solve_quadratic_inequality(const QuadraticFunction& function,
			bool less_equal) {
			if (std::fabs(function.quadratic) <= kIntervalEpsilon) {
				return solve_linear_inequality(
					LinearFunction{ function.constant, function.linear }, less_equal);
			}

			const double discriminant =
				function.linear * function.linear -
				4.0 * function.quadratic * function.constant;
			if (discriminant < -kIntervalEpsilon) {
				const bool satisfied = less_equal
					? function.quadratic < 0.0
					: function.quadratic > 0.0;
				return satisfied ? std::vector<Interval>{ { 0.0, 1.0 } } : std::vector<Interval>{};
			}

			if (std::fabs(discriminant) <= kIntervalEpsilon) {
				const double root = -function.linear / (2.0 * function.quadratic);
				const bool satisfied = less_equal
					? function.quadratic < 0.0
					: function.quadratic > 0.0;
				if (satisfied) {
					return {
						{ -std::numeric_limits<double>::infinity(), root },
						{ root, std::numeric_limits<double>::infinity() }
					};
				}
				return {};
			}

			const double sqrt_discriminant = std::sqrt(std::max(0.0, discriminant));
			double root0 =
				(-function.linear - sqrt_discriminant) / (2.0 * function.quadratic);
			double root1 =
				(-function.linear + sqrt_discriminant) / (2.0 * function.quadratic);
			if (root1 < root0) {
				std::swap(root0, root1);
			}

			if (less_equal) {
				if (function.quadratic > 0.0) {
					return std::vector<Interval>{ { root0, root1 } };
				}
				return {
					{ -std::numeric_limits<double>::infinity(), root0 },
					{ root1, std::numeric_limits<double>::infinity() }
				};
			}

			if (function.quadratic > 0.0) {
				return {
					{ -std::numeric_limits<double>::infinity(), root0 },
					{ root1, std::numeric_limits<double>::infinity() }
				};
			}
			return std::vector<Interval>{ { root0, root1 } };
		}

		/// 将区间裁剪到 [0,1] 范围内
		std::vector<Interval> clip_intervals_to_unit(const std::vector<Interval>& intervals) {
			std::vector<Interval> clipped;
			for (const Interval& interval : intervals) {
				const double start = std::max(0.0, interval.start);
				const double end = std::min(1.0, interval.end);
				if (end - start > kIntervalEpsilon) {
					clipped.push_back({ start, end });
				}
			}
			return normalise_intervals(std::move(clipped));
		}

		/// 构造线性函数表示向量与参数射线的点乘随 t 的变化
		LinearFunction dot_linear(const SPAvector& vector,
			const SPAposition& start,
			const SPAvector& direction,
			const SPAposition& origin) {
			const SPAvector start_offset = start - origin;
			return LinearFunction{
				vector % start_offset,
				vector % direction
			};
		}

		/// 点到线段最短距离平方
		double point_segment_distance_squared(const SPAposition& point,
			const SPAposition& start,
			const SPAposition& end) {
			const SPAvector segment = end - start;
			const double segment_length_squared = segment.len_sq();
			if (segment_length_squared <= 1.0e-18) {
				return (point - start).len_sq();
			}

			const double projection =
				clamp01(((point - start) % segment) / segment_length_squared);
			const SPAposition closest = start + segment * projection;
			return (point - closest).len_sq();
		}

		/// 三角形上离给定点最近的点（经典 7 区域 Voronoi 算法）
		SPAposition closest_point_on_triangle(const SPAposition& point,
			const Triangle3& triangle) {
			const SPAvector ab = triangle.b - triangle.a;
			const SPAvector ac = triangle.c - triangle.a;
			const SPAvector ap = point - triangle.a;
			const double d1 = ab % ap;
			const double d2 = ac % ap;
			if (d1 <= 0.0 && d2 <= 0.0) {
				return triangle.a;
			}

			const SPAvector bp = point - triangle.b;
			const double d3 = ab % bp;
			const double d4 = ac % bp;
			if (d3 >= 0.0 && d4 <= d3) {
				return triangle.b;
			}

			const double vc = d1 * d4 - d3 * d2;
			if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
				const double v = d1 / (d1 - d3);
				return triangle.a + ab * v;
			}

			const SPAvector cp = point - triangle.c;
			const double d5 = ab % cp;
			const double d6 = ac % cp;
			if (d6 >= 0.0 && d5 <= d6) {
				return triangle.c;
			}

			const double vb = d5 * d2 - d1 * d6;
			if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
				const double w = d2 / (d2 - d6);
				return triangle.a + ac * w;
			}

			const double va = d3 * d6 - d5 * d4;
			if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
				const SPAvector bc = triangle.c - triangle.b;
				const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
				return triangle.b + bc * w;
			}

			const double denominator = 1.0 / (va + vb + vc);
			const double v = vb * denominator;
			const double w = vc * denominator;
			return triangle.a + ab * v + ac * w;
		}

		/// 点到三角形最短距离平方
		double point_triangle_distance_squared(const SPAposition& point,
			const Triangle3& triangle) {
			const SPAposition closest = closest_point_on_triangle(point, triangle);
			return (point - closest).len_sq();
		}

		/// 点到包围盒最短距离平方
		double point_box_distance_squared(const SPAposition& point,
			const SPAbox& box) {
			if (!box.is_valid()) {
				return std::numeric_limits<double>::infinity();
			}

			double distance_squared = 0.0;
			for (std::size_t axis = 0; axis < 3U; ++axis) {
				const double coordinate = point[axis];
				if (coordinate < box.min_corner()[axis]) {
					const double delta = box.min_corner()[axis] - coordinate;
					distance_squared += delta * delta;
				}
				else if (coordinate > box.max_corner()[axis]) {
					const double delta = coordinate - box.max_corner()[axis];
					distance_squared += delta * delta;
				}
			}
			return distance_squared;
		}

		/// 合并两个包围盒
		SPAbox merge_boxes(const SPAbox& left, const SPAbox& right) {
			if (!left.is_valid()) {
				return right;
			}
			if (!right.is_valid()) {
				return left;
			}
			return SPAbox(
				SPAposition(std::min(left.min_corner().x(), right.min_corner().x()),
					std::min(left.min_corner().y(), right.min_corner().y()),
					std::min(left.min_corner().z(), right.min_corner().z())),
				SPAposition(std::max(left.max_corner().x(), right.max_corner().x()),
					std::max(left.max_corner().y(), right.max_corner().y()),
					std::max(left.max_corner().z(), right.max_corner().z())));
		}

		/// 按给定边距扩展包围盒
		SPAbox expand_box(const SPAbox& box, double margin) {
			if (!box.is_valid()) {
				return box;
			}
			return SPAbox(
				SPAposition(box.min_corner().x() - margin,
					box.min_corner().y() - margin,
					box.min_corner().z() - margin),
				SPAposition(box.max_corner().x() + margin,
					box.max_corner().y() + margin,
					box.max_corner().z() + margin));
		}

		/// 包围盒质心在指定轴上的坐标
		double box_centroid_coordinate(const SPAbox& box, std::size_t axis) {
			return 0.5 * (box.min_corner()[axis] + box.max_corner()[axis]);
		}

		/// 扩展投影环的 UV 包围盒以包含新 UV 点
		void expand_uv_bounds(ProjectedLoop& loop, const std::array<double, 2>& uv) {
			loop.uv_min[0] = std::min(loop.uv_min[0], uv[0]);
			loop.uv_min[1] = std::min(loop.uv_min[1], uv[1]);
			loop.uv_max[0] = std::max(loop.uv_max[0], uv[0]);
			loop.uv_max[1] = std::max(loop.uv_max[1], uv[1]);
		}

		/// 快速判断 UV 点是否在投影环的 UV 包围盒内
		bool uv_box_contains_point(const ProjectedLoop& loop,
			const std::array<double, 2>& uv,
			double epsilon = 1.0e-8) {
			return uv[0] >= loop.uv_min[0] - epsilon &&
				uv[0] <= loop.uv_max[0] + epsilon &&
				uv[1] >= loop.uv_min[1] - epsilon &&
				uv[1] <= loop.uv_max[1] + epsilon;
		}

		/// 快速判断 UV 线段包围盒是否与投影环包围盒重叠
		bool uv_segment_box_overlaps_loop_box(const std::array<double, 2>& start,
			const std::array<double, 2>& end,
			const ProjectedLoop& loop,
			double epsilon = 1.0e-8) {
			const double segment_min_x = std::min(start[0], end[0]) - epsilon;
			const double segment_max_x = std::max(start[0], end[0]) + epsilon;
			const double segment_min_y = std::min(start[1], end[1]) - epsilon;
			const double segment_max_y = std::max(start[1], end[1]) + epsilon;
			return !(segment_max_x < loop.uv_min[0] ||
				segment_min_x > loop.uv_max[0] ||
				segment_max_y < loop.uv_min[1] ||
				segment_min_y > loop.uv_max[1]);
		}

		/// 判断线段是否与包围盒相交（Slab 方法）
		bool segment_intersects_box(const SPAposition& start,
			const SPAposition& end,
			const SPAbox& box,
			double epsilon = 1.0e-12) {
			if (!box.is_valid()) {
				return false;
			}

			const SPAvector direction = end - start;
			double t_min = 0.0;
			double t_max = 1.0;
			for (std::size_t axis = 0; axis < 3U; ++axis) {
				const double origin = start[axis];
				const double delta = direction[axis];
				const double minimum = box.min_corner()[axis] - epsilon;
				const double maximum = box.max_corner()[axis] + epsilon;
				if (std::fabs(delta) <= epsilon) {
					if (origin < minimum || origin > maximum) {
						return false;
					}
					continue;
				}

				double axis_min = (minimum - origin) / delta;
				double axis_max = (maximum - origin) / delta;
				if (axis_min > axis_max) {
					std::swap(axis_min, axis_max);
				}
				t_min = std::max(t_min, axis_min);
				t_max = std::min(t_max, axis_max);
				if (t_min > t_max) {
					return false;
				}
			}
			return true;
		}

		/// 判断射线是否与包围盒相交
		bool ray_intersects_box(const SPAposition& origin,
			const SPAvector& direction,
			const SPAbox& box,
			double epsilon = 1.0e-12) {
			if (!box.is_valid()) {
				return false;
			}

			double t_min = 0.0;
			double t_max = std::numeric_limits<double>::infinity();
			for (std::size_t axis = 0; axis < 3U; ++axis) {
				const double ray_origin = origin[axis];
				const double ray_direction = direction[axis];
				const double minimum = box.min_corner()[axis] - epsilon;
				const double maximum = box.max_corner()[axis] + epsilon;
				if (std::fabs(ray_direction) <= epsilon) {
					if (ray_origin < minimum || ray_origin > maximum) {
						return false;
					}
					continue;
				}

				double axis_min = (minimum - ray_origin) / ray_direction;
				double axis_max = (maximum - ray_origin) / ray_direction;
				if (axis_min > axis_max) {
					std::swap(axis_min, axis_max);
				}
				t_min = std::max(t_min, axis_min);
				t_max = std::min(t_max, axis_max);
				if (t_min > t_max) {
					return false;
				}
			}
			return t_max >= 0.0;
		}

		/// 为三个顶点构建包围盒
		SPAbox build_triangle_box(const SPAposition& a,
			const SPAposition& b,
			const SPAposition& c) {
			return SPAbox(
				SPAposition(std::min({ a.x(), b.x(), c.x() }),
					std::min({ a.y(), b.y(), c.y() }),
					std::min({ a.z(), b.z(), c.z() })),
				SPAposition(std::max({ a.x(), b.x(), c.x() }),
					std::max({ a.y(), b.y(), c.y() }),
					std::max({ a.z(), b.z(), c.z() })));
		}

		/// 用三个顶点 + 预计算包围盒构建 Triangle3
		Triangle3 make_triangle3(const SPAposition& a,
			const SPAposition& b,
			const SPAposition& c) {
			return Triangle3{ a, b, c, build_triangle_box(a, b, c) };
		}

		/// 递归构建三角形 BVH：沿质心最长轴中位数分割，叶节点阈值 16 个三角形
		int build_triangle_bvh_recursive(std::vector<Triangle3>& triangles,
			std::vector<SurfaceBvhNode>& nodes,
			std::size_t begin,
			std::size_t end) {
			SurfaceBvhNode node;
			node.begin = begin;
			node.end = end;
			for (std::size_t index = begin; index < end; ++index) {
				node.box = merge_boxes(node.box, triangles[index].box);
			}

			const int node_index = static_cast<int>(nodes.size());
			nodes.push_back(node);

			const std::size_t count = end - begin;
			constexpr std::size_t kLeafTriangleCount = 16U;
			if (count <= kLeafTriangleCount) {
				return node_index;
			}

			double centroid_min[3] = {
				std::numeric_limits<double>::infinity(),
				std::numeric_limits<double>::infinity(),
				std::numeric_limits<double>::infinity()
			};
			double centroid_max[3] = {
				-std::numeric_limits<double>::infinity(),
				-std::numeric_limits<double>::infinity(),
				-std::numeric_limits<double>::infinity()
			};
			for (std::size_t index = begin; index < end; ++index) {
				for (std::size_t axis = 0; axis < 3U; ++axis) {
					const double centroid =
						box_centroid_coordinate(triangles[index].box, axis);
					centroid_min[axis] = std::min(centroid_min[axis], centroid);
					centroid_max[axis] = std::max(centroid_max[axis], centroid);
				}
			}

			std::size_t split_axis = 0U;
			double split_extent = centroid_max[0] - centroid_min[0];
			for (std::size_t axis = 1; axis < 3U; ++axis) {
				const double extent = centroid_max[axis] - centroid_min[axis];
				if (extent > split_extent) {
					split_axis = axis;
					split_extent = extent;
				}
			}
			if (split_extent <= 1.0e-12) {
				return node_index;
			}

			const std::size_t midpoint = begin + count / 2U;
			std::nth_element(triangles.begin() + begin,
				triangles.begin() + midpoint,
				triangles.begin() + end,
				[&](const Triangle3& left, const Triangle3& right) {
					return box_centroid_coordinate(left.box, split_axis) <
						box_centroid_coordinate(right.box, split_axis);
				});

			nodes[node_index].left =
				build_triangle_bvh_recursive(triangles, nodes, begin, midpoint);
			nodes[node_index].right =
				build_triangle_bvh_recursive(triangles, nodes, midpoint, end);
			nodes[node_index].begin = 0U;
			nodes[node_index].end = 0U;
			return node_index;
		}

		/// 构建三角形 BVH 的入口函数
		void build_triangle_bvh(std::vector<Triangle3>& triangles,
			std::vector<SurfaceBvhNode>& nodes) {
			nodes.clear();
			if (triangles.empty()) {
				return;
			}
			build_triangle_bvh_recursive(triangles, nodes, 0U, triangles.size());
		}

		/// 构建曲面几何的 BVH 加速结构
		void build_surface_bvh(SurfaceGeometry& geometry) {
			build_triangle_bvh(geometry.triangles, geometry.bvh_nodes);
		}

		/// 遍历 BVH 中满足包围盒谓词的三角形，对每个调用访问器
		template <typename BoxPredicate, typename TriangleVisitor>
		void visit_bvh_triangles(const std::vector<Triangle3>& triangles,
			const std::vector<SurfaceBvhNode>& nodes,
			BoxPredicate&& should_visit_box,
			TriangleVisitor&& visit_triangle) {
			if (nodes.empty()) {
				for (const Triangle3& triangle : triangles) {
					if (should_visit_box(triangle.box)) {
						visit_triangle(triangle);
					}
				}
				return;
			}

			std::vector<int> stack;
			stack.push_back(0);
			while (!stack.empty()) {
				const int node_index = stack.back();
				stack.pop_back();
				const SurfaceBvhNode& node = nodes[node_index];
				if (!should_visit_box(node.box)) {
					continue;
				}
				if (node.is_leaf()) {
					for (std::size_t index = node.begin; index < node.end; ++index) {
						const Triangle3& triangle = triangles[index];
						if (should_visit_box(triangle.box)) {
							visit_triangle(triangle);
						}
					}
					continue;
				}
				if (node.left >= 0) {
					stack.push_back(node.left);
				}
				if (node.right >= 0) {
					stack.push_back(node.right);
				}
			}
		}

		/// 遍历曲面几何中满足条件的三角形
		template <typename BoxPredicate, typename TriangleVisitor>
		void visit_surface_triangles(const SurfaceGeometry& geometry,
			BoxPredicate&& should_visit_box,
			TriangleVisitor&& visit_triangle) {
			visit_bvh_triangles(geometry.triangles, geometry.bvh_nodes,
				std::forward<BoxPredicate>(should_visit_box),
				std::forward<TriangleVisitor>(visit_triangle));
		}

		/// 遍历平面板侧壁三角形
		template <typename BoxPredicate, typename TriangleVisitor>
		void visit_plane_side_triangles(const PlanePanelGeometry& geometry,
			BoxPredicate&& should_visit_box,
			TriangleVisitor&& visit_triangle) {
			visit_bvh_triangles(geometry.side_triangles, geometry.side_bvh_nodes,
				std::forward<BoxPredicate>(should_visit_box),
				std::forward<TriangleVisitor>(visit_triangle));
		}

		/// 构造二次函数：参数射线上的点到平面的带状距离不等式
		QuadraticFunction distance_to_plane_band_function(const SPAposition& start,
			const SPAvector& direction,
			const SPAposition& origin,
			const SPAunit_vector& plane_normal,
			double tolerance) {
			const SPAvector offset = start - origin;
			const double constant = plane_normal % offset;
			const double slope = plane_normal % direction;
			return QuadraticFunction{
				constant * constant - tolerance * tolerance,
				2.0 * constant * slope,
				slope * slope
			};
		}

		/// 同上，使用未归一化平面法向量
		QuadraticFunction distance_to_plane_band_function(const SPAposition& start,
			const SPAvector& direction,
			const SPAposition& origin,
			const SPAvector& plane_normal,
			double tolerance) {
			const SPAvector offset = start - origin;
			const double constant = plane_normal % offset;
			const double slope = plane_normal % direction;
			return QuadraticFunction{
				constant * constant - tolerance * tolerance * plane_normal.len_sq(),
				2.0 * constant * slope,
				slope * slope
			};
		}

		/// 精确计算线段参数空间上与三角形距离 ≤ tolerance 的区间（基于参数函数代数求解）
		std::vector<Interval> exact_segment_triangle_intervals(const SPAposition& start,
			const SPAposition& end,
			const Triangle3& triangle,
			double tolerance) {
			const SPAvector direction = end - start;
			const double tolerance_squared = tolerance * tolerance;

			const SPAvector ab = triangle.b - triangle.a;
			const SPAvector ac = triangle.c - triangle.a;
			const SPAvector bc = triangle.c - triangle.b;
			const double ab_length_squared = ab.len_sq();
			const double ac_length_squared = ac.len_sq();
			const double bc_length_squared = bc.len_sq();
			const SPAvector triangle_normal = ab * ac;

			const LinearFunction d1 = dot_linear(ab, start, direction, triangle.a);
			const LinearFunction d2 = dot_linear(ac, start, direction, triangle.a);
			const LinearFunction d3 = dot_linear(ab, start, direction, triangle.b);
			const LinearFunction d4 = dot_linear(ac, start, direction, triangle.b);
			const LinearFunction d5 = dot_linear(ab, start, direction, triangle.c);
			const LinearFunction d6 = dot_linear(ac, start, direction, triangle.c);

			const QuadraticFunction vc =
				subtract(multiply(d1, d4), multiply(d3, d2));
			const QuadraticFunction vb =
				subtract(multiply(d5, d2), multiply(d1, d6));
			const QuadraticFunction va =
				subtract(multiply(d3, d6), multiply(d5, d4));

			const auto build_point_distance_intervals =
				[&](const SPAposition& point) {
				return clip_intervals_to_unit(solve_quadratic_inequality(
					squared_norm_less_than(start - point, direction, tolerance_squared), true));
				};

			const auto build_edge_distance_intervals =
				[&](const SPAposition& origin, const SPAvector& edge, const LinearFunction& edge_projection) {
				const double edge_length_squared = edge.len_sq();
				const double u_constant = edge_projection.constant / edge_length_squared;
				const double u_slope = edge_projection.slope / edge_length_squared;
				const SPAvector residual_constant = (start - origin) - edge * u_constant;
				const SPAvector residual_slope = direction - edge * u_slope;
				return clip_intervals_to_unit(solve_quadratic_inequality(
					squared_norm_less_than(residual_constant, residual_slope, tolerance_squared), true));
				};

			std::vector<Interval> hit_intervals;
			auto append_feature_intervals = [&](std::vector<Interval> feature_intervals) {
				feature_intervals = clip_intervals_to_unit(std::move(feature_intervals));
				hit_intervals.insert(hit_intervals.end(),
					feature_intervals.begin(), feature_intervals.end());
				};

			std::vector<Interval> vertex_a = build_point_distance_intervals(triangle.a);
			vertex_a = intersect_interval_sets(vertex_a,
				clip_intervals_to_unit(solve_linear_inequality(d1, true)));
			vertex_a = intersect_interval_sets(vertex_a,
				clip_intervals_to_unit(solve_linear_inequality(d2, true)));
			append_feature_intervals(std::move(vertex_a));

			std::vector<Interval> vertex_b = build_point_distance_intervals(triangle.b);
			vertex_b = intersect_interval_sets(vertex_b,
				clip_intervals_to_unit(solve_linear_inequality(d3, false)));
			vertex_b = intersect_interval_sets(vertex_b,
				clip_intervals_to_unit(solve_linear_inequality(subtract(d4, d3), true)));
			append_feature_intervals(std::move(vertex_b));

			std::vector<Interval> vertex_c = build_point_distance_intervals(triangle.c);
			vertex_c = intersect_interval_sets(vertex_c,
				clip_intervals_to_unit(solve_linear_inequality(d6, false)));
			vertex_c = intersect_interval_sets(vertex_c,
				clip_intervals_to_unit(solve_linear_inequality(subtract(d5, d6), true)));
			append_feature_intervals(std::move(vertex_c));

			if (ab_length_squared > 1.0e-18) {
				std::vector<Interval> edge_ab =
					build_edge_distance_intervals(triangle.a, ab, d1);
				edge_ab = intersect_interval_sets(edge_ab,
					clip_intervals_to_unit(solve_linear_inequality(d1, false)));
				edge_ab = intersect_interval_sets(edge_ab,
					clip_intervals_to_unit(solve_linear_inequality(d3, true)));
				edge_ab = intersect_interval_sets(edge_ab,
					clip_intervals_to_unit(solve_quadratic_inequality(vc, true)));
				append_feature_intervals(std::move(edge_ab));
			}

			if (ac_length_squared > 1.0e-18) {
				std::vector<Interval> edge_ac =
					build_edge_distance_intervals(triangle.a, ac, d2);
				edge_ac = intersect_interval_sets(edge_ac,
					clip_intervals_to_unit(solve_linear_inequality(d2, false)));
				edge_ac = intersect_interval_sets(edge_ac,
					clip_intervals_to_unit(solve_linear_inequality(d6, true)));
				edge_ac = intersect_interval_sets(edge_ac,
					clip_intervals_to_unit(solve_quadratic_inequality(vb, true)));
				append_feature_intervals(std::move(edge_ac));
			}

			if (bc_length_squared > 1.0e-18) {
				const LinearFunction bc_projection = subtract(d4, d3);
				std::vector<Interval> edge_bc =
					build_edge_distance_intervals(triangle.b, bc, bc_projection);
				edge_bc = intersect_interval_sets(edge_bc,
					clip_intervals_to_unit(solve_linear_inequality(subtract(d4, d3), false)));
				edge_bc = intersect_interval_sets(edge_bc,
					clip_intervals_to_unit(solve_linear_inequality(subtract(d5, d6), false)));
				edge_bc = intersect_interval_sets(edge_bc,
					clip_intervals_to_unit(solve_quadratic_inequality(va, true)));
				append_feature_intervals(std::move(edge_bc));
			}

			if (triangle_normal.len_sq() > 1.0e-18) {
				std::vector<Interval> face_intervals =
					clip_intervals_to_unit(solve_quadratic_inequality(
						distance_to_plane_band_function(start, direction, triangle.a,
							triangle_normal, tolerance), true));
				face_intervals = intersect_interval_sets(face_intervals,
					clip_intervals_to_unit(solve_quadratic_inequality(va, false)));
				face_intervals = intersect_interval_sets(face_intervals,
					clip_intervals_to_unit(solve_quadratic_inequality(vb, false)));
				face_intervals = intersect_interval_sets(face_intervals,
					clip_intervals_to_unit(solve_quadratic_inequality(vc, false)));
				append_feature_intervals(std::move(face_intervals));
			}

			return normalise_intervals(std::move(hit_intervals));
		}

		/// 从边界环拟合平面参数（原点 + 法向量），遍历三点组合求非退化叉积
		bool fit_plane_from_loop(const Polyline3& loop, SPAposition& origin,
			SPAunit_vector& normal) {
			if (loop.size() < 3U) {
				return false;
			}

			origin = loop.front();
			for (std::size_t i = 1; i < loop.size(); ++i) {
				for (std::size_t j = i + 1; j < loop.size(); ++j) {
					const SPAvector first = loop[i] - origin;
					const SPAvector second = loop[j] - origin;
					const SPAvector cross = first * second;
					if (cross.len() > 1.0e-9) {
						normal = SPAunit_vector(cross);
						return normal.is_valid();
					}
				}
			}
			return false;
		}

		/// 根据法向量构建平面正交基 (u, v)，自动避免退化
		std::pair<SPAunit_vector, SPAunit_vector> build_plane_basis(
			const SPAunit_vector& normal) {
			SPAvector reference(1.0, 0.0, 0.0);
			if (std::fabs(normal % SPAunit_vector(reference)) > 0.9) {
				reference = SPAvector(0.0, 1.0, 0.0);
			}

			const SPAvector tangent = normal * reference;
			const SPAunit_vector u = normalise(tangent);
			const SPAunit_vector v = normalise(normal * u);
			return { u, v };
		}

		/// 将三维点投影到平面的 UV 坐标
		std::array<double, 2> project_to_plane_uv(const SPAposition& point,
			const SPAposition& origin,
			const SPAunit_vector& u_axis,
			const SPAunit_vector& v_axis) {
			const SPAvector offset = point - origin;
			return { offset % u_axis, offset % v_axis };
		}

		/// 判断二维点是否在线段上（叉积共线 + 投影在范围内）
		bool point_on_segment_2d(double px, double py,
			double ax, double ay,
			double bx, double by,
			double epsilon) {
			const double cross = (px - ax) * (by - ay) - (py - ay) * (bx - ax);
			if (std::fabs(cross) > epsilon) {
				return false;
			}

			const double dot = (px - ax) * (bx - ax) + (py - ay) * (by - ay);
			if (dot < -epsilon) {
				return false;
			}

			const double length_squared =
				(bx - ax) * (bx - ax) + (by - ay) * (by - ay);
			return dot <= length_squared + epsilon;
		}

		/// 射线法判断二维点是否在投影环内部
		bool point_in_projected_loop(const std::array<double, 2>& point,
			const ProjectedLoop& loop,
			double epsilon) {
			if (loop.uv_points.size() < 3U) {
				return false;
			}

			bool inside = false;
			for (std::size_t index = 0; index < loop.uv_points.size(); ++index) {
				const auto& current = loop.uv_points[index];
				const auto& next = loop.uv_points[(index + 1U) % loop.uv_points.size()];
				if (point_on_segment_2d(point[0], point[1], current[0], current[1],
					next[0], next[1], epsilon)) {
					return true;
				}

				const bool intersects =
					((current[1] > point[1]) != (next[1] > point[1])) &&
					(point[0] < (next[0] - current[0]) * (point[1] - current[1]) /
						(next[1] - current[1] + 1.0e-30) + current[0]);
				if (intersects) {
					inside = !inside;
				}
			}

			return inside;
		}

		/// 从 PlaneFace 构建平面几何信息（拟合平面 + 投影环 + UV 包围盒）
		PlaneFaceGeometry build_plane_face_geometry(const PlaneFace& face) {
			if (face.boundaries.outer_loops.empty()) {
				throw std::runtime_error("平面板主面缺少外边界，无法构造面域。");
			}

			SPAposition origin;
			SPAunit_vector normal;
			if (!fit_plane_from_loop(face.boundaries.outer_loops.front(), origin, normal)) {
				throw std::runtime_error("平面板主面外边界无法稳定拟合平面。");
			}

			const auto [u_axis, v_axis] = build_plane_basis(normal);
			PlaneFaceGeometry geometry;
			geometry.origin = origin;
			geometry.normal = normal;
			geometry.outward_normal = normal;
			geometry.u_axis = u_axis;
			geometry.v_axis = v_axis;

			auto build_projected = [&](const Polyline3& loop) {
				ProjectedLoop projected;
				projected.world_points = loop;
				for (const SPAposition& point : loop) {
					const auto uv = project_to_plane_uv(point, origin, u_axis, v_axis);
					projected.uv_points.push_back(uv);
					expand_uv_bounds(projected, uv);
				}
				return projected;
				};

			for (const Polyline3& loop : face.boundaries.outer_loops) {
				geometry.outer_loops.push_back(build_projected(loop));
			}
			for (const Polyline3& loop : face.boundaries.inner_loops) {
				geometry.inner_loops.push_back(build_projected(loop));
			}
			return geometry;
		}

		/// 在两组对应边界环之间生成侧壁三角面
		void append_side_wall_triangles(const std::vector<Polyline3>& first_loops,
			const std::vector<Polyline3>& second_loops,
			std::vector<Triangle3>& side_triangles) {
			const std::size_t loop_count = std::min(first_loops.size(), second_loops.size());
			for (std::size_t loop_index = 0; loop_index < loop_count; ++loop_index) {
				const Polyline3& first = first_loops[loop_index];
				const Polyline3& second = second_loops[loop_index];
				if (first.size() < 2U || second.size() < 2U || first.size() != second.size()) {
					continue;
				}

				for (std::size_t point_index = 0; point_index < first.size(); ++point_index) {
					const std::size_t next_index = (point_index + 1U) % first.size();
					const SPAposition& a0 = first[point_index];
					const SPAposition& a1 = first[next_index];
					const SPAposition& b0 = second[point_index];
					const SPAposition& b1 = second[next_index];
					side_triangles.push_back(make_triangle3(a0, a1, b0));
					side_triangles.push_back(make_triangle3(b0, a1, b1));
				}
			}
		}

		/// 从 PlanePanel 构建完整几何（两侧主面 + 确定外法线方向 + 侧壁 + BVH）
		PlanePanelGeometry build_plane_panel_geometry(const PlanePanel& panel) {
			PlanePanelGeometry geometry;
			geometry.main_faces.push_back(build_plane_face_geometry(panel.face_a));
			geometry.main_faces.push_back(build_plane_face_geometry(panel.face_b));
			if (geometry.main_faces.size() == 2U) {
				const SPAvector connector =
					geometry.main_faces[1].origin - geometry.main_faces[0].origin;
				const PlaneFaceGeometry& face_a = geometry.main_faces[0];
				const PlaneFaceGeometry& face_b = geometry.main_faces[1];
				geometry.main_faces[0].outward_normal =
					(connector % face_a.normal) > 0.0
					? normalise(-face_a.normal.vector())
					: face_a.normal;
				geometry.main_faces[1].outward_normal =
					(connector % face_b.normal) > 0.0
					? face_b.normal
					: normalise(-face_b.normal.vector());
			}
			append_side_wall_triangles(panel.face_a.boundaries.outer_loops,
				panel.face_b.boundaries.outer_loops,
				geometry.side_triangles);
			append_side_wall_triangles(panel.face_a.boundaries.inner_loops,
				panel.face_b.boundaries.inner_loops,
				geometry.side_triangles);
			build_triangle_bvh(geometry.side_triangles, geometry.side_bvh_nodes);
			return geometry;
		}

		/// 查找曲面上离给定点最近的点及其法向量
		std::optional<SurfaceClosestPoint> closest_point_on_surface_geometry(
			const SPAposition& point,
			const SurfaceGeometry& geometry) {
			double best_distance_squared = std::numeric_limits<double>::infinity();
			std::optional<SurfaceClosestPoint> best_point;
			visit_surface_triangles(geometry,
				[&](const SPAbox& box) {
					return point_box_distance_squared(point, box) <= best_distance_squared;
				},
				[&](const Triangle3& triangle) {
					const SPAposition closest = closest_point_on_triangle(point, triangle);
					const double distance_squared = (point - closest).len_sq();
					if (distance_squared < best_distance_squared) {
						const SPAvector triangle_normal =
							(triangle.b - triangle.a) * (triangle.c - triangle.a);
						if (triangle_normal.len_sq() <= 1.0e-18) {
							return;
						}
						best_distance_squared = distance_squared;
						best_point = SurfaceClosestPoint{
							closest,
							normalise(triangle_normal)
						};
					}
				});
			return best_point;
		}

		/// 从 SurfacePanel 构建曲面几何（展开三角形 + BVH）
		SurfaceGeometry build_surface_geometry(const SurfacePanel& panel) {
			SurfaceGeometry geometry;
			for (const SurfacePatch& patch : panel.surfaces) {
				for (const Triangle& triangle : patch.triangles) {
					if (triangle[0] >= patch.vertices.size() || triangle[1] >= patch.vertices.size() ||
						triangle[2] >= patch.vertices.size()) {
						continue;
					}
					geometry.triangles.push_back(make_triangle3(patch.vertices[triangle[0]],
						patch.vertices[triangle[1]],
						patch.vertices[triangle[2]]));
				}
			}
			build_surface_bvh(geometry);
			return geometry;
		}

		/// 点到曲面几何的最短距离（BVH 加速裁剪）
		double point_to_surface_geometry_distance(const SPAposition& point,
			const SurfaceGeometry& geometry) {
			double best_distance_squared = std::numeric_limits<double>::infinity();
			visit_surface_triangles(geometry,
				[&](const SPAbox& box) {
					return point_box_distance_squared(point, box) <= best_distance_squared;
				},
				[&](const Triangle3& triangle) {
					best_distance_squared = std::min(best_distance_squared,
						point_triangle_distance_squared(point, triangle));
				});
			return std::sqrt(best_distance_squared);
		}

		/// 精确计算线段与三角形的共面重叠区间
		std::vector<Interval> exact_segment_triangle_coplanar_intervals(
			const SPAposition& start,
			const SPAposition& end,
			const Triangle3& triangle);

		/// Möller-Trumbore 射线-三角形交点检测，返回交点到射线原点距离
		std::optional<double> ray_triangle_intersection_distance(
			const SPAposition& origin,
			const SPAvector& direction,
			const Triangle3& triangle) {
			const SPAvector edge_ab = triangle.b - triangle.a;
			const SPAvector edge_ac = triangle.c - triangle.a;
			const SPAvector p = direction * edge_ac;
			const double determinant = edge_ab % p;
			if (std::fabs(determinant) <= 1.0e-12) {
				return std::nullopt;
			}

			const double inverse_determinant = 1.0 / determinant;
			const SPAvector t = origin - triangle.a;
			const double u = (t % p) * inverse_determinant;
			if (u < -kIntervalEpsilon || u > 1.0 + kIntervalEpsilon) {
				return std::nullopt;
			}

			const SPAvector q = t * edge_ab;
			const double v = (direction % q) * inverse_determinant;
			if (v < -kIntervalEpsilon || u + v > 1.0 + kIntervalEpsilon) {
				return std::nullopt;
			}

			const double distance = (edge_ac % q) * inverse_determinant;
			if (distance <= 1.0e-9) {
				return std::nullopt;
			}
			return distance;
		}

		/// 线段-三角形交点检测，返回交点在线段上的参数 t ∈ [0,1]；共面返回 nullopt
		std::optional<double> segment_triangle_intersection_parameter(
			const SPAposition& start,
			const SPAposition& end,
			const Triangle3& triangle) {
			const SPAvector direction = end - start;
			const SPAvector edge_ab = triangle.b - triangle.a;
			const SPAvector edge_ac = triangle.c - triangle.a;
			const SPAvector triangle_normal = edge_ab * edge_ac;
			if (triangle_normal.len_sq() <= 1.0e-18) {
				return std::nullopt;
			}

			const double start_plane = triangle_normal % (start - triangle.a);
			const double end_plane = triangle_normal % (end - triangle.a);
			const double plane_epsilon =
				1.0e-8 * std::max(1.0, std::sqrt(triangle_normal.len_sq()));
			if (std::fabs(start_plane) <= plane_epsilon &&
				std::fabs(end_plane) <= plane_epsilon) {
				return std::nullopt;
			}

			const SPAvector p = direction * edge_ac;
			const double determinant = edge_ab % p;
			if (std::fabs(determinant) <= 1.0e-12) {
				return std::nullopt;
			}

			const double inverse_determinant = 1.0 / determinant;
			const SPAvector t = start - triangle.a;
			const double u = (t % p) * inverse_determinant;
			if (u < -kIntervalEpsilon || u > 1.0 + kIntervalEpsilon) {
				return std::nullopt;
			}

			const SPAvector q = t * edge_ab;
			const double v = (direction % q) * inverse_determinant;
			if (v < -kIntervalEpsilon || u + v > 1.0 + kIntervalEpsilon) {
				return std::nullopt;
			}

			const double parameter = (edge_ac % q) * inverse_determinant;
			if (parameter < -kIntervalEpsilon || parameter > 1.0 + kIntervalEpsilon) {
				return std::nullopt;
			}
			return clamp01(parameter);
		}

		/// 判断点是否在曲面体内部（先检查边界，再用射线法奇偶检测）
		SolidPointClassification point_in_surface_solid(
			const SPAposition& point,
			const SurfaceGeometry& geometry) {
			ScopedDurationAccumulator timer(g_weld_profiling_data.point_in_solid_ms);
			++g_weld_profiling_data.point_in_solid_call_count;
			constexpr double kBoundaryEpsilon = 1.0e-6;
			const double boundary_epsilon_squared = kBoundaryEpsilon * kBoundaryEpsilon;
			bool on_boundary = false;
			visit_surface_triangles(geometry,
				[&](const SPAbox& box) {
					return point_box_distance_squared(point, box) <= boundary_epsilon_squared;
				},
				[&](const Triangle3& triangle) {
					++g_weld_profiling_data.point_in_solid_boundary_triangle_tests;
					if (on_boundary) {
						return;
					}
					if (point_triangle_distance_squared(point, triangle) <= boundary_epsilon_squared) {
						on_boundary = true;
					}
				});
			if (on_boundary) {
				return SolidPointClassification::OnBoundary;
			}

			const SPAvector ray_direction(0.6113721250000001,
				0.3347818710000000,
				0.7174391230000000);
			std::vector<double> hit_distances;
			visit_surface_triangles(geometry,
				[&](const SPAbox& box) {
					return ray_intersects_box(point, ray_direction, box);
				},
				[&](const Triangle3& triangle) {
					++g_weld_profiling_data.point_in_solid_ray_triangle_tests;
					const auto distance =
						ray_triangle_intersection_distance(point, ray_direction, triangle);
					if (distance.has_value()) {
						hit_distances.push_back(*distance);
					}
				});

			std::sort(hit_distances.begin(), hit_distances.end());
			hit_distances.erase(std::unique(hit_distances.begin(), hit_distances.end(),
				[](double left, double right) {
					return nearly_equal(left, right, 1.0e-7);
				}), hit_distances.end());
			return (hit_distances.size() % 2U == 1U)
				? SolidPointClassification::Inside
				: SolidPointClassification::Outside;
		}

		/// 根据线段长度自适应选取探测步长
		double segment_parameter_probe_step(const SPAposition& start,
			const SPAposition& end) {
			const double segment_length = (end - start).len();
			if (segment_length <= 1.0e-12) {
				return 1.0e-3;
			}
			return std::max(1.0e-9, std::min(1.0e-4, 1.0e-6 / segment_length));
		}

		/// 在线段上采样点判断其在曲面体内/外/边界（逐步扩大步长探测）
		SolidPointClassification sample_segment_classification(
			const SPAposition& start,
			const SPAposition& end,
			double base_t,
			int direction,
			const SurfaceGeometry& geometry) {
			double step = segment_parameter_probe_step(start, end);
			for (int attempt = 0; attempt < 8; ++attempt) {
				const double sample_t = clamp01(base_t + direction * step);
				if (!nearly_equal(sample_t, base_t, step * 0.25)) {
					const SolidPointClassification classification =
						point_in_surface_solid(interpolate_position(start, end, sample_t), geometry);
					if (classification != SolidPointClassification::OnBoundary) {
						return classification;
					}
				}
				step *= 2.0;
			}
			return SolidPointClassification::OnBoundary;
		}

		/// 收集线段穿越曲面体边界的所有事件点（参数 t + Crossing/Touching 类型）
		std::vector<SolidIntersectionEvent> segment_surface_solid_events(
			const SPAposition& start,
			const SPAposition& end,
			const SurfaceGeometry& geometry) {
			ScopedDurationAccumulator timer(g_weld_profiling_data.segment_solid_events_ms);
			++g_weld_profiling_data.segment_solid_event_call_count;
			std::vector<double> parameters;
			visit_surface_triangles(geometry,
				[&](const SPAbox& box) {
					return segment_intersects_box(start, end, box);
				},
				[&](const Triangle3& triangle) {
					++g_weld_profiling_data.segment_event_triangle_tests;
					const auto parameter =
						segment_triangle_intersection_parameter(start, end, triangle);
					if (parameter.has_value() &&
						*parameter > kIntervalEpsilon &&
						*parameter < 1.0 - kIntervalEpsilon) {
						parameters.push_back(*parameter);
					}
				});

			std::sort(parameters.begin(), parameters.end());
			parameters.erase(std::unique(parameters.begin(), parameters.end(),
				[](double left, double right) {
					return nearly_equal(left, right);
				}), parameters.end());

			std::vector<SolidIntersectionEvent> events;
			for (const double parameter : parameters) {
				const SolidPointClassification before =
					sample_segment_classification(start, end, parameter, -1, geometry);
				const SolidPointClassification after =
					sample_segment_classification(start, end, parameter, 1, geometry);
				const SolidIntersectionEvent::Type type =
					(before != SolidPointClassification::OnBoundary &&
						after != SolidPointClassification::OnBoundary &&
						before != after)
					? SolidIntersectionEvent::Type::Crossing
					: SolidIntersectionEvent::Type::Touching;
				events.push_back({ parameter, type });
			}
			return events;
		}

		/// 拓扑共面区间：线段与曲面三角形共面重叠的参数区间集合
		std::vector<Interval> exact_segment_topo_coplanar_intervals(
			const SPAposition& start,
			const SPAposition& end,
			const SurfaceGeometry& geometry) {
			ScopedDurationAccumulator timer(g_weld_profiling_data.topo_coplanar_ms);
			++g_weld_profiling_data.topo_query_count;
			std::vector<Interval> result;
			visit_surface_triangles(geometry,
				[&](const SPAbox& box) {
					return segment_intersects_box(start, end, box);
				},
				[&](const Triangle3& triangle) {
					++g_weld_profiling_data.topo_triangle_tests;
					const std::vector<Interval> intervals =
						exact_segment_triangle_coplanar_intervals(start, end, triangle);
					result.insert(result.end(), intervals.begin(), intervals.end());
				});
			return normalise_intervals(std::move(result));
		}

		/// 计算线段上位于曲面体内部的参数区间（基于穿越事件追踪）
		std::vector<Interval> exact_segment_inside_intervals_on_subsegment(
			const SPAposition& start,
			const SPAposition& end,
			const SurfaceGeometry& geometry) {
			ScopedDurationAccumulator timer(g_weld_profiling_data.inside_ms);
			++g_weld_profiling_data.inside_query_count;
			const std::vector<SolidIntersectionEvent> events =
				segment_surface_solid_events(start, end, geometry);
			const SolidPointClassification start_state =
				sample_segment_classification(start, end, 0.0, 1, geometry);

			bool inside = (start_state == SolidPointClassification::Inside);
			double current_start = 0.0;
			std::vector<Interval> result;
			for (const SolidIntersectionEvent& event : events) {
				if (event.type != SolidIntersectionEvent::Type::Crossing) {
					continue;
				}
				if (inside) {
					result.push_back({ current_start, event.t });
				}
				else {
					current_start = event.t;
				}
				inside = !inside;
			}
			if (inside) {
				result.push_back({ current_start, 1.0 });
			}
			return normalise_intervals(std::move(result));
		}

		/// 点到平面板的最短距离：检查主面投影包含性 + 侧壁三角形距离
		double point_to_plane_panel_distance(const SPAposition& point,
			const PlanePanelGeometry& geometry) {
			double best_distance = std::numeric_limits<double>::infinity();

			for (const PlaneFaceGeometry& face : geometry.main_faces) {
				const SPAvector offset = point - face.origin;
				const double signed_distance = offset % face.normal;
				const auto uv = project_to_plane_uv(point, face.origin, face.u_axis, face.v_axis);

				bool in_outer = false;
				for (const ProjectedLoop& loop : face.outer_loops) {
					if (point_in_projected_loop(uv, loop, 1.0e-8)) {
						in_outer = true;
						break;
					}
				}
				if (!in_outer) {
					continue;
				}

				bool in_inner = false;
				for (const ProjectedLoop& loop : face.inner_loops) {
					if (point_in_projected_loop(uv, loop, 1.0e-8)) {
						in_inner = true;
						break;
					}
				}
				if (!in_inner) {
					best_distance = std::min(best_distance, std::fabs(signed_distance));
				}
			}

			double best_side_squared = std::numeric_limits<double>::infinity();
			visit_plane_side_triangles(geometry,
				[&](const SPAbox& box) {
					return point_box_distance_squared(point, box) <= best_side_squared;
				},
				[&](const Triangle3& triangle) {
					best_side_squared = std::min(best_side_squared,
						point_triangle_distance_squared(point, triangle));
				});
			if (best_side_squared < std::numeric_limits<double>::infinity()) {
				best_distance = std::min(best_distance, std::sqrt(best_side_squared));
			}

			return best_distance;
		}

		/// 二维叉积
		double cross_2d(const std::array<double, 2>& left,
			const std::array<double, 2>& right) {
			return left[0] * right[1] - left[1] * right[0];
		}

		/// 收集 UV 参数射线上与投影环各边的交点参数
		std::vector<double> collect_path_edge_intersections(
			const std::array<double, 2>& start,
			const std::array<double, 2>& end,
			const std::vector<ProjectedLoop>& loops) {
			std::vector<double> parameters = { 0.0, 1.0 };
			const std::array<double, 2> direction{ end[0] - start[0], end[1] - start[1] };
			const bool use_x_axis = std::fabs(direction[0]) >= std::fabs(direction[1]);

			auto maybe_parameter_for_point =
				[&](const std::array<double, 2>& point) -> std::optional<double> {
				if (use_x_axis) {
					if (std::fabs(direction[0]) <= kIntervalEpsilon) {
						return std::nullopt;
					}
					return (point[0] - start[0]) / direction[0];
				}
				if (std::fabs(direction[1]) <= kIntervalEpsilon) {
					return std::nullopt;
				}
				return (point[1] - start[1]) / direction[1];
				};

			for (const ProjectedLoop& loop : loops) {
				if (loop.uv_points.size() < 2U) {
					continue;
				}
				if (!uv_segment_box_overlaps_loop_box(start, end, loop)) {
					continue;
				}
				for (std::size_t index = 0; index < loop.uv_points.size(); ++index) {
					const auto& edge_start = loop.uv_points[index];
					const auto& edge_end =
						loop.uv_points[(index + 1U) % loop.uv_points.size()];
					const std::array<double, 2> edge_direction{
						edge_end[0] - edge_start[0],
						edge_end[1] - edge_start[1]
					};
					const std::array<double, 2> delta{
						edge_start[0] - start[0],
						edge_start[1] - start[1]
					};
					const double denominator = cross_2d(direction, edge_direction);
					if (std::fabs(denominator) <= kIntervalEpsilon) {
						const double collinear_measure = cross_2d(delta, direction);
						if (std::fabs(collinear_measure) > kIntervalEpsilon) {
							continue;
						}
						if (const auto t = maybe_parameter_for_point(edge_start)) {
							parameters.push_back(*t);
						}
						if (const auto t = maybe_parameter_for_point(edge_end)) {
							parameters.push_back(*t);
						}
						continue;
					}

					const double t = cross_2d(delta, edge_direction) / denominator;
					const double u = cross_2d(delta, direction) / denominator;
					if (t >= -kIntervalEpsilon && t <= 1.0 + kIntervalEpsilon &&
						u >= -kIntervalEpsilon && u <= 1.0 + kIntervalEpsilon) {
						parameters.push_back(clamp01(t));
					}
				}
			}

			std::sort(parameters.begin(), parameters.end());
			parameters.erase(std::unique(parameters.begin(), parameters.end(),
				[](double left, double right) {
					return nearly_equal(left, right);
				}), parameters.end());
			return parameters;
		}

		/// 将 UV 射线与投影环的交点参数追加到列表
		void append_path_edge_intersections(std::vector<double>& parameters,
			const std::array<double, 2>& start,
			const std::array<double, 2>& end,
			const std::vector<ProjectedLoop>& loops) {
			const std::array<double, 2> direction{ end[0] - start[0], end[1] - start[1] };
			const bool use_x_axis = std::fabs(direction[0]) >= std::fabs(direction[1]);

			auto maybe_parameter_for_point =
				[&](const std::array<double, 2>& point) -> std::optional<double> {
				if (use_x_axis) {
					if (std::fabs(direction[0]) <= kIntervalEpsilon) {
						return std::nullopt;
					}
					return (point[0] - start[0]) / direction[0];
				}
				if (std::fabs(direction[1]) <= kIntervalEpsilon) {
					return std::nullopt;
				}
				return (point[1] - start[1]) / direction[1];
				};

			for (const ProjectedLoop& loop : loops) {
				if (loop.uv_points.size() < 2U ||
					!uv_segment_box_overlaps_loop_box(start, end, loop)) {
					continue;
				}
				for (std::size_t index = 0; index < loop.uv_points.size(); ++index) {
					const auto& edge_start = loop.uv_points[index];
					const auto& edge_end =
						loop.uv_points[(index + 1U) % loop.uv_points.size()];
					const std::array<double, 2> edge_direction{
						edge_end[0] - edge_start[0],
						edge_end[1] - edge_start[1]
					};
					const std::array<double, 2> delta{
						edge_start[0] - start[0],
						edge_start[1] - start[1]
					};
					const double denominator = cross_2d(direction, edge_direction);
					if (std::fabs(denominator) <= kIntervalEpsilon) {
						const double collinear_measure = cross_2d(delta, direction);
						if (std::fabs(collinear_measure) > kIntervalEpsilon) {
							continue;
						}
						if (const auto t = maybe_parameter_for_point(edge_start)) {
							parameters.push_back(*t);
						}
						if (const auto t = maybe_parameter_for_point(edge_end)) {
							parameters.push_back(*t);
						}
						continue;
					}

					const double t = cross_2d(delta, edge_direction) / denominator;
					const double u = cross_2d(delta, direction) / denominator;
					if (t >= -kIntervalEpsilon && t <= 1.0 + kIntervalEpsilon &&
						u >= -kIntervalEpsilon && u <= 1.0 + kIntervalEpsilon) {
						parameters.push_back(clamp01(t));
					}
				}
			}
		}

		/// 根据法向量选择最稳定的两个投影轴（取法向量分量最小的两个轴）
		std::pair<int, int> dominant_projection_axes(const SPAvector& normal) {
			const double abs_x = std::fabs(normal.x());
			const double abs_y = std::fabs(normal.y());
			const double abs_z = std::fabs(normal.z());
			if (abs_x >= abs_y && abs_x >= abs_z) {
				return { 1, 2 };
			}
			if (abs_y >= abs_x && abs_y >= abs_z) {
				return { 0, 2 };
			}
			return { 0, 1 };
		}

		/// 将三维点投影到指定的两个坐标轴
		std::array<double, 2> project_to_axes(const SPAposition& point,
			int first_axis,
			int second_axis) {
			return { point[first_axis], point[second_axis] };
		}

		/// 将三角形投影到两个坐标轴上构建 ProjectedLoop
		ProjectedLoop build_projected_triangle_loop(const Triangle3& triangle,
			int first_axis,
			int second_axis) {
			ProjectedLoop loop;
			loop.world_points = { triangle.a, triangle.b, triangle.c };
			loop.uv_points = {
				project_to_axes(triangle.a, first_axis, second_axis),
				project_to_axes(triangle.b, first_axis, second_axis),
				project_to_axes(triangle.c, first_axis, second_axis)
			};
			for (const auto& uv : loop.uv_points) {
				expand_uv_bounds(loop, uv);
			}
			return loop;
		}

		/// 计算共面线段与三角形的重叠区间：投影到二维后用边交点采样+包含性测试
		std::vector<Interval> exact_segment_triangle_coplanar_intervals(
			const SPAposition& start,
			const SPAposition& end,
			const Triangle3& triangle) {
			const SPAvector direction = end - start;
			const SPAvector triangle_normal = (triangle.b - triangle.a) * (triangle.c - triangle.a);
			if (triangle_normal.len_sq() <= 1.0e-18) {
				return {};
			}

			const double start_plane = triangle_normal % (start - triangle.a);
			const double end_plane = triangle_normal % (end - triangle.a);
			const double direction_plane = triangle_normal % direction;
			const double plane_epsilon =
				1.0e-8 * std::max(1.0, std::sqrt(triangle_normal.len_sq()));
			if (std::fabs(start_plane) > plane_epsilon ||
				std::fabs(end_plane) > plane_epsilon ||
				std::fabs(direction_plane) > plane_epsilon) {
				return {};
			}

			const auto [first_axis, second_axis] = dominant_projection_axes(triangle_normal);
			const ProjectedLoop projected_triangle =
				build_projected_triangle_loop(triangle, first_axis, second_axis);
			const std::array<double, 2> start_uv =
				project_to_axes(start, first_axis, second_axis);
			const std::array<double, 2> end_uv =
				project_to_axes(end, first_axis, second_axis);
			const std::vector<double> parameters =
				collect_path_edge_intersections(start_uv, end_uv, { projected_triangle });

			std::vector<Interval> result;
			for (std::size_t index = 1; index < parameters.size(); ++index) {
				const double interval_start = parameters[index - 1U];
				const double interval_end = parameters[index];
				if (interval_end - interval_start <= kIntervalEpsilon) {
					continue;
				}

				const double midpoint = 0.5 * (interval_start + interval_end);
				const std::array<double, 2> midpoint_uv{
					start_uv[0] + (end_uv[0] - start_uv[0]) * midpoint,
					start_uv[1] + (end_uv[1] - start_uv[1]) * midpoint
				};
				if (point_in_projected_loop(midpoint_uv, projected_triangle, 1.0e-8)) {
					result.push_back({ interval_start, interval_end });
				}
			}
			return normalise_intervals(std::move(result));
		}

		/// 判断 UV 点是否在面的有效域内（在外轮廓内且不在内孔内）
		bool point_in_face_domain(const std::array<double, 2>& uv,
			const PlaneFaceGeometry& face) {
			bool in_outer = false;
			for (const ProjectedLoop& loop : face.outer_loops) {
				if (uv_box_contains_point(loop, uv) &&
					point_in_projected_loop(uv, loop, 1.0e-8)) {
					in_outer = true;
					break;
				}
			}
			if (!in_outer) {
				return false;
			}
			for (const ProjectedLoop& loop : face.inner_loops) {
				if (uv_box_contains_point(loop, uv) &&
					point_in_projected_loop(uv, loop, 1.0e-8)) {
					return false;
				}
			}
			return true;
		}

		/// 计算线段与平面面板主面的命中区间（带状距离 + UV 域约束）
		std::vector<Interval> exact_segment_face_intervals(const SPAposition& start,
			const SPAposition& end,
			const PlaneFaceGeometry& face,
			double tolerance) {
			const SPAvector direction = end - start;
			const std::vector<Interval> slab_intervals = clip_intervals_to_unit(
				solve_quadratic_inequality(
					distance_to_plane_band_function(start, direction, face.origin,
						face.normal, tolerance),
					true));
			if (slab_intervals.empty()) {
				return {};
			}

			const auto start_uv =
				project_to_plane_uv(start, face.origin, face.u_axis, face.v_axis);
			const auto end_uv =
				project_to_plane_uv(end, face.origin, face.u_axis, face.v_axis);

			std::vector<double> parameters = { 0.0, 1.0 };
			append_path_edge_intersections(parameters, start_uv, end_uv, face.outer_loops);
			append_path_edge_intersections(parameters, start_uv, end_uv, face.inner_loops);
			std::sort(parameters.begin(), parameters.end());
			parameters.erase(std::unique(parameters.begin(), parameters.end(),
				[](double left, double right) {
					return nearly_equal(left, right);
				}), parameters.end());

			std::vector<Interval> domain_intervals;
			for (std::size_t index = 1; index < parameters.size(); ++index) {
				const double interval_start = parameters[index - 1U];
				const double interval_end = parameters[index];
				if (interval_end - interval_start <= kIntervalEpsilon) {
					continue;
				}
				const double midpoint = 0.5 * (interval_start + interval_end);
				const std::array<double, 2> midpoint_uv{
					start_uv[0] + (end_uv[0] - start_uv[0]) * midpoint,
					start_uv[1] + (end_uv[1] - start_uv[1]) * midpoint
				};
				if (point_in_face_domain(midpoint_uv, face)) {
					domain_intervals.push_back({ interval_start, interval_end });
				}
			}

			return intersect_interval_sets(slab_intervals,
				normalise_intervals(std::move(domain_intervals)));
		}

		/// 计算线段与平面板整体的命中区间（主面 + 侧壁三角形）
		std::vector<Interval> exact_segment_plane_panel_intervals(const SPAposition& start,
			const SPAposition& end,
			const PlanePanelGeometry& geometry,
			double tolerance);

		/// 计算线段与曲面三角形的精确距离区间（基于代数不等式求解）
		std::vector<Interval> exact_segment_surface_intervals(const SPAposition& start,
			const SPAposition& end,
			const SurfaceGeometry& geometry,
			double tolerance) {
			ScopedDurationAccumulator timer(g_weld_profiling_data.near_ms);
			++g_weld_profiling_data.near_query_count;
			std::vector<Interval> result;
			visit_surface_triangles(geometry,
				[&](const SPAbox& box) {
					return segment_intersects_box(start, end, expand_box(box, tolerance));
				},
				[&](const Triangle3& triangle) {
					++g_weld_profiling_data.near_triangle_tests;
					const std::vector<Interval> intervals =
						exact_segment_triangle_intervals(start, end, triangle, tolerance);
					result.insert(result.end(), intervals.begin(), intervals.end());
				});
			return normalise_intervals(std::move(result));
		}

		/// 从平面板候选环出发，综合拓扑共面/体内/近距三种方式计算线段与曲面的焊缝区间
		std::vector<Interval> exact_segment_surface_intervals_from_plane_candidate(
			const SPAposition& start,
			const SPAposition& end,
			const SurfaceGeometry& geometry,
			double tolerance,
			const SPAunit_vector& /*outward_normal*/,
			const PlanePanelGeometry& /*source_geometry*/) {
			const std::vector<Interval> topo_intervals =
				exact_segment_topo_coplanar_intervals(start, end, geometry);
			const std::vector<Interval> non_coplanar_intervals =
				complement_on_unit_interval(topo_intervals);

			std::vector<Interval> inside_intervals;
			for (const Interval& interval : non_coplanar_intervals) {
				const SPAposition subsegment_start =
					interpolate_position(start, end, interval.start);
				const SPAposition subsegment_end =
					interpolate_position(start, end, interval.end);
				const std::vector<Interval> local_inside =
					exact_segment_inside_intervals_on_subsegment(
						subsegment_start, subsegment_end, geometry);
				const std::vector<Interval> mapped_inside =
					remap_intervals_to_parent(local_inside, interval);
				inside_intervals.insert(inside_intervals.end(),
					mapped_inside.begin(), mapped_inside.end());
			}
			inside_intervals = normalise_intervals(std::move(inside_intervals));

			std::vector<Interval> covered_intervals = topo_intervals;
			covered_intervals.insert(covered_intervals.end(),
				inside_intervals.begin(), inside_intervals.end());
			covered_intervals = normalise_intervals(std::move(covered_intervals));
			const std::vector<Interval> outside_intervals =
				complement_on_unit_interval(covered_intervals);

			std::vector<Interval> near_intervals;
			for (const Interval& interval : outside_intervals) {
				const SPAposition subsegment_start =
					interpolate_position(start, end, interval.start);
				const SPAposition subsegment_end =
					interpolate_position(start, end, interval.end);
				const std::vector<Interval> local_near =
					exact_segment_surface_intervals(
						subsegment_start, subsegment_end, geometry, tolerance);
				const std::vector<Interval> mapped_near =
					remap_intervals_to_parent(local_near, interval);
				near_intervals.insert(near_intervals.end(),
					mapped_near.begin(), mapped_near.end());
			}
			near_intervals = normalise_intervals(std::move(near_intervals));

			std::vector<Interval> weld_intervals = topo_intervals;
			weld_intervals.insert(weld_intervals.end(),
				inside_intervals.begin(), inside_intervals.end());
			weld_intervals.insert(weld_intervals.end(),
				near_intervals.begin(), near_intervals.end());
			return normalise_intervals(std::move(weld_intervals));
		}

		/// 计算线段与平面板的命中区间（主面带状距离 + 侧壁三角形距离）
		std::vector<Interval> exact_segment_plane_panel_intervals(const SPAposition& start,
			const SPAposition& end,
			const PlanePanelGeometry& geometry,
			double tolerance) {
			std::vector<Interval> result;
			for (const PlaneFaceGeometry& face : geometry.main_faces) {
				const std::vector<Interval> face_intervals =
					exact_segment_face_intervals(start, end, face, tolerance);
				result.insert(result.end(), face_intervals.begin(), face_intervals.end());
			}
			visit_plane_side_triangles(geometry,
				[&](const SPAbox& box) {
					return segment_intersects_box(start, end, expand_box(box, tolerance));
				},
				[&](const Triangle3& triangle) {
					const std::vector<Interval> intervals =
						exact_segment_triangle_intervals(start, end, triangle, tolerance);
					result.insert(result.end(), intervals.begin(), intervals.end());
				});
			return normalise_intervals(std::move(result));
		}

		/// 收集曲面板的所有候选边界环
		std::vector<Polyline3> hanfeng_collect_surface_candidate_loops(
			const SurfacePanel& panel) {
			std::vector<Polyline3> loops;
			for (const SurfacePatch& patch : panel.surfaces) {
				loops.insert(loops.end(), patch.boundaries.outer_loops.begin(),
					patch.boundaries.outer_loops.end());
				loops.insert(loops.end(), patch.boundaries.inner_loops.begin(),
					patch.boundaries.inner_loops.end());
			}
			return loops;
		}

		/// 收集平面板的所有候选边界环（含外法线方向）
		std::vector<PlaneCandidateLoop> hanfeng_collect_plane_candidate_loops(
			const PlanePanelGeometry& geometry) {
			std::vector<PlaneCandidateLoop> loops;
			for (const PlaneFaceGeometry& face : geometry.main_faces) {
				for (const ProjectedLoop& loop : face.outer_loops) {
					loops.push_back(PlaneCandidateLoop{ loop.world_points, face.outward_normal });
				}
				for (const ProjectedLoop& loop : face.inner_loops) {
					loops.push_back(PlaneCandidateLoop{ loop.world_points, face.outward_normal });
				}
			}
			return loops;
		}

		/// 追加点到焊缝折线（距离过近时跳过）
		void hanfeng_append_point_if_needed(Polyline3& polyline,
			const SPAposition& point,
			double tolerance) {
			if (polyline.empty() ||
				std::sqrt((polyline.back() - point).len_sq()) > tolerance * 0.25) {
				polyline.push_back(point);
			}
		}

		/// 计算折线总长度
		double hanfeng_polyline_length(const Polyline3& polyline) {
			double length = 0.0;
			for (std::size_t index = 1; index < polyline.size(); ++index) {
				length += (polyline[index] - polyline[index - 1]).len();
			}
			return length;
		}

		/// 裁剪候选环：在线段参数空间上查询命中区间，连接为连续焊缝折线
		template <typename IntervalQuery>
		std::vector<WeldPolyline> hanfeng_clip_candidate_loop(
			const Polyline3& input_loop,
			double tolerance,
			IntervalQuery&& query_intervals) {
			std::vector<WeldPolyline> result;
			if (input_loop.size() < 2U) {
				return result;
			}

			Polyline3 loop = input_loop;
			if (!(loop.front() == loop.back())) {
				loop.push_back(loop.front());
			}

			Polyline3 current;
			for (std::size_t segment_index = 0; segment_index + 1U < loop.size();
				++segment_index) {
				const SPAposition& start = loop[segment_index];
				const SPAposition& end = loop[segment_index + 1U];
				std::vector<Interval> intervals =
					normalise_intervals(query_intervals(start, end));

				if (current.size() >= 2U &&
					(intervals.empty() || intervals.front().start > kIntervalEpsilon)) {
					if (hanfeng_polyline_length(current) > tolerance * 0.5) {
						result.push_back(WeldPolyline{ current });
					}
					current.clear();
				}

				for (std::size_t interval_index = 0; interval_index < intervals.size();
					++interval_index) {
					const Interval& interval = intervals[interval_index];
					if (interval.end - interval.start <= kIntervalEpsilon) {
						continue;
					}

					const SPAposition hit_start =
						interpolate_position(start, end, interval.start);
					const SPAposition hit_end =
						interpolate_position(start, end, interval.end);

					const bool continues_current =
						!current.empty() &&
						interval.start <= kIntervalEpsilon &&
						std::sqrt((current.back() - hit_start).len_sq()) <= tolerance * 0.25;
					if (!continues_current) {
						if (current.size() >= 2U &&
							hanfeng_polyline_length(current) > tolerance * 0.5) {
							result.push_back(WeldPolyline{ current });
						}
						current.clear();
						hanfeng_append_point_if_needed(current, hit_start, tolerance);
					}
					hanfeng_append_point_if_needed(current, hit_end, tolerance);

					const bool reaches_segment_end =
						interval.end >= 1.0 - kIntervalEpsilon;
					const bool has_gap_after =
						interval_index + 1U < intervals.size() &&
						intervals[interval_index + 1U].start - interval.end > kIntervalEpsilon;
					if (!reaches_segment_end || has_gap_after) {
						if (current.size() >= 2U &&
							hanfeng_polyline_length(current) > tolerance * 0.5) {
							result.push_back(WeldPolyline{ current });
						}
						current.clear();
					}
				}
			}

			if (current.size() >= 2U &&
				hanfeng_polyline_length(current) > tolerance * 0.5) {
				result.push_back(WeldPolyline{ current });
			}
			return result;
		}

		/// 计算点到折线的最短距离
		double hanfeng_point_to_polyline_distance(const SPAposition& point,
			const Polyline3& polyline) {
			if (polyline.empty()) {
				return std::numeric_limits<double>::infinity();
			}
			if (polyline.size() == 1U) {
				return (point - polyline.front()).len();
			}

			double best_distance_squared = std::numeric_limits<double>::infinity();
			for (std::size_t index = 1; index < polyline.size(); ++index) {
				best_distance_squared =
					std::min(best_distance_squared,
						point_segment_distance_squared(point, polyline[index - 1],
							polyline[index]));
			}
			return std::sqrt(best_distance_squared);
		}

		/// 判断两条折线是否等价（端点匹配 + 双向逐点距离检测）
		bool hanfeng_polylines_equivalent(const Polyline3& first,
			const Polyline3& second,
			double tolerance) {
			if (first.size() < 2U || second.size() < 2U) {
				return false;
			}

			const bool same_direction =
				(first.front() - second.front()).len() <= tolerance &&
				(first.back() - second.back()).len() <= tolerance;
			const bool reversed_direction =
				(first.front() - second.back()).len() <= tolerance &&
				(first.back() - second.front()).len() <= tolerance;
			if (!same_direction && !reversed_direction) {
				return false;
			}

			for (const SPAposition& point : first) {
				if (hanfeng_point_to_polyline_distance(point, second) > tolerance) {
					return false;
				}
			}
			for (const SPAposition& point : second) {
				if (hanfeng_point_to_polyline_distance(point, first) > tolerance) {
					return false;
				}
			}
			return true;
		}

		/// 焊缝折线去重：过滤过短折线，移除方向无关的等价重复
		std::vector<WeldPolyline> hanfeng_deduplicate_polylines(
			const std::vector<WeldPolyline>& polylines,
			double tolerance) {
			std::vector<WeldPolyline> unique_polylines;
			for (const WeldPolyline& polyline : polylines) {
				if (polyline.points.size() < 2U ||
					hanfeng_polyline_length(polyline.points) <= tolerance * 0.5) {
					continue;
				}

				bool duplicate = false;
				for (const WeldPolyline& existing : unique_polylines) {
					if (hanfeng_polylines_equivalent(
						polyline.points, existing.points, tolerance)) {
						duplicate = true;
						break;
					}
				}
				if (!duplicate) {
					unique_polylines.push_back(polyline);
				}
			}
			return unique_polylines;
		}

		bool hanfeng_points_touch(const SPAposition& first,
			const SPAposition& second,
			double tolerance) {
			return (first - second).len() <= tolerance;
		}

		Polyline3 hanfeng_oriented_polyline(const Polyline3& polyline,
			bool reverse) {
			if (!reverse) {
				return polyline;
			}
			return Polyline3(polyline.rbegin(), polyline.rend());
		}

		bool hanfeng_try_merge_touching_polylines(WeldPolyline& target,
			const WeldPolyline& candidate,
			double tolerance) {
			if (target.points.size() < 2U || candidate.points.size() < 2U) {
				return false;
			}

			const bool target_back_to_candidate_front =
				hanfeng_points_touch(target.points.back(), candidate.points.front(), tolerance);
			const bool target_back_to_candidate_back =
				hanfeng_points_touch(target.points.back(), candidate.points.back(), tolerance);
			const bool candidate_back_to_target_front =
				hanfeng_points_touch(candidate.points.back(), target.points.front(), tolerance);
			const bool candidate_front_to_target_front =
				hanfeng_points_touch(candidate.points.front(), target.points.front(), tolerance);

			if (target_back_to_candidate_front || target_back_to_candidate_back) {
				const Polyline3 source =
					hanfeng_oriented_polyline(candidate.points, target_back_to_candidate_back);
				target.points.insert(target.points.end(), source.begin() + 1, source.end());
				return true;
			}

			if (candidate_back_to_target_front || candidate_front_to_target_front) {
				const Polyline3 source =
					hanfeng_oriented_polyline(candidate.points, candidate_front_to_target_front);
				Polyline3 merged = source;
				merged.insert(merged.end(), target.points.begin() + 1, target.points.end());
				target.points = std::move(merged);
				return true;
			}

			return false;
		}

		std::vector<WeldPolyline> hanfeng_merge_touching_polylines(
			std::vector<WeldPolyline> polylines,
			double tolerance) {
			bool changed = true;
			while (changed) {
				changed = false;
				for (std::size_t i = 0; i < polylines.size() && !changed; ++i) {
					for (std::size_t j = i + 1U; j < polylines.size(); ++j) {
						if (hanfeng_try_merge_touching_polylines(
							polylines[i], polylines[j], tolerance)) {
							polylines.erase(polylines.begin() + static_cast<std::ptrdiff_t>(j));
							changed = true;
							break;
						}
					}
				}
			}
			return polylines;
		}

		struct FittedWeldSegment {
			std::size_t start_index = 0;
			std::size_t end_index = 0;
			bool is_arc = false;
			double signed_radius = 0.0;
			SPAposition center;
			SPAunit_vector u_axis;
			SPAunit_vector v_axis;
			double start_angle = 0.0;
			double total_angle = 0.0;
		};

		std::vector<SPAposition> dedupe_weld_points(const Polyline3& points,
			double tolerance) {
			std::vector<SPAposition> cleaned;
			for (const SPAposition& point : points) {
				if (cleaned.empty() || (point - cleaned.back()).len() > tolerance) {
					cleaned.push_back(point);
				}
			}
			return cleaned;
		}

		double point_to_line_distance(const SPAposition& point,
			const SPAposition& start,
			const SPAposition& end) {
			const SPAvector segment = end - start;
			const double length_sq = segment.len_sq();
			if (length_sq <= 1.0e-18) {
				return (point - start).len();
			}
			const double t = ((point - start) % segment) / length_sq;
			const SPAposition projected = start + segment * t;
			return (point - projected).len();
		}

		bool fit_line_segment(const std::vector<SPAposition>& points,
			std::size_t start_index,
			std::size_t end_index,
			double tolerance,
			FittedWeldSegment& segment) {
			if (end_index <= start_index) {
				return false;
			}

			const SPAposition& start = points[start_index];
			const SPAposition& end = points[end_index];
			if ((end - start).len() <= tolerance) {
				return false;
			}

			for (std::size_t index = start_index + 1U; index < end_index; ++index) {
				if (point_to_line_distance(points[index], start, end) > tolerance) {
					return false;
				}
			}

			segment = FittedWeldSegment{};
			segment.start_index = start_index;
			segment.end_index = end_index;
			return true;
		}

		bool solve_circle_2d(double x1,
			double y1,
			double x2,
			double y2,
			double x3,
			double y3,
			double& center_x,
			double& center_y) {
			const double determinant =
				2.0 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
			if (std::fabs(determinant) <= 1.0e-12) {
				return false;
			}

			const double p1 = x1 * x1 + y1 * y1;
			const double p2 = x2 * x2 + y2 * y2;
			const double p3 = x3 * x3 + y3 * y3;
			center_x =
				(p1 * (y2 - y3) + p2 * (y3 - y1) + p3 * (y1 - y2)) /
				determinant;
			center_y =
				(p1 * (x3 - x2) + p2 * (x1 - x3) + p3 * (x2 - x1)) /
				determinant;
			return true;
		}

		double normalize_angle_delta(double delta) {
			constexpr double pi = 3.14159265358979323846;
			while (delta <= -pi) {
				delta += 2.0 * pi;
			}
			while (delta > pi) {
				delta -= 2.0 * pi;
			}
			return delta;
		}

		bool fit_arc_segment(const std::vector<SPAposition>& points,
			std::size_t start_index,
			std::size_t end_index,
			double tolerance,
			FittedWeldSegment& segment) {
			if (end_index < start_index + 3U) {
				return false;
			}

			const SPAposition& start = points[start_index];
			const SPAposition& end = points[end_index];
			const SPAvector chord = end - start;
			if (chord.len() <= tolerance) {
				return false;
			}

			SPAvector normal_vector;
			for (std::size_t index = start_index + 1U; index < end_index; ++index) {
				normal_vector = (points[index] - start) * chord;
				if (normal_vector.len() > tolerance * chord.len()) {
					break;
				}
			}
			const SPAunit_vector normal = normalise(normal_vector);
			if (!normal.is_valid()) {
				return false;
			}

			const SPAunit_vector u_axis = normalise(chord);
			const SPAunit_vector v_axis = normalise(normal * u_axis);
			if (!v_axis.is_valid()) {
				return false;
			}

			std::vector<std::array<double, 2>> projected;
			projected.reserve(end_index - start_index + 1U);
			for (std::size_t index = start_index; index <= end_index; ++index) {
				const SPAvector offset = points[index] - start;
				const double plane_distance = std::fabs(offset % normal);
				if (plane_distance > tolerance) {
					return false;
				}
				projected.push_back({ offset % u_axis, offset % v_axis });
			}

			const std::size_t mid_offset = projected.size() / 2U;
			double center_x = 0.0;
			double center_y = 0.0;
			if (!solve_circle_2d(projected.front()[0], projected.front()[1],
				projected[mid_offset][0], projected[mid_offset][1],
				projected.back()[0], projected.back()[1], center_x, center_y)) {
				return false;
			}

			const double radius = std::hypot(
				projected.front()[0] - center_x, projected.front()[1] - center_y);
			if (radius <= tolerance) {
				return false;
			}

			std::vector<double> angles;
			angles.reserve(projected.size());
			double previous_raw_angle = 0.0;
			for (const std::array<double, 2>& point : projected) {
				const double radial_error =
					std::fabs(std::hypot(point[0] - center_x, point[1] - center_y) - radius);
				if (radial_error > tolerance) {
					return false;
				}
				const double angle = std::atan2(point[1] - center_y, point[0] - center_x);
				if (angles.empty()) {
					angles.push_back(angle);
				}
				else {
					angles.push_back(
						angles.back() + normalize_angle_delta(angle - previous_raw_angle));
				}
				previous_raw_angle = angle;
			}

			const double total_angle = angles.back() - angles.front();
			const double angle_span = std::fabs(total_angle);
			constexpr double pi = 3.14159265358979323846;
			constexpr double max_sample_angle = 12.0 * pi / 180.0;
			if (angle_span <= 1.0e-6 || angle_span > pi + 1.0e-6) {
				return false;
			}

			const int direction = total_angle > 0.0 ? 1 : -1;
			for (std::size_t index = 1U; index < angles.size(); ++index) {
				const double delta = angles[index] - angles[index - 1U];
				if ((delta > 0.0 ? 1 : -1) != direction ||
					std::fabs(delta) > max_sample_angle) {
					return false;
				}
			}

			segment = FittedWeldSegment{};
			segment.start_index = start_index;
			segment.end_index = end_index;
			segment.is_arc = true;
			segment.signed_radius = total_angle < 0.0 ? radius : -radius;
			segment.center = start + u_axis.vector() * center_x + v_axis.vector() * center_y;
			segment.u_axis = u_axis;
			segment.v_axis = v_axis;
			segment.start_angle = angles.front();
			segment.total_angle = total_angle;
			return true;
		}

		FittedWeldSegment select_next_weld_segment(
			const std::vector<SPAposition>& points,
			std::size_t start_index,
			double tolerance) {
			FittedWeldSegment best_arc;
			bool has_arc = false;
			for (std::size_t end_index = points.size() - 1U;
				end_index >= start_index + 3U; --end_index) {
				if (fit_arc_segment(points, start_index, end_index, tolerance, best_arc)) {
					has_arc = true;
					break;
				}
				if (end_index == start_index + 3U) {
					break;
				}
			}

			FittedWeldSegment best_line;
			bool has_line = false;
			for (std::size_t end_index = points.size() - 1U;
				end_index >= start_index + 1U; --end_index) {
				if (fit_line_segment(points, start_index, end_index, tolerance, best_line)) {
					has_line = true;
					break;
				}
				if (end_index == start_index + 1U) {
					break;
				}
			}

			if (has_arc &&
				(!has_line || best_arc.end_index - start_index >= best_line.end_index - start_index)) {
				return best_arc;
			}
			if (has_line) {
				return best_line;
			}

			FittedWeldSegment fallback;
			fallback.start_index = start_index;
			fallback.end_index = std::min(start_index + 1U, points.size() - 1U);
			return fallback;
		}

		RxyzCurve build_weld_rxyz_curve(const std::vector<SPAposition>& points,
			const std::vector<FittedWeldSegment>& segments) {
			RxyzCurve curve;
			if (points.empty()) {
				return curve;
			}

			curve.points.reserve(segments.size() + 2U);
			curve.points.push_back(RxyzPoint{ 0.0, 0.0, 0.0, 0.0 });
			for (const FittedWeldSegment& segment : segments) {
				const SPAposition& start = points[segment.start_index];
				curve.points.push_back(RxyzPoint{
					segment.is_arc ? segment.signed_radius : 0.0,
					start.x(), start.y(), start.z() });
			}
			const SPAposition& end = points[segments.empty() ? 0U : segments.back().end_index];
			curve.points.push_back(RxyzPoint{ 0.0, end.x(), end.y(), end.z() });
			curve.points.front().r = static_cast<double>(curve.points.size());
			return curve;
		}

		SPAunit_vector tangent_from_first_segment(const std::vector<SPAposition>& points,
			const FittedWeldSegment& segment) {
			if (!segment.is_arc) {
				return normalise(points[segment.end_index] - points[segment.start_index]);
			}

			const double sin_angle = std::sin(segment.start_angle);
			const double cos_angle = std::cos(segment.start_angle);
			SPAvector tangent =
				segment.u_axis.vector() * (-sin_angle) +
				segment.v_axis.vector() * cos_angle;
			if (segment.total_angle < 0.0) {
				tangent = -tangent;
			}
			return normalise(tangent);
		}

		WeldSpline fit_single_weld_spline(const WeldPolyline& polyline,
			double tolerance) {
			const std::vector<SPAposition> points =
				dedupe_weld_points(polyline.points, tolerance * 0.1);

			WeldSpline spline;
			spline.raw_points = points;
			if (points.empty()) {
				spline.points_rxyz.points.push_back(RxyzPoint{ 1.0, 0.0, 0.0, 0.0 });
				return spline;
			}

			spline.reference_point = points.front();
			if (points.size() == 1U) {
				spline.points_rxyz.points.push_back(RxyzPoint{ 2.0, 0.0, 0.0, 0.0 });
				spline.points_rxyz.points.push_back(RxyzPoint{
					0.0, points.front().x(), points.front().y(), points.front().z() });
				return spline;
			}

			std::vector<FittedWeldSegment> segments;
			std::size_t start_index = 0;
			while (start_index + 1U < points.size()) {
				FittedWeldSegment segment =
					select_next_weld_segment(points, start_index, tolerance);
				if (segment.end_index <= start_index) {
					break;
				}
				segments.push_back(segment);
				start_index = segment.end_index;
			}

			spline.points_rxyz = build_weld_rxyz_curve(points, segments);
			spline.tangent_direction = tangent_from_first_segment(points, segments.front());
			return spline;
		}

	}  // namespace

	/// 重置焊缝性能分析数据为零
	void hanfeng_reset_weld_profiling_data() {
		g_weld_profiling_data = WeldProfilingData{};
	}

	/// 获取当前焊缝性能分析数据的副本
	WeldProfilingData hanfeng_get_weld_profiling_data() {
		return g_weld_profiling_data;
	}

	WeldSplineResult hanfeng_fit_weld_splines(
		const WeldCurveResult& weld_result,
		double tolerance) {
		WeldSplineResult result;
		result.welds.reserve(weld_result.polylines.size());
		for (const WeldPolyline& polyline : weld_result.polylines) {
			result.welds.push_back(fit_single_weld_spline(polyline, tolerance));
		}
		return result;
	}

	/// 焊缝计算主入口：构建几何 → 收集候选 → 裁剪 → 去重，全程带性能计时
	WeldCurveResult hanfeng_compute_plane_surface_weld(
		const PlanePanel& plane_panel,
		const SurfacePanel& surface_panel,
		double tolerance) {
		hanfeng_reset_weld_profiling_data();
		const ProfilingClock::time_point total_start = ProfilingClock::now();

		PlanePanelGeometry plane_geometry;
		{
			ScopedDurationAccumulator timer(g_weld_profiling_data.build_plane_geometry_ms);
			plane_geometry = build_plane_panel_geometry(plane_panel);
		}

		SurfaceGeometry surface_geometry;
		{
			ScopedDurationAccumulator timer(g_weld_profiling_data.build_surface_geometry_ms);
			surface_geometry = build_surface_geometry(surface_panel);
		}
		g_weld_profiling_data.surface_triangle_count = surface_geometry.triangles.size();

		std::vector<Polyline3> surface_candidates;
		{
			ScopedDurationAccumulator timer(
				g_weld_profiling_data.collect_surface_candidates_ms);
			surface_candidates =
				hanfeng_collect_surface_candidate_loops(surface_panel);
		}
		g_weld_profiling_data.surface_candidate_loop_count = surface_candidates.size();
		for (const Polyline3& loop : surface_candidates) {
			g_weld_profiling_data.surface_candidate_segment_count +=
				candidate_loop_segment_count(loop);
		}

		std::vector<PlaneCandidateLoop> plane_candidates;
		{
			ScopedDurationAccumulator timer(
				g_weld_profiling_data.collect_plane_candidates_ms);
			plane_candidates =
				hanfeng_collect_plane_candidate_loops(plane_geometry);
		}
		g_weld_profiling_data.plane_candidate_loop_count = plane_candidates.size();
		for (const PlaneCandidateLoop& candidate : plane_candidates) {
			g_weld_profiling_data.plane_candidate_segment_count +=
				candidate_loop_segment_count(candidate.loop);
		}

		std::vector<WeldPolyline> all_polylines;

		{
			ScopedDurationAccumulator timer(
				g_weld_profiling_data.clip_surface_candidates_ms);
			for (const Polyline3& loop : surface_candidates) {
				const std::vector<WeldPolyline> clipped =
					hanfeng_clip_candidate_loop(loop, tolerance,
						[&](const SPAposition& start, const SPAposition& end) {
							return exact_segment_plane_panel_intervals(
								start, end, plane_geometry, tolerance);
						});
				all_polylines.insert(all_polylines.end(), clipped.begin(), clipped.end());
			}
		}

		{
			ScopedDurationAccumulator timer(
				g_weld_profiling_data.clip_plane_candidates_ms);
			for (const PlaneCandidateLoop& candidate : plane_candidates) {
				const std::vector<WeldPolyline> clipped =
					hanfeng_clip_candidate_loop(candidate.loop, tolerance,
						[&](const SPAposition& start, const SPAposition& end) {
							return exact_segment_surface_intervals_from_plane_candidate(
								start, end, surface_geometry, tolerance,
								candidate.outward_normal, plane_geometry);
						});
				all_polylines.insert(all_polylines.end(), clipped.begin(), clipped.end());
			}
		}

		WeldCurveResult result;
		{
			ScopedDurationAccumulator timer(g_weld_profiling_data.deduplicate_ms);
			result.polylines = hanfeng_deduplicate_polylines(
				hanfeng_merge_touching_polylines(
					hanfeng_deduplicate_polylines(all_polylines, tolerance),
					tolerance),
				tolerance);
		}
		g_weld_profiling_data.total_ms = std::chrono::duration<double, std::milli>(
			ProfilingClock::now() - total_start).count();
		return result;
	}

}  // namespace hanfeng
