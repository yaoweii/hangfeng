// =============================================================================
// hanfeng_util.cpp — hanfeng 工具函数实现
// =============================================================================
//
// 本文件实现了 hanfeng 项目的模型解析与几何计算工具，主要包括：
//   1. 极简 JSON 解析器（JsonValue + JsonParser）
//   2. 模型文件路径解析（resolve_model_json_paths）
//   3. 平面板解析（api_get_plane_panel）
//   4. 曲面板解析（api_get_surface）
//   5. 面板构造与模型数据转换
// =============================================================================

#include "hanfeng/panel.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

namespace hanfeng {

	// =============================================================================
	// 匿名命名空间 — 内部类型和辅助函数
	// =============================================================================

	namespace {

		// JSON 数组和对象的类型别名
		using JsonArray = std::vector<struct JsonValue>;
		using JsonObject = std::unordered_map<std::string, struct JsonValue>;

		/**
		 * @brief 极简 JSON 值类型。
		 *
		 * 使用 std::variant 支持 null/bool/double/string/array/object 六种类型。
		 * 只服务于本项目模型 JSON 的解析。
		 */
		struct JsonValue {
			using Storage =
				std::variant<std::nullptr_t, bool, double, std::string, JsonArray,
				JsonObject>;

			Storage value;

			bool is_object() const { return std::holds_alternative<JsonObject>(value); }
			bool is_array() const { return std::holds_alternative<JsonArray>(value); }
			bool is_string() const { return std::holds_alternative<std::string>(value); }
			bool is_number() const { return std::holds_alternative<double>(value); }

			const JsonObject& as_object() const {
				return std::get<JsonObject>(value);
			}

			const JsonArray& as_array() const { return std::get<JsonArray>(value); }

			const std::string& as_string() const {
				return std::get<std::string>(value);
			}

			double as_number() const { return std::get<double>(value); }
		};

		/// @brief 用于解析模型 JSON 的简单递归下降解析器
		class JsonParser {
		public:
			explicit JsonParser(std::string_view text) : text_(text) {}

			JsonValue parse() {
				skip_whitespace();
				JsonValue root = parse_value();
				skip_whitespace();
				if (pos_ != text_.size()) {
					throw std::runtime_error("JSON 解析失败：根节点后仍存在多余内容。");
				}
				return root;
			}

		private:
			// 根据首字符分发到对应解析函数
			JsonValue parse_value() {
				skip_whitespace();
				if (pos_ >= text_.size()) {
					throw std::runtime_error("JSON 解析失败：意外到达文件末尾。");
				}

				const char ch = text_[pos_];
				if (ch == '{') {
					return parse_object();
				}
				if (ch == '[') {
					return parse_array();
				}
				if (ch == '"') {
					return JsonValue{ parse_string() };
				}
				if (ch == '-' || std::isdigit(static_cast<unsigned char>(ch)) != 0) {
					return JsonValue{ parse_number() };
				}
				if (consume_literal("true")) {
					return JsonValue{ true };
				}
				if (consume_literal("false")) {
					return JsonValue{ false };
				}
				if (consume_literal("null")) {
					return JsonValue{ nullptr };
				}

				throw std::runtime_error("JSON 解析失败：遇到无法识别的值类型。");
			}

			// 解析 JSON 对象 { "key": value, ... }
			JsonValue parse_object() {
				expect('{');
				JsonObject object;
				skip_whitespace();
				if (peek('}')) {
					expect('}');
					return JsonValue{ std::move(object) };
				}

				while (true) {
					skip_whitespace();
					if (!peek('"')) {
						throw std::runtime_error("JSON 解析失败：对象键必须是字符串。");
					}
					const std::string key = parse_string();
					skip_whitespace();
					expect(':');
					JsonValue value = parse_value();
					object.emplace(key, std::move(value));
					skip_whitespace();
					if (peek('}')) {
						expect('}');
						break;
					}
					expect(',');
				}

				return JsonValue{ std::move(object) };
			}

			// 解析 JSON 数组 [value, value, ...]
			JsonValue parse_array() {
				expect('[');
				JsonArray array;
				skip_whitespace();
				if (peek(']')) {
					expect(']');
					return JsonValue{ std::move(array) };
				}

				while (true) {
					array.push_back(parse_value());
					skip_whitespace();
					if (peek(']')) {
						expect(']');
						break;
					}
					expect(',');
				}

				return JsonValue{ std::move(array) };
			}

			// 解析字符串，支持常见转义序列
			std::string parse_string() {
				expect('"');
				std::string result;
				while (pos_ < text_.size()) {
					const char ch = text_[pos_++];
					if (ch == '"') {
						return result;
					}
					if (ch == '\\') {
						if (pos_ >= text_.size()) {
							throw std::runtime_error("JSON 解析失败：字符串转义不完整。");
						}
						const char escaped = text_[pos_++];
						switch (escaped) {
						case '"':
						case '\\':
						case '/':
							result.push_back(escaped);
							break;
						case 'b':
							result.push_back('\b');
							break;
						case 'f':
							result.push_back('\f');
							break;
						case 'n':
							result.push_back('\n');
							break;
						case 'r':
							result.push_back('\r');
							break;
						case 't':
							result.push_back('\t');
							break;
						default:
							throw std::runtime_error(
								"JSON 解析失败：当前实现不支持该字符串转义形式。");
						}
						continue;
					}
					result.push_back(ch);
				}

				throw std::runtime_error("JSON 解析失败：字符串未正常结束。");
			}

			// 解析数值 [-]int[.frac][e/E[+/-]exp]
			double parse_number() {
				const std::size_t start = pos_;
				if (peek('-')) {
					++pos_;
				}
				consume_digits();
				if (peek('.')) {
					++pos_;
					consume_digits();
				}
				if (peek('e') || peek('E')) {
					++pos_;
					if (peek('+') || peek('-')) {
						++pos_;
					}
					consume_digits();
				}

				const std::string token(text_.substr(start, pos_ - start));
				try {
					return std::stod(token);
				}
				catch (const std::exception&) {
					throw std::runtime_error("JSON 解析失败：数值格式非法。");
				}
			}

			void consume_digits() {
				if (pos_ >= text_.size() ||
					std::isdigit(static_cast<unsigned char>(text_[pos_])) == 0) {
					throw std::runtime_error("JSON 解析失败：数值缺少数字部分。");
				}
				while (pos_ < text_.size() &&
					std::isdigit(static_cast<unsigned char>(text_[pos_])) != 0) {
					++pos_;
				}
			}

			// 尝试消费字面量（true/false/null）
			bool consume_literal(std::string_view literal) {
				if (text_.substr(pos_, literal.size()) == literal) {
					pos_ += literal.size();
					return true;
				}
				return false;
			}

			void skip_whitespace() {
				while (pos_ < text_.size() &&
					std::isspace(static_cast<unsigned char>(text_[pos_])) != 0) {
					++pos_;
				}
			}

			bool peek(char expected) const {
				return pos_ < text_.size() && text_[pos_] == expected;
			}

			void expect(char expected) {
				if (!peek(expected)) {
					throw std::runtime_error("JSON 解析失败：缺少预期的结构字符。");
				}
				++pos_;
			}

			std::string_view text_;
			std::size_t pos_ = 0U;
		};

		// -----------------------------------------------------------------------------
		// 模型数据结构
		// -----------------------------------------------------------------------------

		/// @brief 模型文件的 JSON 路径对（边界文件 + 面片文件）
		struct ModelJsonPaths {
			std::filesystem::path boundary_json;  ///< 边界环 JSON 文件路径
			std::filesystem::path panel_json;     ///< 三角网格 JSON 文件路径
		};

		/// @brief 平面板边界环的几何信息（用于分组和区分外轮廓/内孔）
		struct PlaneLoopInfo {
			Polyline3 loop;                 ///< 原始边界环点序列
			SPAposition origin;             ///< 拟合平面原点
			SPAunit_vector normal;          ///< 拟合平面法向量
			double plane_offset = 0.0;      ///< 环质心沿法向量偏移量
			double area_magnitude = 0.0;    ///< 环的有向面积绝对值
		};

		/// @brief 一组共面边界环
		struct LoopGroup {
			SPAposition origin;
			SPAunit_vector normal;
			double plane_offset = 0.0;
			std::vector<PlaneLoopInfo> loops;
		};

		/// @brief 带预计算包围盒的三维三角形（加速距离查询）
		struct Triangle3 {
			SPAposition a;
			SPAposition b;
			SPAposition c;
			SPAbox box;
		};

		/// @brief 投影到平面后的二维边界环（保留三维点和二维 UV 坐标）
		struct ProjectedLoop {
			Polyline3 world_points;
			std::vector<std::array<double, 2>> uv_points;
		};

		/// @brief 平面板单侧主面的几何信息（含投影边界环）
		struct PlaneFaceGeometry {
			SPAposition origin;
			SPAunit_vector normal;
			SPAunit_vector u_axis;
			SPAunit_vector v_axis;
			std::vector<ProjectedLoop> outer_loops;
			std::vector<ProjectedLoop> inner_loops;
		};

		/// @brief 平面板的完整几何（两侧主面 + 侧壁三角形）
		struct PlanePanelGeometry {
			std::vector<PlaneFaceGeometry> main_faces;
			std::vector<Triangle3> side_triangles;
		};

		/// @brief 曲面板的三角网格几何（展开索引 + 预计算包围盒）
		struct SurfaceGeometry {
			std::vector<Triangle3> triangles;
		};

		// -----------------------------------------------------------------------------
		// 文件 I/O 和 JSON 解析辅助
		// -----------------------------------------------------------------------------

		/// 读取文本文件全部内容
		std::string read_text_file(const std::filesystem::path& path) {
			std::ifstream input(path);
			if (!input.is_open()) {
				throw std::runtime_error("无法打开文件: " + path.string());
			}

			std::ostringstream buffer;
			buffer << input.rdbuf();
			return buffer.str();
		}

		/// 从 JSON 对象中获取必需字段
		const JsonValue& require_object_member(const JsonObject& object,
			const std::string& key) {
			const auto iterator = object.find(key);
			if (iterator == object.end()) {
				throw std::runtime_error("JSON 缺少必需字段: " + key);
			}
			return iterator->second;
		}

		/// 从 JSON 值解析三维点 [x, y, z]
		SPAposition parse_position3(const JsonValue& value) {
			if (!value.is_array()) {
				throw std::runtime_error("三维点解析失败：点必须是数组。");
			}
			const JsonArray& array = value.as_array();
			if (array.size() != 3U || !array[0].is_number() || !array[1].is_number() ||
				!array[2].is_number()) {
				throw std::runtime_error("三维点解析失败：点必须是长度为 3 的数值数组。");
			}

			return SPAposition(array[0].as_number(), array[1].as_number(),
				array[2].as_number());
		}

		/// 从 JSON 值解析三维折线
		Polyline3 parse_polyline3(const JsonValue& value) {
			if (!value.is_array()) {
				throw std::runtime_error("折线解析失败：边界环必须是数组。");
			}

			Polyline3 polyline;
			for (const JsonValue& point_value : value.as_array()) {
				polyline.push_back(parse_position3(point_value));
			}
			return polyline;
		}

		/// 从边界 JSON 文件解析所有边界环
		std::vector<Polyline3> parse_boundary_loops(const std::filesystem::path& path) {
			const JsonValue root = JsonParser(read_text_file(path)).parse();
			if (!root.is_object()) {
				throw std::runtime_error("边界文件解析失败：根节点必须是对象。");
			}

			const JsonValue& loops_value =
				require_object_member(root.as_object(), "boundary_loops");
			if (!loops_value.is_array()) {
				throw std::runtime_error("边界文件解析失败：boundary_loops 必须是数组。");
			}

			std::vector<Polyline3> loops;
			for (const JsonValue& loop_value : loops_value.as_array()) {
				loops.push_back(parse_polyline3(loop_value));
			}
			return loops;
		}

		/// 从面片 JSON 文件解析三角网格（vertices + triangles）
		void parse_surface_mesh(const std::filesystem::path& path,
			std::vector<SPAposition>& vertices,
			std::vector<Triangle>& triangles) {
			const JsonValue root = JsonParser(read_text_file(path)).parse();
			if (!root.is_object()) {
				throw std::runtime_error("面片文件解析失败：根节点必须是对象。");
			}

			const JsonObject& object = root.as_object();
			const JsonValue& vertices_value = require_object_member(object, "vertices");
			const JsonValue& triangles_value = require_object_member(object, "triangles");

			if (!vertices_value.is_array()) {
				throw std::runtime_error("面片文件解析失败：vertices 必须是数组。");
			}
			if (!triangles_value.is_array()) {
				throw std::runtime_error("面片文件解析失败：triangles 必须是数组。");
			}

			for (const JsonValue& vertex_value : vertices_value.as_array()) {
				vertices.push_back(parse_position3(vertex_value));
			}

			for (const JsonValue& triangle_value : triangles_value.as_array()) {
				if (!triangle_value.is_array()) {
					throw std::runtime_error("三角面解析失败：triangle 必须是数组。");
				}
				const JsonArray& array = triangle_value.as_array();
				if (array.size() != 3U || !array[0].is_number() || !array[1].is_number() ||
					!array[2].is_number()) {
					throw std::runtime_error(
						"三角面解析失败：triangle 必须是长度为 3 的索引数组。");
				}
				triangles.push_back(Triangle{
					static_cast<std::size_t>(std::llround(array[0].as_number())),
					static_cast<std::size_t>(std::llround(array[1].as_number())),
					static_cast<std::size_t>(std::llround(array[2].as_number())),
					});
			}
		}

		// -----------------------------------------------------------------------------
		// 模型路径解析
		// -----------------------------------------------------------------------------

		/// 根据模型路径（目录或文件）解析出配套的 boundary + panel JSON 路径
		ModelJsonPaths resolve_model_json_paths(const std::filesystem::path& model_path) {
			ModelJsonPaths paths;
			const std::filesystem::path absolute = std::filesystem::absolute(model_path);

			if (std::filesystem::is_directory(absolute)) {
				// 输入是目录 → 遍历查找
				for (const auto& entry : std::filesystem::directory_iterator(absolute)) {
					if (!entry.is_regular_file()) {
						continue;
					}
					const std::string name = entry.path().filename().string();
					if (name.size() >= std::string(".boundary.xyz.json").size() &&
						name.rfind(".boundary.xyz.json") ==
						name.size() - std::string(".boundary.xyz.json").size()) {
						paths.boundary_json = entry.path();
					}
					else if (name.size() >= std::string(".panel.xyz.json").size() &&
						name.rfind(".panel.xyz.json") ==
						name.size() - std::string(".panel.xyz.json").size()) {
						paths.panel_json = entry.path();
					}
				}
			}
			else if (std::filesystem::is_regular_file(absolute)) {
				// 输入是文件 → 推导配套文件
				const std::string name = absolute.filename().string();
				if (name.size() >= std::string(".boundary.xyz.json").size() &&
					name.rfind(".boundary.xyz.json") ==
					name.size() - std::string(".boundary.xyz.json").size()) {
					paths.boundary_json = absolute;
					paths.panel_json = absolute.parent_path() /
						(name.substr(0, name.size() -
							std::string(".boundary.xyz.json")
							.size()) +
							".panel.xyz.json");
				}
				else if (name.size() >= std::string(".panel.xyz.json").size() &&
					name.rfind(".panel.xyz.json") ==
					name.size() - std::string(".panel.xyz.json").size()) {
					paths.panel_json = absolute;
					paths.boundary_json = absolute.parent_path() /
						(name.substr(0, name.size() -
							std::string(".panel.xyz.json")
							.size()) +
							".boundary.xyz.json");
				}
				else {
					throw std::runtime_error(
						"模型路径必须是面板目录、*.panel.xyz.json 或 *.boundary.xyz.json。");
				}
			}
			else {
				throw std::runtime_error("模型路径不存在: " + absolute.string());
			}

			if (paths.boundary_json.empty() || !std::filesystem::exists(paths.boundary_json)) {
				throw std::runtime_error("未找到配套边界文件。");
			}
			if (paths.panel_json.empty() || !std::filesystem::exists(paths.panel_json)) {
				throw std::runtime_error("未找到配套面片文件。");
			}

			return paths;
		}

		// -----------------------------------------------------------------------------
		// 平面拟合与面积计算
		// -----------------------------------------------------------------------------

		/// 从边界环拟合平面参数（原点 + 法向量），返回是否成功
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

		/// 计算点沿法向量的投影偏移量
		double point_offset_along_normal(const SPAposition& point,
			const SPAunit_vector& normal) {
			return (point - SPAposition()) % normal;
		}

		/// 根据法向量构建平面正交基 (u, v)
		std::pair<SPAunit_vector, SPAunit_vector> build_plane_basis(
			const SPAunit_vector& normal) {
			SPAvector reference(1.0, 0.0, 0.0);
			if (std::fabs(normal % SPAunit_vector(reference)) > 0.9) {
				reference = SPAvector(0.0, 1.0, 0.0);  // 避免退化
			}

			const SPAvector tangent = normal * reference;
			const SPAunit_vector u = normalise(tangent);
			const SPAunit_vector v = normalise(normal * u);
			return { u, v };
		}

		/// 使用 Shoelace 公式计算边界环在有向面积绝对值
		double compute_loop_area_magnitude(const Polyline3& loop,
			const SPAposition& origin,
			const SPAunit_vector& normal) {
			if (loop.size() < 3U) {
				return 0.0;
			}

			const auto [u_axis, v_axis] = build_plane_basis(normal);
			double area2 = 0.0;
			for (std::size_t i = 0; i < loop.size(); ++i) {
				const SPAposition& current = loop[i];
				const SPAposition& next = loop[(i + 1U) % loop.size()];
				const SPAvector current_vector = current - origin;
				const SPAvector next_vector = next - origin;
				const double cx = current_vector % u_axis;
				const double cy = current_vector % v_axis;
				const double nx = next_vector % u_axis;
				const double ny = next_vector % v_axis;
				area2 += cx * ny - cy * nx;
			}
			return std::fabs(area2) * 0.5;
		}

		/// 为单个边界环构建几何信息（拟合平面 + 质心 + 偏移量 + 面积）
		PlaneLoopInfo build_plane_loop_info(const Polyline3& loop) {
			SPAposition origin;
			SPAunit_vector normal;
			if (!fit_plane_from_loop(loop, origin, normal)) {
				throw std::runtime_error("平面板边界环无法拟合出有效平面。");
			}

			double sum_x = 0.0;
			double sum_y = 0.0;
			double sum_z = 0.0;
			for (const SPAposition& point : loop) {
				sum_x += point.x();
				sum_y += point.y();
				sum_z += point.z();
			}
			const SPAposition centroid(sum_x / static_cast<double>(loop.size()),
				sum_y / static_cast<double>(loop.size()),
				sum_z / static_cast<double>(loop.size()));

			PlaneLoopInfo info;
			info.loop = loop;
			info.origin = origin;
			info.normal = normal;
			info.plane_offset = point_offset_along_normal(centroid, normal);
			info.area_magnitude = compute_loop_area_magnitude(loop, origin, normal);
			return info;
		}

		/// 判断边界环是否属于某个共面组（法向量平行或反平行 + 偏移量接近）
		bool same_plane_group(const PlaneLoopInfo& loop, const LoopGroup& group) {
			const double cosine = loop.normal % group.normal;
			if (std::fabs(cosine) < 0.999) {
				return false;
			}
			// 法向量反方向时 offset 符号相反，需要校正后再比较
			const double sign = cosine >= 0.0 ? 1.0 : -1.0;
			return std::fabs(loop.plane_offset * sign - group.plane_offset) <= 1.0e-3;
		}

		/// 将共面组构建为 PlaneFace（面积最大为外轮廓，其余为内孔）
		PlaneFace build_plane_face_from_group(LoopGroup group) {
			PlaneFace face;
			std::sort(group.loops.begin(), group.loops.end(),
				[](const PlaneLoopInfo& left, const PlaneLoopInfo& right) {
					return left.area_magnitude > right.area_magnitude;
				});

			if (!group.loops.empty()) {
				face.boundaries.outer_loops.push_back(group.loops.front().loop);
				for (std::size_t index = 1; index < group.loops.size(); ++index) {
					face.boundaries.inner_loops.push_back(group.loops[index].loop);
				}
			}
			return face;
		}

		struct LocalPoint2 {
			double x = 0.0;
			double y = 0.0;
		};

		bool points_close_2d(const LocalPoint2& left,
			const LocalPoint2& right,
			double epsilon = 1.0e-6) {
			return std::fabs(left.x - right.x) <= epsilon &&
				std::fabs(left.y - right.y) <= epsilon;
		}

		void append_local_point_if_needed(std::vector<LocalPoint2>& polyline,
			const LocalPoint2& point,
			double epsilon = 1.0e-6) {
			if (polyline.empty() || !points_close_2d(polyline.back(), point, epsilon)) {
				polyline.push_back(point);
			}
		}

		RxyCurve parse_raw_rxy_curve(const float raw_curve[1000][3], const char* label) {
			const long long raw_count = std::llround(raw_curve[0][0]);
			if (raw_count < 2LL || raw_count > 1000LL) {
				throw std::runtime_error(std::string(label) + " 解析失败：header 点数非法。");
			}

			RxyCurve curve;
			curve.points.reserve(static_cast<std::size_t>(raw_count));
			for (long long index = 0; index < raw_count; ++index) {
				curve.points.push_back(
					RxyPoint{ raw_curve[index][0], raw_curve[index][1], raw_curve[index][2] });
			}
			if (!curve.header_matches_point_count()) {
				throw std::runtime_error(std::string(label) + " 解析失败：header 与实际点数不一致。");
			}
			if (curve.geometry_point_count() < 2U) {
				throw std::runtime_error(std::string(label) + " 解析失败：几何控制点不足。");
			}
			return curve;
		}

		double normalise_angle_delta(double delta) {
			constexpr double kLocalPi = 3.14159265358979323846;
			while (delta <= -kLocalPi) {
				delta += 2.0 * kLocalPi;
			}
			while (delta > kLocalPi) {
				delta -= 2.0 * kLocalPi;
			}
			return delta;
		}

		LocalPoint2 choose_arc_center(const LocalPoint2& start,
			const LocalPoint2& end,
			double signed_radius) {
			const double radius = std::fabs(signed_radius);
			const double dx = end.x - start.x;
			const double dy = end.y - start.y;
			const double chord_length = std::hypot(dx, dy);
			if (chord_length <= 1.0e-9) {
				throw std::runtime_error("rxy 圆弧解析失败：起点与终点重合。");
			}
			if (chord_length > 2.0 * radius + 1.0e-6) {
				throw std::runtime_error("rxy 圆弧解析失败：弦长大于直径。");
			}

			const double mid_x = 0.5 * (start.x + end.x);
			const double mid_y = 0.5 * (start.y + end.y);
			const double height_sq =
				std::max(0.0, radius * radius - 0.25 * chord_length * chord_length);
			const double height = std::sqrt(height_sq);
			const double normal_x = -dy / chord_length;
			const double normal_y = dx / chord_length;
			const LocalPoint2 candidates[2] = {
				{ mid_x + normal_x * height, mid_y + normal_y * height },
				{ mid_x - normal_x * height, mid_y - normal_y * height }
			};
			const bool clockwise = signed_radius > 0.0;

			for (const LocalPoint2& center : candidates) {
				const double start_angle = std::atan2(start.y - center.y, start.x - center.x);
				const double end_angle = std::atan2(end.y - center.y, end.x - center.x);
				const double delta = normalise_angle_delta(end_angle - start_angle);
				if ((clockwise && delta < 0.0) || (!clockwise && delta > 0.0)) {
					return center;
				}
			}

			throw std::runtime_error("rxy 圆弧解析失败：无法根据半径方向确定圆心。");
		}

		void append_arc_segment_samples(std::vector<LocalPoint2>& polyline,
			const LocalPoint2& start,
			const LocalPoint2& end,
			double signed_radius) {
			const LocalPoint2 center = choose_arc_center(start, end, signed_radius);
			const double radius = std::fabs(signed_radius);
			const double start_angle = std::atan2(start.y - center.y, start.x - center.x);
			const double end_angle = std::atan2(end.y - center.y, end.x - center.x);
			double delta = normalise_angle_delta(end_angle - start_angle);
			constexpr double kLocalPi = 3.14159265358979323846;
			if (signed_radius > 0.0 && delta > 0.0) {
				delta -= 2.0 * kLocalPi;
			}
			else if (signed_radius < 0.0 && delta < 0.0) {
				delta += 2.0 * kLocalPi;
			}

			const double max_step = 10.0 * kLocalPi / 180.0;
			const int segment_count =
				std::max(2, static_cast<int>(std::ceil(std::fabs(delta) / max_step)));
			for (int step = 1; step <= segment_count; ++step) {
				const double t = static_cast<double>(step) / static_cast<double>(segment_count);
				const double angle = start_angle + delta * t;
				append_local_point_if_needed(polyline,
					LocalPoint2{ center.x + radius * std::cos(angle),
						center.y + radius * std::sin(angle) });
			}
		}

		std::vector<LocalPoint2> expand_rxy_curve_to_polyline(const RxyCurve& curve) {
			std::vector<LocalPoint2> polyline;
			polyline.reserve(curve.geometry_point_count());
			for (std::size_t index = 1; index + 1 < curve.points.size(); ++index) {
				const RxyPoint& current = curve.points[index];
				const RxyPoint& next = curve.points[index + 1U];
				const LocalPoint2 start{ current.x, current.y };
				const LocalPoint2 end{ next.x, next.y };
				append_local_point_if_needed(polyline, start);
				if (current.defines_arc_to_next()) {
					append_arc_segment_samples(polyline, start, end, current.r);
				}
				else {
					append_local_point_if_needed(polyline, end);
				}
			}

			if (polyline.size() >= 2U && points_close_2d(polyline.front(), polyline.back())) {
				polyline.pop_back();
			}
			if (polyline.size() < 3U) {
				throw std::runtime_error("rxy 曲线展开失败：有效边界点不足。");
			}
			return polyline;
		}

		SPAposition parse_caxis_position_row(const float row[3]) {
			return SPAposition(row[0], row[1], row[2]);
		}

		SPAvector parse_caxis_vector_row(const float row[3], const char* label) {
			const SPAvector vector(row[0], row[1], row[2]);
			if (!normalise(vector).is_valid()) {
				throw std::runtime_error(std::string("cAxis 解析失败：") + label +
					" 轴向量非法。");
			}
			return vector;
		}

		Polyline3 transform_local_polyline(const std::vector<LocalPoint2>& local_points,
			const SPAposition& origin,
			const SPAvector& u_axis,
			const SPAvector& v_axis,
			const SPAvector& offset) {
			Polyline3 world_points;
			world_points.reserve(local_points.size());
			for (const LocalPoint2& local_point : local_points) {
				const SPAposition point =
					origin + u_axis * local_point.x + v_axis * local_point.y + offset;
				world_points.push_back(point);
			}
			return world_points;
		}

		PlaneFace build_plane_face_from_loops(Polyline3 outer_loop,
			std::vector<Polyline3> inner_loops) {
			PlaneFace face;
			face.boundaries.outer_loops.push_back(std::move(outer_loop));
			face.boundaries.inner_loops = std::move(inner_loops);
			return face;
		}

		struct IndexedEdge {
			std::size_t a = 0U;
			std::size_t b = 0U;

			bool operator==(const IndexedEdge& other) const {
				return a == other.a && b == other.b;
			}
		};

		struct IndexedEdgeHash {
			std::size_t operator()(const IndexedEdge& edge) const {
				return std::hash<std::size_t>{}(edge.a) ^
					(std::hash<std::size_t>{}(edge.b) << 1U);
			}
		};

		struct IndexVectorHash {
			std::size_t operator()(const std::vector<std::size_t>& values) const {
				std::size_t seed = 0U;
				for (std::size_t value : values) {
					seed ^= std::hash<std::size_t>{}(value)+0x9e3779b9U + (seed << 6U) +
						(seed >> 2U);
				}
				return seed;
			}
		};

		IndexedEdge canonical_indexed_edge(std::size_t first, std::size_t second) {
			return first <= second ? IndexedEdge{ first, second } : IndexedEdge{ second, first };
		}

		void sort_and_unique_neighbors(
			std::unordered_map<std::size_t, std::vector<std::size_t>>& graph) {
			for (auto& [vertex, neighbors] : graph) {
				(void)vertex;
				std::sort(neighbors.begin(), neighbors.end());
				neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
			}
		}

		std::vector<std::vector<std::size_t>> trace_index_loops(
			std::unordered_map<std::size_t, std::vector<std::size_t>> boundary_graph) {
			sort_and_unique_neighbors(boundary_graph);
			std::vector<std::size_t> start_vertices;
			start_vertices.reserve(boundary_graph.size());
			for (const auto& [vertex, _] : boundary_graph) {
				start_vertices.push_back(vertex);
			}
			std::sort(start_vertices.begin(), start_vertices.end());

			std::unordered_set<IndexedEdge, IndexedEdgeHash> visited_edges;
			std::vector<std::vector<std::size_t>> loops;

			for (std::size_t start : start_vertices) {
				auto graph_iter = boundary_graph.find(start);
				if (graph_iter == boundary_graph.end()) {
					continue;
				}
				for (std::size_t neighbor : graph_iter->second) {
					const IndexedEdge edge = canonical_indexed_edge(start, neighbor);
					if (visited_edges.find(edge) != visited_edges.end()) {
						continue;
					}

					std::vector<std::size_t> loop{ start };
					std::size_t current = start;
					std::optional<std::size_t> previous;

					while (true) {
						auto current_iter = boundary_graph.find(current);
						if (current_iter == boundary_graph.end()) {
							break;
						}
						std::optional<std::size_t> next_point;
						for (std::size_t candidate : current_iter->second) {
							if (previous.has_value() && candidate == *previous) {
								continue;
							}
							if (visited_edges.find(canonical_indexed_edge(current, candidate)) !=
								visited_edges.end()) {
								continue;
							}
							next_point = candidate;
							break;
						}
						if (!next_point.has_value()) {
							break;
						}
						visited_edges.insert(canonical_indexed_edge(current, *next_point));
						previous = current;
						current = *next_point;
						if (current == start) {
							break;
						}
						loop.push_back(current);
					}

					if (loop.size() >= 3U && current == start) {
						loops.push_back(std::move(loop));
					}
				}
			}

			std::sort(loops.begin(), loops.end(),
				[](const std::vector<std::size_t>& left, const std::vector<std::size_t>& right) {
					return left.size() > right.size();
				});
			return loops;
		}

		double indexed_loop_perimeter(const std::vector<std::size_t>& loop,
			const std::vector<SPAposition>& vertices) {
			if (loop.size() < 2U) {
				return 0.0;
			}
			double perimeter = 0.0;
			for (std::size_t index = 0; index < loop.size(); ++index) {
				const SPAposition& first = vertices[loop[index]];
				const SPAposition& second = vertices[loop[(index + 1U) % loop.size()]];
				perimeter += std::sqrt((second - first).len_sq());
			}
			return perimeter;
		}

		std::vector<std::size_t> normalize_loop_key(std::vector<std::size_t> loop) {
			std::sort(loop.begin(), loop.end());
			return loop;
		}

		std::vector<std::vector<std::size_t>> merge_boundary_index_loops(
			const std::vector<std::vector<std::size_t>>& topology_loops,
			const std::vector<std::vector<std::size_t>>& feature_loops,
			const std::vector<SPAposition>& vertices) {
			std::vector<std::vector<std::size_t>> merged;
			std::unordered_set<std::vector<std::size_t>, IndexVectorHash> seen_keys;

			auto append_group = [&](const std::vector<std::vector<std::size_t>>& loops) {
				for (const auto& loop : loops) {
					if (loop.size() < 3U) {
						continue;
					}
					const std::vector<std::size_t> key = normalize_loop_key(loop);
					if (seen_keys.find(key) != seen_keys.end()) {
						continue;
					}
					seen_keys.insert(key);
					merged.push_back(loop);
				}
				};

			append_group(topology_loops);
			append_group(feature_loops);
			std::sort(merged.begin(), merged.end(),
				[&vertices](const std::vector<std::size_t>& left,
					const std::vector<std::size_t>& right) {
						if (left.size() != right.size()) {
							return left.size() > right.size();
						}
						return indexed_loop_perimeter(left, vertices) >
							indexed_loop_perimeter(right, vertices);
				});
			return merged;
		}

		std::optional<SPAunit_vector> indexed_triangle_normal(const Triangle& triangle,
			const std::vector<SPAposition>& vertices) {
			if (triangle[0] >= vertices.size() || triangle[1] >= vertices.size() ||
				triangle[2] >= vertices.size()) {
				return std::nullopt;
			}
			const SPAvector ab = vertices[triangle[1]] - vertices[triangle[0]];
			const SPAvector ac = vertices[triangle[2]] - vertices[triangle[0]];
			const SPAunit_vector normal = normalise(ab * ac);
			if (!normal.is_valid()) {
				return std::nullopt;
			}
			return normal;
		}

		std::vector<std::vector<std::size_t>> smooth_region_boundary_index_loops(
			const std::vector<SPAposition>& vertices,
			const std::vector<Triangle>& triangles,
			double smooth_angle_degrees = 50.0,
			std::size_t max_regions = 2U) {
			if (triangles.empty()) {
				return {};
			}

			std::vector<std::optional<SPAunit_vector>> normals;
			normals.reserve(triangles.size());
			for (const Triangle& triangle : triangles) {
				normals.push_back(indexed_triangle_normal(triangle, vertices));
			}

			std::unordered_map<IndexedEdge, std::vector<std::size_t>, IndexedEdgeHash>
				edge_to_triangles;
			for (std::size_t triangle_index = 0; triangle_index < triangles.size(); ++triangle_index) {
				const Triangle& triangle = triangles[triangle_index];
				edge_to_triangles[canonical_indexed_edge(triangle[0], triangle[1])]
					.push_back(triangle_index);
				edge_to_triangles[canonical_indexed_edge(triangle[1], triangle[2])]
					.push_back(triangle_index);
				edge_to_triangles[canonical_indexed_edge(triangle[2], triangle[0])]
					.push_back(triangle_index);
			}

			const double smooth_cosine =
				std::cos(smooth_angle_degrees * 3.14159265358979323846 / 180.0);
			std::vector<std::vector<std::size_t>> adjacency(triangles.size());
			for (const auto& [edge, triangle_indices] : edge_to_triangles) {
				(void)edge;
				if (triangle_indices.size() != 2U) {
					continue;
				}
				const auto& first_normal = normals[triangle_indices[0]];
				const auto& second_normal = normals[triangle_indices[1]];
				if (!first_normal.has_value() || !second_normal.has_value()) {
					continue;
				}
				if (std::fabs(*first_normal % *second_normal) < smooth_cosine) {
					continue;
				}
				adjacency[triangle_indices[0]].push_back(triangle_indices[1]);
				adjacency[triangle_indices[1]].push_back(triangle_indices[0]);
			}

			std::vector<std::vector<std::size_t>> components;
			std::vector<bool> seen(triangles.size(), false);
			for (std::size_t triangle_index = 0; triangle_index < triangles.size(); ++triangle_index) {
				if (seen[triangle_index]) {
					continue;
				}
				std::vector<std::size_t> stack{ triangle_index };
				seen[triangle_index] = true;
				std::vector<std::size_t> component;
				while (!stack.empty()) {
					const std::size_t current = stack.back();
					stack.pop_back();
					component.push_back(current);
					for (std::size_t neighbor : adjacency[current]) {
						if (!seen[neighbor]) {
							seen[neighbor] = true;
							stack.push_back(neighbor);
						}
					}
				}
				components.push_back(std::move(component));
			}

			std::sort(components.begin(), components.end(),
				[](const std::vector<std::size_t>& left,
					const std::vector<std::size_t>& right) {
						return left.size() > right.size();
				});

			std::vector<std::size_t> component_index(triangles.size(), std::size_t(-1));
			for (std::size_t index = 0; index < components.size(); ++index) {
				for (std::size_t triangle_index : components[index]) {
					component_index[triangle_index] = index;
				}
			}

			std::vector<std::vector<std::size_t>> region_loops;
			std::unordered_set<std::vector<std::size_t>, IndexVectorHash> seen_loop_keys;
			const std::size_t region_limit = std::min(max_regions, components.size());
			for (std::size_t region_index = 0; region_index < region_limit; ++region_index) {
				if (components[region_index].size() < 4U) {
					continue;
				}
				std::unordered_map<std::size_t, std::vector<std::size_t>> boundary_graph;
				for (const auto& [edge, triangle_indices] : edge_to_triangles) {
					if (triangle_indices.size() != 2U) {
						continue;
					}
					const std::size_t first_region = component_index[triangle_indices[0]];
					const std::size_t second_region = component_index[triangle_indices[1]];
					if (((first_region == region_index) == (second_region == region_index))) {
						continue;
					}
					boundary_graph[edge.a].push_back(edge.b);
					boundary_graph[edge.b].push_back(edge.a);
				}

				for (const auto& loop : trace_index_loops(std::move(boundary_graph))) {
					if (loop.size() < 3U) {
						continue;
					}
					const std::vector<std::size_t> key = normalize_loop_key(loop);
					if (seen_loop_keys.find(key) != seen_loop_keys.end()) {
						continue;
					}
					seen_loop_keys.insert(key);
					region_loops.push_back(loop);
				}
			}

			std::sort(region_loops.begin(), region_loops.end(),
				[](const std::vector<std::size_t>& left,
					const std::vector<std::size_t>& right) {
						return left.size() > right.size();
				});
			if (region_loops.size() > max_regions) {
				region_loops.resize(max_regions);
			}
			return region_loops;
		}

		std::vector<Polyline3> reconstruct_surface_boundary_loops(
			const std::vector<SPAposition>& vertices,
			const std::vector<Triangle>& triangles) {
			std::unordered_map<IndexedEdge, std::size_t, IndexedEdgeHash> edge_counts;
			std::unordered_map<std::size_t, std::vector<std::size_t>> boundary_graph;
			for (const Triangle& triangle : triangles) {
				edge_counts[canonical_indexed_edge(triangle[0], triangle[1])] += 1U;
				edge_counts[canonical_indexed_edge(triangle[1], triangle[2])] += 1U;
				edge_counts[canonical_indexed_edge(triangle[2], triangle[0])] += 1U;
			}

			for (const auto& [edge, count] : edge_counts) {
				if (count != 1U) {
					continue;
				}
				boundary_graph[edge.a].push_back(edge.b);
				boundary_graph[edge.b].push_back(edge.a);
			}

			const std::vector<std::vector<std::size_t>> topology_loops =
				trace_index_loops(std::move(boundary_graph));
			const std::vector<std::vector<std::size_t>> feature_loops =
				smooth_region_boundary_index_loops(vertices, triangles);
			const std::vector<std::vector<std::size_t>> merged_loops =
				merge_boundary_index_loops(topology_loops, feature_loops, vertices);

			std::vector<Polyline3> loops;
			loops.reserve(merged_loops.size());
			for (const auto& loop_indices : merged_loops) {
				Polyline3 loop;
				loop.reserve(loop_indices.size());
				for (std::size_t vertex_index : loop_indices) {
					if (vertex_index < vertices.size()) {
						loop.push_back(vertices[vertex_index]);
					}
				}
				if (loop.size() >= 3U) {
					loops.push_back(std::move(loop));
				}
			}
			return loops;
		}

		void assign_surface_boundaries_from_loops(const std::vector<Polyline3>& loops,
			FaceBoundaries& boundaries) {
			const std::size_t outer_loop_count = std::min<std::size_t>(2U, loops.size());
			boundaries.outer_loops.insert(boundaries.outer_loops.end(),
				loops.begin(),
				loops.begin() + outer_loop_count);
			boundaries.inner_loops.insert(boundaries.inner_loops.end(),
				loops.begin() + outer_loop_count,
				loops.end());
		}

		// -----------------------------------------------------------------------------
		// 线性插值与距离计算
		// -----------------------------------------------------------------------------

		/// 两点之间线性插值：t=0 返回 start，t=1 返回 end
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

		/// 点到线段的最短距离平方
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

		/// 点到三角形的最短距离平方（经典 7 区域算法）
		double point_triangle_distance_squared(const SPAposition& point,
			const Triangle3& triangle) {
			const SPAvector ab = triangle.b - triangle.a;
			const SPAvector ac = triangle.c - triangle.a;
			const SPAvector ap = point - triangle.a;
			const double d1 = ab % ap;
			const double d2 = ac % ap;
			if (d1 <= 0.0 && d2 <= 0.0) {
				return ap.len_sq();
			}

			const SPAvector bp = point - triangle.b;
			const double d3 = ab % bp;
			const double d4 = ac % bp;
			if (d3 >= 0.0 && d4 <= d3) {
				return bp.len_sq();
			}

			const double vc = d1 * d4 - d3 * d2;
			if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
				const double v = d1 / (d1 - d3);
				const SPAposition closest = triangle.a + ab * v;
				return (point - closest).len_sq();
			}

			const SPAvector cp = point - triangle.c;
			const double d5 = ab % cp;
			const double d6 = ac % cp;
			if (d6 >= 0.0 && d5 <= d6) {
				return cp.len_sq();
			}

			const double vb = d5 * d2 - d1 * d6;
			if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
				const double w = d2 / (d2 - d6);
				const SPAposition closest = triangle.a + ac * w;
				return (point - closest).len_sq();
			}

			const double va = d3 * d6 - d5 * d4;
			if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
				const SPAvector bc = triangle.c - triangle.b;
				const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
				const SPAposition closest = triangle.b + bc * w;
				return (point - closest).len_sq();
			}

			const double denominator = 1.0 / (va + vb + vc);
			const double v = vb * denominator;
			const double w = vc * denominator;
			const SPAposition closest = triangle.a + ab * v + ac * w;
			return (point - closest).len_sq();
		}

		/// 点到包围盒的最短距离平方
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

		// -----------------------------------------------------------------------------
		// 平面投影与包含性测试
		// -----------------------------------------------------------------------------

		/// 将三维点投影到平面的 UV 坐标系
		std::array<double, 2> project_to_plane_uv(const SPAposition& point,
			const SPAposition& origin,
			const SPAunit_vector& u_axis,
			const SPAunit_vector& v_axis) {
			const SPAvector offset = point - origin;
			return { offset % u_axis, offset % v_axis };
		}

		/// 判断二维点是否在线段上
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

		// -----------------------------------------------------------------------------
		// 面板几何构建
		// -----------------------------------------------------------------------------

		/// 从 PlaneFace 构建平面几何信息（拟合平面 + 投影边界环）
		PlaneFaceGeometry build_plane_face_geometry(const PlaneFace& face) {
			if (face.boundaries.outer_loops.empty()) {
				throw std::runtime_error("平面板主面缺少外边界。");
			}

			SPAposition origin;
			SPAunit_vector normal;
			if (!fit_plane_from_loop(face.boundaries.outer_loops.front(), origin, normal)) {
				throw std::runtime_error("平面板主面无法拟合平面。");
			}

			const auto [u_axis, v_axis] = build_plane_basis(normal);
			PlaneFaceGeometry geometry;
			geometry.origin = origin;
			geometry.normal = normal;
			geometry.u_axis = u_axis;
			geometry.v_axis = v_axis;

			auto build_projected = [&](const Polyline3& loop) {
				ProjectedLoop projected;
				projected.world_points = loop;
				for (const SPAposition& point : loop) {
					projected.uv_points.push_back(
						project_to_plane_uv(point, origin, u_axis, v_axis));
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

		/// 在两组对应边界环之间生成侧壁三角形
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

		/// 从 PlanePanel 构建完整几何（两侧主面 + 侧壁）
		PlanePanelGeometry build_plane_panel_geometry(const PlanePanel& panel) {
			PlanePanelGeometry geometry;
			geometry.main_faces.push_back(build_plane_face_geometry(panel.face_a));
			geometry.main_faces.push_back(build_plane_face_geometry(panel.face_b));
			append_side_wall_triangles(panel.face_a.boundaries.outer_loops,
				panel.face_b.boundaries.outer_loops,
				geometry.side_triangles);
			append_side_wall_triangles(panel.face_a.boundaries.inner_loops,
				panel.face_b.boundaries.inner_loops,
				geometry.side_triangles);
			return geometry;
		}

		/// 从 SurfacePanel 构建曲面几何（展开索引三角形）
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
			return geometry;
		}

		// -----------------------------------------------------------------------------
		// 点到几何体的距离
		// -----------------------------------------------------------------------------

		/// 点到曲面几何的最短距离（包围盒加速裁剪）
		double point_to_surface_geometry_distance(const SPAposition& point,
			const SurfaceGeometry& geometry) {
			double best_distance_squared = std::numeric_limits<double>::infinity();
			for (const Triangle3& triangle : geometry.triangles) {
				if (point_box_distance_squared(point, triangle.box) > best_distance_squared) {
					continue;
				}
				best_distance_squared =
					std::min(best_distance_squared,
						point_triangle_distance_squared(point, triangle));
			}
			return std::sqrt(best_distance_squared);
		}

		/// 点到平面面板的最短距离（检查主面距离 + 侧壁距离）
		double point_to_plane_panel_distance(const SPAposition& point,
			const PlanePanelGeometry& geometry) {
			double best_distance = std::numeric_limits<double>::infinity();

			// 检查每个主面：点必须在外轮廓内且不在内孔内
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

			// 检查侧壁三角形
			double best_side_squared = std::numeric_limits<double>::infinity();
			for (const Triangle3& triangle : geometry.side_triangles) {
				if (point_box_distance_squared(point, triangle.box) > best_side_squared) {
					continue;
				}
				best_side_squared =
					std::min(best_side_squared,
						point_triangle_distance_squared(point, triangle));
			}
			if (best_side_squared < std::numeric_limits<double>::infinity()) {
				best_distance = std::min(best_distance, std::sqrt(best_side_squared));
			}

			return best_distance;
		}

		// -----------------------------------------------------------------------------
		// 焊缝辅助函数
		// -----------------------------------------------------------------------------

		/// 收集曲面板的所有候选边界环
		std::vector<Polyline3> collect_surface_candidate_loops(const SurfacePanel& panel) {
			std::vector<Polyline3> loops;
			for (const SurfacePatch& patch : panel.surfaces) {
				loops.insert(loops.end(), patch.boundaries.outer_loops.begin(),
					patch.boundaries.outer_loops.end());
				loops.insert(loops.end(), patch.boundaries.inner_loops.begin(),
					patch.boundaries.inner_loops.end());
			}
			return loops;
		}

		/// 收集平面板的所有候选边界环（face_a + face_b）
		std::vector<Polyline3> collect_plane_candidate_loops(const PlanePanel& panel) {
			std::vector<Polyline3> loops;
			loops.insert(loops.end(), panel.face_a.boundaries.outer_loops.begin(),
				panel.face_a.boundaries.outer_loops.end());
			loops.insert(loops.end(), panel.face_a.boundaries.inner_loops.begin(),
				panel.face_a.boundaries.inner_loops.end());
			loops.insert(loops.end(), panel.face_b.boundaries.outer_loops.begin(),
				panel.face_b.boundaries.outer_loops.end());
			loops.insert(loops.end(), panel.face_b.boundaries.inner_loops.begin(),
				panel.face_b.boundaries.inner_loops.end());
			return loops;
		}

		/// 追加点到折线（避免追加过近的重复点）
		void append_point_if_needed(Polyline3& polyline,
			const SPAposition& point,
			double tolerance) {
			if (polyline.empty() ||
				std::sqrt((polyline.back() - point).len_sq()) > tolerance * 0.25) {
				polyline.push_back(point);
			}
		}

		/// 计算折线总长度
		double polyline_length(const Polyline3& polyline) {
			double length = 0.0;
			for (std::size_t index = 1; index < polyline.size(); ++index) {
				length += (polyline[index] - polyline[index - 1]).len();
			}
			return length;
		}

		/// 二分法定位命中/未命中的过渡参数（24 次迭代，精度约 2^-24）
		template <typename HitPredicate>
		double locate_transition_parameter(const SPAposition& start,
			const SPAposition& end,
			double t0,
			double t1,
			bool start_hit,
			HitPredicate&& is_hit) {
			double left = t0;
			double right = t1;
			for (int iteration = 0; iteration < 24; ++iteration) {
				const double mid = 0.5 * (left + right);
				const bool mid_hit =
					is_hit(interpolate_position(start, end, mid));
				if (mid_hit == start_hit) {
					left = mid;
				}
				else {
					right = mid;
				}
			}
			return 0.5 * (left + right);
		}

		/// 计算点到折线的最短距离
		double point_to_polyline_distance(const SPAposition& point,
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

	}  // namespace

	// =============================================================================
	// 公开 API
	// =============================================================================

	/**
	 * @brief 解析平面板数据。
	 *
	 * 流程：解析路径 → 解析边界环 → 拟合平面 → 按共面分两组 → 按面积分外轮廓/内孔。
	 */
	PlanePanel api_get_plane_panel(const std::filesystem::path& model_path) {
		const ModelJsonPaths paths = resolve_model_json_paths(model_path);
		const std::vector<Polyline3> loops = parse_boundary_loops(paths.boundary_json);
		if (loops.size() < 2U) {
			throw std::runtime_error("平面板解析失败：至少需要两条边界环来表示两侧主面。");
		}

		// 为每个环计算几何信息，按共面关系分组
		std::vector<LoopGroup> groups;
		for (const Polyline3& loop : loops) {
			const PlaneLoopInfo info = build_plane_loop_info(loop);
			bool assigned = false;
			for (LoopGroup& group : groups) {
				if (same_plane_group(info, group)) {
					group.loops.push_back(info);
					assigned = true;
					break;
				}
			}
			if (!assigned) {
				LoopGroup group;
				group.origin = info.origin;
				group.normal = info.normal;
				group.plane_offset = info.plane_offset;
				group.loops.push_back(info);
				groups.push_back(std::move(group));
			}
		}

		if (groups.size() != 2U) {
			throw std::runtime_error(
				"平面板解析失败：当前实现要求边界环可稳定分成两个主面平面。");
		}

		// 按偏移量排序：face_a（偏移小）在前，face_b（偏移大）在后
		const SPAunit_vector reference_normal = groups.front().normal;
		std::sort(groups.begin(), groups.end(),
			[&reference_normal](const LoopGroup& left, const LoopGroup& right) {
				const double left_offset =
					point_offset_along_normal(left.origin, reference_normal);
				const double right_offset =
					point_offset_along_normal(right.origin, reference_normal);
				return left_offset < right_offset;
			});

		PlanePanel panel;
		panel.face_a = build_plane_face_from_group(groups[0]);
		panel.face_b = build_plane_face_from_group(groups[1]);
		return panel;
	}

	PlanePanel api_get_plane_panel(const float rxyPltBndry1[1000][3],
		const float rxyholes[20][1000][3],
		const float cAxis[4][3],
		float thickdis1,
		float thickdis2) {
		const RxyCurve outer_curve = parse_raw_rxy_curve(rxyPltBndry1, "平面板外边界");
		const std::vector<LocalPoint2> outer_local_points =
			expand_rxy_curve_to_polyline(outer_curve);

		const long long raw_hole_count = std::llround(rxyholes[0][0][0]);
		if (raw_hole_count < 0LL || raw_hole_count > 19LL) {
			throw std::runtime_error("平面板内孔解析失败：内孔数量超出支持范围。");
		}

		const SPAposition origin = parse_caxis_position_row(cAxis[0]);
		const SPAvector u_axis = parse_caxis_vector_row(cAxis[1], "u");
		const SPAvector v_axis = parse_caxis_vector_row(cAxis[2], "v");
		const SPAunit_vector w_axis = normalise(parse_caxis_vector_row(cAxis[3], "w"));
		const SPAvector face_a_offset = w_axis.vector() * static_cast<double>(thickdis1);
		const SPAvector face_b_offset = w_axis.vector() * static_cast<double>(-thickdis2);

		std::vector<Polyline3> inner_face_a_loops;
		std::vector<Polyline3> inner_face_b_loops;
		inner_face_a_loops.reserve(static_cast<std::size_t>(raw_hole_count));
		inner_face_b_loops.reserve(static_cast<std::size_t>(raw_hole_count));
		for (long long hole_index = 1; hole_index <= raw_hole_count; ++hole_index) {
			const RxyCurve hole_curve = parse_raw_rxy_curve(
				rxyholes[hole_index], "平面板内孔边界");
			const std::vector<LocalPoint2> hole_local_points =
				expand_rxy_curve_to_polyline(hole_curve);
			inner_face_a_loops.push_back(transform_local_polyline(
				hole_local_points, origin, u_axis, v_axis, face_a_offset));
			inner_face_b_loops.push_back(transform_local_polyline(
				hole_local_points, origin, u_axis, v_axis, face_b_offset));
		}

		PlanePanel panel;
		panel.face_a = build_plane_face_from_loops(
			transform_local_polyline(outer_local_points, origin, u_axis, v_axis, face_a_offset),
			std::move(inner_face_a_loops));
		panel.face_b = build_plane_face_from_loops(
			transform_local_polyline(outer_local_points, origin, u_axis, v_axis, face_b_offset),
			std::move(inner_face_b_loops));
		return panel;
	}

	/**
	 * @brief 解析曲面板数据。
	 *
	 * 流程：解析路径 → 解析三角网格 → 解析边界环 → 前 2 条为外轮廓，其余为内孔。
	 */
	SurfacePanel api_get_surface_panel(const std::filesystem::path& model_path) {
		const ModelJsonPaths paths = resolve_model_json_paths(model_path);

		SurfacePatch patch;
		parse_surface_mesh(paths.panel_json, patch.vertices, patch.triangles);

		const std::vector<Polyline3> loops = parse_boundary_loops(paths.boundary_json);
		assign_surface_boundaries_from_loops(loops, patch.boundaries);

		SurfacePanel panel;
		panel.surfaces.push_back(std::move(patch));
		return panel;
	}

	SurfacePanel api_get_surface(const std::filesystem::path& model_path) {
		return api_get_surface_panel(model_path);
	}

	SurfacePanel api_get_surface_panel(const float vertices[][3],
		int vertex_count,
		const int triangles[][3],
		int triangle_count,
		const float normals[][3]) {
		(void)normals;
		if (vertex_count < 0) {
			throw std::runtime_error("曲面板构造失败：顶点数量不能为负。");
		}
		if (triangle_count < 0) {
			throw std::runtime_error("曲面板构造失败：三角形数量不能为负。");
		}

		SurfacePatch patch;
		patch.vertices.reserve(static_cast<std::size_t>(vertex_count));
		for (int index = 0; index < vertex_count; ++index) {
			patch.vertices.emplace_back(vertices[index][0], vertices[index][1], vertices[index][2]);
		}

		patch.triangles.reserve(static_cast<std::size_t>(triangle_count));
		for (int index = 0; index < triangle_count; ++index) {
			const int i0 = triangles[index][0];
			const int i1 = triangles[index][1];
			const int i2 = triangles[index][2];
			if (i0 < 0 || i1 < 0 || i2 < 0 || i0 >= vertex_count || i1 >= vertex_count ||
				i2 >= vertex_count) {
				throw std::runtime_error("曲面板构造失败：三角形索引越界。");
			}
			patch.triangles.push_back(Triangle{ static_cast<std::size_t>(i0),
				static_cast<std::size_t>(i1),
				static_cast<std::size_t>(i2) });
		}

		const std::vector<Polyline3> loops =
			reconstruct_surface_boundary_loops(patch.vertices, patch.triangles);
		assign_surface_boundaries_from_loops(loops, patch.boundaries);

		SurfacePanel panel;
		panel.surfaces.push_back(std::move(patch));
		return panel;
	}

}  // namespace hanfeng
