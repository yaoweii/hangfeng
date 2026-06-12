// =============================================================================
// model_io_test.cpp — 模型 I/O 测试（平面板和曲面板的文件解析）
// =============================================================================
//
// 测试覆盖：
//   - api_get_plane 从模型文件中正确读取平面板数据
//   - api_get_surface 从模型文件中正确读取曲面板数据
//   - api_get_surface 正确将多余边界环分类为内孔
//
// 这些测试依赖 model/ 目录下的实际模型文件作为测试数据。
// =============================================================================

#include <cassert>
#include <filesystem>

#include "hanfeng/panel.hpp"
#include "test_common.hpp"

namespace {

	template <std::size_t N>
	void zero_point_array(float(&points)[N][3]) {
		for (std::size_t index = 0; index < N; ++index) {
			points[index][0] = 0.0f;
			points[index][1] = 0.0f;
			points[index][2] = 0.0f;
		}
	}

	template <std::size_t N>
	void zero_triangle_array(int(&triangles)[N][3]) {
		for (std::size_t index = 0; index < N; ++index) {
			triangles[index][0] = 0;
			triangles[index][1] = 0;
			triangles[index][2] = 0;
		}
	}

	bool polyline_contains_point(const hanfeng::Polyline3& loop,
		double x,
		double y,
		double z,
		double epsilon = 1.0e-6) {
		for (const hanfeng::SPAposition& point : loop) {
			if (nearly_equal(point.x(), x, epsilon) && nearly_equal(point.y(), y, epsilon) &&
				nearly_equal(point.z(), z, epsilon)) {
				return true;
			}
		}
		return false;
	}

	void zero_curve(float curve[1000][3]) {
		for (int row = 0; row < 1000; ++row) {
			curve[row][0] = 0.0f;
			curve[row][1] = 0.0f;
			curve[row][2] = 0.0f;
		}
	}

	void zero_holes(float holes[20][1000][3]) {
		for (int hole = 0; hole < 20; ++hole) {
			zero_curve(holes[hole]);
		}
	}

	void set_identity_caxis(float cAxis[4][3]) {
		cAxis[0][0] = 0.0f;
		cAxis[0][1] = 0.0f;
		cAxis[0][2] = 0.0f;
		cAxis[1][0] = 1.0f;
		cAxis[1][1] = 0.0f;
		cAxis[1][2] = 0.0f;
		cAxis[2][0] = 0.0f;
		cAxis[2][1] = 1.0f;
		cAxis[2][2] = 0.0f;
		cAxis[3][0] = 0.0f;
		cAxis[3][1] = 0.0f;
		cAxis[3][2] = 1.0f;
	}

	void set_rotated_caxis(float cAxis[4][3]) {
		cAxis[0][0] = 10.0f;
		cAxis[0][1] = 20.0f;
		cAxis[0][2] = 30.0f;
		cAxis[1][0] = 0.0f;
		cAxis[1][1] = 1.0f;
		cAxis[1][2] = 0.0f;
		cAxis[2][0] = 0.0f;
		cAxis[2][1] = 0.0f;
		cAxis[2][2] = 1.0f;
		cAxis[3][0] = 1.0f;
		cAxis[3][1] = 0.0f;
		cAxis[3][2] = 0.0f;
	}

	void set_rectangle_boundary_curve(float curve[1000][3]) {
		zero_curve(curve);
		curve[0][0] = 6.0f;
		curve[1][0] = 0.0f;
		curve[1][1] = 0.0f;
		curve[1][2] = 0.0f;
		curve[2][0] = 0.0f;
		curve[2][1] = 4.0f;
		curve[2][2] = 0.0f;
		curve[3][0] = 0.0f;
		curve[3][1] = 4.0f;
		curve[3][2] = 2.0f;
		curve[4][0] = 0.0f;
		curve[4][1] = 0.0f;
		curve[4][2] = 2.0f;
		curve[5][0] = 0.0f;
		curve[5][1] = 0.0f;
		curve[5][2] = 0.0f;
	}

	void set_hole_curve(float curve[1000][3]) {
		zero_curve(curve);
		curve[0][0] = 6.0f;
		curve[1][0] = 0.0f;
		curve[1][1] = 1.0f;
		curve[1][2] = 0.5f;
		curve[2][0] = 0.0f;
		curve[2][1] = 2.0f;
		curve[2][2] = 0.5f;
		curve[3][0] = 0.0f;
		curve[3][1] = 2.0f;
		curve[3][2] = 1.5f;
		curve[4][0] = 0.0f;
		curve[4][1] = 1.0f;
		curve[4][2] = 1.5f;
		curve[5][0] = 0.0f;
		curve[5][1] = 1.0f;
		curve[5][2] = 0.5f;
	}

	void set_arc_boundary_curve(float curve[1000][3]) {
		zero_curve(curve);
		curve[0][0] = 5.0f;
		curve[1][0] = 1.0f;
		curve[1][1] = 1.0f;
		curve[1][2] = 0.0f;
		curve[2][0] = 0.0f;
		curve[2][1] = 0.0f;
		curve[2][2] = -1.0f;
		curve[3][0] = 0.0f;
		curve[3][1] = 0.0f;
		curve[3][2] = 0.0f;
		curve[4][0] = 0.0f;
		curve[4][1] = 1.0f;
		curve[4][2] = 0.0f;
	}

}  // namespace

/**
 * @brief 测试 api_get_plane_panel 从模型文件中正确解析平面板。
 *
 * 读取 model/plane_panels/FB03C-2D-PLATE1 目录下的模型文件，
 * 验证：
 * - 两侧主面（face_a、face_b）各有 1 条外轮廓环
 * - 外轮廓环各有 28 个点
 * - face_a 的 z 坐标约为 9385.0（较低的 Z 面）
 * - face_b 的 z 坐标约为 9396.0（较高的 Z 面）
 */
void test_api_get_plane_panel_reads_plane_panel_from_model_files() {
	const std::filesystem::path model_path =
		std::filesystem::path(HANFENG_SOURCE_DIR) /
		"model/plane_panels/FB03C-2D-PLATE1";
	const hanfeng::PlanePanel panel = hanfeng::api_get_plane_panel(model_path);

	// 两侧主面各有 1 条外轮廓
	assert(panel.face_a.boundaries.outer_loops.size() == 1U);
	assert(panel.face_b.boundaries.outer_loops.size() == 1U);
	// 每条外轮廓有 28 个点
	assert(panel.face_a.boundaries.outer_loops.front().size() == 28U);
	assert(panel.face_b.boundaries.outer_loops.front().size() == 28U);
	// 验证 Z 坐标（face_a 在低面，face_b 在高面）
	assert(nearly_equal(panel.face_a.boundaries.outer_loops.front().front().z(),
		9385.0));
	assert(nearly_equal(panel.face_b.boundaries.outer_loops.front().front().z(),
		9396.0));
}

/**
 * @brief 测试 api_get_surface 从模型文件中正确解析曲面板。
 *
 * 读取 model/surface_panels/FB03C-PL268-CPLATE 目录下的模型文件，
 * 验证：
 * - 1 个曲面面片
 * - 648 个顶点、1292 个三角形
 * - 2 条外轮廓环、0 条内孔
 */
void test_api_get_surface_reads_surface_panel_from_model_files() {
	const std::filesystem::path model_path =
		std::filesystem::path(HANFENG_SOURCE_DIR) /
		"model/surface_panels/FB03C-PL268-CPLATE";
	const hanfeng::SurfacePanel panel = hanfeng::api_get_surface_panel(model_path);

	assert(panel.surfaces.size() == 1U);
	assert(panel.surfaces.front().vertices.size() == 648U);
	assert(panel.surfaces.front().triangles.size() == 1292U);
	assert(panel.surfaces.front().boundaries.outer_loops.size() == 2U);
	assert(panel.surfaces.front().boundaries.inner_loops.empty());
}

/**
 * @brief 测试 api_get_surface 将多余边界环正确分类为内孔。
 *
 * 读取 model/surface_panels/FB03C-166F-PLATE1 目录下的模型文件，
 * 验证：
 * - 1 个曲面面片
 * - 2 条外轮廓环（业务约定：最多 2 条）
 * - 12 条内孔轮廓环（其余全部归为内孔）
 */
void test_api_get_plane_reads_fb03c_166f_plate1_as_plane_panel() {
	const std::filesystem::path model_path =
		std::filesystem::path(HANFENG_SOURCE_DIR) /
		"model/plane_panels/FB03C-166F-PLATE1";
	const hanfeng::PlanePanel panel = hanfeng::api_get_plane_panel(model_path);

	assert(panel.face_a.boundaries.outer_loops.size() == 1U);
	assert(panel.face_b.boundaries.outer_loops.size() == 1U);
	assert(panel.face_a.boundaries.inner_loops.size() == 6U);
	assert(panel.face_b.boundaries.inner_loops.size() == 6U);
}

void test_api_get_plane_panel_groups_axis_aligned_noisy_model_loops() {
	struct ExpectedPanel {
		const char* name;
		std::size_t inner_loop_count_per_face;
	};

	const ExpectedPanel panels[] = {
		{ "FB03C-0LB-PLATE1", 4U },
		{ "FB03C-162F-PLATE1", 1U },
		{ "FB03C-162FA-CLIP2", 1U },
		{ "FB03C-162FA-CLIP5", 1U },
		{ "FB03C-162FA-PLATE2", 1U },
		{ "FB03C-163F-PLATE1", 1U },
		{ "FB03C-164FE-PLATE1", 1U },
	};

	for (const ExpectedPanel& expected : panels) {
		const std::filesystem::path model_path =
			std::filesystem::path(HANFENG_SOURCE_DIR) /
			"model/plane_panels" / expected.name;
		const hanfeng::PlanePanel panel = hanfeng::api_get_plane_panel(model_path);

		assert(panel.face_a.boundaries.outer_loops.size() == 1U);
		assert(panel.face_b.boundaries.outer_loops.size() == 1U);
		assert(panel.face_a.boundaries.inner_loops.size() ==
			expected.inner_loop_count_per_face);
		assert(panel.face_b.boundaries.inner_loops.size() ==
			expected.inner_loop_count_per_face);
	}
}

void test_api_get_plane_panel_builds_faces_from_rxy_boundary_input() {
	float boundary[1000][3];
	float holes[20][1000][3];
	float cAxis[4][3];
	set_rectangle_boundary_curve(boundary);
	zero_holes(holes);
	set_rotated_caxis(cAxis);

	const hanfeng::PlanePanel panel =
		hanfeng::api_get_plane_panel(boundary, holes, cAxis, 2.0f, 3.0f);

	assert(panel.face_a.boundaries.outer_loops.size() == 1U);
	assert(panel.face_b.boundaries.outer_loops.size() == 1U);
	assert(panel.face_a.boundaries.inner_loops.empty());
	assert(panel.face_b.boundaries.inner_loops.empty());

	const auto& face_a = panel.face_a.boundaries.outer_loops.front();
	const auto& face_b = panel.face_b.boundaries.outer_loops.front();
	assert(face_a.size() == 4U);
	assert(face_b.size() == 4U);

	assert(nearly_equal(face_a[0].x(), 12.0));
	assert(nearly_equal(face_a[0].y(), 20.0));
	assert(nearly_equal(face_a[0].z(), 30.0));
	assert(nearly_equal(face_a[1].x(), 12.0));
	assert(nearly_equal(face_a[1].y(), 24.0));
	assert(nearly_equal(face_a[1].z(), 30.0));
	assert(nearly_equal(face_a[2].x(), 12.0));
	assert(nearly_equal(face_a[2].y(), 24.0));
	assert(nearly_equal(face_a[2].z(), 32.0));

	assert(nearly_equal(face_b[0].x(), 7.0));
	assert(nearly_equal(face_b[0].y(), 20.0));
	assert(nearly_equal(face_b[0].z(), 30.0));
	assert(nearly_equal(face_b[2].x(), 7.0));
	assert(nearly_equal(face_b[2].y(), 24.0));
	assert(nearly_equal(face_b[2].z(), 32.0));
}

void test_api_get_plane_panel_builds_inner_holes_from_rxy_input() {
	float boundary[1000][3];
	float holes[20][1000][3];
	float cAxis[4][3];
	set_rectangle_boundary_curve(boundary);
	zero_holes(holes);
	set_identity_caxis(cAxis);

	holes[0][0][0] = 1.0f;
	set_hole_curve(holes[1]);

	const hanfeng::PlanePanel panel =
		hanfeng::api_get_plane_panel(boundary, holes, cAxis, 1.5f, 0.5f);

	assert(panel.face_a.boundaries.inner_loops.size() == 1U);
	assert(panel.face_b.boundaries.inner_loops.size() == 1U);

	const auto& face_a_hole = panel.face_a.boundaries.inner_loops.front();
	const auto& face_b_hole = panel.face_b.boundaries.inner_loops.front();
	assert(face_a_hole.size() == 4U);
	assert(face_b_hole.size() == 4U);
	assert(nearly_equal(face_a_hole[0].x(), 1.0));
	assert(nearly_equal(face_a_hole[0].y(), 0.5));
	assert(nearly_equal(face_a_hole[0].z(), 1.5));
	assert(nearly_equal(face_b_hole[2].x(), 2.0));
	assert(nearly_equal(face_b_hole[2].y(), 1.5));
	assert(nearly_equal(face_b_hole[2].z(), -0.5));
}

void test_api_get_plane_panel_samples_arc_segments_into_polylines() {
	float boundary[1000][3];
	float holes[20][1000][3];
	float cAxis[4][3];
	set_arc_boundary_curve(boundary);
	zero_holes(holes);
	set_identity_caxis(cAxis);

	const hanfeng::PlanePanel panel =
		hanfeng::api_get_plane_panel(boundary, holes, cAxis, 0.0f, 0.0f);

	assert(panel.face_a.boundaries.outer_loops.size() == 1U);
	const auto& loop = panel.face_a.boundaries.outer_loops.front();
	assert(loop.size() > 4U);
	assert(nearly_equal(loop.front().x(), 1.0));
	assert(nearly_equal(loop.front().y(), 0.0));
	assert(nearly_equal(loop.front().z(), 0.0));

	bool found_arc_sample = false;
	for (const hanfeng::SPAposition& point : loop) {
		if (point.y() < -1.0e-3 && point.x() > 1.0e-3) {
			found_arc_sample = true;
			break;
		}
	}
	assert(found_arc_sample);
}

void test_api_get_surface_panel_builds_single_topology_boundary_loop() {
	float vertices[8][3];
	float normals[8][3];
	int triangles[8][3];
	zero_point_array(vertices);
	zero_point_array(normals);
	zero_triangle_array(triangles);

	vertices[0][0] = 0.0f;
	vertices[0][1] = 0.0f;
	vertices[0][2] = 0.0f;
	vertices[1][0] = 1.0f;
	vertices[1][1] = 0.0f;
	vertices[1][2] = 0.0f;
	vertices[2][0] = 1.0f;
	vertices[2][1] = 1.0f;
	vertices[2][2] = 0.0f;
	vertices[3][0] = 0.0f;
	vertices[3][1] = 1.0f;
	vertices[3][2] = 0.0f;
	for (int index = 0; index < 4; ++index) {
		normals[index][2] = 1.0f;
	}

	triangles[0][0] = 0;
	triangles[0][1] = 1;
	triangles[0][2] = 2;
	triangles[1][0] = 0;
	triangles[1][1] = 2;
	triangles[1][2] = 3;

	const hanfeng::SurfacePanel panel =
		hanfeng::api_get_surface_panel(vertices, 4, triangles, 2, normals);

	assert(panel.surfaces.size() == 1U);
	assert(panel.surfaces.front().vertices.size() == 4U);
	assert(panel.surfaces.front().triangles.size() == 2U);
	assert(panel.surfaces.front().boundaries.outer_loops.size() == 1U);
	assert(panel.surfaces.front().boundaries.inner_loops.empty());

	const auto& loop = panel.surfaces.front().boundaries.outer_loops.front();
	assert(loop.size() == 4U);
	assert(polyline_contains_point(loop, 0.0, 0.0, 0.0));
	assert(polyline_contains_point(loop, 1.0, 0.0, 0.0));
	assert(polyline_contains_point(loop, 1.0, 1.0, 0.0));
	assert(polyline_contains_point(loop, 0.0, 1.0, 0.0));
}

void test_api_get_surface_panel_falls_back_to_smooth_region_boundaries() {
	float vertices[16][3];
	float normals[16][3];
	int triangles[20][3];
	zero_point_array(vertices);
	zero_point_array(normals);
	zero_triangle_array(triangles);

	vertices[0][0] = 0.5f; vertices[0][1] = 0.5f; vertices[0][2] = 1.0f;  // top_center
	vertices[1][0] = 0.5f; vertices[1][1] = 0.5f; vertices[1][2] = 0.0f;  // bottom_center
	vertices[2][0] = 0.0f; vertices[2][1] = 0.0f; vertices[2][2] = 1.0f;
	vertices[3][0] = 1.0f; vertices[3][1] = 0.0f; vertices[3][2] = 1.0f;
	vertices[4][0] = 1.0f; vertices[4][1] = 1.0f; vertices[4][2] = 1.0f;
	vertices[5][0] = 0.0f; vertices[5][1] = 1.0f; vertices[5][2] = 1.0f;
	vertices[6][0] = 0.0f; vertices[6][1] = 0.0f; vertices[6][2] = 0.0f;
	vertices[7][0] = 1.0f; vertices[7][1] = 0.0f; vertices[7][2] = 0.0f;
	vertices[8][0] = 1.0f; vertices[8][1] = 1.0f; vertices[8][2] = 0.0f;
	vertices[9][0] = 0.0f; vertices[9][1] = 1.0f; vertices[9][2] = 0.0f;
	for (int index = 0; index < 10; ++index) {
		normals[index][2] = 1.0f;
	}

	const int triangle_data[16][3] = {
		{ 0, 2, 3 }, { 0, 3, 4 }, { 0, 4, 5 }, { 0, 5, 2 },
		{ 1, 7, 6 }, { 1, 8, 7 }, { 1, 9, 8 }, { 1, 6, 9 },
		{ 2, 6, 7 }, { 2, 7, 3 }, { 3, 7, 8 }, { 3, 8, 4 },
		{ 4, 8, 9 }, { 4, 9, 5 }, { 5, 9, 6 }, { 5, 6, 2 }
	};
	for (int index = 0; index < 16; ++index) {
		triangles[index][0] = triangle_data[index][0];
		triangles[index][1] = triangle_data[index][1];
		triangles[index][2] = triangle_data[index][2];
	}

	const hanfeng::SurfacePanel panel =
		hanfeng::api_get_surface_panel(vertices, 10, triangles, 16, normals);

	assert(panel.surfaces.size() == 1U);
	assert(panel.surfaces.front().boundaries.outer_loops.size() == 2U);
	assert(panel.surfaces.front().boundaries.inner_loops.empty());
	assert(panel.surfaces.front().boundaries.outer_loops[0].size() == 4U);
	assert(panel.surfaces.front().boundaries.outer_loops[1].size() == 4U);
}

void test_api_get_surface_panel_classifies_extra_loops_as_inner_loops() {
	float vertices[20][3];
	float normals[20][3];
	int triangles[24][3];
	zero_point_array(vertices);
	zero_point_array(normals);
	zero_triangle_array(triangles);

	vertices[0][0] = 0.5f; vertices[0][1] = 0.5f; vertices[0][2] = 1.0f;
	vertices[1][0] = 0.5f; vertices[1][1] = 0.5f; vertices[1][2] = 0.0f;
	vertices[2][0] = 0.0f; vertices[2][1] = 0.0f; vertices[2][2] = 1.0f;
	vertices[3][0] = 1.0f; vertices[3][1] = 0.0f; vertices[3][2] = 1.0f;
	vertices[4][0] = 1.0f; vertices[4][1] = 1.0f; vertices[4][2] = 1.0f;
	vertices[5][0] = 0.0f; vertices[5][1] = 1.0f; vertices[5][2] = 1.0f;
	vertices[6][0] = 0.0f; vertices[6][1] = 0.0f; vertices[6][2] = 0.0f;
	vertices[7][0] = 1.0f; vertices[7][1] = 0.0f; vertices[7][2] = 0.0f;
	vertices[8][0] = 1.0f; vertices[8][1] = 1.0f; vertices[8][2] = 0.0f;
	vertices[9][0] = 0.0f; vertices[9][1] = 1.0f; vertices[9][2] = 0.0f;
	vertices[10][0] = 3.0f; vertices[10][1] = 0.0f; vertices[10][2] = 0.0f;
	vertices[11][0] = 4.0f; vertices[11][1] = 0.0f; vertices[11][2] = 0.0f;
	vertices[12][0] = 4.0f; vertices[12][1] = 1.0f; vertices[12][2] = 0.0f;
	vertices[13][0] = 3.0f; vertices[13][1] = 1.0f; vertices[13][2] = 0.0f;
	for (int index = 0; index < 14; ++index) {
		normals[index][2] = 1.0f;
	}

	const int triangle_data[18][3] = {
		{ 0, 2, 3 }, { 0, 3, 4 }, { 0, 4, 5 }, { 0, 5, 2 },
		{ 1, 7, 6 }, { 1, 8, 7 }, { 1, 9, 8 }, { 1, 6, 9 },
		{ 2, 6, 7 }, { 2, 7, 3 }, { 3, 7, 8 }, { 3, 8, 4 },
		{ 4, 8, 9 }, { 4, 9, 5 }, { 5, 9, 6 }, { 5, 6, 2 },
		{ 10, 11, 12 }, { 10, 12, 13 }
	};
	for (int index = 0; index < 18; ++index) {
		triangles[index][0] = triangle_data[index][0];
		triangles[index][1] = triangle_data[index][1];
		triangles[index][2] = triangle_data[index][2];
	}

	const hanfeng::SurfacePanel panel =
		hanfeng::api_get_surface_panel(vertices, 14, triangles, 18, normals);

	assert(panel.surfaces.size() == 1U);
	assert(panel.surfaces.front().boundaries.outer_loops.size() == 2U);
	assert(panel.surfaces.front().boundaries.inner_loops.size() == 1U);
	assert(panel.surfaces.front().boundaries.outer_loops[0].size() == 4U);
	assert(panel.surfaces.front().boundaries.outer_loops[1].size() == 4U);
	assert(panel.surfaces.front().boundaries.inner_loops[0].size() == 4U);
}

void test_api_get_surface_panel_rejects_out_of_range_triangle_indices() {
	float vertices[4][3];
	float normals[4][3];
	int triangles[2][3];
	zero_point_array(vertices);
	zero_point_array(normals);
	zero_triangle_array(triangles);

	vertices[0][0] = 0.0f;
	vertices[1][0] = 1.0f;
	vertices[2][1] = 1.0f;
	normals[0][2] = 1.0f;
	normals[1][2] = 1.0f;
	normals[2][2] = 1.0f;

	triangles[0][0] = 0;
	triangles[0][1] = 1;
	triangles[0][2] = 3;

	bool thrown = false;
	try {
		(void)hanfeng::api_get_surface_panel(vertices, 3, triangles, 1, normals);
	}
	catch (const std::runtime_error&) {
		thrown = true;
	}
	assert(thrown);
}
