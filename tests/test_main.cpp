// =============================================================================
// test_main.cpp - 测试入口
// =============================================================================
//
// 当前项目使用最简单的“手写测试函数 + assert”方式组织测试。
// 将 `HANFENG_WELD_ONLY_TESTS` 设为 1 时，仅运行真实模型的
// 平面板/曲面板焊缝导出测试；设为 0 时运行全部测试。
// =============================================================================

#ifndef HANFENG_WELD_ONLY_TESTS
#define HANFENG_WELD_ONLY_TESTS 1
#endif

// --- 基础几何测试 ---
void test_spa_position_and_vector_arithmetic();
void test_spa_box_semantics();

// --- 数据结构布局测试 ---
void test_rxy_curve_semantics();
void test_plane_panel_layout();
void test_surface_panel_layout();
void test_weld_curve_result_layout();

// --- 模型解析测试 ---
void test_api_get_plane_panel_reads_plane_panel_from_model_files();
void test_api_get_surface_reads_surface_panel_from_model_files();
void test_api_get_plane_reads_fb03c_166f_plate1_as_plane_panel();
void test_api_get_plane_panel_builds_faces_from_rxy_boundary_input();
void test_api_get_plane_panel_builds_inner_holes_from_rxy_input();
void test_api_get_plane_panel_samples_arc_segments_into_polylines();
void test_api_get_surface_panel_builds_single_topology_boundary_loop();
void test_api_get_surface_panel_falls_back_to_smooth_region_boundaries();
void test_api_get_surface_panel_classifies_extra_loops_as_inner_loops();
void test_api_get_surface_panel_rejects_out_of_range_triangle_indices();

// --- 焊缝计算测试 ---
void test_api_get_plane_surface_weld_detects_surface_boundary_on_plane_face();
void test_compute_plane_surface_weld_from_geometry_module();
void test_api_get_plane_surface_weld_detects_plane_boundary_coplanar_with_surface_face();
void test_api_get_plane_surface_weld_detects_plane_boundary_inside_surface_solid();
void test_compute_plane_surface_weld_detects_exact_band_without_sampling_loss();
void test_api_get_plane_surface_weld_detects_plane_boundary_near_surface_exterior_side();

// --- 真实模型导出测试 ---
void test_weld_with_real_models_and_export();

#if HANFENG_WELD_ONLY_TESTS

int main() {
	test_weld_with_real_models_and_export();
	return 0;
}

#else

int main() {
	test_spa_position_and_vector_arithmetic();
	test_spa_box_semantics();

	test_rxy_curve_semantics();
	test_plane_panel_layout();
	test_surface_panel_layout();
	test_weld_curve_result_layout();

	test_api_get_plane_panel_reads_plane_panel_from_model_files();
	test_api_get_surface_reads_surface_panel_from_model_files();
	test_api_get_plane_reads_fb03c_166f_plate1_as_plane_panel();
	test_api_get_plane_panel_builds_faces_from_rxy_boundary_input();
	test_api_get_plane_panel_builds_inner_holes_from_rxy_input();
	test_api_get_plane_panel_samples_arc_segments_into_polylines();
	test_api_get_surface_panel_builds_single_topology_boundary_loop();
	test_api_get_surface_panel_falls_back_to_smooth_region_boundaries();
	test_api_get_surface_panel_classifies_extra_loops_as_inner_loops();
	test_api_get_surface_panel_rejects_out_of_range_triangle_indices();

	test_api_get_plane_surface_weld_detects_surface_boundary_on_plane_face();
	test_compute_plane_surface_weld_from_geometry_module();
	test_api_get_plane_surface_weld_detects_plane_boundary_coplanar_with_surface_face();
	test_api_get_plane_surface_weld_detects_plane_boundary_inside_surface_solid();
	test_compute_plane_surface_weld_detects_exact_band_without_sampling_loss();
	test_api_get_plane_surface_weld_detects_plane_boundary_near_surface_exterior_side();
	return 0;
}

#endif
