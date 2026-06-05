// =============================================================================
// data_structures_layout_test.cpp — 数据结构布局和语义的测试
// =============================================================================
//
// 测试覆盖：
//   - RxyCurve 的曲线编码语义（header 点数、直线/圆弧/截断判断）
//   - PlanePanel 的布局（face_a、face_b、外轮廓、内孔）
//   - SurfacePanel 的布局（SurfacePatch 的顶点、三角形、边界）
//   - WeldCurveResult 的布局（WeldPolyline 的点序列）
//
// 使用 assert 宏进行断言。
// =============================================================================

#include <cassert>

#include "hanfeng/panel.hpp"
#include "hanfeng/weld.hpp"

/**
 * @brief 测试 RxyCurve 的编码语义。
 *
 * 构造一条包含 5 个点（1 个 header + 4 个几何点）的曲线：
 * - header: r=5.0（声明 5 个点）
 * - points[1]: r=0.0 → 直线
 * - points[2]: r=6.0 → 圆弧
 * - points[3]: r=2.0 → 截断标记
 * - points[4]: r=0.0 → 直线
 */
void test_rxy_curve_semantics() {
  hanfeng::RxyCurve curve{
      {hanfeng::RxyPoint{5.0, 0.0, 0.0},   // header: r=5 表示总共 5 个点
       hanfeng::RxyPoint{0.0, 0.0, 0.0},   // r=0: 直线
       hanfeng::RxyPoint{6.0, 10.0, 0.0},  // r=6: 圆弧
       hanfeng::RxyPoint{2.0, 20.0, 5.0},  // r=2: 截断标记
       hanfeng::RxyPoint{0.0, 30.0, 5.0}}}; // r=0: 直线

  // header 的 r 值等于数组大小
  assert(curve.header_matches_point_count());
  // 声明点数 = 5（含 header）
  assert(curve.declared_point_count() == 5U);
  // 几何点数 = 4（不含 header）
  assert(curve.geometry_point_count() == 4U);
  // 验证各几何点的曲线类型判断
  assert(curve.points.at(1).defines_line_to_next());        // r=0 → 直线
  assert(curve.points.at(2).defines_arc_to_next());         // r=6 → 圆弧
  assert(curve.points.at(3).defines_truncation_to_next());  // r=2 → 截断
}

/**
 * @brief 测试 PlanePanel 的布局。
 *
 * 构造一个平面板，face_a 在 z=0 平面，face_b 在 z=2 平面。
 * face_a 有一个外轮廓矩形和一个内孔矩形。
 */
void test_plane_panel_layout() {
  hanfeng::PlanePanel panel{};
  // face_a 外轮廓：z=0 平面上的 10x5 矩形
  panel.face_a.boundaries.outer_loops.push_back(
      {hanfeng::SPAposition(0.0, 0.0, 0.0),
       hanfeng::SPAposition(10.0, 0.0, 0.0),
       hanfeng::SPAposition(10.0, 5.0, 0.0),
       hanfeng::SPAposition(0.0, 5.0, 0.0)});
  // face_b 外轮廓：z=2 平面上的 10x5 矩形
  panel.face_b.boundaries.outer_loops.push_back(
      {hanfeng::SPAposition(0.0, 0.0, 2.0),
       hanfeng::SPAposition(10.0, 0.0, 2.0),
       hanfeng::SPAposition(10.0, 5.0, 2.0),
       hanfeng::SPAposition(0.0, 5.0, 2.0)});
  // face_a 内孔：z=0 平面上的 2x2 矩形
  panel.face_a.boundaries.inner_loops.push_back(
      {hanfeng::SPAposition(2.0, 1.0, 0.0),
       hanfeng::SPAposition(4.0, 1.0, 0.0),
       hanfeng::SPAposition(4.0, 3.0, 0.0),
       hanfeng::SPAposition(2.0, 3.0, 0.0)});

  // 验证布局
  assert(panel.face_a.boundaries.outer_loops.size() == 1U);
  assert(panel.face_b.boundaries.outer_loops.size() == 1U);
  assert(panel.face_a.boundaries.inner_loops.size() == 1U);
}

/**
 * @brief 测试 SurfacePanel 的布局。
 *
 * 构造一个曲面板，包含一个面片（3 个顶点、1 个三角形、1 条外轮廓环）。
 */
void test_surface_panel_layout() {
  hanfeng::SurfacePanel panel{};
  hanfeng::SurfacePatch patch{};
  // 3 个顶点
  patch.vertices = {
      hanfeng::SPAposition(0.0, 0.0, 0.0),
      hanfeng::SPAposition(1.0, 0.0, 0.0),
      hanfeng::SPAposition(0.0, 1.0, 0.0),
  };
  // 1 个三角形（索引 0, 1, 2）
  patch.triangles = {
      hanfeng::Triangle{0U, 1U, 2U},
  };
  // 外轮廓环
  patch.boundaries.outer_loops.push_back(
      {hanfeng::SPAposition(0.0, 0.0, 0.0),
       hanfeng::SPAposition(1.0, 0.0, 0.0),
       hanfeng::SPAposition(0.0, 1.0, 0.0)});
  panel.surfaces.push_back(patch);

  // 验证布局
  assert(panel.surfaces.size() == 1U);
  assert(panel.surfaces.front().vertices.size() == 3U);
  assert(panel.surfaces.front().triangles.front()[2] == 2U);
  assert(panel.surfaces.front().boundaries.outer_loops.size() == 1U);
}

/**
 * @brief 测试 WeldCurveResult 的布局。
 *
 * 构造一个焊缝结果，包含 1 条折线（3 个点）。
 */
void test_weld_curve_result_layout() {
  hanfeng::WeldCurveResult result{};
  // 一条从 (0,0,0) 到 (2,0,0) 的焊缝折线
  result.polylines.push_back(
      hanfeng::WeldPolyline{{hanfeng::SPAposition(0.0, 0.0, 0.0),
                             hanfeng::SPAposition(1.0, 0.0, 0.0),
                             hanfeng::SPAposition(2.0, 0.0, 0.0)}});

  assert(result.polylines.size() == 1U);
  assert(result.polylines.front().points.size() == 3U);
}
