# =============================================================================
# test_dxf_exporter.py — DXF 导出器的 Python 单元测试
# =============================================================================
#
# 测试覆盖：
#   - 默认平滑角度常量
#   - DXF 块解析（3DFACE 三角面 + POLYLINE 网格折线）
#   - 边界环重建（拓扑边界 + 光滑区域边界）
#   - 面板分类（平面/曲面）
#   - OBJ 文本生成（顶点去重 + 面片写入）
#   - rxyz 曲线采样
#   - 圆弧段拟合
#   - 拟合 UI HTML 生成
#   - 演示 HTML 生成
#   - 端到端 DXF 导出流程
#
# 使用 Python 标准库 unittest 框架。
# =============================================================================

import json
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

from dxf_exporter import (
    _fit_arc_segment,
    BlockMesh,
    DEFAULT_SMOOTH_ANGLE_DEGREES,
    build_sampled_rxyz_curve,
    build_obj_text,
    classify_panel,
    export_dxf_models,
    fit_plane_boundary_loop,
    generate_demo_html,
    generate_fit_ui_html,
    parse_dxf_blocks,
    reconstruct_boundary_loops,
)


class DxfExporterTests(unittest.TestCase):
    """DXF 导出器测试套件。"""

    def test_default_smooth_angle_degrees_is_raised(self):
        """验证默认平滑角度为 50.0 度。"""
        self.assertEqual(DEFAULT_SMOOTH_ANGLE_DEGREES, 50.0)

    def test_parse_dxf_blocks_reads_block_faces_and_mesh_polyline(self):
        """测试 DXF 块解析：从 DXF 文本中提取 3DFACE 三角面和 POLYLINE 网格折线。

        构造一个包含 TEST-BLOCK 的 DXF 文本，该块内含有：
        - 1 个 3DFACE（三角面）
        - 1 个 POLYLINE（网格折线，2 个顶点）
        - 1 个 INSERT 引用
        验证解析后能正确提取块名、三角形顶点坐标和折线顶点。
        """
        # DXF 文本：包含 BLOCKS 段和 ENTITIES 段的完整 DXF 结构
        text = """  0
SECTION
  2
BLOCKS
  0
BLOCK
  8
0
  2
TEST-BLOCK
 70
64
 10
0.0
 20
0.0
 30
0.0
  3
TEST-BLOCK
  0
3DFACE
 10
0.0
 20
0.0
 30
0.0
 11
1.0
 21
0.0
 31
0.0
 12
1.0
 22
1.0
 32
0.0
 13
0.0
 23
0.0
 33
0.0
  0
POLYLINE
 70
16
 71
2
 72
4
  0
VERTEX
 10
0.0
 20
0.0
 30
0.0
  0
VERTEX
 10
1.0
 20
0.0
 30
0.0
  0
SEQEND
  0
ENDBLK
  0
ENDSEC
  0
SECTION
  2
ENTITIES
  0
INSERT
  2
TEST-BLOCK
 10
0.0
 20
0.0
 30
0.0
  0
SEQEND
  0
ENDSEC
  0
EOF
"""
        blocks = parse_dxf_blocks(text.splitlines())
        self.assertEqual(set(blocks), {"TEST-BLOCK"})
        mesh = blocks["TEST-BLOCK"]
        # 验证三角形：顶点为 (0,0,0), (1,0,0), (1,1,0)
        self.assertEqual(len(mesh.triangles), 1)
        self.assertEqual(
            mesh.triangles[0],
            ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)),
        )
        # 验证网格折线：2 个顶点
        self.assertEqual(len(mesh.mesh_polylines), 1)
        self.assertEqual(mesh.mesh_polylines[0], [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)])

    def test_reconstruct_boundary_loops_returns_single_square_loop(self):
        """测试边界环重建：两个三角形组成一个正方形，应重建出 1 条 4 点边界环。"""
        triangles = [
            ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)),
            ((0.0, 0.0, 0.0), (1.0, 1.0, 0.0), (0.0, 1.0, 0.0)),
        ]

        loops = reconstruct_boundary_loops(triangles)

        self.assertEqual(len(loops), 1)
        self.assertEqual(len(loops[0]), 4)
        self.assertEqual(
            set(loops[0]),
            {
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (1.0, 1.0, 0.0),
                (0.0, 1.0, 0.0),
            },
        )

    def test_reconstruct_boundary_loops_falls_back_to_smooth_region_boundaries(self):
        """测试边界环重建回退到光滑区域边界。

        构造一个八面体的三角网格（上下各 4 个三角形 + 4 个侧面三角形），
        由于侧面三角形之间不共享完整边，拓扑边界为空，
        回退到基于法向量光滑性的区域边界检测，应得到 2 条边界环（顶面和底面）。
        """
        # 定义八面体的关键顶点
        top_center = (0.5, 0.5, 1.0)
        bottom_center = (0.5, 0.5, 0.0)
        top_a = (0.0, 0.0, 1.0)
        top_b = (1.0, 0.0, 1.0)
        top_c = (1.0, 1.0, 1.0)
        top_d = (0.0, 1.0, 1.0)
        bottom_a = (0.0, 0.0, 0.0)
        bottom_b = (1.0, 0.0, 0.0)
        bottom_c = (1.0, 1.0, 0.0)
        bottom_d = (0.0, 1.0, 0.0)
        # 上半部 4 个三角形
        triangles = [
            (top_center, top_a, top_b),
            (top_center, top_b, top_c),
            (top_center, top_c, top_d),
            (top_center, top_d, top_a),
            # 下半部 4 个三角形（法向量朝下）
            (bottom_center, bottom_b, bottom_a),
            (bottom_center, bottom_c, bottom_b),
            (bottom_center, bottom_d, bottom_c),
            (bottom_center, bottom_a, bottom_d),
            # 侧面 4 个三角形
            (top_a, bottom_a, bottom_b),
            (top_a, bottom_b, top_b),
            (top_b, bottom_b, bottom_c),
            (top_b, bottom_c, top_c),
            (top_c, bottom_c, bottom_d),
            (top_c, bottom_d, top_d),
            (top_d, bottom_d, bottom_a),
            (top_d, bottom_a, top_a),
        ]

        loops = reconstruct_boundary_loops(triangles)

        # 回退到光滑区域边界：顶面和底面各 1 条 4 点环
        self.assertEqual(len(loops), 2)
        self.assertEqual(sorted(len(loop) for loop in loops), [4, 4])

    def test_reconstruct_boundary_loops_merges_topology_and_smooth_region_boundaries(self):
        """测试边界环重建合并拓扑边界和光滑区域边界。

        在八面体基础上增加一个完全分离的正方形（2 个三角形），
        拓扑边界应检测出分离正方形的环，光滑区域边界应检测出顶面和底面的环，
        最终合并为 3 条边界环。
        """
        # 八面体顶点（同上）
        top_center = (0.5, 0.5, 1.0)
        bottom_center = (0.5, 0.5, 0.0)
        top_a = (0.0, 0.0, 1.0)
        top_b = (1.0, 0.0, 1.0)
        top_c = (1.0, 1.0, 1.0)
        top_d = (0.0, 1.0, 1.0)
        bottom_a = (0.0, 0.0, 0.0)
        bottom_b = (1.0, 0.0, 0.0)
        bottom_c = (1.0, 1.0, 0.0)
        bottom_d = (0.0, 1.0, 0.0)
        # 分离正方形的顶点
        detached_a = (3.0, 0.0, 0.0)
        detached_b = (4.0, 0.0, 0.0)
        detached_c = (4.0, 1.0, 0.0)
        detached_d = (3.0, 1.0, 0.0)
        triangles = [
            (top_center, top_a, top_b),
            (top_center, top_b, top_c),
            (top_center, top_c, top_d),
            (top_center, top_d, top_a),
            (bottom_center, bottom_b, bottom_a),
            (bottom_center, bottom_c, bottom_b),
            (bottom_center, bottom_d, bottom_c),
            (bottom_center, bottom_a, bottom_d),
            (top_a, bottom_a, bottom_b),
            (top_a, bottom_b, top_b),
            (top_b, bottom_b, bottom_c),
            (top_b, bottom_c, top_c),
            (top_c, bottom_c, bottom_d),
            (top_c, bottom_d, top_d),
            (top_d, bottom_d, bottom_a),
            (top_d, bottom_a, top_a),
            # 分离的正方形
            (detached_a, detached_b, detached_c),
            (detached_a, detached_c, detached_d),
        ]

        loops = reconstruct_boundary_loops(triangles)

        # 3 条环：1 条拓扑边界（分离正方形）+ 2 条光滑区域边界（顶面和底面）
        self.assertEqual(len(loops), 3)
        self.assertEqual(sorted(len(loop) for loop in loops), [4, 4, 4])

    def test_reconstruct_boundary_loops_keeps_small_triangle_boundary(self):
        """测试单个三角形的边界环保留。

        只有一个三角形时，拓扑边界就是三角形的 3 条边，
        应重建出 1 条 3 点边界环。
        """
        triangles = [
            ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
        ]

        loops = reconstruct_boundary_loops(triangles)

        self.assertEqual(len(loops), 1)
        self.assertEqual(len(loops[0]), 3)
        self.assertEqual(
            set(loops[0]),
            {
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (0.0, 1.0, 0.0),
            },
        )

    def test_classify_panel_marks_coplanar_mesh_as_plane(self):
        """测试面板分类：共面三角网格应被分类为 'plane'。"""
        # 两个共面三角形（都在 y=0 平面上）
        triangles = [
            ((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 1.0, 1.0)),
            ((0.0, 0.0, 0.0), (0.0, 1.0, 1.0), (0.0, 0.0, 1.0)),
        ]
        boundary_loops = [[
            (0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 1.0, 1.0),
            (0.0, 0.0, 1.0),
        ]]

        category = classify_panel(triangles, boundary_loops)

        self.assertEqual(category, "plane")

    def test_classify_panel_marks_fb03c_166f_plate1_as_plane(self):
        project_root = Path(__file__).resolve().parents[1]
        blocks = parse_dxf_blocks(
            (project_root / "FB03C.dxf").read_text(
                encoding="utf-8",
                errors="ignore",
            ).splitlines()
        )
        mesh = blocks["FB03C-166F-PLATE1"]
        boundary_loops = reconstruct_boundary_loops(mesh.triangles)

        category = classify_panel(mesh.triangles, boundary_loops)

        self.assertEqual(category, "plane")

    def test_build_obj_text_deduplicates_vertices_and_writes_faces(self):
        """测试 OBJ 文本生成：顶点去重并正确写入面片索引。

        两个三角形共享顶点，OBJ 中应有 4 个唯一顶点和 2 个面片。
        """
        mesh = BlockMesh(
            name="TEST-BLOCK",
            triangles=[
                ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)),
                ((0.0, 0.0, 0.0), (1.0, 1.0, 0.0), (0.0, 1.0, 0.0)),
            ],
        )

        obj_text = build_obj_text(mesh)

        # 验证 OBJ 格式：对象名、4 个顶点、2 个面片
        self.assertEqual(obj_text.splitlines()[0], "o TEST-BLOCK")
        self.assertEqual(obj_text.count("\nv "), 4)
        self.assertIn("f 1 2 3", obj_text)
        self.assertIn("f 1 3 4", obj_text)

    def test_build_sampled_rxyz_curve_closes_loop_with_zero_r_segments(self):
        """测试 rxyz 曲线采样：3 个点应生成 5 个控制点的闭合曲线。

        采样规则：
        - header 点：r = 总点数 (5)
        - 每个几何点：r = 0（直线段）
        - 末尾自动闭合回起点
        """
        points = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (1.0, 1.0, 0.0),
        ]

        curve = build_sampled_rxyz_curve(points)

        self.assertEqual(
            curve,
            [
                [5.0, 0.0, 0.0, 0.0],   # header: r=5
                [0.0, 0.0, 0.0, 0.0],   # 第 1 个点
                [0.0, 1.0, 0.0, 0.0],   # 第 2 个点
                [0.0, 1.0, 1.0, 0.0],   # 第 3 个点
                [0.0, 0.0, 0.0, 0.0],   # 闭合回第 1 个点
            ],
        )

    def test_fit_plane_boundary_loop_compresses_clockwise_arc_into_positive_radius(self):
        """测试平面边界环拟合：顺时针圆弧应拟合为正半径。

        输入为 1/4 圆弧（从 (1,0) 到 (0,-1) 的 5 个点），
        拟合后应得到：
        - 拟合质量为 'excellent'
        - 第一段为圆弧，半径约 1.0
        - 正半径（顺时针方向）
        """
        loop = [
            (1.0, 0.0, 0.0),
            (0.92388, -0.382683, 0.0),
            (0.707107, -0.707107, 0.0),
            (0.382683, -0.92388, 0.0),
            (0.0, -1.0, 0.0),
            (0.0, 0.0, 0.0),
        ]

        fitted = fit_plane_boundary_loop(loop)

        # 验证拟合质量
        self.assertEqual(fitted["metrics"]["status"], "excellent")
        self.assertLess(fitted["metrics"]["max_error"], 1.0e-3)
        # 第一段应为圆弧，半径约 1.0
        self.assertEqual(fitted["fitted_segments"][0]["kind"], "arc")
        self.assertGreater(fitted["fitted_segments"][0]["signed_radius"], 0.0)
        self.assertAlmostEqual(fitted["fitted_segments"][0]["radius"], 1.0, places=3)
        self.assertAlmostEqual(fitted["fitted_points_rxyz"][1][0], 1.0, places=5)

    def test_fit_arc_segment_rejects_varying_curvature_points(self):
        """测试圆弧拟合拒绝曲率变化的点集。

        输入点集的曲率不均匀（不是同一段圆弧），
        拟合应返回 None。
        """
        points_uv = [
            (0.0, 0.0),
            (0.001352, 35.173633),
            (0.405205, 63.560787),
            (0.634178, 140.277653),
            (0.001292, 228.742529),
            (-0.634234, 246.395744),
            (-0.405278, 266.078648),
            (0.213253, 349.429079),
            (0.001069, 427.403444),
            (37.496868, 470.180137),
        ]

        self.assertIsNone(_fit_arc_segment(points_uv))

    def test_fit_arc_segment_rejects_sparse_large_sweep_arc_candidates(self):
        """测试圆弧拟合拒绝稀疏大角度圆弧候选。

        输入点数太少（4 个点）且分布在大角度弧段上，
        拟合应返回 None。
        """
        points_uv = [
            (137.548654, 928.117483),
            (130.898985, 941.319471),
            (-165.25056, 1054.537426),
            (-179.011405, 1049.138377),
        ]

        self.assertIsNone(_fit_arc_segment(points_uv))

    def test_generate_fit_ui_html_contains_overlay_and_quality_summary(self):
        """测试拟合 UI HTML 生成：验证关键 HTML 元素是否存在。

        构造一个包含平面面板拟合数据的输入，
        验证生成的 HTML 包含原始边界、拟合边界、效果判断等关键部分，
        以及 Canvas 绑定和圆弧标注相关代码。
        """
        html = generate_fit_ui_html(
            [
                {
                    "name": "PLANE-A",
                    "plane_basis": {
                        "origin": [0.0, 0.0, 0.0],
                        "u": [1.0, 0.0, 0.0],
                        "v": [0.0, 1.0, 0.0],
                        "normal": [0.0, 0.0, 1.0],
                    },
                    "loops": [
                        {
                            "loop_index": 0,
                            "sampled_points_uv": [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]],
                            "fitted_segments": [
                                {
                                    "kind": "line",
                                    "start_uv": [0.0, 0.0],
                                    "end_uv": [1.0, 0.0],
                                    "max_error": 0.0,
                                },
                                {
                                    "kind": "arc",
                                    "start_uv": [1.0, 0.0],
                                    "end_uv": [1.0, 1.0],
                                    "center_uv": [0.5, 0.5],
                                    "mid_uv": [1.2, 0.5],
                                    "radius": 0.5,
                                    "signed_radius": 0.5,
                                    "clockwise": True,
                                    "max_error": 0.0,
                                }
                            ],
                            "metrics": {
                                "source_point_count": 3,
                                "fitted_segment_count": 2,
                                "fitted_control_point_count": 2,
                                "compression_ratio": 0.333333,
                                "mean_error": 0.0,
                                "max_error": 0.0,
                                "tolerance": 0.001,
                                "status": "excellent",
                            },
                        }
                    ],
                }
            ]
        )

        # 验证 HTML 中包含所有关键元素
        self.assertIn("原始边界", html)
        self.assertIn("拟合边界", html)
        self.assertIn("效果判断", html)
        self.assertIn('id="fitCanvas"', html)
        self.assertIn("sampled_points_uv", html)
        self.assertIn("fitted_segments", html)
        self.assertIn("圆弧段编号", html)
        self.assertIn("drawArcCallouts", html)
        self.assertIn("callout-anchor", html)
        self.assertIn("callout-elbow", html)
        self.assertIn("seg.kind === 'arc'", html)

    def test_generate_demo_html_contains_panel_switch_and_rotation_controls(self):
        """测试演示 HTML 生成：验证面板选择器和旋转控制是否存在。

        构造一个平面面板的演示数据，验证生成的 HTML 包含：
        - 面板选择下拉框
        - Canvas 2D 查看器（不是 WebGL）
        - 旋转控制滑块
        - 边界模式切换
        - 缩放控制
        - 线框投影相关函数
        """
        html = generate_demo_html(
            [
                {
                    "name": "TEST-BLOCK",
                    "category": "plane",
                    "vertex_count": 4,
                    "triangle_count": 2,
                    "boundary_loop_count": 1,
                    "mesh_polyline_count": 0,
                    "bbox": {
                        "min_x": 0.0,
                        "max_x": 1.0,
                        "min_y": 0.0,
                        "max_y": 1.0,
                        "min_z": 0.0,
                        "max_z": 0.0,
                    },
                    "vertices": [
                        [0.0, 0.0, 0.0],
                        [1.0, 0.0, 0.0],
                        [1.0, 1.0, 0.0],
                        [0.0, 1.0, 0.0],
                    ],
                    "triangles": [[0, 1, 2], [0, 2, 3]],
                    "boundary_loops": [[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]]],
                    "mesh_polylines": [],
                }
            ]
        )

        # 验证面板选择器
        self.assertIn('id="panelSelect"', html)
        # 验证 Canvas 查看器
        self.assertIn('id="viewerCanvas"', html)
        # 验证鼠标交互事件
        self.assertIn("pointerdown", html)
        self.assertIn("wheel", html)
        self.assertIn("contextmenu", html)
        # 验证旋转控制
        self.assertIn("yawSlider", html)
        # 验证边界显示控制
        self.assertIn("showBoundary", html)
        self.assertIn('id="boundaryMode"', html)
        self.assertIn("原始边界", html)
        self.assertIn("可见边界", html)
        # 验证缩放控制
        self.assertIn('id="zoomIn"', html)
        self.assertIn('id="zoomOut"', html)
        self.assertIn('id="resetView"', html)
        self.assertIn('id="zoomValue"', html)
        # 验证不使用 WebGL（使用 Canvas 2D）
        self.assertNotIn('id="panelButtons"', html)
        self.assertIn("getContext('2d')", html)
        self.assertNotIn("getContext('webgl')", html)
        self.assertNotIn("DEPTH_TEST", html)
        # 验证线框投影相关函数
        self.assertIn("transformPositions", html)
        self.assertIn("pointInTriangle", html)
        self.assertIn("segmentVisibility", html)
        self.assertIn("drawVisibleBoundarySegment", html)
        # 验证边界模式状态
        self.assertIn("state.boundaryMode", html)
        self.assertIn("state.boundaryMode === 'raw'", html)
        self.assertIn("panel.category === 'plane'", html)
        # 验证状态栏
        self.assertIn('id="viewerStatus"', html)
        self.assertIn("Canvas 2D 线框投影", html)
        self.assertIn("getBoundingClientRect()", html)
        self.assertIn("min-height: 420px;", html)

    def test_export_dxf_models_writes_each_panel_into_its_own_directory(self):
        """测试端到端 DXF 导出：验证每个面板写入独立目录。

        构造一个包含 PANEL-A 块的 DXF 文件（含 2 个共面三角形和 2 个不共面三角形），
        导出到临时目录，验证：
        - 平面板和曲面板目录结构正确
        - PANEL-A 目录下生成所有必要文件（.obj, .boundary.xyz.json, .panel.xyz.json, .sampled.rxyz.json, .fitted.rxyz.json）
        - metadata.json 正确记录面板信息
        - 演示 HTML 文件生成
        - 拟合结果中质量为 'excellent'
        """
        # DXF 文本：PANEL-A 块，包含 2 个三角面
        dxf_text = """  0
SECTION
  2
BLOCKS
  0
BLOCK
  8
0
  2
PANEL-A
 70
64
 10
0.0
 20
0.0
 30
0.0
  3
PANEL-A
  0
3DFACE
 10
0.0
 20
0.0
 30
0.0
 11
1.0
 21
0.0
 31
0.0
 12
1.0
 22
1.0
 32
0.0
 13
0.0
 23
0.0
 33
0.0
  0
3DFACE
 10
0.0
 20
0.0
 30
1.0
 11
1.0
 21
0.0
 31
1.0
 12
1.0
 22
1.0
 32
1.0
 13
0.0
 23
0.0
 33
1.0
  0
ENDBLK
  0
ENDSEC
  0
SECTION
  2
ENTITIES
  0
INSERT
  2
PANEL-A
 10
0.0
 20
0.0
 30
0.0
  0
SEQEND
  0
ENDSEC
  0
EOF
"""
        with TemporaryDirectory() as tmp:
            root = Path(tmp)
            dxf_path = root / "sample.dxf"
            model_root = root / "model"
            demo_root = root / "demo"
            dxf_path.write_text(dxf_text, encoding="utf-8")

            # 执行导出
            metadata = export_dxf_models(dxf_path, model_root, demo_root)

            # 验证目录结构
            plane_root = model_root / "plane_panels"
            surface_root = model_root / "surface_panels"
            panel_dir = plane_root / "PANEL-A"
            self.assertTrue(plane_root.is_dir())
            self.assertTrue(surface_root.is_dir())
            self.assertTrue(panel_dir.is_dir())
            # 验证生成的文件
            self.assertTrue((panel_dir / "PANEL-A.obj").exists())
            self.assertTrue((panel_dir / "PANEL-A.boundary.xyz.json").exists())
            self.assertTrue((panel_dir / "PANEL-A.panel.xyz.json").exists())
            self.assertTrue((panel_dir / "PANEL-A.boundary.sampled.rxyz.json").exists())
            self.assertTrue((panel_dir / "PANEL-A.boundary.fitted.rxyz.json").exists())
            # 验证 metadata
            self.assertEqual(metadata["panels"][0]["directory"], panel_dir.as_posix())
            written = json.loads((model_root / "metadata.json").read_text(encoding="utf-8"))
            self.assertEqual(written["panels"][0]["directory"], panel_dir.as_posix())
            # 验证演示文件
            self.assertTrue((demo_root / "index.html").exists())
            self.assertTrue((root / "ui.html").exists())

            # 验证拟合结果
            fitted_payload = json.loads(
                (panel_dir / "PANEL-A.boundary.fitted.rxyz.json").read_text(encoding="utf-8")
            )
            self.assertEqual(fitted_payload["name"], "PANEL-A")
            self.assertEqual(fitted_payload["category"], "plane")
            self.assertEqual(fitted_payload["loops"][0]["metrics"]["status"], "excellent")


if __name__ == "__main__":
    unittest.main()
