# History Index

## 2026-06-13 曲面板-曲面板焊缝接口与真实模型测试
**文件**: [2026-06-13-surface-surface-weld-api-and-real-model-tests.md](2026-06-13-surface-surface-weld-api-and-real-model-tests.md)

新增曲面板-曲面板焊缝检查接口，包含 `SurfacePanel + SurfacePanel` 核心接口和 raw mesh 便利重载；复用现有曲面边界、BVH、候选裁剪、端点合并、去重和 `rxyz` 后处理。测试入口拆分为 `HANFENG_PLANE_SURFACE_REAL_MODEL_TESTS` 与 `HANFENG_SURFACE_SURFACE_REAL_MODEL_TESTS` 两个真实模型宏，默认只跑轻量测试。曲面-曲面真实模型测试完成 85 个曲面板的 3570 组不重复配对，导出 194 组有焊缝结果，并生成 JSON/JS/HTML。

**关键词**: 曲面板-曲面板焊缝, SurfacePanel, raw mesh, 真实模型宏, HTML可视化, rxyz, 边界贴靠

---

## 2026-06-13 FB03C 焊缝导出与平面板分组修复
**文件**: [2026-06-13-fb03c-weld-export-plane-grouping-fix.md](2026-06-13-fb03c-weld-export-plane-grouping-fix.md)

基于新的 `FB03C.dxf` 重新生成 `model/`，将真实模型焊缝 HTML/JSON 导出改为只输出存在焊缝的组合；修复焊缝 polyline 端点在容差内相接但未合并的问题；进一步定位并修复 7 个轴对齐平面板因拟合法向微小偏斜导致 `same_plane_group()` 误分成 4 组的问题。修复后可测试平面板从 186 个恢复为 193 个，可测试组合从 15810 增加到 16405，有焊缝导出组合从 270 增加到 287。
**关键词**: FB03C, 焊缝导出, HTML过滤, 端点合并, same_plane_group, 平面板分组, 轴对齐平面, model更新

---

## 2026-06-13 焊缝折线转 rxyz 样条后处理

**文件**: [2026-06-13-weld-rxyz-spline-postprocess.md](2026-06-13-weld-rxyz-spline-postprocess.md)

在焊缝检测和去重完成后，新增 `hanfeng_fit_weld_splines()` 将每条焊缝折线后处理为 `rxyz` 样条，并为每条焊缝输出一组 `reference_point` 和 `tangent_direction`。真实模型导出结果同步追加 `points_rxyz`、`reference_point`、`tangent_direction` 字段。

**关键词**: 焊缝后处理, rxyz, 样条拟合, 圆弧恢复, 切线方向, WeldSpline

---

所有历史工作文档索引，按日期倒序排列。查找历史工作时先看本文件。

---

## 2026-06-06 生产数据到平面/曲面板构造函数

**文件**: [2026-06-06-production-data-plane-surface-panel-constructor.md](2026-06-06-production-data-plane-surface-panel-constructor.md)

新增 `api_get_plane_panel()` 和 `api_get_surface_panel()` 两个面向生产输入的构造接口，不再依赖 `model` 目录中间文件，可直接从生产侧原始几何数据构造 `PlanePanel` / `SurfacePanel`。

**关键词**: 平面板, 曲面板, 构造函数, 生产数据, rxy, mesh, 边界提取

---

## 2026-06-05 焊缝测试与可视化调试

**文件**: [2026-06-05-weld-test-visualization-debug.md](2026-06-05-weld-test-visualization-debug.md)

12 组焊缝测试 + HTML 可视化完成，包含 `same_plane_group` 法向量分组 bug 修复、file:// 协议问题修复、平面板 mesh 显示修复。记录了 FB03C-166F-PLATE3 分段疑问。

**关键词**: 焊缝测试, HTML可视化, bug修复, same_plane_group, 分段, Canvas 2D

---

## 2026-06-03 平面板-曲面板焊缝性能优化

**文件**: [2026-06-03-plane-surface-weld-performance-optimization.md](2026-06-03-plane-surface-weld-performance-optimization.md)

通过 profiling 定位 `near` 阶段为瓶颈（占 86%-92%），分两轮引入曲面侧 BVH 和平面侧 BVH，整批真实模型导出从 ~54.6s 降到 ~1.1s，总提速 ~50x。

**关键词**: 性能优化, BVH, profiling, near区间, 焊缝提速

---

## 2026-06-03 平面板-曲面板焊缝实体语义重构

**文件**: [2026-06-03-plane-surface-weld-solid-intersection-refactor.md](2026-06-03-plane-surface-weld-solid-intersection-refactor.md)

将焊缝判定从"距离+法向启发式"切换为"I_topo_coplanar ∪ I_inside ∪ I_near"实体语义。支持"内部也算焊缝"和"拓扑共面直接命中"，不再依赖 delta·n 判断。

**关键词**: 实体语义重构, inside/outside, 拓扑共面, near区间, 焊缝判定

---

## 2026-06-02 平面边界线与曲面板焊缝判定设计

**文件**: [2026-06-02-plane-surface-weld-solid-intersection-design.md](2026-06-02-plane-surface-weld-solid-intersection-design.md)

焊缝实体语义的设计文档，定义了 I_topo_coplanar / I_inside / I_near 三类区间、总体流程、数据结构、数值鲁棒性要求和测试方案。

**关键词**: 设计文档, 焊缝判定, 实体语义, 区间求解, 数据结构

---

## 2026-05-27 FB03C-166F-PLATE1 平面板分类修正

**文件**: [2026-05-27-fb03c-166f-plate1-plane-panel-fix.md](2026-05-27-fb03c-166f-plate1-plane-panel-fix.md)

修正 `dxf_exporter.py` 的 `classify_panel()` 分类逻辑，解决"带厚度、带多个孔的平面板"被误判为曲面板的问题。修改后平面板 3 个、曲面板 5 个、组合数 3x5=15。

**关键词**: 板件分类, PLATE1, 平面板修正, classify_panel, dxf_exporter

---

## 2026-05-12 RXYZ 曲面板生成器设计

**文件**: [2026-05-12-rxyz-surface-panel-design.md](2026-05-12-rxyz-surface-panel-design.md)

将平面 rxyz 控制点数组转换为闭合三角管状曲面 mesh 的设计文档，包含输入语义、几何解释、弧线构造、中心线采样、mesh 生成和输出文件定义。

**关键词**: RXYZ, 曲面板, 设计文档, 管状mesh, 弧线, OBJ, demo

---

## 2026-05-12 RXYZ 曲面板生成器实现计划

**文件**: [2026-05-12-rxyz-surface-panel-plan.md](2026-05-12-rxyz-surface-panel-plan.md)

RXYZ 曲面板生成器的 TDD 实现计划，9 个任务从脚手架到端到端验证，使用 Node.js 标准库，无外部依赖。

**关键词**: 实现计划, TDD, Node.js, 曲面板生成器, 测试
