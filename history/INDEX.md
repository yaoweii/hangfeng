# History Index

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
