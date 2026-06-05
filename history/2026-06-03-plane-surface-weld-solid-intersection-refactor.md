# 2026-06-03 平面板-曲面板焊缝实体语义重构记录

## 背景

本次调整针对 `plane -> surface` 焊缝求解路径。

原实现的核心判断方式是：

- 先求线段到曲面三角网的 `distance <= tolerance` 区间
- 再用 `outward normal`、最近点方向、回穿平面板等启发式做过滤

这套逻辑对“线段靠近曲面板外表面”的情况有效，但不符合当前业务语义：

- 线段如果已经进入曲面板实体内部，也应该算焊缝
- 线段如果与曲面板表面存在拓扑级别共面重叠，也应该直接算焊缝
- 不能再只靠最近距离与法向关系来推断命中

用户进一步明确了一个约束：

- 当前求交使用的是拓扑层面的求交，不是几何层面的“无限延拓求交”
- 有交的情况，当前交点一定同时在线段上、也在板上

因此，本次重构改为严格按实体语义求：

```text
I_weld = I_topo_coplanar ∪ I_inside ∪ I_near
```

## 根因

旧实现的问题集中在 `src/hafeng_geometry.cpp` 的
`exact_segment_surface_intervals_from_plane_candidate()`。

旧路径本质上是：

1. 先求整段 `distance <= tolerance`
2. 对每个命中区间中点，找曲面最近点
3. 用 `delta · outward_normal` 判断“是否朝外命中”
4. 再用一段回穿检测排除“从平面板内部碰到曲面”的情况

这会导致两类错误：

- 真正位于曲面板实体内部的线段被排除
- 拓扑共面重叠段必须绕回距离启发式才能命中，语义不稳定

## 修改内容

### 1. 重写 plane -> surface 单段命中区间求解器

文件：`src/hafeng_geometry.cpp`

保留原有 weld polyline 拼接框架，只替换“单条候选线段如何求命中区间”。

新逻辑为：

1. 先求 `I_topo_coplanar`
2. 对非共面剩余子区间求 `I_inside`
3. 用补集求 `I_outside`
4. 仅对 `I_outside` 子区间求 `I_near`
5. 合并为最终 `I_weld`

### 2. 新增实体内外判定与穿透事件分类

文件：`src/hafeng_geometry.cpp`

新增内部工具：

- `SolidPointClassification`
- `SolidIntersectionEvent`
- `point_in_surface_solid()`
- `segment_surface_solid_events()`
- `exact_segment_inside_intervals_on_subsegment()`

实现方式：

- 用闭合三角网对点做 inside / outside / on-boundary 判定
- 对线段与三角面求实际穿透参数
- 对交点前后做微小采样，区分 `Crossing` 与 `Touching`
- 仅 `Crossing` 事件翻转 inside / outside 状态

### 3. 新增拓扑共面区间求解

文件：`src/hafeng_geometry.cpp`

新增：

- `exact_segment_triangle_coplanar_intervals()`
- `exact_segment_topo_coplanar_intervals()`

处理原则：

- 只接受真实线段与三角面域的正长度重叠
- 不接受仅仅“落在同一无限平面”的几何共面
- 共面区间直接记入焊缝，不再经过 inside/outside 或法向过滤

### 4. 删除旧的 outward-normal 启发式过滤作用

文件：`src/hafeng_geometry.cpp`

旧实现中的以下思路已退出主判定逻辑：

- 最近点法向方向过滤
- `delta · outward_normal` 硬判定
- 回穿平面板的二次排除

`exact_segment_surface_intervals_from_plane_candidate()` 现在按实体区间语义直接生成结果。

### 5. 补充并重构合成测试夹具

文件：

- `tests/plane_surface_weld_test.cpp`
- `tests/test_main.cpp`

新增闭合盒体曲面板夹具，用来明确表达实体语义，而不是只用一张开口矩形面片。

新增/调整的测试重点覆盖：

- 平面板边界与曲面板表面拓扑共面时应命中
- 平面板边界整段位于曲面板实体内部时应命中
- 平面板边界整段位于实体外部但在容差带内时应命中
- 原有 `surface -> plane` 路径的基础回归仍保留
- 精确 band 回归继续保留

同时，测试中的 polyline 匹配也做了调整：

- 不再要求命中段必须刚好成为整条 polyline 的首尾
- 改为允许它作为 polyline 中的一个连续子段出现

这样更贴合当前 polyline 拼接后的实际输出形态。

## 修改后的行为

当前 `plane -> surface` 路径的业务语义变为：

- 线段与曲面板表面存在拓扑级别共面重叠：算焊缝
- 线段位于曲面板闭合实体内部：算焊缝
- 线段位于曲面板实体外部，但到曲面板边界距离不超过容差：算焊缝
- 单点相切且无正长度区间：不生成焊缝段

## 验证结果

本轮已执行并通过：

- `cmake --build build --config Debug --target hanfeng`
- `build\\Debug\\hanfeng.exe`
- `cmake --build build --config Release --target plane_surface_weld`
- `build\\Release\\plane_surface_weld.exe`

验证到两类结果：

### 1. Debug 单元测试通过

说明：

- 新增的实体语义合成测试通过
- 旧有基础回归未被破坏

### 2. 真实模型导出流程跑通

重新生成了：

- `result/weld_results.json`
- `result/weld_results.js`
- `result/weld_viewer.html`

本轮真实模型导出统计为：

- 平面板数量：`3`
- 曲面板数量：`5`
- 总组合数：`15`

其中部分组合的焊缝段数发生变化，属于本次实体语义重构的直接结果：

- `FB03C-166F-PLATE1 x FB03C-PL250-CPLATE`：`3 -> 2`
- `FB03C-166F-PLATE3 x FB03C-PL251-CPLATE`：`3 -> 2`
- `FB03C-2D-PLATE1 x FB03C-PL250-CPLATE`：`1 -> 2`

用户已查看结果并确认“现在结果正确”。

## 涉及文件

本次核心涉及：

- `src/hafeng_geometry.cpp`
- `tests/plane_surface_weld_test.cpp`
- `tests/test_main.cpp`
- `result/weld_results.json`
- `result/weld_results.js`
- `result/weld_viewer.html`

## 结论

这次重构把 `plane -> surface` 焊缝判定从“距离 + 法向启发式过滤”切换为“拓扑共面 + 实体 inside/outside + outside near”的实体语义实现。

结果上：

- 支持“内部也算焊缝”
- 支持“拓扑共面直接命中”
- 不再依赖 `delta · n` 这类不稳定启发式作为主判定依据
- 与当前业务语义一致

当前结果已经过合成测试、真实模型导出验证，并已由用户确认结果正确。
