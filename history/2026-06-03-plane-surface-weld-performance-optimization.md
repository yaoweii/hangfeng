# 2026-06-03 平面板-曲面板焊缝性能优化记录

## 背景

在前一轮 `plane -> surface` 焊缝求解语义重构完成后，结果已经正确，但真实模型导出耗时仍然偏高。

从 `result/weld_viewer.html` 和 `result/weld_results.json` 可以直接观察到：

- 存在焊缝的组合耗时明显偏长
- 即使最终没有焊缝输出，部分组合仍然非常慢
- 加载模型的时间很短，主要时间消耗发生在焊缝求解阶段

因此，本轮工作的目标不是再改语义，而是在不改变结果的前提下，系统性降低 `plane -> surface` 路径的计算成本。

## 第一阶段：定位真实瓶颈

### 1. 增加细分 profiling

为 `plane -> surface` 求解路径补充分阶段计时，细分出：

- `topo_ms`
- `inside_ms`
- `near_ms`
- `clip_plane_candidates_ms`
- `clip_surface_candidates_ms`

相关改动主要在：

- `include/hanfeng/hafeng_geometry.hpp`
- `src/hafeng_geometry.cpp`
- `tests/plane_surface_weld_test.cpp`

### 2. profiling 结论

初次 profiling 后，瓶颈非常明确：

- `clip_plane_candidates_ms` 占 `compute_weld` 的约 `86% ~ 92%`
- 其中 `near_ms` 单独就占 `compute_weld` 的约 `85% ~ 91%`
- `inside_ms` 约为 `1%`
- `topo_ms` 约为 `0.3%`

最慢组合 `FB03C-166F-PLATE1 x FB03C-PL250-CPLATE` 的典型数据为：

- `compute_weld = 8563.8 ms`
- `clip_plane_candidates = 7832.8 ms`
- `near = 7715.4 ms`
- `inside = 85.3 ms`
- `topo = 31.2 ms`

这说明第一刀必须先砍 `near`，而不是先动 `inside` 或 `topo`。

## 第二阶段：曲面侧 BVH 优化

### 1. 根因

当时的 `SurfaceGeometry` 只有平铺三角列表，没有空间加速结构。

这导致：

- 共面判定会反复全扫三角面
- inside/outside 判定会反复全扫三角面
- `near` 区间求解也会反复全扫三角面

实际成本近似表现为：

`平面候选线段数 × 曲面三角面数 × 多轮重复扫描`

例如：

- `FB03C-166F-PLATE1` 的平面候选段数约 `964`
- `FB03C-PL250-CPLATE` 的曲面三角面数约 `2996`

这意味着仅 `near` 阶段就会触发数百万级别的三角测试工作量。

### 2. 修改内容

在 `src/hafeng_geometry.cpp` 中为曲面三角网增加 BVH，并将以下路径切换为 BVH 查询：

- `point_in_surface_solid()`
- `segment_surface_solid_events()`
- `exact_segment_topo_coplanar_intervals()`
- `exact_segment_surface_intervals()`

### 3. 第一轮优化结果

真实模型整批导出耗时从约 `54.6s` 降到约 `6.2s`，接近 `8.8x` 提速。

典型组合变化：

- `PLATE1 x PL250`: `8563.8 ms -> 741.2 ms`
- `PLATE1 x PL251`: `7927.8 ms -> 701.0 ms`
- `PLATE3 x PL250`: `5919.4 ms -> 491.5 ms`
- `2D-PLATE1 x PL250`: `460.0 ms -> 43.8 ms`

### 4. 新瓶颈

第一轮优化后，新的主要耗时转移到了：

- `clip_surface_candidates_ms`

也就是“曲面板边界去裁剪平面板”这一侧。

## 第三阶段：平面侧 BVH 与粗筛优化

### 1. 修改方向

针对新的热点，继续在 `src/hafeng_geometry.cpp` 中完成第二轮优化：

- 为平面侧壁三角面增加 BVH
- 将 `exact_segment_plane_panel_intervals()` 的 side-triangle 遍历切换为 BVH 查询
- 为 `ProjectedLoop` 增加 UV 包围盒
- 在 `point_in_face_domain()` 和边交点收集前增加 bbox 粗筛
- 避免在 `exact_segment_face_intervals()` 中重复拷贝 `outer_loops + inner_loops`

### 2. 第二轮优化结果

真实模型整批导出耗时进一步从约 `6.2s` 降到约 `1.1s`。

相对最初版本，总提速接近 `50x`。

典型组合变化：

- `PLATE1 x PL250`: `741.2 ms -> 3.1 ms`
- `PLATE1 x PL251`: `701.0 ms -> 2.3 ms`
- `PLATE3 x PL250`: `491.5 ms -> 1.8 ms`

此时 `compute_weld` 本身已经非常轻，单组大多降到了 `0.3 ms ~ 3 ms` 量级。

## 最终结果

### 1. 存在焊缝的组合已进入 10ms 内

当前数据集内，存在焊缝的 3 组真实组合，`compute_weld` 分别为：

- `FB03C-166F-PLATE1 x FB03C-PL250-CPLATE`: `3.0855 ms`
- `FB03C-166F-PLATE3 x FB03C-PL251-CPLATE`: `2.4944 ms`
- `FB03C-2D-PLATE1 x FB03C-PL250-CPLATE`: `1.3586 ms`

如果看单组总耗时（包含加载），这 3 组也都在 `10 ms` 内：

- `7.8878 ms`
- `6.2805 ms`
- `4.7468 ms`

### 2. 当前剩余瓶颈

在双侧 BVH 完成后，主要时间已不再集中在焊缝核心求解，而更多落在：

- 模型加载
- 几何构建
- 组合间未复用的固定成本

这意味着本轮优化已经解决了主要算法级热点。

## 验证

本轮已执行并通过：

- `cmake --build build --config Release --target plane_surface_weld`
- `cmake --build build --config Debug --target hanfeng`
- `build\\Debug\\hanfeng.exe`
- `build\\Release\\plane_surface_weld.exe`

同时重新生成并核对了：

- `result/weld_results.json`
- `result/weld_results.js`
- `result/weld_viewer.html`

在结果层面，本轮优化的原则是：

- 不改变焊缝语义
- 不改变正确结果
- 只降低求解成本

用户已确认当前结果正确，且性能提升符合预期。

## 涉及文件

本轮核心涉及：

- `include/hanfeng/hafeng_geometry.hpp`
- `src/hafeng_geometry.cpp`
- `tests/plane_surface_weld_test.cpp`
- `result/weld_results.json`
- `result/weld_results.js`
- `result/weld_viewer.html`

## 结论

本轮工作完成了从“结果正确但很慢”到“结果正确且速度可用”的性能收敛。

核心方法不是继续堆启发式，而是：

1. 先用 profiling 定位真实热点
2. 再按热点路径分两轮引入空间加速结构
3. 最终把 `plane -> surface` 焊缝求解从多轮全表扫描，压缩到 BVH 驱动的局部查询

最终效果是：

- 整批真实模型导出从约 `54.6s` 降到约 `1.1s`
- 总体提速接近 `50x`
- 存在焊缝的真实组合 `compute_weld` 全部进入 `10ms` 内

到这里，`plane -> surface` 路径已经从“语义正确”进一步推进到“工程上可接受且可交付”的状态。
