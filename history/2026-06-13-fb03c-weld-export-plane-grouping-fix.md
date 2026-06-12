# 2026-06-13 FB03C 焊缝导出与平面板分组修复

## 背景

本轮工作基于新的 `FB03C.dxf` 测试数据。该 DXF 改动很大，因此先重新生成 `model/` 中的平面板和曲面板数据，然后运行真实模型的平面板-曲面板焊缝测试。

新的模型统计：

- 总板件数：278
- 平面板：193
- 曲面板：85
- 理论组合数：193 x 85 = 16405

`model/` 目录是生成产物，当前在 `.gitignore` 中，不会出现在 `git status` 里。

## 已完成工作

### 1. 更新模型数据

使用新的 `FB03C.dxf` 重新导出了 `model/`。

确认旧测试中关心的板件仍然存在：

- 平面板：
  - `FB03C-166F-PLATE1`
  - `FB03C-166F-PLATE3`
  - `FB03C-2D-PLATE1`
- 曲面板：
  - `FB03C-PL13-CPLATE`
  - `FB03C-PL250-CPLATE`
  - `FB03C-PL251-CPLATE`
  - `FB03C-PL267-CPLATE`
  - `FB03C-PL268-CPLATE`

### 2. 真实模型焊缝结果只导出有焊缝的组合

修改了真实模型测试导出逻辑：

- 仍然测试所有可解析的平面板 x 曲面板组合。
- HTML/JSON/JS 只导出 `weld_result.polyline_count > 0` 的组合。
- 顶层 JSON 新增统计字段：
  - `combination_count`
  - `tested_combination_count`
  - `exported_combination_count`
  - `export_filter`

输出文件：

- `result/weld_results.json`
- `result/weld_results.js`
- `result/weld_viewer.html`

### 3. 修复焊缝端点相接但未合并的问题

问题：如果两条焊缝 polyline 的端点在容差范围内相同，之前没有合并，导致一条连续焊缝被拆成多段。

修复位置：

- `src/plane_surface_hanfeng_util.cpp`

新增逻辑：

- `hanfeng_points_touch()`
- `hanfeng_oriented_polyline()`
- `hanfeng_try_merge_touching_polylines()`
- `hanfeng_merge_touching_polylines()`

合并支持四种端点连接关系：

- target.back -> candidate.front
- target.back -> candidate.back
- candidate.back -> target.front
- candidate.front -> target.front

合并时会自动处理 candidate 方向，并跳过重复端点。

新增回归测试：

- `test_compute_plane_surface_weld_merges_touching_weld_polylines()`

测试构造两个相邻曲面 patch，它们分别与同一平面产生两段端点相接的焊缝。修复前输出 2 条 polyline，修复后输出 1 条连续 polyline。

### 4. 定位并修复 7 个平面板不能解析的问题

初始全量运行时有 7 个平面板无法通过 C++ 的 `api_get_plane_panel()`：

- `FB03C-0LB-PLATE1`
- `FB03C-162F-PLATE1`
- `FB03C-162FA-CLIP2`
- `FB03C-162FA-CLIP5`
- `FB03C-162FA-PLATE2`
- `FB03C-163F-PLATE1`
- `FB03C-164FE-PLATE1`

报错：

```text
平面板解析失败：当前实现要求边界环可稳定分成两个主面平面。
```

最初诊断时，这些板被 C++ 分成了 4 个平面组。但进一步检查最后一个例子 `FB03C-164FE-PLATE1` 后发现，正确分组应该是：

- g0：绿色 + 蓝色
- g1：黄色 + 红色

也就是这些边界环实际应按主面坐标聚成 2 组，而不是 4 组。

根因：

`same_plane_group()` 原来比较的是：

```cpp
loop.plane_offset * sign == group.plane_offset
```

但 `loop.plane_offset` 是在 loop 自己拟合出的法向下计算的，`group.plane_offset` 是在 group 法向下计算的。对于坐标值很大的轴对齐板件，只要 loop 拟合法向有极小倾斜，offset 就会被放大成明显差值，导致同一主面上的 loop 被拆成不同 group。

典型例子：

- `FB03C-164FE-PLATE1`
  - loop 1 和 loop 2 都在 `x ~= 287869`
  - loop 0 和 loop 3 都在 `x ~= 287855`
  - 但原逻辑因 offset 差约 `0.0022 > 1e-3` 拆成 4 组

修复方式：

- 保留原来的严格平面 offset 分组。
- 若严格 offset 比较失败，并且两个法向都近似轴对齐且主轴一致，则使用 loop/group 质心在主轴坐标上的距离进行兜底判断。
- 兜底主轴坐标容差为 `1.0e-2`。

修复位置：

- `src/panel.cpp`

新增字段：

- `PlaneLoopInfo::centroid`
- `LoopGroup::centroid`

新增辅助函数：

- `dominant_axis_index()`
- `coordinate_at_axis()`
- `normal_is_axis_aligned()`

新增回归测试：

- `test_api_get_plane_panel_groups_axis_aligned_noisy_model_loops()`

测试覆盖上述 7 个板，并断言：

- 每个板可以解析为 2 个主面。
- `FB03C-0LB-PLATE1` 每面 4 个 inner loop。
- 其他 6 个板每面 1 个 inner loop。

## 当前验证结果

构建：

```text
cmake --build build --config Debug --target hanfeng
```

结果：通过。

默认测试程序：

```text
.\build\Debug\hanfeng.exe
```

结果：通过。

修复前：

- 可测试平面板数量：186
- 可测试曲面板数量：85
- 可测试组合数：15810
- 导出有焊缝组合：270
- 7 个平面板解析失败

修复后：

- 可测试平面板数量：193
- 可测试曲面板数量：85
- 可测试组合数：16405
- 导出有焊缝组合：287
- 7 个平面板均可解析
- 不再打印 `ERROR loading plane`

JSON 导出过滤检查：

- `actual_exported == exported_combination_count`
- `zero_weld_exports == 0`

## 生成的可视化文件

真实焊缝结果：

- `result/weld_viewer.html`

七个平面板的诊断 demo：

- `result/failed_plane_panels_demo.html`

注意：`failed_plane_panels_demo.html` 是在修复分组前生成的诊断图，展示的是旧逻辑下的 4 组分裂现象。修复后如果还需要对比图，应重新生成一个“修复后 2 组分组”的版本。

## 当前代码改动范围

主要相关文件：

- `src/panel.cpp`
- `src/plane_surface_hanfeng_util.cpp`
- `tests/model_io_test.cpp`
- `tests/plane_surface_weld_test.cpp`
- `tests/test_main.cpp`

另外，本轮之前和过程中还存在与焊缝 rxyz 样条后处理相关的改动：

- `include/hanfeng/hanfeng.hpp`
- `include/hanfeng/plane_surface_hanfeng_util.hpp`
- `include/hanfeng/weld.hpp`
- `src/hanfeng.cpp`

## 后续注意点

1. `same_plane_group()` 的兜底逻辑目前只针对近似轴对齐主面。这样改动范围较小，避免影响一般斜板的几何分组。
2. 如果后续出现非轴对齐但同类“拟合法向略偏导致 offset 放大”的板件，需要设计更通用的平面距离判断，而不是继续扩大 axis-aligned 兜底。
3. 修复后测试组合数增加到 16405，HTML 导出组合增加到 287，属于预期结果，因为之前失败的 7 个平面板现在参与了真实焊缝测试。
4. `result/` 和 `model/` 都是生成产物，后续如果要交付可视化结果，需要确认是否需要单独保存或纳入版本控制。
