# 2026-06-13 焊缝折线转 rxyz 样条后处理

## 目标

在平面板与曲面板焊缝检测完成之后，将最终得到的每条焊缝折线重新拟合为 `rxyz` 曲线表示。

本次实现明确保持两阶段流程：

- 第一阶段：沿用现有焊缝检测逻辑，输出 `WeldCurveResult.polylines`
- 第二阶段：对检测完成并去重后的焊缝折线做后处理，输出 `WeldSplineResult`

这样避免把求交检测算法和结果表达转换耦合在一起。

## 新增接口

新增底层后处理函数：

```cpp
WeldSplineResult hanfeng_fit_weld_splines(
    const WeldCurveResult& weld_result,
    double tolerance = 1.0e-3);
```

新增对外一站式接口：

```cpp
WeldSplineResult api_get_plane_surface_weld_splines(
    const PlanePanel& plane_panel,
    const SurfacePanel& surface_panel,
    double tolerance = 1.0,
    double fit_tolerance = 1.0e-3);
```

其中 `api_get_plane_surface_weld_splines()` 内部先调用现有 `api_get_plane_surface_weld()` 完成焊缝检测，再调用 `hanfeng_fit_weld_splines()` 做 `xyz -> rxyz` 后处理。

## 新增数据结构

在 `include/hanfeng/weld.hpp` 中新增：

```cpp
struct WeldSpline {
    Polyline3 raw_points;
    RxyzCurve points_rxyz;
    SPAposition reference_point;
    SPAunit_vector tangent_direction;
};

struct WeldSplineResult {
    std::vector<WeldSpline> welds;
};
```

每条焊缝只输出一组 `reference_point` 和 `tangent_direction`。

当前规则：

- `reference_point`：取该条焊缝的第一个有效几何点
- `tangent_direction`：取该点处的焊缝切线方向并单位化
- 直线首段：切线为 `p1 - p0`
- 圆弧首段：切线为圆弧在起点处的方向

## rxyz 拟合规则

输入是最终焊缝折线 `Polyline3`。

后处理会按顺序贪心选择下一段：

1. 优先尝试将连续点合并为圆弧段
2. 如果圆弧不成立，则尝试合并为直线段
3. 无法长段合并时退化为单条直线段

圆弧拟合约束：

- 点集必须近似共面
- 点集必须近似共圆
- 角度方向必须单调
- 单段相邻采样角度不得超过约 `12°`
- 这个阈值用于兼容当前模型边界按约 `10°` 离散的来源
- 只接受不超过半圆的圆弧段，避免用单个 `r` 半径表达时产生歧义

输出 `rxyz` 语义沿用项目已有定义：

- header 点的 `r` 为总点数
- 几何点 `r == 0` 表示到下一点为直线段
- 几何点 `r != 0 && abs(r) != 2` 表示到下一点为圆弧段，`r` 为带方向的半径

## 导出结果

真实模型焊缝导出仍保留原有 `points` 折线字段，同时在每条焊缝结果中追加：

```json
{
  "points_rxyz": [[...]],
  "reference_point": [x, y, z],
  "tangent_direction": [dx, dy, dz]
}
```

顶层 JSON 额外记录：

```json
"weld_fit_tolerance": 0.001
```

输出文件：

- `result/weld_results.json`
- `result/weld_results.js`
- `result/weld_viewer.html`

## 测试覆盖

新增测试：

```cpp
test_hanfeng_fit_weld_splines_converts_line_and_arc_polylines()
```

覆盖内容：

- 直线焊缝折线压缩为两个几何控制点的 `rxyz`
- 按 `10°` 采样的 90° 圆弧折线恢复为一段圆弧 `rxyz`
- 每条焊缝生成 `reference_point`
- 每条焊缝生成单位化 `tangent_direction`

该测试已加入默认焊缝测试入口，在真实模型导出前执行。

## 修改文件

- `include/hanfeng/weld.hpp`
- `include/hanfeng/plane_surface_hanfeng_util.hpp`
- `include/hanfeng/hanfeng.hpp`
- `src/plane_surface_hanfeng_util.cpp`
- `src/hanfeng.cpp`
- `tests/plane_surface_weld_test.cpp`
- `tests/test_main.cpp`

## 验证结果

已执行：

- `cmake --build build --config Debug --target hanfeng`
- `.\build\Debug\hanfeng.exe`
- `result/weld_results.json` 解析抽查
- `cmake --build build --config Release --target hanfeng`

验证结论：

- Debug 构建通过
- 新增样条后处理测试通过
- 15 组真实模型组合导出完成
- 存在焊缝的 3 个组合均包含 `points_rxyz`、`reference_point`、`tangent_direction`
- Release 构建通过

## 注意事项

- `FB03C.dxf` 在本次任务前后仍显示为 modified，但本次工作未修改该文件。
- 当前 `rxyz` 拟合是结果后处理，不改变焊缝检测、裁剪和去重逻辑。
