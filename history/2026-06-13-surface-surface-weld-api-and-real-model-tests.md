# 2026-06-13 曲面板-曲面板焊缝接口与真实模型测试

## 背景

本轮工作在已有平面板-曲面板焊缝计算基础上，新增曲面板-曲面板焊缝检查接口。目标是保持现有两层接口风格：

- 构造层：原始三角网格通过 `api_get_surface_panel()` 转换为 `SurfacePanel`
- 焊缝层：`SurfacePanel + SurfacePanel` 计算焊缝

同时提供 raw mesh 便利重载，允许生产侧直接传入两组三角面片。

## 新增接口

新增对象接口：

```cpp
WeldCurveResult api_get_surface_surface_weld(
    const SurfacePanel& first_panel,
    const SurfacePanel& second_panel,
    double tolerance = 1.0);

WeldSplineResult api_get_surface_surface_weld_splines(
    const SurfacePanel& first_panel,
    const SurfacePanel& second_panel,
    double tolerance = 1.0,
    double fit_tolerance = 1.0e-3);
```

新增 raw mesh 便利接口：

```cpp
WeldCurveResult api_get_surface_surface_weld(
    const float first_vertices[][3],
    int first_vertex_count,
    const int first_triangles[][3],
    int first_triangle_count,
    const float first_normals[][3],
    const float second_vertices[][3],
    int second_vertex_count,
    const int second_triangles[][3],
    int second_triangle_count,
    const float second_normals[][3],
    double tolerance = 1.0);
```

样条版本同样支持 raw mesh 输入，并复用已有 `hanfeng_fit_weld_splines()`。

## 算法语义

v1 采用“边界贴靠优先”语义：

- 候选焊缝来自两块曲面板的 `outer_loops` 和 `inner_loops`
- first 边界裁剪到 second 曲面，second 边界裁剪到 first 曲面
- 命中区间包含拓扑共面、闭合曲面体内、容差内近距
- 最终复用已有端点合并、去重和 rxyz 后处理

本轮不计算任意三角面内部相交但不落在曲面边界上的交线。

## 真实模型测试宏

测试入口改为两个独立宏：

```text
HANFENG_PLANE_SURFACE_REAL_MODEL_TESTS
HANFENG_SURFACE_SURFACE_REAL_MODEL_TESTS
```

行为：

- 两个宏都为 `0`：只跑轻量单元/合成测试
- `HANFENG_PLANE_SURFACE_REAL_MODEL_TESTS=1`：只跑平面板-曲面板真实模型全量测试
- `HANFENG_SURFACE_SURFACE_REAL_MODEL_TESTS=1`：只跑曲面板-曲面板真实模型全量测试
- 两个宏都为 `1`：依次跑两个真实模型测试，不跑轻量测试

对应 CMake 选项已加入 `CMakeLists.txt`。

## 曲面-曲面真实模型导出

新增 `test_surface_surface_weld_with_real_models()`。

真实模型统计：

- 曲面板数量：85
- 曲面板不重复配对组合：3570
- 有焊缝导出组合：194

导出文件：

- `result/surface_surface_weld_results.json`
- `result/surface_surface_weld_results.js`
- `result/surface_surface_weld_viewer.html`

HTML viewer 展示曲面-曲面有焊缝组合列表、焊缝折线、参考点和切线方向摘要。

## 修改范围

- `CMakeLists.txt`
- `include/hanfeng/hanfeng.hpp`
- `include/hanfeng/plane_surface_hanfeng_util.hpp`
- `src/hanfeng.cpp`
- `src/plane_surface_hanfeng_util.cpp`
- `tests/plane_surface_weld_test.cpp`
- `tests/test_main.cpp`
- `spec/define.md`
- `README.md`

## 验证结果

已执行：

- 默认轻量模式：
  - `cmake -S . -B build -DHANFENG_PLANE_SURFACE_REAL_MODEL_TESTS=OFF -DHANFENG_SURFACE_SURFACE_REAL_MODEL_TESTS=OFF`
  - `cmake --build build --config Debug --target hanfeng`
  - `.\build\Debug\hanfeng.exe`
- 曲面-曲面真实模型模式：
  - `cmake -S . -B build -DHANFENG_PLANE_SURFACE_REAL_MODEL_TESTS=OFF -DHANFENG_SURFACE_SURFACE_REAL_MODEL_TESTS=ON`
  - `cmake --build build --config Debug --target hanfeng`
  - `.\build\Debug\hanfeng.exe`
- 平面-曲面真实模型模式：
  - `cmake -S . -B build -DHANFENG_PLANE_SURFACE_REAL_MODEL_TESTS=ON -DHANFENG_SURFACE_SURFACE_REAL_MODEL_TESTS=OFF`
  - `cmake --build build --config Debug --target hanfeng`
- 默认轻量模式恢复后：
  - `cmake --build build --config Release --target hanfeng`
  - `git diff --check`

结论：

- 默认模式不再跑 16405 组真实模型测试
- 曲面-曲面真实模型模式完成 3570 组配对并导出 194 组有焊缝结果
- 平面-曲面真实模型模式可编译
- Release 构建通过
- `git diff --check` 通过，仅有 LF/CRLF 提示
