# hanfeng

平面板与曲面板、曲面板与曲面板之间的焊缝折线计算库。输入平面板（rxy 编码）和曲面板（三角网格）的几何数据，输出两块板交界的焊缝位置及拟合样条。

## 项目结构

```
hanfeng/
├── include/hanfeng/          # 公共头文件
│   ├── geometry.hpp           # 三维几何基础类型（SPAposition / SPAvector / SPAbox）
│   ├── panel.hpp              # 面板数据结构（PlanePanel / SurfacePanel）与模型读取接口
│   ├── weld.hpp               # 焊缝结果数据结构
│   ├── plane_surface_hanfeng_util.hpp  # 焊缝计算核心
│   └── hanfeng.hpp            # 对外 API 入口
├── src/                       # 实现源文件
├── tests/                     # 测试
├── demo/                      # 可视化演示（HTML + Python）
├── spec/                      # 数据规范定义
├── dxf_exporter.py            # DXF 文件解析与导出工具
└── CMakeLists.txt
```

## 核心模块

### geometry — 三维几何基础

沿用 ACIS/GME 命名风格的轻量几何类型：

| 类型 | 说明 |
|------|------|
| `SPAposition` | 三维空间位置点 |
| `SPAvector` | 三维向量 |
| `SPAunit_vector` | 单位向量（构造时自动归一化） |
| `SPAbox` | 轴对齐包围盒（AABB） |

运算符：`%` 点乘、`*` 叉乘/数乘、`|` 包围盒并集、`&` 交集、`&&` 相交判断。

### panel — 面板数据模型

- **PlanePanel**：带厚度的平面板，包含 face_a / face_b 两侧边界
- **SurfacePanel**：曲面板，由多个 `SurfacePatch`（三角网格 + 边界环）组成
- 曲线编码：`rxy`（二维）/ `rxyz`（三维），`r == 0` 为直线，`r != 0` 为圆弧，`|r| ≈ 2` 为截断标记

### weld — 焊缝计算

- `WeldCurveResult`：焊缝折线结果（多条折线）
- `WeldSplineResult`：焊缝样条拟合结果（含参考点与切线方向）
- 平面板-曲面板焊缝：候选来自平面板和曲面板边界
- 曲面板-曲面板焊缝：v1 采用“边界贴靠”语义，候选来自两块曲面板的边界环

## 对外 API

```cpp
#include <hanfeng/hanfeng.hpp>

// 从模型文件构造面板
hanfeng::PlanePanel plane = hanfeng::api_get_plane_panel("path/to/plane_model");
hanfeng::SurfacePanel surface = hanfeng::api_get_surface_panel("path/to/surface_model");

// 计算焊缝折线
hanfeng::WeldCurveResult welds = hanfeng::api_get_plane_surface_weld(plane, surface, /*tolerance=*/1.0);

// 拟合焊缝样条
hanfeng::WeldSplineResult splines = hanfeng::api_get_plane_surface_weld_splines(plane, surface);

// 曲面板-曲面板焊缝
hanfeng::SurfacePanel surface_a = hanfeng::api_get_surface_panel("path/to/surface_a_model");
hanfeng::SurfacePanel surface_b = hanfeng::api_get_surface_panel("path/to/surface_b_model");

hanfeng::WeldCurveResult surface_welds =
    hanfeng::api_get_surface_surface_weld(surface_a, surface_b, /*tolerance=*/1.0);

hanfeng::WeldSplineResult surface_splines =
    hanfeng::api_get_surface_surface_weld_splines(surface_a, surface_b);
```

也可直接从原始数组构造面板：

```cpp
// 从 rxy 数组构造平面板
hanfeng::PlanePanel plane = hanfeng::api_get_plane_panel(rxyPltBndry1, rxyholes, cAxis, thickdis1, thickdis2);

// 从三角网格数组构造曲面板
hanfeng::SurfacePanel surface = hanfeng::api_get_surface_panel(vertices, vertex_count, triangles, triangle_count, normals);
```

曲面板-曲面板焊缝也提供 raw mesh 便利重载，可直接传入两组三角网格；接口内部会先调用 `api_get_surface_panel(...)` 构造 `SurfacePanel`。

## 构建

依赖 CMake 3.21+、C++17 编译器。

```bash
cmake -B build -S .
cmake --build build --config Release
```

构建产物为可执行文件 `hanfeng`（含内置测试）。

## 运行测试

```bash
./build/hanfeng              # 或 build/Release/hanfeng.exe (MSVC)
```

默认构建下，测试覆盖基础几何运算、数据结构布局、模型 I/O、平面-曲面焊缝计算、曲面-曲面焊缝计算；默认不跑真实模型全量导出。

真实模型测试由两个 CMake 选项控制：

```bash
# 只跑平面板-曲面板真实模型全量测试
cmake -S . -B build -DHANFENG_PLANE_SURFACE_REAL_MODEL_TESTS=ON -DHANFENG_SURFACE_SURFACE_REAL_MODEL_TESTS=OFF
cmake --build build --config Release

# 只跑曲面板-曲面板真实模型全量测试
cmake -S . -B build -DHANFENG_PLANE_SURFACE_REAL_MODEL_TESTS=OFF -DHANFENG_SURFACE_SURFACE_REAL_MODEL_TESTS=ON
cmake --build build --config Release
```

两个选项都为 `OFF` 时运行轻量测试；任一选项为 `ON` 时只运行对应真实模型测试。

## 可视化

真实模型测试会在 `result/` 下生成可视化和数据文件：

- 平面板-曲面板：`weld_viewer.html`、`weld_results.json`、`weld_results.js`
- 曲面板-曲面板：`surface_surface_weld_viewer.html`、`surface_surface_weld_results.json`、`surface_surface_weld_results.js`

`demo/` 目录也提供独立 HTML 可视化页面，可直接在浏览器中打开查看板件与焊缝结果。

`dxf_exporter.py` 可从 DXF 文件解析板件几何数据并导出为项目使用的模型格式。

## 规范

数据结构、字段含义与输入输出格式定义见 `spec/define.md`。开发时以 spec 为准。
