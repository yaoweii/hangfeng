---
name: debug-weld-results
description: 当前焊缝测试与可视化的工作进展，包含 bug 修复记录和待排查问题
type: project
originSessionId: 5d07616f-f44a-41d0-b96d-8aa17d6a3083
---
## 当前工作：焊缝测试 + HTML 可视化

### 已完成
- `tests/plane_surface_weld_test.cpp`: 新增 `test_weld_with_real_models_and_export()` 函数
  - 读取 `model/metadata.json`，遍历 2×6=12 组平面板×曲面板组合
  - 带 `<chrono>` 计时调用 `api_get_plane` / `api_get_surface` / `api_get_plane_surface_weld`
  - 输出 `result/weld_results.json`、`result/weld_results.js`、`result/weld_viewer.html`
  - HTML 数据完全内联（兼容 file:// 协议），Canvas 2D 投影渲染
  - 平面板绿色线框 + 曲面板蓝色线框 + 边界环红色 + 焊缝鲜艳色标注段号
- `tests/test_main.cpp`: `PLANE_SURFACE_TEST` 宏条件编译
- `CMakeLists.txt`: 新增 `plane_surface_weld` 构建目标
- 12 组焊缝全部成功运行

### Bug 修复记录

1. **`same_plane_group` 法向量反方向分组错误**
   - 文件: `src/hanfeng_util.cpp:640`
   - 原因: FB03C-166F-PLATE3 的 8 条边界环法向量有 [-1,0,0] 和 [1,0,0] 两种方向，`plane_offset` 符号相反导致同一平面被分为 4 组而非 2 组
   - 修复: 用点积正负乘以 sign 对齐 offset 后再比较

2. **HTML 无法显示数据（file:// 协议问题）**
   - 原因: `fetch()` 和 `<script src>` 在 file:// 下被浏览器安全策略拦截
   - 修复: 将 JSON 数据直接内联为 `var WELD_DATA = {...};` 嵌入 HTML 的 `<script>` 标签

3. **只显示一个模型**
   - 原因: 平面板只输出边界环，没有三角网格数据
   - 修复: 在测试中用 `api_get_surface()` 额外读取平面板 mesh，JSON 新增 `plane_vertices` / `plane_triangles`，HTML 渲染平面板绿色 mesh

### 当前问题：FB03C-166F-PLATE3 × FB03C-PL251-CPLATE 焊缝分段

用户观察到 3 段焊缝看似连续，分析结果：

| 段 | X | 起点 Y,Z | 终点 Y,Z | 长度 |
|---|---|---|---|---|
| #0 | 289689 | 1171.80, 9507.48 | 1173.47, 9508.40 | 1.9mm |
| #1 | 289689 | 583.84, 11001.00 | 581.83, 11000.50 | 2.1mm |
| #2 | 289675 | 1173.33, 9508.32 | 583.57, 11000.90 | 1604.9mm |

- 段0/段1 在 X=289689 面（face_b），极短（~2mm）
- 段2 在 X=289675 面（face_a），长线（~1.6m）
- Y/Z 坐标基本对齐，X 差 14mm = 平面板厚度
- **疑问**: 段0和段1是否应该被合并到段2中？或者它们是独立的焊缝片段？
- 可能原因: 焊缝计算分别在 face_a 和 face_b 上独立进行，产生分段结果

### 关键文件位置

| 文件 | 说明 |
|---|---|
| `tests/plane_surface_weld_test.cpp` | 焊缝测试 + HTML 生成 |
| `tests/test_main.cpp` | PLANE_SURFACE_TEST 宏 |
| `CMakeLists.txt` | plane_surface_weld 目标 |
| `src/hanfeng_util.cpp:640` | same_plane_group 修复 |
| `result/weld_viewer.html` | 内联数据的可视化 HTML |
| `result/weld_results.json` | 12 组焊缝结果 |
| `demo/index.html` | 参考的 Canvas 2D 渲染 |

### 构建与运行命令

```bash
cmake -B build -G "Visual Studio 18 2026"
cmake --build build --config Release --target plane_surface_weld
./build/Release/plane_surface_weld.exe
```
