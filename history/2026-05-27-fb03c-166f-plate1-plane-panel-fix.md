# 2026-05-27 FB03C-166F-PLATE1 平面板分类修正记录

## 背景

用户确认 `FB03C-166F-PLATE1` 实际是平面板，不是曲面板。

原先项目中该板件被放在曲面板集合中，导致两类问题：

- `model/metadata.json` 将其标记为 `surface`
- 真实模型焊缝测试按“平面板 × 曲面板”组合遍历时，把它错误地当成曲面板参与检测

## 根因

问题出在 `dxf_exporter.py` 的 `classify_panel()` 分类逻辑。

旧逻辑中：

- 若三角网完全共面，则判为 `plane`
- 若边界环数量不等于 2，则直接判为 `surface`

这会误伤“带厚度、带多个孔的平面板”。

`FB03C-166F-PLATE1` 的真实数据特征是：

- 三角网并非单一共面，因此不会命中第一条
- 边界环很多，共 14 个，因此被第二条直接误判为 `surface`
- 但这些边界环实际全部落在两组互相平行的平面上，符合平面板语义

## 修改内容

### 1. 修正板件分类逻辑

文件：`dxf_exporter.py`

调整 `classify_panel()`：

- 不再要求“边界环数量必须恰好为 2”
- 改为检查所有边界环是否都能归并到两组平行平面
- 若所有边界环都落在两组不同的平行平面上，则判为 `plane`
- 否则判为 `surface`

新增辅助函数：

- `_loop_plane_offset()`

### 2. 补充 Python 回归测试

文件：`tests/test_dxf_exporter.py`

新增基于真实 DXF 数据的测试：

- `test_classify_panel_marks_fb03c_166f_plate1_as_plane`

作用：

- 直接读取 `FB03C.dxf`
- 取出 `FB03C-166F-PLATE1`
- 重建边界环并执行分类
- 断言结果必须为 `plane`

### 3. 修正 C++ 模型读取测试

文件：`tests/model_io_test.cpp`

将原先把 `FB03C-166F-PLATE1` 当成曲面板读取的测试，改为按平面板读取：

- 从 `model/plane_panels/FB03C-166F-PLATE1` 调用 `api_get_plane()`
- 断言 `face_a` / `face_b` 的外轮廓各 1 个
- 断言两侧内孔轮廓各 6 个

### 4. 同步测试入口

文件：`tests/test_main.cpp`

- 注册新的平面板读取测试
- 移除旧的 `FB03C-166F-PLATE1` 曲面板测试入口引用

### 5. 重建模型与结果产物

重新执行导出和焊缝批量计算后，以下产物已同步刷新：

- `model/metadata.json`
- `model/plane_panels/FB03C-166F-PLATE1/`
- `demo/index.html`
- `ui.html`
- `result/weld_results.json`
- `result/weld_results.js`
- `result/weld_viewer.html`

## 修正后的数据状态

当前板件集合为：

平面板：

- `FB03C-166F-PLATE1`
- `FB03C-166F-PLATE3`
- `FB03C-2D-PLATE1`

曲面板：

- `FB03C-PL13-CPLATE`
- `FB03C-PL250-CPLATE`
- `FB03C-PL251-CPLATE`
- `FB03C-PL267-CPLATE`
- `FB03C-PL268-CPLATE`

对应真实模型焊缝组合数为：

- `3 × 5 = 15`

`FB03C-166F-PLATE1` 现在只作为平面板参与 5 组焊缝检测，不再作为曲面板出现。

## 验证结果

已执行并通过：

- `python -m unittest tests/test_dxf_exporter.py`
- `cmake --build build --config Release`
- `build\\Release\\hanfeng.exe`
- `build\\Release\\plane_surface_weld.exe`

焊缝批量结果验证到：

- 平面板数量：`3`
- 曲面板数量：`5`
- 总组合数：`15`

其中 `FB03C-166F-PLATE1` 的焊缝组合为：

- `FB03C-166F-PLATE1 x FB03C-PL13-CPLATE`
- `FB03C-166F-PLATE1 x FB03C-PL250-CPLATE`
- `FB03C-166F-PLATE1 x FB03C-PL251-CPLATE`
- `FB03C-166F-PLATE1 x FB03C-PL267-CPLATE`
- `FB03C-166F-PLATE1 x FB03C-PL268-CPLATE`

## 结论

这次修正已经覆盖：

- 板件分类逻辑
- Python 导出回归测试
- C++ 模型读取测试
- 真实模型焊缝批量检测
- 导出结果与可视化产物

`FB03C-166F-PLATE1` 已从“误归类的曲面板”恢复为“正确的平面板”。
