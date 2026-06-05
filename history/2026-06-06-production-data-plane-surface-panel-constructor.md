# 实现生产数据到平面/曲面板构造函数

## 工作概述

本轮工作新增了两个面向生产输入的构造接口：

- `api_get_plane_panel(...)`
- `api_get_surface_panel(...)`

目标是让系统不再只依赖 `model` 目录下的中间文件，而是可以直接从生产侧提供的原始几何数据构造现有的 `PlanePanel` / `SurfacePanel`，继续复用后续焊缝计算流程。

---

## 一、平面板生产构造接口

### 1. 新增接口

在 `include/hanfeng/hanfeng_util.hpp` / `src/hanfeng_util.cpp` 中新增了平面板生产输入重载：

```cpp
PlanePanel api_get_plane_panel(const float rxyPltBndry1[1000][3],
    const float rxyholes[20][1000][3],
    const float cAxis[4][3],
    float thickdis1,
    float thickdis2);
```

### 2. 输入语义

- `rxyPltBndry1`：局部坐标系下的外边界 `rxy`
- `rxyholes`：局部坐标系下的内孔 `rxy`，`rxyholes[0][0][0]` 为孔数量
- `cAxis[4][3]`：局部平面到三维世界坐标的基
  - 第 0 行：原点
  - 第 1 行：`u`
  - 第 2 行：`v`
  - 第 3 行：`w`
- `thickdis1`：沿 `w+` 偏移
- `thickdis2`：沿 `w-` 偏移

### 3. 实现内容

- 增加了原始 `[r, x, y]` 数组到 `RxyCurve` 的解析
- 增加了 `rxy` 曲线到局部二维折线的展开
- 支持直线段和圆弧段
- 圆弧段在接口内部离散成折线，以兼容当前项目的边界消费方式
- 将局部二维点通过 `cAxis` 映射到三维
- 构造：
  - `face_a = base + w * thickdis1`
  - `face_b = base - w * thickdis2`
- 外边界写入两侧面的 `outer_loops`
- 内孔写入两侧面的 `inner_loops`

### 4. 测试覆盖

新增并通过了以下测试：

- 直接由矩形 `rxy` 构造双面平面板
- 直接由内孔 `rxy` 构造双面内孔
- 含圆弧段输入时可正确离散为折线

---

## 二、曲面板生产构造接口

### 1. 新增接口

在 `include/hanfeng/hanfeng_util.hpp` / `src/hanfeng_util.cpp` 中新增了曲面板生产输入接口：

```cpp
SurfacePanel api_get_surface_panel(const float vertices[][3],
    int vertex_count,
    const int triangles[][3],
    int triangle_count,
    const float normals[][3]);
```

### 2. 输入语义

- `vertices`：顶点坐标
- `vertex_count`：顶点数
- `triangles`：三角面片索引
- `triangle_count`：三角面片数
- `normals`：每个点的法线坐标

当前实现中：

- 点法线已作为生产接口输入接收
- 边界提取主流程仍以三角面片拓扑为主
- 这样可以最大程度对齐现有 `model` 目录中的边界生成逻辑

### 3. 边界提取实现

新接口的边界提取逻辑参考并对齐了现有 `model` 生成链路中 `dxf_exporter.py` 的实现：

#### 3.1 拓扑边界提取

- 统计所有三角形边
- 只出现一次的边视为拓扑边界
- 构造成边界图
- 追踪闭合边界环

#### 3.2 Smooth-region 边界补充

- 按三角片法向夹角构造光滑区域连通分量
- 对主要区域提取区域交界边
- 将其追踪为补充边界环

#### 3.3 边界合并与分类

- 合并拓扑边界与 smooth-region 边界
- 去重
- 按当前项目约定分类：
  - 前 2 条为 `outer_loops`
  - 其余为 `inner_loops`

### 4. 实现效果

新增接口可直接从生产侧 mesh 数据构造 `SurfacePanel`，并自动补出项目现有语义下的边界信息，不需要再依赖外部 `.boundary.xyz.json` 文件。

### 5. 测试覆盖

新增并通过了以下测试：

- 简单矩形 mesh 可提取单条拓扑边界
- smooth-region 回退场景可提取两条主边界
- 三条边界环场景可正确分为 2 条外边界 + 1 条内孔
- 三角形索引越界会正确抛错

---

## 三、兼容性处理

为了避免影响现有流程，本轮工作保留了原有文件读取接口：

- `api_get_plane_panel(const std::filesystem::path& model_path)`
- `api_get_surface(const std::filesystem::path& model_path)`

也就是说：

- 原有从 `model` 目录读取的流程继续可用
- 新增生产输入重载用于直接构造
- 两套入口最终都会产出相同数据结构，便于复用后续焊缝算法

---

## 四、验证结果

本轮改动已完成以下验证：

- `cmake --build build --config Debug --target hanfeng`
- `.\build\Debug\hanfeng.exe`
- `cmake --build build --config Debug --target plane_surface_weld`

以上验证均已通过。

---

## 五、当前结论

当前项目已经具备：

- 从生产 `rxy` 数据直接构造 `PlanePanel`
- 从生产 mesh 数据直接构造 `SurfacePanel`
- 两者都能继续进入现有焊缝计算流程

后续如果需要，还可以继续补两类工作：

- 将曲面板接口进一步改成固定长度生产数组签名
- 根据生产系统的实际入参格式，再补一层更薄的适配包装
