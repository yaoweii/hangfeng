# 平面边界线与曲面板焊缝判定实现设计

日期：2026-06-02

## 1. 背景

当前平面板到曲面板的焊缝判定，已经具备以下基础能力：

- 能对线段与三角 mesh 的 `distance <= tolerance` 求精确命中区间。
- 能对平面板候选边界线段逐段生成 weld polyline。
- 能对真实模型批量导出 `weld_results.json` 与 HTML 可视化结果。

但现有方法仍以“到曲面壳面的最近距离”为核心，这在当前业务语义下不够：

- 如果平面边界线段已经进入曲面板实体内部，该段也应当算焊缝。
- 如果平面边界线段与曲面板表面存在拓扑级别的共面重叠，该段也应当直接算焊缝。

只依赖距离会导致根本问题：

- 一条线段整段在曲面板内部。
- 另一条线段整段在曲面板外部，但与曲面板距离相同。
- 两者的 `d_min / d_max` 可能完全相同，但业务结论相反。

因此，最终判定不能只建立在“到曲面壳面的距离”上，而必须区分：

- 是否与目标表面存在拓扑级别共面重叠。
- 是否位于目标曲面板实体内部。
- 是否位于目标曲面板实体外部但距离边界不超过容差。


## 2. 目标

对于一条平面板候选边界线段 `p(t), t ∈ [0,1]`，求出最终焊缝参数区间：

```text
I_weld = I_topo_coplanar ∪ I_inside ∪ I_near
```

其中：

- `I_topo_coplanar`：线段与曲面板表面存在拓扑级别共面重叠的参数区间。
- `I_inside`：线段位于曲面板实体内部的参数区间。
- `I_near`：线段位于曲面板实体外部，但到曲面板边界的最小距离 `<= tolerance` 的参数区间。

最终目标是得到：

- 整段焊缝
- 整段非焊缝
- 或多段离散焊缝子区间

并把这些区间映射回 3D polyline。


## 3. 前提与假设

本方案默认以下条件成立：

1. 曲面板 mesh 可作为闭合实体使用。
2. 曲面板三角面片朝向一致，可支持实体内外判定。
3. 三角 mesh 没有严重自交。
4. “拓扑级别共面”可以通过表面面域重叠识别出来。

如果第 1 或第 2 条不成立，则本方案不能直接落地，需要先修复 mesh。


## 4. 核心定义

### 4.1 候选线段

对平面板任一候选边界线段，记：

```text
p(t) = p0 + t * (p1 - p0),  t ∈ [0,1]
```

### 4.2 拓扑级别共面区间

`I_topo_coplanar` 定义为：

```text
I_topo_coplanar = { t ∈ [0,1] | p(t) 与曲面板表面面域存在拓扑级别共面重叠 }
```

这里的“拓扑级别共面”不是指“恰好落在同一个无限平面里”，而是指：

- 线段的一段真实落在目标板的表面面域上。
- 该重叠具有正长度。
- 该段可以直接视为焊缝命中。

### 4.3 内部区间

`I_inside` 定义为：

```text
I_inside = { t ∈ [0,1] | p(t) 位于曲面板实体内部 }
```

### 4.4 外部近距离区间

`I_near` 定义为：

```text
I_near = { t ∈ [0,1] | p(t) 位于曲面板实体外部 且 dist(p(t), surface_boundary) <= tolerance }
```

注意：

- `I_topo_coplanar` 优先于普通 inside/outside 推断。
- `I_near` 只在实体外部求。
- 内部点不需要再看距离，直接由 `I_inside` 负责。

### 4.5 最终焊缝区间

```text
I_weld = union(I_topo_coplanar, I_inside, I_near)
```


## 5. 总体流程

### 步骤 1：构造曲面板实体查询对象

输入：

- 曲面板 `SurfacePanel`

预处理得到：

- `SurfaceGeometry`：所有三角面片
- `ClosedSolidQuery`：支持以下操作
  - `point_in_solid(point)`
  - `segment_surface_intersections(start, end)`
  - `segment_triangle_distance_intervals(start, end, tolerance)`
  - `segment_topo_coplanar_intervals(start, end)`

如果后续性能不足，可在该层引入 BVH。

### 步骤 2：对每条平面候选边界线段先求 `I_topo_coplanar`

对单条线段，先识别它与曲面板表面之间是否存在拓扑级别共面重叠：

1. 检测线段与哪些三角面片属于同一局部表面面域。
2. 识别是否形成正长度重叠区间。
3. 记录为 `I_topo_coplanar`。

这类区间直接算焊缝，不需要再经过 inside/outside 推断。

### 步骤 3：对剩余部分求 `I_inside`

对单条线段：

1. 求线段与曲面板边界三角面的所有交点参数 `t_intersections`。
2. 去重、排序。
3. 判断线段起点 `p(0)` 是 inside 还是 outside。
4. 沿参数轴扫描，每遇到一次真正穿透交点，就翻转 inside/outside 状态。
5. 记录所有 inside 状态覆盖的参数区间，形成 `I_inside`。

### 步骤 4：求 `I_outside`

```text
I_outside = [0,1] \ union(I_topo_coplanar, I_inside)
```

`I_outside` 可能是：

- 空
- 一个区间
- 多个离散区间

### 步骤 5：只在 `I_outside` 上求 `I_near`

对 `I_outside` 中的每个子区间：

1. 调用现有的精确 `segment × triangle mesh` 距离区间求解。
2. 求得满足 `distance <= tolerance` 的参数区间。
3. 把这些区间裁切在当前 outside 子区间内。
4. 合并得到最终 `I_near`。

### 步骤 6：合并

```text
I_weld = union(I_topo_coplanar, I_inside, I_near)
```

合并规则：

- 重叠区间合并。
- 端点接触且间隙在数值容差内时合并。
- 零长度区间丢弃。

### 步骤 7：映射回 3D polyline

对 `I_weld` 中每个区间 `[ta, tb]`：

- 计算 `p(ta)` 和 `p(tb)`。
- 与相邻边段拼接。
- 得到最终 weld polyline。


## 6. `I_topo_coplanar` 的详细实现

### 6.1 语义

这里的“共面”只接受拓扑级别共面，不接受单纯几何共面。

也就是说，不是判断：

- 线段和某个三角形恰好落在同一个无限平面。

而是判断：

- 线段的一段真实落在目标板的表面面域上。
- 该段和表面面域形成正长度重叠。

满足该条件时，这段直接算焊缝。

### 6.2 与普通相交的区别

普通穿透相交是“点事件”：

- 进入一次。
- 离开一次。
- inside/outside 状态翻转。

拓扑级别共面是“区间事件”：

- 不是单个交点。
- 而是一段连续重叠参数区间。
- 不应按 crossing 交点去翻转 inside/outside。

### 6.3 处理原则

若线段与目标表面存在拓扑级别共面重叠区间：

- 直接把该区间加入 `I_topo_coplanar`。
- 不再要求它满足 `I_inside`。
- 也不再要求通过 `distance <= tol` 重新证明。

### 6.4 实现建议

第一版可采用保守实现：

1. 识别共面三角面片集合。
2. 对每个共面三角面片，求线段与三角面域的参数重叠区间。
3. 将相邻且属于同一拓扑表面区域的区间合并。
4. 去掉零长度区间。

如果后续需要更严格的“同一表面区域”语义，可在三角邻接层面补充拓扑分组。


## 7. `I_inside` 的详细实现

### 7.1 需要解决的问题

`I_inside` 的关键不是最近距离，而是：

- 线段何时进入实体。
- 线段何时离开实体。
- 线段是否整段都在实体内。

### 7.2 交点求解

对线段与每个三角形求交，得到参数 `t`。

需要注意：

1. 单点相切不能简单当作 inside/outside 翻转。
2. 命中三角形边或顶点时，可能多个三角形给出同一个 `t`，要去重。
3. 共面重合不按普通穿透交点处理，已由 `I_topo_coplanar` 单独负责。

交点分类建议分成：

- `crossing`：真正穿透实体边界。
- `touching`：仅相切，不翻转。

### 7.3 起点 inside/outside 判定

对线段起点 `p(0)` 调用：

```text
point_in_solid(p(0))
```

得到初始状态：

- inside
- outside
- on-boundary

若 `on-boundary`：

- 向线段方向做一个极小偏移 `p(eps)` 重新判定。
- 用偏移后的状态作为扫描初值。

### 7.4 扫描规则

设排序后的有效穿透交点为：

```text
t0 < t1 < t2 < ... < tn
```

扫描状态机：

- 初始状态由 `point_in_solid(p(0+eps))` 决定。
- 每遇到一个 `crossing` 交点，翻转一次 inside/outside。
- 当前状态为 inside 时，记录参数区间。

例如：

- 起点 outside，交点为 `t1, t2`
  - `I_inside = [t1, t2]`

- 起点 inside，无交点
  - `I_inside = [0,1]`

- 起点 outside，交点为 `t1, t2, t3, t4`
  - `I_inside = [t1, t2] ∪ [t3, t4]`

### 7.5 与共面区间的关系

拓扑级别共面区间已经在 `I_topo_coplanar` 中单独处理，因此：

- `I_inside` 的扫描逻辑不负责识别共面焊缝。
- 共面区间不应被误当成 crossing 交点来翻转 inside/outside。


## 8. `I_near` 的详细实现

### 8.1 定义

只对 outside 区间求：

```text
I_near = { t | p(t) outside solid 且 dist(p(t), surface mesh) <= tolerance }
```

### 8.2 实现来源

当前代码已经具备：

- `segment × triangle` 精确区间求解。
- `segment × surface mesh` 多三角区间并集求解。

因此不需要重写距离内核，只需调整使用方式：

1. 先求全线段 `distance <= tol` 区间。
2. 再与 `I_outside` 求交。

或者更省：

1. 只对 `I_outside` 子区间分别求 `distance <= tol`。
2. 再合并。

推荐第二种，因为可以减少不必要计算。


## 9. 所有情况如何判断与处理

### 情况 A：整段在实体外，且整段距离都大于容差

- `I_topo_coplanar = ∅`
- `I_inside = ∅`
- `I_near = ∅`
- 结果：整段 `pass`

### 情况 B：整段在实体外，且整段距离都小于容差

- `I_topo_coplanar = ∅`
- `I_inside = ∅`
- `I_near = [0,1]`
- 结果：整段焊缝

### 情况 C：整段在实体内

- `I_topo_coplanar = ∅`
- `I_inside = [0,1]`
- `I_near` 不必计算
- 结果：整段焊缝

### 情况 D：整段与目标表面拓扑级别共面重叠

- `I_topo_coplanar = [0,1]`
- 结果：整段焊缝

### 情况 E：先在外，再进入实体，再离开实体

- `I_inside = [t_enter, t_exit]`
- outside 两侧再分别计算 `I_near`
- 结果：`I_inside ∪ 两侧外部近距离区间`

### 情况 F：整段都在外，但距离出现多段进入/离开容差带

例如：

```text
>tol -> <=tol -> >tol -> <=tol -> >tol
```

则：

- `I_inside = ∅`
- `I_near = [t1,t2] ∪ [t3,t4]`
- 结果：两段焊缝

### 情况 G：一部分在内部，一部分在外部近距离带

- `I_inside` 非空
- `I_near` 非空
- 结果：取并集并合并相邻区间

### 情况 H：一部分拓扑级别共面，一部分在内部或外部近距离带

- `I_topo_coplanar` 非空
- `I_inside` 或 `I_near` 非空
- 结果：三者取并集

### 情况 I：相切

- 仅单点接触
- 若没有正长度区间，则丢弃
- 不生成焊缝段

### 情况 J：起点在边界上

- 使用 `p(eps)` 判 inside/outside 初始状态
- 再进入扫描


## 10. 数据结构建议

### 10.1 新增实体查询结果

```cpp
struct SolidIntersectionEvent {
    double t;
    enum class Type {
        Crossing,
        Touching
    } type;
};
```

### 10.2 区间结构

继续复用现有：

```cpp
struct Interval {
    double start;
    double end;
};
```

### 10.3 新增接口

```cpp
bool point_in_surface_solid(const SPAposition& point,
                            const SurfaceGeometry& geometry);

std::vector<Interval> exact_segment_topo_coplanar_intervals(
    const SPAposition& start,
    const SPAposition& end,
    const SurfaceGeometry& geometry);

std::vector<SolidIntersectionEvent> segment_surface_solid_events(
    const SPAposition& start,
    const SPAposition& end,
    const SurfaceGeometry& geometry);

std::vector<Interval> exact_segment_inside_intervals(
    const SPAposition& start,
    const SPAposition& end,
    const SurfaceGeometry& geometry);
```


## 11. 与现有代码的集成方式

当前 `plane -> surface` 路径大致是：

- 收集平面候选边界环。
- 对每条边界线段调用 `exact_segment_surface_intervals...`。
- 拼出 weld polyline。

改造后建议变成：

1. 保留现有候选边界环流程。
2. 对每条线段先求 `I_topo_coplanar`。
3. 对剩余部分求 `I_inside`。
4. 求 `I_outside`。
5. 对 `I_outside` 求 `I_near`。
6. 合并成 `I_weld`。
7. 用 `I_weld` 驱动 polyline 拼接。

这样可以最大程度复用已有的：

- 区间合并
- polyline 拼接
- 去重

需要替换的只是一条线段的“命中区间求解器”。


## 12. 数值与鲁棒性要求

### 12.1 容差

- 距离判断：`distance <= tolerance + eps`
- 参数比较：`eps_t = 1e-9`
- 点在边界上的 inside/outside 采样偏移：`eps_p = 1e-6 ~ 1e-5`

### 12.2 去重

交点参数去重时：

- `|t_i - t_j| <= eps_t` 视为同一点

### 12.3 零长度区间

若：

```text
end - start <= eps_t
```

则丢弃。

### 12.4 相邻区间合并

若两个区间满足：

```text
next.start <= current.end + eps_t
```

则合并。


## 13. 性能建议

第一版可先正确实现，再做加速。

建议性能优化顺序：

1. 复用现有 triangle AABB 剪枝。
2. 为 surface mesh 建 BVH。
3. `point_in_solid` 使用 BVH 加速射线求交。
4. `segment_surface_solid_events` 使用 BVH 剪枝三角求交。
5. 仅在 `I_outside` 子区间上做 `I_near`。


## 14. 测试方案

至少覆盖以下测试：

### 14.1 合成测试

1. 整段在实体外且远
2. 整段在实体外且近
3. 整段在实体内
4. 整段与目标表面拓扑级别共面
5. 一次进入一次离开
6. 多次进入/离开容差带，得到多段 `I_near`
7. 起点在边界上
8. 相切但无正长度
9. 拓扑级别共面区间与 inside/near 区间相邻或重叠

### 14.2 回归测试

1. 当前已有的“精确 band”测试不能回退
2. “plane boundary inside surface” 合成测试要改为基于实体语义验证
3. `FB03C-166F-PLATE3 × FB03C-PL251-CPLATE` 作为真实模型回归样本

### 14.3 导出验证

重新生成：

- `result/weld_results.json`
- `result/weld_viewer.html`

并核对：

- 线段整段在内部时能保留
- 外部多段近距离区间能分别保留
- 拓扑级别共面段能直接保留
- 不再依赖 `delta · n_surface` 作为硬过滤


## 15. 实施顺序

建议按下面顺序实现：

1. 增加 `exact_segment_topo_coplanar_intervals`
2. 增加 `point_in_surface_solid`
3. 增加 `segment_surface_solid_events`
4. 实现 `exact_segment_inside_intervals`
5. 在 `plane -> surface` 路径接入 `I_topo_coplanar`
6. 对剩余部分接入 `I_inside`
7. 只对 `I_outside` 求 `I_near`
8. 合并成 `I_weld`
9. 跑合成测试
10. 跑真实模型导出回归


## 16. 本方案的取舍

### 优点

- 语义直接，对“内部也算焊缝”支持天然。
- 拓扑级别共面可以直接命中，不需要再绕回距离启发式。
- 不再需要围绕 `d_min / d_max / delta·n` 写大量启发式规则。
- 输出天然是区间，适合当前 weld polyline 生成流程。

### 风险

- 前提依赖闭合 mesh 与一致朝向。
- 拓扑级别共面识别实现复杂。
- 切触与边界重合情形需要仔细做鲁棒处理。
- 若 mesh 有缺陷，inside/outside 判定可能不稳定。

### 结论

只要目标曲面板可以被视为闭合实体，并且能识别拓扑级别共面区间，这个方案比“距离 + 点乘启发式判断”更清晰、更稳定，也更符合当前业务语义。
