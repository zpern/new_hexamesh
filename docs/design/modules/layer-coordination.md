# 层数协调与停止传播设计

## 1. 文档目的

本文定义 BoundaryMesh 阶段 07 的相邻层数协调和停止传播模型。本阶段在规则层逐层生成期间维护每个 Wall 源面的最大允许层数，使共享源边的两个面最终层数差不超过外部配置值。

本文只描述层数上限、共享边邻接、传播顺序和生成事务，不生成 Pyramid/Tetra 过渡单元，也不处理只共享顶点的复杂角点。

## 2. 设计目标

阶段 07 必须满足：

1. 外部可配置共享边两侧最大允许层数差，默认值为 1；
2. 参数允许为 0，表示共享边两侧必须等层；
3. 只沿 GrowthPatch 内共享完整源边的面传播，不沿仅共享顶点的面传播；
4. 顶点请求层数、质量失败、碰撞停止和邻接约束全部参与传播；
5. 已知请求上限在生成前传播，运行时停止在当前层提交前动态传播；
6. 每个源面的允许层数只减不增，不删除已经提交的历史单元；
7. 直接停止原因优先，邻接原因不得覆盖质量或碰撞等直接原因；
8. 队列和最终结果不依赖源面遍历顺序；
9. 程序级错误保持当前层零提交；
10. 最终结果继续使用已有 `FaceGrowthRecord`。

## 3. 非目标

阶段 07 不负责：

- 沿仅共享顶点的面传播；
- 识别停止后形成的过渡区域；
- 生成 Pyramid/Tetra；
- 回滚或裁剪已经提交的体单元；
- 记录触发邻居、完整传播路径或队列调试历史；
- PLY、VTK、命令行和真实性能测试。

## 4. 公共配置和结果语义

`RegularLayerGrowthOptions` 增加：

```cpp
std::uint32_t max_neighbor_layer_difference{1}; // 共享边两侧最大允许层数差
```

该值为无符号整数：

- `0` 表示共享边两侧最终层数相同；
- `1` 是默认值，表示最多相差一层；
- `2` 及以上允许更宽的过渡。

`FaceStopReason` 增加：

```cpp
NeighborLayerConstraint // 因相邻源面的层数上限传播而提前停止
```

状态分类：

- 达到原始逐点请求上限：`Completed + VertexLayerLimit`；
- 低于原始请求且只由传播限制：`Stopped + NeighborLayerConstraint`；
- 直接质量失败：保留已有质量原因；
- 直接碰撞：保留 `Collision`。

`stop_layer` 仍表示第一个未接受的目标层号。

## 5. 面层数约束

内部 `FaceLayerConstraint` 至少保存：

```cpp
struct FaceLayerConstraint
{
    SurfaceFaceId source_face_id{};        // 输入 Wall 面编号
    std::uint32_t requested_layer_count{}; // 原始逐点参数决定的面层数上限
    std::uint32_t allowed_layer_count{};   // 传播后当前最大允许层数
    FaceLayerLimitKind limit_kind{};       // 用户请求、直接停止或邻接约束
    FaceStopReason direct_reason{};         // 质量或碰撞等直接停止原因
};
```

源面的原始请求层数为该面所有源顶点请求的最小值：

```text
requested(face) = min(layer_count(vertex) for vertex in face)
```

`allowed_layer_count` 初始等于 `requested_layer_count`，后续只允许降低。

最终结果不公开约束表，不增加重复的 `FaceGrowthState`。现有 `FaceGrowthRecord` 已足以表达接受层数、最终状态、停止原因和停止层。

## 6. 共享边邻接图

`TerminationPropagator` 从 `SurfaceTopology` 构造 GrowthPatch 内面邻接图。

只有以下情况建立传播边：

- 两个源面都属于当前 GrowthPatch；
- 两个源面在 `SurfaceTopology` 中共享同一条完整边。

Triangle–Triangle、Triangle–Quad 和 Quad–Quad 均按同一规则处理。只共享一个顶点不建立传播边。Patch 外部的 Wall、Farfield 和 Symmetry 面不加入该传播图。

每个邻接列表按 `SurfaceFaceId` 排序并去重。

## 7. 传播算法

对于共享边相邻面 `current` 和 `neighbor`：

```text
neighbor.allowed = min(
    neighbor.allowed,
    current.allowed + max_neighbor_layer_difference)
```

加法必须使用更宽整数或饱和计算，禁止 `uint32_t` 回绕。

只有上限实际降低时，邻面才重新入队。队列按 `SurfaceFaceId` 确定性处理。由于所有上限均为非负整数且只会降低，传播必然终止。

该过程计算不超过原始请求和直接停止限制的最大可行上限分布。例如：

```text
请求层数：2 — 10 — 10
最大差值：1
传播结果：2 —  3 —  4
```

多个低层限制同时存在时，对每个面取所有传播路径给出的最严格上限。

## 8. 初始化传播

生成开始前执行：

1. 逐面计算 `requested_layer_count`；
2. 构造 Patch 共享边邻接图；
3. 将 `allowed_layer_count` 初始化为请求值；
4. 对全部已知请求上限执行一次传播。

因此较低的用户请求不会等到运行时完成以后才影响邻域，受限面不会生成明知最终必须删除的层。

## 9. 运行时传播和单层数据流

目标层为 `target_layer` 时，严格执行：

1. Stepper 在动态几何计算前检查逐面允许上限；
2. `current_layer >= allowed_layer_count` 的面不参与预推出；
3. 上限等于请求值时记录正常完成，上限低于请求值时记录邻接停止；
4. 对剩余面执行预推出和质量检查；
5. 直接质量失败面上限降为 `target_layer - 1`，保留直接质量原因并传播；
6. 删除传播后 `allowed_layer_count < target_layer` 的候选；
7. 对剩余候选检查原始表面和历史外露边界；
8. 碰撞面上限降为 `target_layer - 1`，保留 `Collision` 并传播；
9. 再次删除本层已经不允许提交的候选；
10. 对剩余候选执行同层自碰撞；
11. 同层相撞双方上限均降为 `target_layer - 1`，传播并最终过滤；
12. 原子提交最终候选。

如果传播后：

```text
allowed_layer_count < target_layer
```

本层候选不得提交，停止层为 `target_layer`。

如果：

```text
allowed_layer_count == target_layer
```

本层可以提交；下一层开始前以 `NeighborLayerConstraint` 停止。

传播不会要求删除已经提交的历史单元。运行时直接停止发生在当前目标层提交以前，因此相邻活动面不会已经超出新上限。

## 10. 与阶段 06 的接口衔接

为了让传播约束退出的候选不成为同层幽灵障碍，`LayerCollisionChecker` 分为：

```cpp
filterAgainstObstacles(...) // 原始 Wall/Farfield 与历史外露边界
filterSelfCollisions(...)   // 当前剩余候选之间
```

阶段 06 单独实施时连续调用两个操作。阶段 07 接入后，在两个操作之间应用动态停止传播并压缩候选集合。

质量停止传播发生在两个碰撞操作以前。这样质量失败或邻接退出的候选也不会进入同层候选树。

## 11. 停止原因优先级

停止原因按直接性处理：

```text
直接质量失败      → 质量原因
直接碰撞          → Collision
达到自身请求上限  → VertexLayerLimit
仅由邻接传播限制  → NeighborLayerConstraint
```

邻接传播只降低允许层数，不能覆盖已经存在的直接原因。一个面同时受到多条传播路径限制时不记录路径来源。

## 12. 错误处理

新增程序错误：

```cpp
struct InvalidFaceConstraintState
{
    SurfaceFaceId source_face_id{}; // 状态不一致的源 Wall 面
    std::uint32_t layer{};          // 发现错误时的目标层号
};
```

以下情况返回 `RegularLayerGrowthError`：

- GrowthPatch 源面在拓扑中不存在；
- 约束表缺少或重复源面；
- Stepper 或碰撞事件引用未知源面；
- 当前 Front 层号高于面已接受或允许层数；
- 压缩后的 Front 与约束表映射不一致。

程序级错误发生时当前层零提交。传播没有不收敛错误。

## 13. 测试要求

### 13.1 约束和邻接

- 面请求层数取顶点最小值；
- 三种 Triangle/Quad 共享边组合；
- 仅共享顶点不邻接；
- 不同 Patch 连通分量互不传播；
- Patch 外面不进入传播图；
- 输入顺序不改变邻接结果；
- 缺失、重复和未知源面返回错误。

### 13.2 传播器

- 差值 0、1、2 的链式传播；
- 环形邻接收敛；
- 多源限制取最严格上限；
- 动态质量和碰撞停止传播；
- 上限只减不增；
- 直接原因优先；
- 邻接停止原因正确；
- 队列和输入顺序不变量；
- 最大 `uint32_t` 差值不溢出。

### 13.3 生成器集成

- 已知请求在生成前传播；
- 差值 0 阻止邻面提交同一目标层；
- 差值 1 允许邻面多提交一层；
- 质量、原始/历史碰撞和同层碰撞传播；
- 因传播退出的候选不进入同层树；
- 完成、直接停止和邻接停止记录正确；
- 无孤立顶点、半提交单元或错误外露面；
- Triangle/Quad 混合链；
- 复杂角点只沿共享边传播；
- Debug、Release 完整 CTest 回归。

真实性能和 PLY 案例属于阶段 10。

## 14. 完成边界

阶段 07 完成后，每个 GrowthPatch 连通分量内的最终源面层数满足外部最大共享边层数差，且全部动态停止在提交前以确定性方式传播。

本阶段只协调规则层结果，不识别过渡区域，也不生成过渡单元。
