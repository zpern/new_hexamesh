# Symmetry 与 Internal 滑移曲面生长设计

## 目标

将当前仅存在于独立模块和测试中的平面滑移约束接入正式边界层生成流程，并参考旧 BLMesh 的 `symmetry.h`、`NormalSmoothStrategy`、`GenerateBLMesh()`、`PropagateNode()` 与 `UpdateSymmetry()`，完整支持以下行为：

- `.bc.txt` 输入可声明 Symmetry 与 Internal Zone；
- Symmetry 与 Internal 使用同一套轴对齐平面或离散曲面滑移算法；
- 方向和步长光滑后重新约束最终生长方向；
- 最终候选位置二次投影到全部关联滑移曲面；
- 原始碰撞索引排除 Symmetry 与 Internal；
- 滑移 region 归属逐层继承；
- 每层生成并保留 Symmetry 与 Internal 侧面；
- 完整远场边界保留滑移侧面，边界层顶面仍只包含 BoundaryLayerInterface。

本设计沿用当前工程的 `SurfaceMesh`、`GrowthPatch`、`GrowthFront`、`Result`、错误模型和空间模块。复用参考工程的算法与处理顺序，但不直接引入其 BLMesh 数据结构或整套 libigl 依赖。

## 当前状态与缺口

当前工程已经具备：

- `SurfaceBoundaryKind::Symmetry` 与 `SurfaceBoundaryKind::Internal`；
- `PatchVertex::sliding_region_ids` 和 `FrontVertexBoundary::sliding_region_ids`；
- Patch 构建时收集 Wall 顶点邻接的 Symmetry/Internal region；
- 逐层前沿压缩时保留滑移 region；
- `SlidingConstraintBuilder` 的共面平面识别、单平面切向约束、双平面交线约束和过约束检查；
- 原始碰撞索引排除 Symmetry。

当前缺口是：

- `.bc.txt` 只接受 `Far:` 和 `Wall:`；
- 正式规则层生长不调用 `SlidingConstraintBuilder`；
- 非轴对齐或曲面 region 不受支持；
- 方向/步长光滑后没有重新施加滑移约束；
- 最终候选位置没有投影回滑移几何；
- Internal 尚未从原始碰撞索引排除；
- 没有等价于参考工程 `UpdateSymmetry()` 的逐层侧面生成；
- 最终完整边界不保留 Symmetry/Internal 侧面。

## 输入兼容

边界映射文件增加两个可选段：

```text
Wall:
1
Far:
2
Symmetry:
3
4
Internal:
5
```

规则如下：

- 保持 `Wall:` 和 `Far:` 的原有语义与格式；
- `Symmetry:` 映射到 `SurfaceBoundaryKind::Symmetry`；
- `Internal:` 映射到 `SurfaceBoundaryKind::Internal`；
- Zone ID 继续写入 `SurfaceBoundaryTag::region_id`；
- 每个可用 Zone 必须恰好出现一次；
- 重复 Zone、未知段、非法 ID、缺失 Zone 的错误行为保持现有严格解析策略；
- 旧的只含 Wall/Far 文件无需修改。

## 滑移几何模型

### Region 语义

每个被前沿引用的 Symmetry 或 Internal `region_id` 建立一个 `SlidingSurface`。region ID 在整个 `SurfaceMesh` 中作为滑移几何键；同一 region 的离散三角形和四边形共同描述一个连续物理表面。

`SlidingSurface` 至少保存：

- `region_id`；
- 原始 `SurfaceBoundaryKind`，仅允许 Symmetry 或 Internal；
- `AxisX`、`AxisY`、`AxisZ` 或 `Curved` 类型；
- 轴对齐平面的常量坐标；
- 曲面三角化结果；
- 曲面三角形包围盒及最近点加速结构；
- 平均边长、参考长度和几何容差。

### 轴对齐平面识别

按参考 `symmetry.h::judgyType()` 建立参数：

```text
reference_length = 0.02 * average_edge_length
axis_epsilon     = 0.1  * reference_length
```

按 X、Y、Z 顺序检查 region 全部顶点的坐标跨度：

```text
max(axis) - min(axis) < axis_epsilon
```

第一个满足条件的轴确定为 `AxisX`、`AxisY` 或 `AxisZ`。常量坐标使用该 region 全部唯一顶点对应坐标的稳定平均值，并要求所有顶点到解析平面的距离不超过几何容差。

如果三个轴均不满足，region 自动建立为 `Curved`，不返回“不支持非轴对齐平面”错误。

### 曲面三角化与最近点投影

- 三角形直接作为投影图元；
- 四边形采用当前工程统一的确定性对角线策略拆分；
- 退化图元、非有限坐标或越界顶点使 region 构建失败；
- 扩展当前空间模块，为三角形集合提供确定性最近点查询；
- 最近点算法覆盖三角形内部、三条边和三个顶点；
- 距离相同的候选按源面 ID、局部三角形 ID 稳定择优；
- 不引入参考工程的 libigl `AABB` 或 BLMesh `BinaryTree` 数据类型。

## Patch 与 Front 归属

GrowthPatch 仍然只由 Wall 面构成。对每个被 Wall 选中的源顶点，遍历完整 `SurfaceTopology::vertexFaces()`，收集所有邻接 Symmetry/Internal 面的 `region_id`，排序去重后写入 `PatchVertex::sliding_region_ids`。

GrowthFrontBuilder 将归属复制到 `FrontVertexBoundary::sliding_region_ids`。后续 compact、停止传播、规则层和过渡流程必须原样继承。Symmetry/Internal 面不作为生长顶面进入 `GrowthFront::faces`。

## 每层生长数据流

正式规则层的顺序为：

```text
计算原始生长方向
-> GrowthFieldSmoother 同时光滑方向与实际步长
-> 使用 SlidingSurface 约束光滑后的最终方向
-> current_position + actual_height * constrained_direction
-> 将最终候选位置投影到全部关联 SlidingSurface
-> 各向同性停止判断
-> Prism/Hexa 质量判断
-> 碰撞判断
-> 接受节点与体单元
-> 生成/更新外露滑移侧面
```

质量、停止与碰撞阶段必须使用二次投影后的最终坐标。

### 最终方向约束

零个滑移 region：只验证并归一化光滑结果。

一个轴对齐面：将对应坐标分量清零后归一化：

- AxisX：`direction.x = 0`；
- AxisY：`direction.y = 0`；
- AxisZ：`direction.z = 0`。

一个曲面：参考 `SymmetryPlane::adjustNormal()`：

```text
endpoint              = current_position + smoothed_direction
projected_endpoint    = project(endpoint, curved_surface)
constrained_direction = normalize(projected_endpoint - current_position)
```

多个滑移面：按升序 region ID 使用确定性顺序迭代方向端点投影。轴对齐面使用解析投影，曲面使用最近点投影。结果退化或非有限时返回可诊断错误。

三个独立轴向平面会完全锁死三维方向，返回 `OverConstrainedGrowthVertex`。一般曲面组合不预先用固定数量判定过约束，而以迭代收敛和结果方向是否退化为准。

### 最终候选位置投影

参考 `PropagateNode()` 与 `ProjectFinalSymmetryPosition()`：

```text
raw_candidate = current_position
              + actual_height * constrained_direction
```

一个关联面时直接投影一次，并复投影检查表面残差。

多个关联面时，从 `raw_candidate` 开始，按升序 region ID 依次投影到每个表面，最多迭代 20 次。每轮计算：

- 本轮位置变化；
- 投影后点到所有关联表面的最大残差。

两者都不超过投影容差时才收敛。投影容差由前沿有效长度容差和机器精度建立，不使用与模型尺度无关的固定绝对值作为唯一判据。

缺失 region、最近点查询失败、非有限坐标、方向退化或迭代不收敛时，停止受影响顶点关联的候选面，不创建未约束的新节点。错误携带 layer、front/source vertex 与相关 region。

## 碰撞语义

构建原始表面碰撞索引时主动跳过：

```cpp
SurfaceBoundaryKind::Symmetry
SurfaceBoundaryKind::Internal
```

两类表面都是滑移几何，不作为阻挡边界层的实体障碍。Wall、Farfield 及其他类别保持当前碰撞语义。

动态外露边界中的滑移侧面用于边界拓扑和最终输出；同一 region 上按约束生成的相邻候选不因接触该滑移侧面而被判为非法自碰撞。其他跨 region 或非拓扑相邻接触仍遵循现有碰撞规则。

## 逐层滑移侧面

本阶段实现参考工程 `UpdateSymmetry()` 的等价能力，并同时适用于 Symmetry 和 Internal。

每次接受一层生长后，对上一层前沿的每条规范无向边：

1. 求两个端点 `sliding_region_ids` 的交集；
2. 对每个共同 region 查找其 `SlidingSurface`；
3. 若两个端点均在本层成功生成上层点，则使用下层边和上层边建立 Quad；
4. 侧面方向根据相邻体单元外向约定确定，而不是依赖输入边遍历顺序；
5. 使用原始滑移类别和 region 标记：

```cpp
SurfaceBoundaryTag{original_kind, region_id}
```

其中 `original_kind` 分别保持 Symmetry 或 Internal。

局部停止、层数不一致或过渡模板改变上下层拓扑时，不强行建立退化 Quad。最终侧面从实际已接受体单元的外露边界提取，并用端点共同滑移 region 分类；必要时形成三角形或过渡面，从而避免孔洞。

相邻体单元生成的内部重复面使用规范化顶点键成对抵消。真正位于滑移边界上的外露面只出现一次，因此保留。

## 最终边界输出

### 完整远场边界

现有 `farfield_boundary` 表示交给后续远场体网格流程的完整边界，而不只是原始 Farfield。修改后包含：

- 原始 `SurfaceBoundaryKind::Farfield` 面；
- 最终 `SurfaceBoundaryKind::BoundaryLayerInterface`；
- 逐层生成的 `SurfaceBoundaryKind::Symmetry` 侧面；
- 逐层生成的 `SurfaceBoundaryKind::Internal` 侧面。

每个滑移侧面保留原始 region ID。

### 边界层顶面

`boundary_layer_top` 仍只从完整边界中提取 `BoundaryLayerInterface`，不包含 Farfield、Symmetry 或 Internal。该输出用于单独观察边界层最外层顶面，而不是表达完整封闭边界。

### 边界层体网格

边界层体网格继续保存生成的体单元。若 VTK 元数据结构支持表面类别，滑移侧面的类别和 region 同步写出；不通过修改体单元类型表达边界条件。

## 错误模型

保留并扩展现有滑移错误：

- `SlidingInputMismatch`：front 引用不存在或类别冲突的 region；
- `InvalidSlidingSurface`：退化面、非法顶点、无法三角化或无法建立加速结构；
- `UndefinedConstrainedDirection`：最终约束方向退化或非有限；
- `OverConstrainedGrowthVertex`：三个独立轴向约束完全锁死顶点；
- 新增曲面投影失败与不收敛错误，包含 layer、顶点、regions、迭代次数、位置变化和最大残差。

构建阶段的全局几何错误使流水线失败。逐层某一顶点的投影失败按局部停止处理，并传播到关联面的停止记录；不得静默使用未投影候选位置继续生长。

## 测试策略

### 输入测试

- 原有 Wall/Far 文件保持通过；
- Symmetry/Internal 段正确映射类别与 Zone region；
- 四种段可任意合法排序；
- 重复 Zone、未知段、非法 ID 和缺失 Zone保持严格报错。

### 几何与投影单元测试

- X、Y、Z 类型识别；
- 轴向跨度容差边界；
- 三轴均不满足时建立 Curved；
- 三角形内部、边和顶点最近点；
- Quad 确定性三角化；
- 曲面方向端点投影；
- 一个轴向面、两个轴向面及三个轴向面过约束；
- 平面与曲面、双曲面的确定性交替投影；
- 投影不收敛、缺失 region、退化几何与非有限输入。

### 正式生长集成测试

- 证明约束发生在 `GrowthFieldSmoother` 之后；
- 最终候选位置发生二次投影；
- X/Y/Z 坐标经过多层生长无累计漂移；
- 曲面节点经过多层生长保持在容差范围内；
- Symmetry 与 Internal 属性逐层继承；
- 两类表面均从原始碰撞索引排除；
- 投影失败产生局部停止而不是越过滑移面。

### 侧面与输出测试

- 单条滑移边每层生成一个正确朝向 Quad；
- Symmetry Quad 保留 Symmetry 和 region；
- Internal Quad 保留 Internal 和 region；
- 相邻体单元内部重复面抵消；
- 局部停止和过渡区无退化面、重复面或边界孔洞；
- `farfield_boundary` 包含 Farfield、BoundaryLayerInterface、Symmetry 和 Internal；
- `boundary_layer_top` 只包含 BoundaryLayerInterface；
- 现有 Wall/Farfield 和过渡流水线不回归。

## 非目标

本阶段不做以下工作：

- 直接复制 BLMesh、BLNode、libigl AABB 或旧二叉树类型；
- 周期边界处理；
- 将 Symmetry 与 Internal 合并成同一个公开边界枚举；
- 让 Symmetry/Internal 面进入 Wall GrowthFront；
- 将 Symmetry/Internal 当作原始实体碰撞障碍；
- 修改未涉及本功能的多法向或过渡模板策略。

## 验收标准

给定含 Wall、Farfield、Symmetry 和 Internal Zone 的合法 CGNS 表面及映射文件：

1. 输入类别和 region 被正确保留；
2. Wall 边界层顶点在每次方向/步长光滑后受对应滑移几何约束；
3. 每个最终候选点在质量和碰撞判断前投影到全部关联滑移面；
4. 轴对齐面和离散曲面均可多层稳定生长；
5. 原始 Symmetry/Internal 不阻挡生长；
6. 每层生成拓扑正确、类别正确的 Symmetry/Internal 侧面；
7. 完整远场边界包含滑移侧面，边界层顶面不包含滑移侧面；
8. 投影或几何失败可诊断且不会产生未约束节点；
9. 全部新增测试及现有测试通过。
