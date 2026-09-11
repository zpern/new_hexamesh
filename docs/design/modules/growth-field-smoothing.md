# 生长法向与步长平滑设计

## 1. 目标

本模块统一负责每一层活动前沿的原始法向选择、法向平滑和步长平滑。设计参考旧工程以下实现：

- `BLNode::GetNormal(MBLNode *pNodes, int type)`；
- 非 `OLD` 的 `SimpleNormalSmoothStrategy::SmoothNormalOnce(...)`；
- `NormalSmoothStrategy::SmoothNormal()`；
- 非 `OLD` 的 `BLMesh::SmoothHeightRatio(...)`。

本次改动需要满足：

1. 第 0 层和后续所有层使用同一套多候选原始法向选择算法；
2. Triangle 和 Quad 均可参与法向选择与平滑；
3. 仅使用当前仍能生长的活动前沿建立邻域；
4. 法向平滑尽量复刻参考工程的实际数值；
5. 步长平滑保留参考算法思想，但采用同步更新以消除遍历顺序和线程时序影响；
6. 后续层基准步长继承上一层实际步长；
7. 法向、步长等生成状态只属于活动前沿，不污染通用 `SurfaceMesh` 和 `VolumeMesh`；
8. 复杂角点只做标记，本阶段不实现 Pyramid/Tetra 过渡。

## 2. 数据结构

### 2.1 GrowthFrontVertex

将活动前沿中与一个点相关的平行数组合并为专用点结构：

```cpp
struct GrowthFrontVertex
{
    Point3 position;                    // 当前层坐标
    Point3 root_position;               // 第 0 层 Wall 点坐标
    VertexId source_vertex_id{};        // 输入 Wall 顶点编号
    FrontVertexBoundary boundary;       // 当前点继承的边界约束
    Vector3 direction{Vector3::Zero()}; // 最近一次平滑后的生长方向
    Scalar actual_height{};             // 最近一层真正采用的推出步长
    Scalar visibility_cosine{1};        // 原始法向对关联面的最小点积
    bool complex_corner{};              // 是否属于低可见性复杂角点
};
```

第 0 层的 `direction` 为零向量，`actual_height` 为零。它们表示尚不存在下层继承法向和尚未执行过推出，而不是有效的生长结果。

`root_position` 对应参考工程持续继承的 `DecentID`。法向平滑强度使用当前点到初始 Wall 点的累计空间距离，而不是仅使用上一层步长。

### 2.2 GrowthFront

```cpp
struct GrowthFront
{
    std::uint32_t layer{};                 // 当前层号，第 0 层为输入 Wall
    std::vector<GrowthFrontVertex> vertices; // 当前紧凑活动点
    std::vector<SurfaceFace> faces;        // 索引当前 vertices 的活动面
    std::vector<SurfaceFaceId> source_face_ids; // 活动面到输入 Wall 面的映射
};
```

不保留旧的 `vertices`、`source_vertex_ids` 和 `vertex_boundaries` 三组平行数组兼容接口。本工程不要求保持旧 API 兼容。

### 2.3 FrontAdjacency

每一层从当前活动 `GrowthFront::faces` 构建一次临时邻接：

```cpp
struct FrontAdjacency
{
    std::vector<std::vector<std::size_t>> vertex_neighbors; // 每个点的一环邻点
    std::vector<std::vector<std::size_t>> vertex_incident_faces; // 每个点的活动关联面
};
```

邻点按当前前沿点下标排序并去重。该对象只在当前层有效，不跨层保存。初始 `SurfaceTopology` 虽然能够推导源表面邻接，但不能直接代替它，因为局部停止和层数上限会逐层删除活动面。

## 3. 模块边界

保留并扩展 `computeGrowthDirections()`，由它根据当前活动前沿、面几何评价和邻接完成多候选原始法向选择，并输出原始方向、可见性及复杂角点标记。

新增独立的 `GrowthFieldSmoother`。它输入当前活动前沿、当前邻接、多候选原始法向结果和本层基准步长，输出平滑法向及实际步长。

`GrowthFieldSmoother` 负责：

- 参考数值的法向动态平滑；
- 一轮同步步长平滑；
- 相关输入和输出的有限性、单位化及映射检查。

该模块不负责：

- 修改点坐标；
- 构造 Prism/Hexa；
- 判断体单元质量；
- 碰撞检测；
- 局部停止传播；
- Pyramid/Tetra 过渡。

`RegularLayerStepper` 负责组织本层流程并使用平滑结果预推出。最终 `VolumeMesh` 仍只保存坐标、体单元和体单元元数据。

## 4. 每层处理流程

每层严格按以下顺序执行：

1. 根据面层数约束筛选本层活动面；
2. 将活动面压缩为紧凑 `GrowthFront`；
3. 重新计算当前层所有活动面的几何；
4. 从活动面构建 `FrontAdjacency`；
5. 为每个活动点选择多候选原始法向；
6. 对全部活动点执行动态法向平滑；
7. 根据外部生长参数和上一层实际步长计算本层基准步长；
8. 对本层基准步长执行一轮同步平滑；
9. 使用平滑法向和实际步长预推出候选点；
10. 评价候选 Prism/Hexa 的质量；
11. 质量合格的候选进入后续碰撞过滤；
12. 最终接受点携带本层方向和实际步长进入下一层。

质量或碰撞阶段删除活动面以后不在同一层重新执行平滑。该面及只属于该面的点从下一层开始不再参与邻域。

## 5. 面法向

Triangle 和 Quad 的单位面法向均复用当前 `FrontEvaluation`。

Triangle 遵循输入面顶点顺序的右手法则。Quad 固定沿 `v0-v2` 分成两个三角形：

```text
A1 = 0.5 * (v1 - v0) × (v2 - v0)
A2 = 0.5 * (v2 - v0) × (v3 - v0)
A  = A1 + A2
n  = A / |A|
```

一个 Quad 合成一个单位面法向，在节点法向计算中只贡献一次。不会因为内部拆成两个三角形而获得双倍权重。

参考工程将反向叉积称为外法向，但本工程已经规定沿输入 Wall 面绕序使用右手法则，因此不额外反转现有面法向。

## 6. 多候选原始法向

### 6.1 可见性

对顶点 `i` 的任意单位候选方向 `d`，定义：

```text
visibility(i, d) = min(n_f · d)
```

其中 `n_f` 是该点全部当前活动关联面的单位法向。数值越大，说明候选方向与最不利关联面越一致。

### 6.2 候选顺序和阈值

按参考工程的顺序尝试：

1. 上一层平滑法向：仅第 1 层以后存在，阈值 `cos(30°)`；
2. `SimpleNormal`：阈值 `cos(30°)`；
3. `NaiveNormal`：阈值 `cos(30°)`；
4. `CenterNormal`：阈值 `cos(10°)`；
5. `GeometryNormal`：阈值 `cos(1°)`。

某个候选严格超过对应阈值后立即采用。没有候选达到阈值时，采用所有有效候选中可见性最大的方向。候选包含非有限值或无法单位化时，该候选无效，但不妨碍继续尝试其他候选。全部候选均无效才返回错误。

参考函数的 `type` 参数实际上没有参与分支，本设计不增加无意义的 `type` 参数。

### 6.3 SimpleNormal

```text
d_simple = normalize(Σ n_f)
```

每个活动 Triangle 或 Quad 的单位法向等权贡献一次，不使用面积权重或顶点角度权重。

### 6.4 NaiveNormal

按照活动关联面的稳定顺序，以 `25°` 为阈值将法向归入方向组。每组内部先平均并单位化，不同组的代表法向再等权相加并单位化。这样大量方向近似的细分面不会仅凭数量压倒其他几何方向。

### 6.5 CenterNormal

先寻找点积最小、方向最分离的两个活动关联面法向，再沿参考工程的角点中心构造求得候选方向。

Triangle 在当前顶点处使用其前后两个相邻顶点形成的两条边。Quad 同样只使用当前顶点的前一个和后一个相邻顶点形成的真实边，忽略对角顶点。该扩展修复了参考实现只可靠支持三角形的问题，同时保持中心方向构造含义不变。

### 6.6 GeometryNormal

沿用参考工程的二元组和三元组候选构造方式，并以可见性最大为选择目标。它是复杂角点的后级候选，不替代前面的快速候选。

### 6.7 复杂角点

最终原始方向的可见性写入 `visibility_cosine`。当：

```text
visibility_cosine < cos(30°)
```

时设置 `complex_corner=true`。本阶段继续平滑和预推出，不直接停止；其关联面可由当前 `FrontAdjacency` 推导，供未来 Task 8 的 Pyramid/Tetra 过渡使用。

## 7. 法向平滑

法向平滑参考非 `OLD` 的 `SimpleNormalSmoothStrategy`，但统一采用双缓冲同步更新。

### 7.1 一环权重

对当前点 `i` 和邻点 `j`：

```text
d2_ij            = |p_j - p_i|²
average_d2_i     = average(d2_ij)
distance_ratio   = average_d2_i / d2_ij
normal_alignment = |normalize(p_j - p_i) · n_j|
beita_scale      = visibility_j * 2 / π
influence        = distance_ratio^(3 + 2 * normal_alignment)
                   / beita_scale²
weight           = 0.5 + influence / (1 + influence)
```

加权邻点法向为：

```text
neighbor_normal_i = normalize(Σ weight_ij * n_j)
```

分母接近零、非有限幂运算或零长度邻边均作为明确错误处理，不允许 NaN 传播到候选网格。

### 7.2 平滑强度

当前点局部尺寸取其全部活动关联面的最短边。Triangle 检查三条边，Quad 检查四条边。

```text
cumulative_height = |position - root_position|
height_size_ratio = cumulative_height / minimum_front_size
smoothing_strength = 2 + (1.7^height_size_ratio - 1) * 15
```

随后：

```text
smooth_contribution = smoothing_strength * neighbor_normal
candidate = normalize(original + smooth_contribution)
```

第 1 层生成前累计高度为零，因此平滑强度从参考值 `2` 开始。

### 7.3 最大偏转限制

```text
original_visible_angle = acos(clamp(visibility(original), -1, 1))
maximum_deviation = max(0, (π/2 - original_visible_angle) * 2/3)
minimum_original_alignment = cos(maximum_deviation)
```

若平滑候选偏离原始方向过多，使用固定 40 次二分搜索缩小邻点贡献比例，直到满足最大偏转限制。

### 7.4 可见性回退

参考非 `OLD` 代码实际将：

```text
35 * 0.8 * π / 180 ≈ 0.488692
```

直接与法向点积比较，而不是使用 `cos(28°)`。为复刻实际数值，本实现保留 `0.488692` 这一比较语义。

如果候选可见性低于该值，最多执行 11 次：

```text
candidate = normalize(candidate + 0.7 * current_direction)
```

仍无法满足时恢复到本轮原始方向。

### 7.5 动态迭代

- 内部最多执行 100 轮，不提供外部配置；
- 前 4 轮处理全部活动点；
- 某点本轮新旧法向点积小于 `0.9985` 时，该点及其一环邻点进入下一轮；
- 后续只处理仍需平滑的局部区域；
- 记录每轮待处理点数量；记录数超过 12 后，如果本轮数量与十条记录之前相同，则按参考逻辑提前结束；
- 每轮先读取旧缓冲计算全部新方向，再一次性写入新缓冲；
- 算法结果不得依赖点或邻接遍历顺序。

本阶段不新增对称面处理逻辑，保留工程现有对称数据，不扩大本次改动范围。

## 8. 步长继承

本层基准步长定义为步长平滑之前的初始值。

目标层为第 1 层时：

```text
base_height_i = profile.first_height
```

目标层大于第 1 层时：

```text
base_height_i = current_vertex.actual_height
                * profile.growth_ratio
```

因此后续层继承上一层真正采用的平滑步长，不再使用 `first_height * pow(growth_ratio, layer - 1)`。

## 9. 步长同步平滑

法向平滑完成后，每层只执行一轮步长平滑。

对所有活动点先使用基准步长预测位置：

```text
q_j = p_j + base_height_j * smoothed_direction_j
```

对点 `i`：

```text
predicted_height_i = average((q_j - p_i) · smoothed_direction_i)
relative_difference_i = (predicted_height_i - base_height_i)
                        / base_height_i
correction_i = 1 / (1 + exp(-0.5 * relative_difference_i)) - 0.5
actual_height_i = base_height_i * (1 + correction_i)
```

显式限制：

```text
0.5 * base_height_i <= actual_height_i <= 1.5 * base_height_i
```

实际实现采用完整闭区间 `[0.5, 1.5] * base_height_i`。Logistic 计算使用数值稳定形式，避免大指数溢出。

全部 `q_j` 使用同一个输入缓冲。所有实际步长计算完成后一次性写入，不允许前面顶点的结果影响后面顶点。

## 10. 预推出与状态继承

```text
candidate_position_i = current_position_i
                       + actual_height_i * smoothed_direction_i
```

通过质量和碰撞过滤的下一层点继承：

- `position = candidate_position`；
- `root_position` 不变；
- `source_vertex_id` 不变；
- `boundary` 不变；
- `direction = smoothed_direction`；
- `actual_height = 本层实际步长`；
- 本层重新计算的 `visibility_cosine` 和 `complex_corner`。

一个点同时属于接受面和停止面时，只要仍属于至少一个接受面就进入下一层。只属于停止面的点被紧凑前沿删除。被删除点及被删除面从下一层开始不再参与平滑。

## 11. 错误与停止语义

以下情况返回生成错误并终止本次生成：

- 前沿坐标、根坐标、方向或步长包含非有限值；
- 面顶点编号越界；
- 活动点没有活动关联面或一环邻点；
- 面法向、候选法向或平滑法向无法单位化；
- 上一层实际步长、本层基准步长或平滑步长不是有限正数；
- 源点找不到生长参数；
- 前沿、评价和邻接的层号或规模不一致；
- 权重、指数、Logistic 或几何构造产生非法数值。

公共错误枚举和结构字段添加中文 `//` 注释。错误至少记录层号、当前前沿点下标和 `source_vertex_id`。

低可见性复杂角点不是错误。候选单元退化、反转、局部翻转、skewness 超限、碰撞以及邻面层数差传播继续采用现有逐面局部停止语义。

## 12. 测试设计

### 12.1 邻接测试

- Triangle、Quad 和混合前沿；
- 邻点排序去重；
- 关联面映射；
- 已删除面不参与；
- 非法顶点引用。

### 12.2 原始法向测试

- 平面区域选择 `SimpleNormal`；
- 后续层优先继承上一层法向；
- `NaiveNormal` 的 `25°` 分组；
- Triangle `CenterNormal`；
- Quad 广义 `CenterNormal`；
- `GeometryNormal` 及最大可见性兜底；
- 低可见性复杂角点标记；
- Triangle/Quad 右手方向。

### 12.3 法向平滑测试

- 固定几何的参考数值；
- 邻点距离、方向和可见性的权重；
- 累计高度与最短边控制的强度；
- 最大偏转限制和可见性回退；
- `0.9985` 动态迭代判定；
- 前 4 轮全量、后续局部和 100 轮上限；
- 调整点及邻接遍历顺序后结果不变；
- 输出有限且为单位向量。

### 12.4 步长测试

- 第 1 层使用 `first_height`；
- 后续层使用上一层实际步长乘 `growth_ratio`；
- Logistic 系数 `0.5`；
- 单轮同步更新；
- `[0.5, 1.5]` 限制；
- 活动前沿隔离；
- 非有限或非正步长错误。

### 12.5 集成和回归

- 完整生长前沿流水线；
- 规则层质量和碰撞局部停止；
- Debug 全量构建与 CTest；
- Release 全量构建与 CTest；
- IO 关闭配置构建；
- `2dot5_cf.cgns + .bc.txt` 实际案例；
- VTK 有限坐标、单元数量和停止原因统计；
- 改动前后体单元数、碰撞停止数、运行时间和峰值内存对比。

实现应通过局部单元测试和全量回归验证；本文件只保留当前算法和接口约束，不记录开发流程或提交要求。
