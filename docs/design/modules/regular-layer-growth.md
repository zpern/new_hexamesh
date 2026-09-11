# Prism/Hexa 等层规则生长设计

## 1. 文档目的

本文定义当前 BoundaryMesh 的规则边界层生长模型。实现从 Wall `GrowthPatch` 和初始 `GrowthFront` 出发，根据逐顶点参数逐层生成 Prism 与 Hexa，并使用体单元质量评价决定每个源面是否继续生长。

本文只描述模块边界、公共数据、状态变化和错误语义，不规定碰撞检测、停止传播以及 Pyramid/Tetra 过渡算法。

## 2. 设计目标

本阶段必须满足以下要求：

1. 每个 Wall 源顶点独立接收首层高度、增长率和请求层数；
2. 一次只预生成一层，使后续碰撞检测可以插入候选生成与提交之间；
3. Triangle 生成 Prism，Quad 生成 Hexa；
4. 候选单元在提交前调用阶段 04 的质量评价；
5. 一个源面质量不合格时只停止该面，不直接停止相邻面；
6. 最终返回独立的 `VolumeMesh`，并显式记录源顶点与各层体网格顶点的对应关系；
7. 区分正常完成、局部停止和程序级错误；
8. 每层以事务方式提交，程序级错误不得产生半层结果。

## 3. 非目标

规则层生成本身不负责：

- 对称方向约束和候选位置投影；
- 候选层与原始表面、已有体网格或当前前沿的碰撞检测；
- 相邻源面之间的停止传播和层数协调；
- 局部停止后开口的 Pyramid/Tetra 过渡；
- PLY、VTK 或命令行输入输出；
- 向调用方已有的 `VolumeMesh` 直接追加数据。

阶段 03 已有的对称约束数据保留。约束生长将在后续设计中通过独立约束器接入单层推进过程，本阶段不调用它。

## 4. 生长参数

### 4.1 逐顶点参数

```cpp
struct VertexGrowthProfile
{
    Scalar first_height{};          // 第一层生长步长
    Scalar growth_ratio{1};         // 相邻层生长步长的倍率
    std::uint32_t layer_count{};    // 外部请求的最大生长层数
};

struct SourceVertexGrowthProfile
{
    VertexId source_vertex_id{};    // GrowthPatch 中的源 Wall 顶点编号
    VertexGrowthProfile profile;    // 该源顶点继承的全部生长参数
};
```

第 `k` 层的点步长为：

```text
height(k) = first_height × growth_ratio^(k - 1), k >= 1
```

例如：

```text
第 1 层：first_height
第 2 层：first_height × growth_ratio
第 3 层：first_height × growth_ratio²
```

允许 `growth_ratio < 1`、`growth_ratio == 1` 和 `growth_ratio > 1`。`layer_count == 0` 合法，表示该顶点不请求生成边界层。

### 4.2 参数完整性

每个 `GrowthPatch` 顶点必须恰好对应一条输入参数：

- 缺失参数是错误；
- 同一源顶点重复输入是错误；
- 输入不属于当前 Patch 的源顶点是错误；
- `first_height` 必须是有限正数；
- `growth_ratio` 必须是有限正数；
- 计算得到的某层步长必须是有限数。

参数整理结果应建立按源顶点查询的参数表，后续推进器不重复扫描原始输入数组。

## 5. 模块划分

### 5.1 GrowthProfileBuilder

`GrowthProfileBuilder` 只负责验证外部逐点参数并建立查询表。它不计算前沿方向、不生成候选顶点，也不修改网格。

### 5.2 RegularLayerStepper

`RegularLayerStepper` 一次只尝试生成一层。它接收当前 Front、当前逐面状态和参数查询表，返回本层候选结果，不直接修改最终 `VolumeMesh`。

单层推进器负责：

1. 根据逐点 `layer_count` 筛选完整活动面；
2. 在计算方向前移除已达到层数上限的面；
3. 对剩余活动 Front 重新计算面几何和角度加权节点方向；
4. 根据各点当前层步长产生候选位置；
5. 构造候选 Prism/Hexa；
6. 调用阶段 04 的质量评价；
7. 汇总合格面、停止面、完成面和下一层 Front。

### 5.3 RegularLayerGenerator

`RegularLayerGenerator` 管理完整生长过程。它初始化第 0 层，循环调用 Stepper，并把成功的单层结果原子提交到最终输出。

Generator 负责：

- 初始化 `VolumeMesh`、`LayerVertexTable` 和生长记录；
- 循环推进直到没有活动面；
- 只保存至少被一个合格面引用的候选顶点；
- 为合格 Prism/Hexa 写入单元元数据；
- 更新逐点和逐面实际接受层数；
- 在程序级错误出现时放弃当前整层。

## 6. 面与顶点的活动规则

### 6.1 顶点资格

设当前 Front 表示第 `current_layer` 层。源顶点能够生成下一层的条件是：

```text
current_layer < profile.layer_count
```

### 6.2 面资格

Triangle 或 Quad 只有在其所有顶点都能生成下一层时，整个面才是本层合格候选面。不能生成完整 Prism/Hexa 的面不得部分推进。

例如，一个 Triangle 的三个顶点请求层数分别为 `4/5/6`：

- 第 1 至第 4 层生成完整 Prism；
- 尝试第 5 层时，第一个顶点已达到上限；
- 该 Triangle 正常完成，只接受 4 层；
- 其余两个顶点只有仍被其他活动面引用时，才能通过其他面继续生长。

### 6.3 局部质量停止

候选单元质量不合格时，只停止对应源面：

- 不提交该面的候选单元；
- 该面不进入下一层 Front；
- 共用顶点仍可被其他合格面引用并继续生长；
- 候选顶点只有至少被一个合格面引用时才写入最终网格；
- 停止面与继续面之间形成的台阶或开口留给后续过渡阶段。

停止面必须在下一层方向计算前移除，不能继续影响活动顶点的角度加权方向。

## 7. 生长方向和候选位置

生长方向固定使用当前活动 Front 面顶点顺序的右手法向，不提供反向生长配置。

每层都必须基于当前活动 Front 的实际坐标重新计算：

1. 面法向和面几何；
2. 当前活动关联面的角度权重；
3. 节点生长方向。

候选位置为：

```text
candidate = current_position + current_step × current_direction
```

本阶段不执行对称方向约束或位置投影。

## 8. 体单元顶点顺序

公共顶点顺序必须与阶段 04 的质量评价完全一致，Stepper 不得通过交换顶点掩盖候选单元翻转。

### 8.1 Triangle 到 Prism

当前层 Triangle 为 `v0, v1, v2`，下一层对应点为 `v0', v1', v2'`：

```cpp
Prism{
    v0, v1, v2,
    v0_prime, v1_prime, v2_prime
};
```

底面为 `0-1-2`，顶面为 `3-4-5`，生长边为 `0-3`、`1-4`、`2-5`。

### 8.2 Quad 到 Hexa

当前层 Quad 为 `v0, v1, v2, v3`，下一层对应点为 `v0', v1', v2', v3'`：

```cpp
Hexa{
    v0, v1, v2, v3,
    v0_prime, v1_prime, v2_prime, v3_prime
};
```

底面为 `0-1-2-3`，顶面为 `4-5-6-7`，生长边为 `0-4`、`1-5`、`2-6`、`3-7`。

## 9. 质量接受规则

阶段 05 复用 `VolumeCellQualityOptions`，其中最大 skewness 由外部输入。

调用阶段 04 后存在两类结果：

```text
评价成功且 acceptable == false
    → 正常局部停止
    → 写入 FaceGrowthRecord
    → 继续处理其他面

评价返回 failure(error)
    → 程序级错误
    → 当前整层不提交
    → Generator 返回 failure
```

正常局部停止原因包括候选单元退化、整体反转、局部翻转和 skewness 超限。

## 10. 单层事务

单层推进结果概念上包含：

```cpp
struct LayerStepResult
{
    std::uint32_t layer{};                              // 本次尝试生成的目标层号
    GrowthFront next_front;                             // 仅由合格面组成的下一层 Front
    std::vector<std::size_t> previous_front_vertex_indices; // 下一层点对应的上一层局部点下标
    std::vector<std::size_t> previous_front_face_indices;   // 下一层面对应该层输入面的下标
    std::vector<FaceStopEvent> stopped_faces;           // 因质量不合格停止的面
    std::vector<FaceStopEvent> completed_faces;         // 因顶点层数上限完成的面
};
```

该结构表达的是逻辑职责，具体实现可补充候选单元和提交所需映射，但不得让 Stepper 直接修改调用方的最终网格。

单层流程为：

```text
当前 Front
→ 按 layer_count 筛选完整活动面
→ 移除达到上限的面
→ 对 EligibleFront 重新计算方向
→ 生成候选点
→ Triangle 生成 Prism，Quad 生成 Hexa
→ 阶段 04 质量评价
→ 仅由合格面构造 next_front
→ Generator 原子提交该层
```

任何程序级错误发生时，本层候选顶点、候选单元、层映射和接受层数都不得部分写入最终结果。

## 11. 最终输出

```cpp
struct LayerVertexRecord
{
    VertexId source_vertex_id{};              // 输入 Wall 顶点编号
    std::vector<VertexId> layer_vertex_ids;   // 从第 0 层开始的实际体网格顶点编号
};

using LayerVertexTable = std::vector<LayerVertexRecord>;

struct RegularLayerGrowthResult
{
    VolumeMesh mesh;                          // 独立的规则边界层体网格
    LayerVertexTable layer_vertices;          // 源顶点到实际层顶点的显式映射
    std::vector<VertexGrowthRecord> vertices; // 逐源顶点请求值和实际接受层数
    std::vector<FaceGrowthRecord> faces;       // 逐源面状态、层数和停止原因
};
```

`VolumeMesh` 包含第 0 层 Wall 顶点和所有被合格单元引用的后续顶点。层顶点编号不得通过固定数量公式隐式推导。

## 12. 状态记录

```cpp
enum class FaceGrowthStatus
{
    Active,     // 仍可尝试生成下一层
    Completed,  // 因请求层数上限正常完成
    Stopped     // 因候选单元质量不合格提前停止
};

enum class FaceStopReason
{
    None,                       // 未停止
    VertexLayerLimit,           // 至少一个顶点达到请求层数上限
    DegenerateCandidate,        // 候选单元退化
    ReversedCandidate,          // 候选单元整体反转
    LocallyInvertedCandidate,   // 候选单元局部翻转
    SkewnessExceeded            // 候选单元最大 skewness 超限
};

struct FaceGrowthRecord
{
    SurfaceFaceId source_face_id{};        // 对应输入 Wall 面编号
    std::uint32_t accepted_layer_count{};  // 实际提交的规则层数量
    FaceGrowthStatus status{};             // 当前最终状态
    FaceStopReason stop_reason{};           // 完成或停止原因
    std::uint32_t stop_layer{};             // 首个未被接受的目标层号
};

struct VertexGrowthRecord
{
    VertexId source_vertex_id{};            // 对应输入 Wall 顶点编号
    VertexGrowthProfile profile;            // 外部请求的完整参数
    std::uint32_t accepted_layer_count{};   // 实际进入最终网格的最大层数
};
```

外部请求层数与实际接受层数必须分别保留。

## 13. 错误模型

参数错误单独建模：

```cpp
using GrowthProfileError = std::variant<
    MissingVertexGrowthProfile,
    DuplicateVertexGrowthProfile,
    UnknownVertexGrowthProfile,
    InvalidFirstHeight,
    InvalidGrowthRatio>;
```

高层错误保留底层诊断：

```cpp
using RegularLayerGrowthError = std::variant<
    GrowthProfileFailure,
    FrontEvaluationFailure,
    GrowthDirectionFailure,
    CellEvaluationFailure,
    NonFiniteLayerHeight,
    InvalidLayerFrontMapping,
    VolumeVertexIdOverflow>;
```

包装错误必须附带适用的源顶点、源面和层号。达到层数上限、候选质量不合格以及没有活动面都属于正常结果，不返回 `failure`。

## 14. 公共入口

```cpp
struct RegularLayerGrowthOptions
{
    VolumeCellQualityOptions cell_quality; // 阶段 04 的候选单元质量参数
};

Result<RegularLayerGrowthResult, RegularLayerGrowthError>
generateRegularLayers(
    const GrowthPatch &patch,
    const GrowthFront &initial_front,
    const std::vector<SourceVertexGrowthProfile> &profiles,
    const RegularLayerGrowthOptions &options);
```

最高循环层数不重复作为独立参数输入。Generator 根据逐点 `layer_count` 和逐面活动状态自然结束。

## 15. 测试范围

### 15.1 参数测试

- 缺失、重复和多余参数；
- 非法 `first_height` 和 `growth_ratio`；
- 小于、等于和大于 1 的增长率；
- `layer_count == 0`；
- 多层步长计算产生 NaN 或 Inf。

### 15.2 基本生长测试

- 单 Triangle 多层 Prism；
- 单 Quad 多层 Hexa；
- Triangle/Quad 混合 Front；
- 各顶点使用不同首层高度和增长率；
- 固定右手生长方向、公共顶点顺序和正有向体积；
- `LayerVertexTable`、单元层号和源面元数据。

### 15.3 局部停止测试

- Triangle 顶点层数为 `4/5/6` 时只生成 4 层；
- 达到层数上限后正常完成；
- 退化、整体反转、局部翻转和 skewness 超限只停止当前面；
- 共享顶点的一面停止、另一面继续；
- 无合格面引用的候选顶点不提交；
- 局部开口不在本阶段生成过渡单元。

### 15.4 事务测试

- Stepper 不修改输入 Front；
- Generator 在某层发生程序级错误时不提交半层；
- 所有面完成或停止后自动退出；
- VertexId 溢出被明确报告。

真实 PLY 案例留到正式 IO、碰撞与过渡模块具备后的集成阶段。

## 16. 后续扩展点

阶段 06 在候选生成与提交之间加入碰撞检查，不改变 Stepper 的单层事务边界。后续对称约束可以作为独立候选位置约束器接入。局部层数协调、停止传播和 Pyramid/Tetra 填充分别由后续阶段处理。
