# Prism/Hexa 候选单元有向体积与质量评价设计

## 1. 目标

阶段 04 为阶段 05 的预推出流程提供独立的候选体单元评价函数。阶段 05 在正式提交 Prism 或 Hexa 前，将候选顶点坐标传入本模块；本模块使用固定子四面体有向体积判断退化、整体反转和局部翻转，并使用组成面的 equiangular skewness 判断质量是否合格。

本阶段只支持 Prism 和 Hexa。Tetra 与 Pyramid 延后到阶段 09，并根据实际过渡模板单独设计。

本设计明确不使用 Jacobian、归一化 Jacobian、形函数采样或高斯积分。

## 2. 模块边界

```text
BoundaryMesh::Quality
    ├── BoundaryMesh::Core
    └── BoundaryMesh::Surface
```

`Quality` 只接收单个候选单元的连续坐标，不依赖 `GrowthFront`、`GrowthPatch`、`VolumeMesh` 或面片停止状态。它不写日志、不创建线程、不修改全局数据，单次评价不进行动态内存分配。

阶段 05 负责批量并行、候选接受、正式网格写入和停止状态传播。

## 3. 公共顶点顺序

### 3.1 Prism

```text
底面：0, 1, 2
顶面：3, 4, 5
生长边：0→3、1→4、2→5
```

标准正向 Prism：

```text
0=(0,0,0)  1=(1,0,0)  2=(0,1,0)
3=(0,0,1)  4=(1,0,1)  5=(0,1,1)
```

### 3.2 Hexa

```text
底面：0, 1, 2, 3
顶面：4, 5, 6, 7
生长边：0→4、1→5、2→6、3→7
```

标准正向 Hexa：

```text
0=(0,0,0)  1=(1,0,0)  2=(1,1,0)  3=(0,1,0)
4=(0,0,1)  5=(1,0,1)  6=(1,1,1)  7=(0,1,1)
```

## 4. 固定子四面体分解

Prism 固定拆成三个四面体：

```text
0: (0,1,2,3)
1: (1,2,3,4)
2: (2,3,4,5)
```

上述子四面体在标准 Prism 中均为正，总体积为 `0.5`。

Hexa 固定沿体对角线 `0–6` 拆成六个四面体：

```text
0: (0,1,2,6)
1: (0,2,3,6)
2: (0,3,7,6)
3: (0,7,4,6)
4: (0,4,5,6)
5: (0,5,1,6)
```

上述子四面体在单位 Hexa 中均为正，总体积为 `1`。

分解顺序属于公共确定性契约。`worst_subtet_index` 永远使用该顺序中的下标。

## 5. 子四面体有向体积

四面体 `(a,b,c,d)` 的六倍有向体积为：

```text
signed_volume_6 = dot(b-a, cross(c-a, d-a))
signed_volume   = signed_volume_6 / 6
```

所有判断严格与零比较，不使用绝对容差或相对容差：

```text
signed_volume_6 > 0  → 正体积
signed_volume_6 < 0  → 负体积
signed_volume_6 == 0 → 退化
```

因此，一个非常薄但计算结果仍为正的候选保持有效；一个几乎压平但结果为很小负数的候选判为翻转，而不是退化。这是本阶段明确采用的浮点语义。

## 6. 有效性分类

依次累计所有子四面体是否出现正、负和零体积，分类优先级固定为：

```text
同时存在正体积和负体积       → LocallyInverted
没有正负混合但存在零体积     → Degenerate
所有子四面体均为负           → Reversed
所有子四面体均为正           → Valid
```

即使同时存在零体积，只要还存在正负混合，仍优先报告 `LocallyInverted`。

总体有向体积等于固定分解中所有子四面体有向体积之和。总体积只用于诊断，不能覆盖子四面体分类；总体积为正不代表不存在局部翻转。

## 7. Equiangular skewness

保留阶段 04 已有的面 equiangular skewness：

```text
Triangle 理想角 = 60°
Quad 理想角     = 90°
```

单面 skewness：

```text
max(
    (theta_max - theta_ideal) / (pi - theta_ideal),
    (theta_ideal - theta_min) / theta_ideal)
```

Prism 取两个三角面和三个四边形面的最大值；Hexa 取六个四边形面的最大值。结果限制到 `[0,1]`。

面边是否退化同样严格与零比较，调用面算法时长度容差固定为 `0`。若子四面体分类已经无效且面角计算遇到零长度边，则 skewness 记为 `1`，作为正常候选拒绝，不升级为程序错误。

## 8. 公共数据结构

```cpp
using PrismPoints = std::array<Point3, 6>;
using HexaPoints = std::array<Point3, 8>;

struct VolumeCellQualityOptions
{
    Scalar maximum_skewness{0.95}; // 可接受候选的最大面偏斜度
};
```

```cpp
enum class VolumeCellValidity
{
    Valid,          // 所有子四面体均为正
    Degenerate,     // 无正负混合，但至少一个子四面体为零
    Reversed,       // 所有子四面体均为负
    LocallyInverted // 子四面体同时出现正体积和负体积
};
```

```cpp
struct VolumeCellEvaluation
{
    VolumeCellValidity validity{VolumeCellValidity::Degenerate}; // 几何有效性分类
    Scalar signed_volume{};                                      // 所有子四面体有向体积之和
    Scalar minimum_subtet_signed_volume{};                       // 最小子四面体有向体积
    Scalar maximum_subtet_signed_volume{};                       // 最大子四面体有向体积
    std::size_t worst_subtet_index{};                            // 最小子体积的最早固定下标
    Scalar skewness{};                                           // 所有组成面的最大偏斜度
    bool acceptable{};                                           // 几何有效且 skewness 不超限
};
```

相同最小子体积出现多次时，保留固定分解顺序中最早的下标。

## 9. 公开接口

```cpp
Result<VolumeCellEvaluation, VolumeCellEvaluationError>
evaluatePrism(
    const PrismPoints &points,
    const VolumeCellQualityOptions &options = {});

Result<VolumeCellEvaluation, VolumeCellEvaluationError>
evaluateHexa(
    const HexaPoints &points,
    const VolumeCellQualityOptions &options = {});
```

两个函数均无状态、可重入且线程安全。

```cpp
acceptable =
    validity == VolumeCellValidity::Valid &&
    skewness <= options.maximum_skewness;
```

## 10. 错误模型

以下属于调用或数值错误，返回 `Result::failure`：

- `maximum_skewness` 不是有限数或不在 `[0,1]`；
- 输入顶点包含 NaN 或 Infinity；
- 坐标差、叉积、点积、除以六或总体积累加产生 NaN/Infinity；
- 对几何有效候选计算 skewness 时产生 NaN/Infinity。

错误只携带错误类别、Prism/Hexa 类别、非法配置值、可选局部顶点下标和可选子四面体下标，不再携带 Jacobian 采样位置。

以下属于正常候选拒绝，返回 `Result::success(evaluation)`：

- 子四面体严格为零；
- 所有子四面体为负；
- 子四面体正负混合；
- skewness 超过阈值。

## 11. 阶段 05 调用流程

```text
预推出候选 Prism/Hexa
        ↓
调用阶段 04 evaluator
        ↓
failure：报告调用或数值错误
success 且 acceptable=true：正式提交候选
success 且 acceptable=false：丢弃候选并停止对应面片生长
```

本阶段不缩短层高重试，也不修改停止传播范围。

## 12. 文件组织

```text
include/boundary_mesh/surface/face_skewness.hpp
src/surface/face_skewness.cpp

include/boundary_mesh/quality/volume_cell_evaluation.hpp
include/boundary_mesh/quality/volume_cell_evaluation_error.hpp
include/boundary_mesh/quality/volume_cell_evaluator.hpp

src/quality/volume_cell_evaluator_common.cpp
src/quality/prism_evaluator.cpp
src/quality/hexa_evaluator.cpp
```

不再保留 Jacobian 形函数、参考采样点、归一化或高斯积分帮助代码。

## 13. 测试

单元测试必须覆盖：

- 理想三角形和四边形 skewness 为零；
- 标准 Prism 三个子体积为正，总体积 `0.5`；
- 单位 Hexa 六个子体积为正，总体积 `1`；
- 极薄但子体积仍为正的 Prism/Hexa 为 `Valid`；
- 所有子体积为负得到 `Reversed`；
- 子体积正负混合得到 `LocallyInverted`；
- 存在严格零子体积且无正负混合得到 `Degenerate`；
- 正负混合与零同时存在时优先得到 `LocallyInverted`；
- 有效但 skewness 超限时 `acceptable=false`；
- NaN、Infinity、配置错误和有向体积溢出返回精确错误；
- 最小子体积相同时保留最早 `worst_subtet_index`；
- 整体平移不改变体积、分类或 skewness；
- 整体缩放后分类和 skewness 不变，总体积按三次方缩放。

集成测试模拟阶段 05 的预推出接受与停止决策，但不写入正式 `VolumeMesh`。

Release benchmark 对固定 Prism/Hexa 重复评价至少一百万次并输出吞吐量，不设置机器相关的 CTest 时间阈值。

## 14. 完成边界

阶段 04 完成时：

- `BoundaryMesh::Quality` 只用固定子四面体有向体积评价 Prism/Hexa 几何有效性；
- 不存在任何 Jacobian、归一化 Jacobian、形函数采样或高斯积分接口与实现；
- 面 skewness 仍作为唯一质量指标；
- Debug 与 Release 单元测试、集成测试和既有回归全部通过；
- 单次评价不动态分配、不写日志、不创建线程。

阶段 04 不负责候选生成、正式体网格提交、前沿更新、碰撞检测、局部停止传播、层高重试或 Tetra/Pyramid 评价。
