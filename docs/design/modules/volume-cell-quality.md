# Prism/Hexa 候选单元质量评价设计

## 1. 目标

阶段 04 为边界层预推出流程提供独立的候选体单元评价模块。阶段 05 在正式提交每个 Prism 或 Hexa 之前，将候选顶点坐标传给本模块；本模块返回几何有效性、Jacobian 诊断、有向体积和 equiangular skewness。阶段 05 根据结果接受候选单元，或者丢弃候选并停止对应面片继续生长。

本阶段只支持 Prism 和 Hexa。Tetra 与 Pyramid 的质量评价延后到阶段 09，避免在过渡模板尚未确定时提前固定接口。

## 2. 模块边界

新增 CMake target：

```text
BoundaryMesh::Quality
```

依赖方向：

```text
BoundaryMesh::Quality
    ├── BoundaryMesh::Core
    └── BoundaryMesh::Surface
```

`Quality` 不依赖 `GrowthFront`、`GrowthPatch`、`VolumeMesh` 或任何停止状态。它不写日志、不修改全局数据、不启动线程，并且在单元评价期间不进行动态内存分配。

`Surface` 提供三角形和四边形的无状态 equiangular skewness 算法，避免 Prism 与 Hexa 重复实现面角公式。

阶段 05 负责：

- 生成候选层顶点；
- 从 Triangle 构造候选 Prism；
- 从 Quad 构造候选 Hexa；
- 对活动面进行批量并行调度；
- 调用质量评价函数；
- 接受候选或停止对应面片；
- 将接受的顶点和单元写入正式网格。

候选质量不合格时直接停止该面片，本阶段和阶段 05 均不尝试缩短本层高度后重试。

## 3. 公开数据结构

候选单元直接使用连续坐标，不通过正式网格索引：

```cpp
using PrismPoints = std::array<Point3, 6>;
using HexaPoints = std::array<Point3, 8>;
```

配置：

```cpp
struct VolumeCellQualityOptions
{
    Scalar relative_jacobian_tolerance{1e-12};
    Scalar relative_length_tolerance{1e-12};
    Scalar maximum_skewness{0.95};
};
```

几何有效性：

```cpp
enum class VolumeCellValidity
{
    Valid,
    Degenerate,
    Reversed,
    LocallyInverted
};
```

Jacobian 采样位置：

```cpp
enum class JacobianSampleKind
{
    Vertex,
    Center,
    IntegrationPoint
};

struct JacobianSampleLocation
{
    JacobianSampleKind kind{JacobianSampleKind::Center};
    std::size_t index{};
};
```

完整评价：

```cpp
struct VolumeCellEvaluation
{
    VolumeCellValidity validity{VolumeCellValidity::Degenerate};
    Scalar signed_volume{};
    Scalar minimum_jacobian{};
    Scalar maximum_jacobian{};
    Scalar minimum_normalized_jacobian{};
    Scalar maximum_normalized_jacobian{};
    Scalar skewness{};
    JacobianSampleLocation worst_jacobian_location;
    bool acceptable{};
};
```

接受条件固定为：

```cpp
acceptable =
    validity == VolumeCellValidity::Valid &&
    skewness <= options.maximum_skewness;
```

原始 Jacobian 和有向体积保留物理量纲。归一化 Jacobian 只用于稳定判断方向和退化，不增加新的质量等级；本阶段唯一的质量指标是 skewness。

## 4. 公开函数

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

两个函数无状态、可重入且线程安全。阶段 05 可以在外部并行调用，但单次调用内部不得创建线程。

## 5. 公共顶点顺序

阶段 04 将以下顺序写入 `mesh_volume.hpp`，作为后续所有模块的公共不变量。

### 5.1 Prism

```text
底面：0, 1, 2
顶面：3, 4, 5
生长边：0→3、1→4、2→5
```

参考坐标：

```text
0 = (0, 0, -1)    3 = (0, 0, 1)
1 = (1, 0, -1)    4 = (1, 0, 1)
2 = (0, 1, -1)    5 = (0, 1, 1)
```

参数域满足 `r >= 0`、`s >= 0`、`r+s <= 1`、`-1 <= t <= 1`。

### 5.2 Hexa

```text
底面：0, 1, 2, 3
顶面：4, 5, 6, 7
生长边：0→4、1→5、2→6、3→7
```

参考坐标：

```text
0 = (-1,-1,-1)    4 = (-1,-1, 1)
1 = ( 1,-1,-1)    5 = ( 1,-1, 1)
2 = ( 1, 1,-1)    6 = ( 1, 1, 1)
3 = (-1, 1,-1)    7 = (-1, 1, 1)
```

标准直 Prism 与标准 Hexa 在上述顺序下必须产生正 Jacobian。

## 6. Jacobian 采样

形函数把参考坐标映射到实际坐标：

```text
x(r,s,t) = Σ Ni(r,s,t) xi
```

Jacobian 的三列为实际坐标对三个参考坐标的偏导。每个检查点计算原始行列式：

```cpp
determinant = J.determinant();
```

### 6.1 Prism 检查点

- 六个参考顶点；
- 中心 `(1/3, 1/3, 0)`；
- 三角形三点积分规则 `(1/6,1/6)`、`(2/3,1/6)`、`(1/6,2/3)` 与厚度方向 `t=±1/sqrt(3)` 的笛卡尔组合。

共 13 个检查位置，其中六个积分点同时用于有向体积。

### 6.2 Hexa 检查点

- 八个参考顶点；
- 中心 `(0,0,0)`；
- `r,s,t` 分别取 `±1/sqrt(3)` 的八个 `2×2×2` 高斯点。

共 17 个检查位置，其中八个高斯点同时用于有向体积。

采样顺序必须固定，使 `worst_jacobian_location` 在不同运行和平台上保持确定性。最差位置按最小归一化 Jacobian 选择；数值相同时保留采样顺序中较早的位置。

## 7. 归一化与数值容差

边界层单元可能很薄，禁止使用“最大边长的三次方”作为唯一 Jacobian 容差，否则会误拒绝高长宽比但方向正确的单元。

对 Jacobian 三列 `a,b,c` 计算：

```text
local_scale = |a| |b| |c|
normalized_jacobian = det(J) / local_scale
```

若任一列长度为零、非有限数，或者三者乘积发生数值下溢，则该检查点直接标记为退化。不得使用单元最大边长去单独限制某一 Jacobian 列，否则会误拒绝方向正确的薄层单元。其余情况按无量纲相对容差判断：

```text
normalized >  tolerance  → positive
normalized < -tolerance  → negative
其他                     → degenerate
```

极薄但正交的边界层单元虽然原始 `det(J)` 很小，归一化值仍接近 1，因此不会仅因层高很小而被拒绝。

单元特征长度定义为候选顶点之间的最大两点距离。面角计算前使用特征长度乘以 `relative_length_tolerance` 判断数值上不可分辨的零长度边；若特征长度本身为零，整个单元直接退化。点积除法后的余弦必须限制到 `[-1,1]`。这个长度容差只服务坐标差和面角稳定性，不参与 Jacobian 列的高宽比限制。

## 8. 有向体积

有向体积只通过积分点 Jacobian 计算：

```text
signed_volume = Σ weight_i det(J_i)
```

Prism 使用三角形三点规则乘以厚度方向两点 Gauss 规则；Hexa 使用 `2×2×2` Gauss 规则。不再将单元拆成四面体，也不为体积重复计算 Jacobian。

有向体积是整体诊断量。几何有效性仍以所有规定检查点的 Jacobian 符号为主，因为总体积为正不能排除局部折叠。

## 9. Equiangular skewness

三角形理想角为 `pi/3`，四边形理想角为 `pi/2`。单面 skewness 为：

```text
max(
    (theta_max - theta_ideal) / (pi - theta_ideal),
    (theta_ideal - theta_min) / theta_ideal)
```

结果限制在 `[0,1]`。

Prism 评价两个三角形面和三个四边形面；Hexa 评价六个四边形面。体单元 skewness 取所有面的最大值。

实现先收集角余弦，利用 `acos` 的单调性只对最小角和最大角调用 `acos`：Hexa 至多两次，Prism 对三角形和四边形各两次。若存在零长度边，单元标记为退化，skewness 固定为 1，`acceptable=false`。

## 10. 有效性分类

所有检查点归纳出 `has_positive`、`has_negative` 和 `has_degenerate`。分类优先级固定为：

```text
同时有正和负             → LocallyInverted
无正负混合但存在退化点   → Degenerate
全部为负                 → Reversed
全部为正                 → Valid
```

正负混合即使同时包含零点，也优先报告 `LocallyInverted`，因为该状态最明确地说明单元内部发生折叠。

几何无效与质量不合格必须区分：

```text
validity != Valid                    → 几何无效
validity == Valid 且 skewness 超限   → 几何有效但质量不合格
```

阶段 05 对两种结果都丢弃候选并停止对应面片，但记录不同停止原因。

## 11. 错误模型

以下属于调用或数值错误，返回 `Result::failure`：

- 容差不是有限正数；
- `maximum_skewness` 不在 `[0,1]`；
- 输入顶点包含 NaN 或无穷值；
- 中间计算产生非有限数值。

错误类型携带配置值、单元类型、局部顶点下标或 Jacobian 采样位置。

以下属于正常候选拒绝，返回 `Result::success(evaluation)`：

- Jacobian 退化；
- 整体反转；
- 局部翻转；
- skewness 超过阈值。

库代码不打印日志，也不通过异常跨越公开 API。

## 12. 文件组织

```text
include/boundary_mesh/surface/face_skewness.hpp
src/surface/face_skewness.cpp

include/boundary_mesh/quality/volume_cell_evaluation.hpp
include/boundary_mesh/quality/volume_cell_evaluation_error.hpp
include/boundary_mesh/quality/volume_cell_evaluator.hpp

src/quality/prism_evaluator.cpp
src/quality/hexa_evaluator.cpp
src/quality/volume_cell_evaluator_common.cpp
```

Prism/Hexa 的形函数和参考采样点分别保存在各自实现文件中；配置验证、归一化、符号分类和结果组装放在公共实现中。所有新增公共字段和枚举值必须带中文行尾注释。

## 13. 测试

单元测试覆盖：

- 理想三角形和四边形的 skewness 为 0；
- skewness 公式、退化边和 `[0,1]` 范围；
- 标准 Prism 体积 0.5、全正 Jacobian、skewness 0；
- 单位 Hexa 体积 1、全正 Jacobian、skewness 0；
- 极薄但正交的 Prism/Hexa 仍有效；
- 整体顶点顺序反转得到 `Reversed`；
- 单个顶点折入得到 `LocallyInverted`；
- 上表面压到底面得到 `Degenerate`；
- 几何有效但 skewness 超限时 `acceptable=false`；
- NaN、Infinity 和非法配置精确返回错误；
- 坐标整体放大或缩小时有效性和 skewness 不变；
- 最差 Jacobian 位置具有稳定索引。

集成测试只模拟阶段 05 的预推出调用：构造候选坐标、调用 evaluator、验证合格候选可提交且不合格候选会被拒绝。本阶段不真正写入 `VolumeMesh` 或改变 `GrowthFront`。

性能基准以独立可执行程序提供，不设置机器相关的 CTest 时间阈值。基准使用 Release 构建批量重复评价固定候选并报告吞吐量；无动态分配和 Jacobian 复用由代码结构与审查保证。

## 14. 完成边界

阶段 04 完成时：

- `BoundaryMesh::Quality` 可以独立评价单个候选 Prism/Hexa；
- Jacobian、体积、skewness 和接受结论来自同一次确定性计算；
- 函数可由阶段 05 在外部安全并行调用；
- 所有单元测试、集成测试和既有回归测试通过。

阶段 04 不实现候选生成、体网格提交、前沿更新、面片停止传播、碰撞检测、层高缩短重试以及 Tetra/Pyramid 评价。
