# 各向同性高度停止设计

## 1. 目的

边界层逐层生长时，候选 Prism 或 Hexa 的侧向生长尺度可能逐渐接近当前底面的面内尺度。此时继续生成高长宽比边界层单元的意义已经降低，应保留当前候选单元，并停止对应源面后续生长。

本功能通过可配置的无量纲阈值 `isotropic_height` 判断是否达到各向同性状态。

## 2. 判据

对已经通过体单元质量检查的候选单元计算：

```text
isotropic_ratio = average_side_length / sqrt(current_base_area)
```

其中：

- Prism 使用 3 条底面顶点到对应顶面顶点的侧边欧氏长度平均值；
- Hexa 使用 4 条底面顶点到对应顶面顶点的侧边欧氏长度平均值；
- `current_base_area` 是当前层活动前沿底面的动态面积，不使用初始 Wall 面面积；
- 三角形和四边形面积通过现有统一表面评估函数获得。

停止条件为：

```text
isotropic_ratio >= isotropic_height
```

相等时同样停止后续生长。

## 3. 接受和停止语义

各向同性判据在候选单元通过质量检查后执行。达到阈值不否定当前候选单元：

1. 当前 Prism 或 Hexa 写入最终体网格；
2. 当前顶面写入最终外露边界；
3. 对应源面不进入下一层 `GrowthFront`；
4. 当前单元成为该源面的最后一个边界层单元；
5. 停止原因记录为 `FaceStopReason::IsotropicHeightReached`；
6. 该停止参与现有 `max_neighbor_layer_difference` 邻域传播。

若当前接受的是第 `L` 层，则：

```text
accepted_layer_count = L
stop_layer = L + 1
```

`stop_layer` 表示第一个未接受、也不会再尝试的层号。

## 4. 数据流与职责

`RegularLayerStepper` 负责在质量检查通过后计算各向同性比值，并将结果明确分为：

```cpp
continuing_faces          // 当前单元接受，顶面进入下一层
isotropic_stopped_faces  // 当前单元接受，但在本层终止
```

`LayerStepResult` 必须同时携带这两类结果，避免把“已经接受但终止”的面混入普通失败面。普通失败面没有可提交单元，而各向同性停止面必须提交当前单元。

`RegularLayerGenerator` 负责：

- 提交两类结果对应的体单元；
- 只用 `continuing_faces` 构建下一层活动前沿；
- 将两类结果的顶面都交给外露边界跟踪逻辑；
- 对 `isotropic_stopped_faces` 登记停止原因并触发邻域层差传播。

不采用“先进入下一层再停止”的方式，以免产生无意义的下一层尝试和延迟的停止层语义；也不在 Generator 中重复计算几何判据。

## 5. 外部参数

生成配置新增：

```cpp
double isotropic_height{1.0};
```

独立命令行程序新增可选参数：

```text
--isotropic-height <value>
```

默认值为 `1.0`。输入必须同时满足：

```text
isfinite(isotropic_height) && isotropic_height > 0
```

零、负数、NaN 和无穷值均作为参数错误处理，生成流程不得启动。

命令行配置摘要输出：

```text
isotropic_height=1
```

最终停止统计增加：

```text
stop_isotropic_height=<count>
```

`count` 表示直接因各向同性判据停止的源面数量，不包含随后因邻域层差约束停止的面。新增停止原因后，所有按枚举数量建立的固定数组和输出映射必须同步扩展，避免索引越界。

## 6. 错误处理

- 参数非法：在命令行或公共配置验证入口返回明确错误；
- 当前底面评估失败：沿用现有前沿几何失败处理，不尝试计算比值；
- 当前底面面积非正或非有限：沿用现有退化面停止原因；
- 侧边长度或最终比值非有限：视为候选几何无效，不归类为各向同性停止；
- 质量检查失败优先于各向同性判断，不能保留质量不合格单元。

## 7. 测试范围

单元和集成测试必须覆盖：

1. Prism 比值小于阈值时继续生长；
2. Prism 比值等于阈值时保留当前单元并停止；
3. Prism 比值大于阈值时保留当前单元并停止；
4. Hexa 使用 4 条对应侧边的平均长度；
5. 判据使用当前层动态底面面积，而不是初始 Wall 面面积；
6. 停止后的当前体单元和最终外露顶面均被保留；
7. 停止面不进入下一层 `GrowthFront`；
8. `IsotropicHeightReached`、`accepted_layer_count` 和 `stop_layer` 记录正确；
9. 各向同性停止能够触发现有邻域层差传播；
10. 默认阈值为 `1.0`，命令行能够覆盖默认值；
11. 零、负数、NaN 和无穷阈值均被拒绝；
12. 新增停止原因后的统计数组和文本输出不存在越界或遗漏。

实现完成后必须运行全部 Debug 和 Release 回归测试。

## 8. 范围边界

本次功能只增加各向同性高度停止，不改变：

- 质量评价算法及 skewness 阈值；
- 碰撞检测算法；
- 步长和法向平滑算法；
- 邻域传播的最大允许层数差定义；
- Pyramid/Tetra 过渡单元生成。
