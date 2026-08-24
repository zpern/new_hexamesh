# 步长平滑状态分离设计

## 目标

修正当前步长平滑将上一层修正逐层累乘的问题，使层间步长语义与 `blmesh` 一致：上一层修正只参与本层邻点位置预测，本层最终步长始终基于未修正的理论基准步长。

## 方案比较

1. **分离理论步长和临时预测步长（采用）**：平滑器接收两组数组，数据语义直接、无需在前沿顶点中长期保存修正比例。
2. 在 `GrowthFrontVertex` 中新增 `height_ratio`：更接近 `BLNode`，但会扩大持久状态并增加停止、压缩和映射过程的维护成本。
3. 完全不继承上一层修正：实现最简单，但邻点预测不再复刻 `blmesh`，会丢失上一层局部厚度信息。

## 数据定义

- `reference_heights[i]`：第 `k` 层未修正理论步长，等于顶点配置的 `first_height * growth_ratio^(k - 1)`。
- `provisional_heights[i]`：进入本层时继承上一层修正的临时步长。第一层等于 `reference_heights[i]`；后续层等于上一层 `actual_height * growth_ratio`。
- `actual_heights[i]`：本层平滑后的最终推出步长。

## 算法

1. 使用 `provisional_heights` 和已平滑法向构造邻点预计顶面位置。
2. 将邻点预计位置沿当前节点法向投影并求平均，得到 `predicted_height`。
3. 使用理论步长计算相对差：

   `relative = (predicted_height - reference_height) / reference_height`

4. 保留当前 Logistic 响应：

   `correction = sigmoid(0.5 * relative) - 0.5`

5. 使用乘法生成本层实际步长：

   `actual_height = reference_height * (1 + correction)`

6. 继续将实际步长限制在理论基准步长的 `50%～150%`。

这样本层新修正会替换上一层修正，而不会与旧修正连乘。

## 接口和错误处理

`GrowthFieldSmoother::smooth` 将原有单个 `base_heights` 参数拆为：

```cpp
const std::vector<Scalar> &reference_heights,
const std::vector<Scalar> &provisional_heights
```

两组数组都必须与活动前沿顶点数一致，且所有值有限并大于零。任一条件不满足时沿用现有 `GrowthFieldSmoothingError` 返回机制，不抛出异常。

## 范围限制

- 不实现 `blmesh::FixHightRatio` 的首层局部尺寸修正。
- 不改变法向平滑、活动前沿提取、质量检测、碰撞检测、各向同性停止和邻域层差传播。
- 不增加外部参数。

## 验证

1. 单元测试构造 `reference_height != provisional_height` 的输入，确认最终步长以 `reference_height` 为基准；旧实现必须失败。
2. 验证输入数组数量和值的错误路径。
3. 运行完整 Debug 回归测试。
4. 使用 Release 和固定参数测试 `2dot5_cf`：首层步长 `0.1`、增长率 `1.2`、20 层、最大 skewness `0.9`、各向同性阈值 `1.0`。
5. 报告逐层新增单元数、总单元数、停止原因和质量直方图，并与乘法累计版本的 `631453` 个体单元对比。
