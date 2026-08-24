# 步长平滑乘法实验实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将步长平滑修正从除法切换为乘法，并用 `2dot5_cf` 验证实际生成结果。

**Architecture:** 不改变 `GrowthFieldSmoother` 接口和数据流，只替换最终实际步长公式。测试直接复算同一预测步长和 Logistic 修正，确保乘法语义不能被除法实现误通过。

**Tech Stack:** C++17、CMake、CTest、CGNS、ASCII Legacy VTK。

## Global Constraints

- Logistic 响应系数保持为 `0.5`。
- 实际步长继续限制在本层基准步长的 `50%～150%`。
- 只使用当前活动前沿，不改变法向平滑、质量、碰撞和停止逻辑。
- 不增加配置开关。

---

### Task 1: 切换步长公式并测试 2dot5

**Files:**
- Modify: `tests/unit/growth/growth_field_smoother_test.cpp`
- Modify: `src/growth/growth_field_smoother.cpp`

**Interfaces:**
- Consumes: `GrowthFieldSmoother::smooth(..., const std::vector<Scalar>& base_heights)`
- Produces: `SmoothedGrowthFields::actual_heights`，其值采用 `base_height * (1 + correction)`

- [ ] **Step 1: 写出乘法公式测试**

将测试辅助函数末尾改为：

```cpp
return std::clamp(
    base_heights[vertex_index] *
        (Scalar{1} + correction),
    Scalar{0.5} * base_heights[vertex_index],
    Scalar{1.5} * base_heights[vertex_index]);
```

- [ ] **Step 2: 验证 RED**

运行：

```powershell
cmake --build build --config Debug --target boundary_mesh_growth_field_smoother_test
ctest --test-dir build -C Debug -R "^boundary_mesh_growth_field_smoother_test$" --output-on-failure
```

预期：测试以返回码 `10` 失败，因为生产代码仍采用除法。

- [ ] **Step 3: 最小实现**

将生产公式改为：

```cpp
const Scalar actual = std::clamp(
    base_heights[index] *
        (Scalar{1} + correction),
    Scalar{0.5} * base_heights[index],
    Scalar{1.5} * base_heights[index]);
```

- [ ] **Step 4: 验证 GREEN 和完整回归**

运行：

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

预期：构建成功且全部测试通过。

- [ ] **Step 5: 构建 Release 并测试真实算例**

运行：

```powershell
cmake --build build --config Release --target boundary_mesh_cli
.\build\Release\boundary_mesh_cli.exe --input "C:\Users\zpern\Desktop\todo\jiuyuan-quailty\test_case\2dot5_cf\2dot5_cf.cgns" --first-height 0.1 --growth-ratio 1.2 --layer-count 20 --maximum-skewness 0.9 --isotropic-height 1.0 --output-prefix "C:\Users\zpern\Desktop\todo\jiuyuan-quailty\test_case\2dot5_cf\2dot5_cf_height_multiply"
```

预期：命令成功，输出逐层新增单元数、总单元数和停止原因，并生成两个 VTK 文件。

- [ ] **Step 6: 检查并提交**

```powershell
git diff --check
git status --short
git add src/growth/growth_field_smoother.cpp tests/unit/growth/growth_field_smoother_test.cpp
git commit -m "fix: multiply by smoothed height ratio"
```
