# 测试目录布局设计

## 1. 目标

测试目录按照被测模块组织，使测试文件的位置与生产代码职责保持一致。开发者可以仅通过路径判断测试属于 Core、Mesh、Surface、Growth 还是跨模块集成流程。

## 2. 固定目录结构

```text
tests/
├── unit/
│   ├── core/
│   │   ├── core_types_test.cpp
│   │   └── result_test.cpp
│   ├── mesh/
│   │   ├── surface_mesh_test.cpp
│   │   ├── volume_mesh_test.cpp
│   │   ├── surface_topology_types_test.cpp
│   │   ├── surface_topology_validation_test.cpp
│   │   ├── surface_topology_builder_test.cpp
│   │   └── surface_topology_edge_error_test.cpp
│   ├── surface/
│   │   └── face_evaluation_test.cpp
│   └── growth/
│       └── GrowthPatch、GrowthFront、方向和约束的单元测试
└── integration/
    └── 跨两个或更多模块的完整流程测试
```

## 3. 分类规则

- `unit/core`：基础类型、`Result` 等不属于具体网格算法的基础设施。
- `unit/mesh`：表面/体网格数据结构、离散拓扑及拓扑输入验证。
- `unit/surface`：针对单个 Triangle/Quad 的无状态表面算法。
- `unit/growth`：GrowthPatch、GrowthFront、动态评价、方向和对称约束。
- `integration`：同时经过多个模块的端到端流程，不按最终调用模块放入 unit。

测试目录不重复 `boundary_mesh` 层级。公共头文件已经通过 `<boundary_mesh/...>` 表达该命名空间，再增加 `tests/unit/boundary_mesh/...` 只会制造无收益的深层目录。

## 4. CMake 规则

测试 target 名称保持稳定，只修改源文件路径。例如：

```cmake
add_executable(
    boundary_mesh_surface_topology_validation_test
    unit/mesh/surface_topology_validation_test.cpp
)
```

目录整理不重命名 CTest test name，不改变测试行为，也不改变生产 target 依赖。新测试从创建时就放入对应模块目录。

## 5. 完成标准

- 现有 8 个测试全部移动到 `unit/core` 或 `unit/mesh`；
- Task 2 的新测试位于 `unit/surface`；
- 生长相关单元测试位于 `unit/growth`；
- `tests/unit` 根目录不再直接存放 `.cpp`；
- CMake 重新配置、Debug 构建和全部 CTest 通过；
- 文件移动在 Git 中识别为 rename，测试 target 名称保持不变。
