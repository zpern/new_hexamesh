# Multi-Normal 独立模块设计

## 1. 目的

将当前位于 `growth` 目录和 `boundary_mesh_boundary_layer` 编译目标中的多法向拆点、拓扑改造与过渡体算法拆成独立模块。此次调整只改变代码归属、编译边界和上层调用位置，不改变多法向算法、规则层算法、命令行参数或网格输出语义。

## 2. 模块边界

新增独立静态库：

```cmake
boundary_mesh_multi_normal
BoundaryMesh::MultiNormal
```

源文件和公共头文件分别放置于：

```text
src/multi_normal/
include/boundary_mesh/multi_normal/
```

以下实现归入 MultiNormal：

- 多法向拆分规划；
- BLMesh 拆点、几何、相交检查与拓扑缝合适配代码；
- incident face fan；
- 分裂后的表面拓扑构造；
- 含分裂点四边形的三角形化；
- 过渡体构造和相交回缩；
- transformed front 构造；
- 多法向体与规则层体网格合并；
- `third/blmesh_mnormal` 中为上述算法编译的实现文件。

`BoundaryMesh::BoundaryLayer` 只保留普通前沿、方向、规则层生长、碰撞、停止传播、顶面和 Farfield 等规则附面层能力，不再编译任何 `multi_normal_*` 或 BLMesh 多法向源码。

## 3. 依赖方向

依赖关系固定为：

```text
Core / Surface / Quality / Spatial
                    ↓
           BoundaryLayer
                    ↓
            MultiNormal ← IO（仅调试输出）
                    ↓
        Transition / CLI
```

MultiNormal 可以复用 `GrowthFront`、前沿评价及其他 BoundaryLayer 基础能力；BoundaryLayer 不得包含或链接 MultiNormal。上层协调模块同时链接 BoundaryLayer 和 MultiNormal，并负责按顺序调用两者。

这避免了 `BoundaryLayer ↔ MultiNormal` 循环依赖，也使关闭多法向时规则层库可以单独使用。

## 4. 公共接口与命名

公共 include 路径由：

```cpp
#include <boundary_mesh/growth/multi_normal_*.hpp>
```

调整为：

```cpp
#include <boundary_mesh/multi_normal/multi_normal_*.hpp>
```

BLMesh 适配头和 `incident_face_fan.hpp` 同样进入 `boundary_mesh/multi_normal/`。本次不修改 `MultiNormalOptions`、`MultiNormalTransitionResult`、错误类型、函数名称或 `boundary_mesh` 命名空间，以限制迁移风险。

## 5. 调用与数据流

多法向处理仍在拓扑建立后、规则层生成前由调用方显式执行：

```text
Wall surface/topology
→ build GrowthPatch/GrowthFront
→ generateMultiNormalTransition
→ transformed_front
→ generateRegularLayers
→ ReservedLayer 重构
→ mergeMultiNormalAndRegularMeshes
→ 最终体网格、顶面和 Farfield
```

原 `boundary_layer_generator` 同时编排 MultiNormal 和 Regular，因此不再作为 BoundaryLayer 库的组成部分。其头文件迁至 `include/boundary_mesh/transition/boundary_layer_generator.hpp`，实现迁至 `src/transition/boundary_layer_generator.cpp`，并编译进 `boundary_mesh_transition`。对外函数签名与行为保持不变，调用方只更新 include 路径。

## 6. CMake 调整

1. 从 `boundary_mesh_boundary_layer` 源文件列表移除全部 MultiNormal、BLMesh 适配和第三方多法向实现。
2. 新增 `boundary_mesh_multi_normal`，为其配置 BLMesh 私有 include 目录。
3. `boundary_mesh_multi_normal` 链接其实际使用的 Core、Surface、Quality、Spatial、IO 和 BoundaryLayer 能力。
4. `boundary_mesh_transition` 链接 `BoundaryMesh::MultiNormal` 与 `BoundaryMesh::BoundaryLayer`。
5. 将组合生成器移动并编译到 `boundary_mesh_transition`，消除 BoundaryLayer 对 MultiNormal 的反向依赖。
6. 将调试 VTK 输出所需的 IO 依赖从 BoundaryLayer 转移给 MultiNormal；BoundaryLayer 不再因为多法向调试输出而链接 IO。
7. 保留现有顶层库别名和命令行入口；新增模块别名供单独使用和测试。

## 7. 测试迁移与约束

测试文件按职责迁移到：

```text
tests/unit/multi_normal/
tests/integration/multi_normal/
```

现有断言不因目录迁移而改变。新增一个模块边界检查，至少验证：

- `boundary_mesh_boundary_layer` 的源列表不含 MultiNormal 或 BLMesh 多法向实现；
- MultiNormal 公共头文件不再位于 `include/boundary_mesh/growth/`；
- 使用者通过 `BoundaryMesh::MultiNormal` 可以独立链接多法向入口；
- 上层组合流程仍能执行 MultiNormal → Regular → Merge。

迁移完成后执行完整 Release 构建和全部 CTest。现有 72 项测试必须全部通过；新增边界测试也必须通过。

## 8. 兼容性与非目标

本次不做以下工作：

- 不修改拆点判定、Skewness 策略、回缩规则或单元分解算法；
- 不更改 `--multi-normal` 开关及默认值；
- 不改变 CGNS 只向附面层模块传入 Wall 面的行为；
- 不重命名现有公共 C++ 类型和函数；
- 不抽取新的通用 Front 基础库；
- 不清理与本次迁移无关的现有文件或用户改动。

## 9. 完成标准

满足以下条件视为完成：

1. 仓库中不存在 `src/growth/multi_normal_*`、`include/boundary_mesh/growth/multi_normal_*` 或位于 growth 下的 BLMesh 多法向适配实现；
2. `boundary_mesh_boundary_layer` 可在不链接 MultiNormal 的情况下构建和使用规则层功能；
3. `BoundaryMesh::MultiNormal` 是独立可链接目标；
4. 上层调用顺序及输出结构保持不变；
5. Release 全量构建成功，所有测试通过。
