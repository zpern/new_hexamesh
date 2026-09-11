# BoundaryMesh 当前架构

## 1. 项目定位

BoundaryMesh 是一个 C++17 边界层网格生成库。它从封闭的三角形/四边形表面网格出发，沿表面法向逐层生成 Prism 和 Hexa 层，并在局部停止、碰撞、层数不一致或拓扑过渡处生成 Pyramid、Tetra 等过渡单元。

当前实现重点是：混合表面与体网格、封闭表面拓扑、生长前沿、法向和步长平滑、规则层生长、空间碰撞检查、过渡单元构造、CGNS/VTK 输入输出和命令行运行路径。

## 2. 代码组织

```text
include/boundary_mesh/   公共 C++ 接口
src/                     实现
tests/                   单元测试和集成测试
apps/                    可执行程序入口
third/                   Eigen、CGNS、HDF5、geom 等第三方依赖
cmake/                   CMake 辅助模块
scripts/                 验证和网格检查脚本
docs/                    设计、使用和维护文档
```

| 模块 | 职责 |
|---|---|
| `core` | 基础索引、标量、结果类型、错误和公共配置 |
| `mesh` | 表面/体网格、面单元和拓扑数据 |
| `surface` | 面几何评价、法向、角度和局部表面操作 |
| `growth` | GrowthPatch、GrowthFront、法向和步长平滑 |
| `quality` | Prism、Hexa、Pyramid、Tetra 的几何质量与有效性 |
| `spatial` | AABB、空间索引和候选碰撞查询 |
| `sliding` | 过渡顶点的表面滑移和约束求解 |
| `multi_normal` | 多法向角点和多分支生长支持 |
| `transition` | 层数差异、停止传播和过渡单元构造 |
| `boundary_layer` | 逐层生成流程、事务提交和最终边界整理 |
| `io` | CGNS/VTK 读取、写出和边界条件映射 |
| `cli` | 命令行参数、运行编排和诊断输出 |

## 3. 依赖方向

```text
core → mesh/surface/quality/spatial/sliding/multi_normal
     → growth → transition → boundary_layer → io/cli
```

低层模块不得依赖 CLI 或具体输入格式。生成算法依赖抽象网格和几何接口，IO 只负责把外部数据转换成这些模型。

## 4. 核心数据流

1. 读取表面网格并验证坐标、索引、面类型和封闭性。
2. 构建确定性的表面拓扑、边邻接和边界元数据。
3. 初始化第 0 层 `GrowthFront`，计算面法向、顶点方向和初始步长。
4. 根据邻接、曲率、对称面和用户参数平滑方向与步长。
5. 生成 Prism/Hexa 候选，并执行质量检查和碰撞检查。
6. 将局部停止、共享边约束和最大层数约束传播到相邻面。
7. 对层数不一致的边界构造 Pyramid/Tetra/滑移过渡候选。
8. 通过统一有效性检查后，以事务方式提交该层；失败则回退本层候选。
9. 生成最终远场边界，写出体网格和诊断信息。

候选单元只有在拓扑、几何质量、碰撞和方向检查全部通过后才会进入最终网格。中间状态不直接污染已提交层。

## 5. 生长状态与停止语义

`GrowthFront` 表示当前正在生长的表面前沿，包含当前层顶点、面、邻接和每个顶点的方向/步长状态。每一轮生成下一层候选前沿；成功提交后，下一层成为新的当前前沿。

停止是局部的，但必须通过共享边邻接协调。常见停止原因包括达到层数/高度、步长过小、候选质量不足、真实碰撞以及输入拓扑或几何无效。合法共享面接触不能被误判为碰撞；无法安全判定时，优先停止局部生长并保留可验证网格。

## 6. 质量与拓扑不变量

- 所有索引必须引用有效顶点或面。
- 输入表面必须满足项目要求的封闭性和方向一致性。
- 每个已提交体单元必须通过有向体积和退化检查。
- 过渡单元必须保持接口节点一致，并满足方向约定。
- 生成失败不得留下半提交层。
- 输出顺序和诊断结果应尽量确定性，便于回归测试。

## 7. 构建与测试

顶层 CMake 提供 `BoundaryMesh::Core`、`Mesh`、`Surface`、`Growth`、`Quality`、`Spatial`、`Sliding`、`MultiNormal`、`Transition`、`BoundaryLayer`、`IO` 和 CLI 支持库，以及测试目标和命令行程序。

```powershell
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

`tests/unit/` 覆盖基础类型、拓扑、几何质量和局部算法；`tests/integration/` 覆盖规则生长、增量过渡、IO 和完整流水线；`scripts/` 提供 VTK/ParaView 输出的独立几何检查。

## 8. 文档维护规则

架构文档描述当前代码，不记录每一次提交的实施细节。实验、已完成计划和临时诊断不应堆积在主文档目录；需要保留时放入发布说明或独立历史归档。
