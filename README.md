# BoundaryMesh 程序维护文档

本文面向第一次接手本项目的 C++ 开发者，以当前源码和 CMake 为准；无法确认的信息明确标为“待确认”。

# 1. 项目简介

`BoundaryMesh` 是 C++17 边界层体网格生成程序及静态库集合。它读取带边界分区的二维 CGNS 非结构表面网格，从 `Wall` 三角形/四边形表面向外生长体单元，并生成供后续远场体网格流程使用的边界表面。

程序按首层高度、增长率和请求层数生长，同时拒绝退化、反转、局部翻转、偏斜过大或碰撞的候选单元，传播相邻面层差限制，并可在复杂角点执行多法向拆分。规则层试生长后，程序协调相邻面层数，用三角形/四边形模板生成保留层过渡单元。

输入：

- 一个 CGNS 文件：单个 Base，`CellDimension=2`、`PhysicalDimension=3`，非结构 Zone，面仅支持 `TRI_3`/`QUAD_4`；
- 同目录同主名的 `<主名>.bc.txt`，将每个 Zone 标为 `Wall`、`Farfield`、`Symmetry` 或 `Internal`；
- 首层高度、增长率、层数及可选质量/协调/多法向参数。

输出：

- `<prefix>_boundary_layer.vtk`：混合体边界层网格（可含 Tetra/Pyramid/Prism/Hexa）；
- `<prefix>_farfield_boundary.vtk`：完整边界，包含 Farfield、BoundaryLayerInterface、Symmetry 和 Internal；
- `<prefix>_boundary_layer_top.vtk`：只包含最终 BoundaryLayerInterface 外露顶面；
- 标准输出中的输入规模、单元计数和停止原因统计。

```text
CGNS + .bc.txt + CLI 参数
 -> 读取、合并显式连接及跨 Zone 精确重合顶点
 -> 验证闭合流形表面并构建拓扑
 -> 提取 Wall GrowthPatch -> 建立第 0 层 GrowthFront
 -> 可选多法向拆点/过渡
 -> 规则层试生长 -> 相邻层数协调
 -> Triangle/Quad 保留层过渡模板
 -> 合并网格 -> 输出 3 个 legacy VTK
```

# 2. 开发环境与依赖

| 项目 | 代码可确认的要求 |
| --- | --- |
| CMake | 3.20 或更高 |
| C/C++ | 工程启用 C 和 C++；项目代码为 C++17，关闭编译器扩展 |
| 编译器 | 支持 C++17；未限定厂商或最低版本 |
| Eigen | `third/eigen`，头文件依赖 |
| tiger_geom | `third/geom`，空间相交私有依赖 |
| HDF5 | `third/hdf5`，当前子模块 1.14.6；CGNS IO 开启时使用 |
| CGNS | `third/cgns`，当前子模块 4.5.2 |
| BLMesh 多法向代码 | `third/blmesh_mnormal`，源码直接编入边界层库 |

CMake 优先复用父工程提供的 `BoundaryMesh::CGNS`、CGNS target 或 `tiger_geom`；独立构建则使用 `third/`。内置 HDF5/CGNS 为静态库，并关闭工具、示例、Fortran、Java及依赖自身测试。

- Windows：仓库当前已有 Visual Studio 多配置构建产物；MSVC 使用 `/utf-8 /W0`。推荐 VS 2022，但最低 MSVC 版本待确认。
- Linux/macOS：CMake 未禁止，非 MSVC 使用 `-w`；官方支持/CI 状态待确认。
- 环境变量：独立构建无需专用变量；`TIGER_ROOT_DIR` 由顶层 CMake设置为源码根。
- 现状：默认关闭全部编译警告，这不代表推荐实践。

# 3. 项目目录结构

```text
new_boundaryMesh/
├── CMakeLists.txt                 # targets、开关和测试入口
├── cmake/                         # 编译选项、第三方接入
├── apps/boundary_mesh_cli.cpp     # main()
├── include/boundary_mesh/
│   ├── core/                      # 标量、ID、Result
│   ├── mesh/                      # 表面/体网格与拓扑
│   ├── surface/                   # 表面几何/skewness
│   ├── quality/                   # Prism/Hexa 质量
│   ├── spatial/                   # AABB 和三角接触
│   ├── growth/                    # 前沿、规则层和碰撞
│   ├── boundary_layer/            # 完整附面层流程编排
│   ├── multi_normal/              # 多法向起始区
│   ├── transition/                # 层协调和过渡模板
│   └── io/                        # CGNS 读取、VTK 写出
├── src/                           # 与 include 基本镜像的实现
│   ├── cli/ mesh/ surface/ quality/ spatial/ growth/
│   └── boundary_layer/ multi_normal/ transition/ io/
├── tests/
│   ├── unit/                      # 按模块的单元测试
│   ├── integration/               # 流水线测试
│   └── helpers/                   # CGNS fixture
├── benchmarks/                    # 质量和 CGNS benchmark
├── docs/design/                   # 架构/模块设计记录
├── docs/plans/                    # 历史计划，不是运行配置
└── third/                         # Eigen/geom/HDF5/CGNS/BLMesh
```

建议阅读顺序：本 README -> `src/cli/boundary_mesh_command.cpp` -> `src/boundary_layer/boundary_layer_generator.cpp` -> 各阶段公开头文件 -> 实现与对应测试。历史计划可能落后于源码。

# 4. 编译方法

先在源码根目录取完整依赖：

```bash
git submodule update --init --recursive
```

确认 `third/eigen`、`geom`、`hdf5`、`cgns`、`blmesh_mnormal` 非空。前四项在 `.gitmodules` 中；BLMesh 的来源/版本方式待确认。

## Windows（Visual Studio）

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 `
  -DBOUNDARY_MESH_ENABLE_CGNS_IO=ON -DBUILD_TESTING=ON
cmake --build build --config Debug --parallel
ctest --test-dir build -C Debug --output-on-failure

cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
```

CLI 通常位于 `build/Debug/boundary_mesh_cli.exe` 或 `build/Release/boundary_mesh_cli.exe`；静态库位于对应配置目录。

## Linux/macOS（单配置生成器）

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug \
  -DBOUNDARY_MESH_ENABLE_CGNS_IO=ON -DBUILD_TESTING=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure

cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release \
  -DBOUNDARY_MESH_ENABLE_CGNS_IO=ON -DBUILD_TESTING=ON
cmake --build build-release --parallel
```

CLI 通常为 `build-release/boundary_mesh_cli`。实际发行验证状态待确认。

关闭 CGNS：

```bash
cmake -S . -B build-no-io -DBOUNDARY_MESH_ENABLE_CGNS_IO=OFF -DBUILD_TESTING=ON
cmake --build build-no-io --parallel
```

此时 `boundary_mesh_io` 只含 VTK writer；不会生成 CGNS reader、CLI、CGNS 测试/benchmark。

| target | 产物/职责 |
| --- | --- |
| `boundary_mesh_core` | 拓扑/网格基础静态库 |
| `boundary_mesh_surface` | 面几何静态库 |
| `boundary_mesh_quality` | 体质量静态库 |
| `boundary_mesh_spatial` | 碰撞空间结构静态库 |
| `boundary_mesh_sliding_surface` | Symmetry/Internal 区域建模、方向约束与位置投影静态库 |
| `boundary_mesh_boundary_layer` | 生长主静态库 |
| `boundary_mesh_transition` | 保留层/模板静态库 |
| `boundary_mesh_io` | VTK；开关 ON 时也含 CGNS |
| `boundary_mesh_cli_support` / `boundary_mesh_cli` | CLI 库/程序，仅 CGNS ON |
| `boundary_mesh_cgns_pipeline_benchmark` | CGNS benchmark，仅 CGNS ON |
| `boundary_mesh_volume_cell_quality_benchmark` | 质量 benchmark，仅 BUILD_TESTING ON |

CTest 不默认运行两个 benchmark，也不运行需外部路径的 `boundary_mesh_cgns_real_case_test`。

# 5. 程序运行方法

入口为 `apps/boundary_mesh_cli.cpp::main()`，实际逻辑为 `boundary_mesh::runBoundaryMeshCommand()`。

```text
boundary_mesh_cli --input FILE --first-height VALUE
  --growth-ratio VALUE --layer-count COUNT
  [--maximum-skewness VALUE]
  [--max-neighbor-layer-difference COUNT]
  [--isotropic-height VALUE]
  [--multi-normal true|false]
  [--debuglog true|false]
  [--output-prefix PATH]
```

| 参数 | 约束/默认值 |
| --- | --- |
| `--input` | 必填，CGNS 路径 |
| `--first-height` | 必填，有限且 > 0 |
| `--growth-ratio` | 必填，有限且 > 0 |
| `--layer-count` | 必填，uint32 且 > 0 |
| `--maximum-skewness` | 默认 0.95，范围 [0,1] |
| `--max-neighbor-layer-difference` | 默认 1，uint32 |
| `--isotropic-height` | 默认 1，必须 > 0 |
| `--multi-normal` | 默认 false，仅 true/false/1/0 |
| `--debuglog` | 默认 false；启用后写出 `<输出前缀>_debug.txt`，记录每个 Wall 源面的停止原因 |
| `--output-prefix` | 默认 `<输入目录>/<输入主名>` |

参数必须严格按“名字 值”成对出现；重复、未知或缺值返回退出码 2。

`.bc.txt` 示例（Zone 名须为正整数）：

```text
Wall:
1
2
Far:
3
Symmetry:
4
Internal:
5
```

`case.cgns` 对应 `case.bc.txt`。每个 Zone 必须恰好出现一次；不支持注释或其他未知段名。`Symmetry:` 和 `Internal:` 区域约束与 Wall 共点的边界层顶点沿对应表面滑移。

滑移面按 region 建立。轴面识别沿用参考工程参数：`reference_length = 0.02 * average_edge_length`，`axis_epsilon = 0.1 * reference_length`，依次识别 X/Y/Z 常量面；三个轴均不满足时启用三角曲面最近点算法。方向/步长平滑完成后施加方向约束，质量与碰撞检查前再投影最终候选位置。多滑移面按 region ID 升序迭代，最多 20 次；不收敛时以 `SlidingProjectionFailure` 局部停止相关面。

```powershell
.\build\Release\boundary_mesh_cli.exe `
  --input C:\mesh\case.cgns `
  --first-height 0.001 --growth-ratio 1.2 --layer-count 10 `
  --output-prefix C:\mesh\out\case
```

退出码：0 成功；2 参数；3 CGNS/映射读取；4 拓扑；5 Patch/Front；6 生长；7 输出目录/VTK。CLI 当前只打印阶段级错误，不打印底层结构化 error。

# 6. 程序整体架构

```text
CLI -> BoundaryLayer
          ├── Growth
          ├── Transition -> Growth
          └── MultiNormal -> Growth

Core / Surface / Quality / Spatial / SlidingSurface
          ↑
   Growth、Transition、MultiNormal
```

```text
main -> runBoundaryMeshCommand
 -> readCgnsSurface
 -> SurfaceTopologyBuilder::build
 -> GrowthPatchBuilder::build
 -> GrowthFrontBuilder::buildInitial
 -> generateBoundaryLayers
    -> generateMultiNormalTransition（可选）
    -> generateIncrementalBoundaryLayers
       -> generateRegularLayers
       -> LayerTransitionResolver（逐层固定点）
       -> buildProvisionalTransition
       -> finalizeIncrementalLayerTopology
    -> mergeMultiNormalAndRegularMeshes
 -> writeLegacyVtk（三次）
```

`SurfaceMesh` 使用全局 ID；`GrowthPatch` 保存 Wall 源 ID；`GrowthFront` 改用紧凑局部下标并保留回源映射；`LayerVertexTable` 连接源点/分支、层号和体网格点。最终合并会再次重映射 ID，不能把不同阶段的整数 ID 混用。

# 7. 各模块功能说明

## 7.1 core

`types.hpp` 定义 `Scalar=double`、Eigen 三维点/向量及 uint32 的 Vertex/Edge/SurfaceFace/VolumeCell ID。ID 表示对应容器下标。

### `Result<T,E>`

用 `std::variant` 表示成功值或结构化错误。接口：`success/failure/hasValue/value/error`。错误态调用 `value()`（或反之）抛 `std::logic_error`，必须先检查。

## 7.2 mesh

### `SurfaceMesh`

保存 `Triangle/Quad`、坐标和逐面 `SurfaceBoundaryTag`。`faces` 与 `face_tags` 必须同长；绕序决定法向和邻面方向。

### `SurfaceTopologyBuilder` / `SurfaceTopology`

验证非空、有限坐标、引用、面退化/重复、每边恰有两面、共享边方向相反。输出升序规范边、edge-to-face、face-to-edge、face-to-neighbor、vertex-to-face 快照。网格变更后必须整体重建。

### `VolumeMesh`

保存 Tetra/Pyramid/Prism/Hexa 及同长 `CellMetadata`。单元角色明确区分 `RegularLayer`、`MultiNormalTransition` 和 `LayerTransition`，并记录源面与层号。Tetra 使用有向基面 `0-1-2` 与正体积侧顶点 `3`；Pyramid 使用连续环绕基面 `0-1-2-3` 与正体积侧顶点 `4`。四类单元的顺序分别由 `TetraVertexOrder`、`PyramidVertexOrder`、`PrismVertexOrder`、`HexaVertexOrder` 注释约定，质量算法依赖它；负体积作为反转诊断保留，不自动交换节点。

## 7.3 surface / quality

`evaluateTriangle/evaluateQuad` 计算法向、面积、角和尺度；两个 `*EquiangularSkewness` 计算面偏斜度。

`evaluatePrism/evaluateHexa` 以固定子四面体有向体积区分 `Valid/Degenerate/Reversed/LocallyInverted`，并以组成面最大 skewness 判断 `acceptable`。`RegularLayerStepper` 提交前调用。

## 7.4 io

### `readCgnsSurface()`

读取坐标和 TRI_3/QUAD_4。Zone 名必须解析成唯一正 uint32。各 Zone 顶点先独立保存，再按显式 `Abutting1to1` + `PointList/PointListDonor` 连接用并查集合并；连接点坐标要求完全相同。显式连接处理后，还会补充合并不同 Zone 间 X/Y/Z 完全相同的顶点（`+0/-0` 等价）；不做容差焊接，也不合并同一 Zone 内的重复坐标。最终按 `(zone_id, element_id)` 排面并压缩点 ID。

内部 `readBoundaryConditionMap()` 要求所有 Zone 恰好映射一次，当前 `region_id=zone_id`。`writeLegacyVtk()` 重载支持表面/体网格。

## 7.5 growth：Patch、Front、方向场

### `GrowthPatchBuilder` / `GrowthPatch`

提取 Wall 面/点。Patch 点按源 VertexId、源面按 SurfaceFaceId 排序；点记录相邻 Symmetry/Internal region，供滑移约束和逐层侧面标记使用。

### `GrowthFrontBuilder` / `GrowthFront`

构建第 0 层紧凑活动前沿。`faces` 引用局部点，`source_face_ids` 映射回输入；`GrowthFrontVertex` 保存当前/根坐标、源点、方向、实际步长、可见性、角点和多法向分支。

`FrontEvaluator` 评价几何；`buildFrontAdjacency` 建一环；`computeGrowthDirections` 选初始方向；`GrowthFieldSmoother` 平滑方向/高度；随后对 Symmetry/Internal 施加滑移方向约束，并把最终候选位置二次投影到滑移面，再进行各项质量与碰撞判断。轴对齐区域使用解析投影；不满足 X/Y/Z 判定的区域使用三角曲面最近点投影。

## 7.6 growth：规则层

### `GrowthProfileTable`

`GrowthProfileBuilder` 保证每个 Patch 源点恰有一个合法 profile，并计算各层高度，检查有限性。

### `RegularLayerStepper`

执行一层预推出：构造候选点和 Prism/Hexa，做滑移约束、质量、固定障碍碰撞和同层自碰撞，生成仅含合格面的紧凑 `next_front`，返回新旧局部映射和停止事件。Symmetry/Internal 不进入普通 `CollisionIndex`，而由独立静态索引按 region 检查非法穿越，并放行授权的顶点、真实物理边和完整侧面接触；投影失败只停止受影响的局部面。多法向预处理及多法向过渡生成阶段不执行该相交检测；进入逐层生长后，常规单元和层差过渡区的联合碰撞检测都使用静态滑移索引。

### `RegularLayerGenerator` / `generateRegularLayers()`

多层事务控制器：初始化 profile、对称约束、原表面碰撞索引、面上限和终止传播；循环 step；提交合格单元/`LayerVertexTable`；维护 `ExposedBoundaryTracker`；最后构造真实顶面和 Farfield。错误时不返回部分成功结果。

辅助职责：`LayerCollisionChecker` 过滤非法接触；`FaceLayerConstraintTable` 保存逐面上限；`TerminationPropagator` 传播直接停止/最大邻层差；`ExposedBoundaryTracker` 增量维护外露面。

## 7.7 growth：多法向

`generateMultiNormalTransition()` 是入口：建立 `IncidentFaceFan`，规划拆分分支，构造多分支拓扑，处理 Quad/三角化，生成过渡体，并通过长度调整尝试消除相交。结果含过渡体、`transformed_front`、源点/分支映射、面来源、前沿局部点到体点映射。

实现复用 `third/blmesh_mnormal`，并有 `blmesh_*` 适配层；修改时同时检查 parity 测试。

## 7.8 transition

`LayerTransitionResolver` 在每一候选层内执行角点压制、临时过渡构造、联合碰撞检查和依赖高面回退，直到保留集合稳定。

`buildTriangleSideTransition()` 以及三个 Quad 增量模板只处理相邻层差为一的局部拓扑。`LayerQuadDiagonalTable` 保证同一层面由顶盖和侧向模板共享规范对角线。

完整用例入口属于 `boundary_layer` 模块；`transition` 只负责层差处理，不依赖顶层编排模块。

## 7.9 spatial

`Aabb/BinaryAabbTree` 做确定性 broad phase；`classifyTriangleContact/hasIllegalTriangleContact` 分类 narrow phase 并过滤合法共享拓扑；`CollisionIndex` 保存三角图元和 owner。

# 8. 核心数据结构

| 结构 | 表示/生命周期 | 关键映射 |
| --- | --- | --- |
| `SurfaceMesh` | 全流程只读输入表面 | VertexId/FaceId 等于容器下标；tags 同长 |
| `SurfaceTopology` | 某版 SurfaceMesh 的派生快照 | 所有邻接引用源 ID；网格改动即失效 |
| `GrowthPatch` | Wall 子集 | 保存排序源 ID，不复制面几何 |
| `GrowthFront` | 某层活动紧凑表面 | 面引用局部点；显式映回源点/面 |
| `GrowthFrontVertex` | 活动点状态 | root 不变、position 随层变；分支键为 source + branch |
| `LayerVertexTable` | 源点/分支到逐层体点 | `layer_vertex_ids[0]` 是第 0 层 |
| `VolumeMesh` | 阶段或最终混合体 | cell 与 metadata 同长；合并后点 ID 可变 |
| `RegularLayerGrowthResult` | 规则层事务结果 | mesh、逐点/面状态、top/farfield、诊断 |
| `MultiNormalTransitionResult` | 多法向结果/透传前沿 | transformed-front 到 transition volume 映射 |
| `BoundaryLayerGenerationResult` | 完整生成结果 | 合并后的体网格、顶面和远场边界 |
| `Result<T,E>` | 函数返回期 | 访问前检查 `hasValue()` |

# 9. 关键程序流程

## 9.1 读取与拓扑

```text
readCgnsSurface
 -> 校验 Base/维度/Zone/坐标/单元
 -> 读 .bc.txt
 -> 读各 Zone -> 显式连接合并 -> 跨 Zone 精确坐标补充合并 -> 稳定排序/压缩 ID
 -> SurfaceTopologyBuilder::build
 -> 闭流形/方向检查和邻接生成
```

同坐标不会自动焊接，必须有受支持连接。拓扑拒绝开放边，因此输入需为闭表面，而不是孤立 Wall 片。

## 9.2 Patch/Front

```text
选择 Wall -> 收集排序源面/点 -> 记录 Symmetry region
 -> 源点到紧凑点映射 -> 重映射 Wall 面 -> layer=0
```

## 9.3 多法向

```text
关联面扇 -> 拆分策略 -> 多分支拓扑/前沿
 -> 过渡体/Quad 处理 -> 相交检测和长度解析 -> 映射输出
```

`enabled=false` 仍走同一 API，`applied=false` 并透传等价前沿。

## 9.4 规则层试生长

```text
验证 profile/options -> 对称/邻接/碰撞索引
 -> 每层：评价 -> 方向/高度平滑 -> 预推出
 -> 质量 -> 碰撞 -> 终止传播 -> 提交 -> 压缩下一层
 -> 请求层数到达或活动前沿为空
```

`IsotropicHeightReached` 是“接受当前单元后停止后续层”；其他质量/碰撞停止通常表示首个未接受层。

## 9.5 逐层过渡和输出

```text
每层候选 -> 停止集合 -> 角点压制
 -> 临时顶盖/侧向模板 -> 联合碰撞 -> 回退至固定点
 -> 提交规则单元 -> 最终拓扑物化
 -> 去内部重复顶三角 -> 多法向/规则网格合并
 -> Farfield + top -> 3 个 VTK
```

# 10. 新人修改功能时从哪里入手

| 修改目标 | 优先文件/模块 |
| --- | --- |
| CLI 参数/默认值/退出码 | `src/cli/boundary_mesh_command.*` |
| CGNS 类型/跨 Zone 合并 | `src/io/cgns_surface_reader.cpp`、CGNS error 头 |
| `.bc.txt`/边界类别 | `src/io/boundary_condition_map.*`、`mesh_surface.hpp` |
| VTK 格式 | `src/io/legacy_vtk_writer.cpp` |
| 表面合法性/邻接 | `src/mesh/surface_topology_builder.cpp` |
| Wall/Patch | `growth_patch_builder.cpp` |
| 初始前沿/映射 | `growth_front_builder.cpp`、`growth_front.hpp` |
| 方向/角点/滑移约束 | `growth_direction.cpp`、`incident_face_fan.cpp`、`src/sliding/*` |
| 方向/高度平滑 | `growth_field_smoother.cpp`、`skewness_direction_refiner.cpp` |
| 高度公式 | `growth_profile_builder.cpp` |
| 单层候选接受 | `regular_layer_stepper.cpp` |
| 多层循环/提交 | `regular_layer_generator.cpp`、`regular_layer_growth.hpp` |
| Prism/Hexa 质量 | `src/quality/*_evaluator.cpp`、face skewness |
| 碰撞语义 | `collision_boundary_policy.*`、`src/spatial/*`、`layer_collision_checker.cpp` |
| 相邻停止传播 | `face_layer_constraint.cpp`、`termination_propagator.cpp` |
| 多法向拆点 | `multi_normal_*`、`third/blmesh_mnormal` |
| 层协调/高边 | `layer_transition_resolver.cpp`、`quad_high_neighbor_selector.cpp` |
| 临时过渡边界 | `provisional_transition_builder.cpp` |
| 过渡单元 | `triangle_side_transition.cpp`、`incremental_transition_templates.cpp` |
| 完整流水线 | `src/boundary_layer/*` |
| targets/依赖 | 顶层 CMake、`cmake/Dependencies.cmake` |

修改后先跑对应 unit，再跑相关 integration，最后全量 CTest。

# 11. 容易踩坑的地方

1. **ID 作用域不同。** 源 ID、前沿局部下标、体点 ID 都是整数，必须经 source 映射、LayerVertexTable 或多法向映射转换。
2. **面绕序是输入。** 它影响法向、生长、有向体积；共享边在两面中须反向。不要排序面顶点。
3. **拓扑是快照。** SurfaceMesh 改动后必须重建。
4. **输入须闭合流形。** 每边恰有两面；仅 Wall patch 会产生 BoundaryEdge。
5. **Zone 名须为唯一正 uint32。** `wall`、`Zone1` 均失败。
6. **跨 Zone 不做容差焊接。** 显式连接优先；缺少连接时仅自动合并坐标完全相同的跨 Zone 顶点，同一 Zone 内不自动合并。
7. **.bc.txt 须全覆盖且无注释。**
8. **Symmetry/Internal 都是滑移面。** 二者使用相同约束算法但保留各自 kind；与 Wall 相邻的侧面会逐层生成并保留。
   滑移几何与投影属于 `BoundaryMesh::SlidingSurface`；Internal 双层邻接仍属于 Core。普通障碍由 `CollisionIndex` 处理，Symmetry/Internal 的常规层非法相交由 Spatial 的独立静态索引处理。
9. **trial mesh 不等于最终 mesh。** 后者经过协调、模板重建、合并。
10. **Prism/Hexa 顶点顺序不可随意改。**
11. **停止可能是接受后停止。** 尤其 IsotropicHeightReached。
12. **排序是确定性/二分查找约束。** Patch、profile、映射需保持约定。
13. **ID/层数为 uint32。** 不要绕过溢出检查。
14. **默认关闭警告。**
15. **CLI 隐藏底层 error。** 复杂失败应调试 Result 的 variant。

# 12. 调试建议

- 读取：观察 `CgnsSurfaceError.code/path/zone_id/element_id/detail`，核对同名映射。
- 拓扑：观察具体 `SurfaceTopologyError`、规范边、两关联面及局部方向。
- Patch/Front：比较 Patch 源 ID 与 Front source 映射；确认面引用局部点。
- 方向：在 `FrontEvaluator::evaluate`、`computeGrowthDirections`、`GrowthFieldSmoother::smooth` 断点，观察 position/root/direction/actual_height/visibility。
- 停止：在 `RegularLayerStepper::step` 质量/碰撞分支观察 `VolumeCellEvaluation`、`FaceStopEvent`、点序和 source face。
- 协调：观察 accepted layer、face constraints、termination 邻居和 high-edge 下标。
- 多法向：观察 `(source_vertex_id, branch_id)`、vertex mapping、face origins、front-volume IDs；库 API 可启用中间 VTK，CLI 尚未暴露 debug 参数。
- 合并：核对合并前后点数、所有 cell 引用、top remap 和 mesh merge。
- 优先调试最小测试 target，再从完整 CLI 复现。

# 13. 后续维护建议

## 现有代码事实

- 主要阶段有单元/集成测试；benchmark 和真实 CGNS 案例不在默认 CTest。
- CLI 为所有 Wall 点设置同一 profile，没有逐点配置入口。
- CLI 仅输出阶段级失败；结构化错误留在库 API。
- 边界映射支持 Wall/Far/Symmetry/Internal；生成结果还使用 BoundaryLayerInterface 标记边界层外顶面。
- `boundary_mesh_boundary_layer` 同时含规则生长、多法向、碰撞、远场构造，并直接编译 BLMesh 源码，耦合较强。
- 没有 install/export/package 规则，尚未形成可安装 SDK。
- 默认关闭警告；历史 docs 可能描述旧入口。

## 改进建议

1. 为 error variant 实现集中 formatter，并让 CLI 输出错误码、源实体和层号。
2. 若需扩展边界配置 schema，可增加注释和 region/zone 分离，同时保留当前四段格式的兼容测试。
3. 将 BLMesh 适配和第三方源码封装为独立 target，明确版本。
4. CI 覆盖 MSVC 与 GCC/Clang，并增加开启警告的 job；确认后记录官方平台矩阵。
5. 增加 install/export 及消费方 smoke test。
6. 将真实 CGNS 数据作为可选 fixture 接入 CI，补大网格、极端 ID/层数和错误消息回归。
7. 拆分 CLI 的配置、pipeline 和序列化职责，降低单点编排耦合。
8. 定期标记历史计划，以当前 API/测试同步架构文档和 README。

维护本文时，以源码、CMake 和测试确认事实；版本、平台支持或外部约定无法确认时继续标“待确认”。
