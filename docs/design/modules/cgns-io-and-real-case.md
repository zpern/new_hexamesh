# CGNS IO、命令行与真实案例设计

## 1. 文档目的

本文档定义 BoundaryMesh 的正式文件 IO 边界、命令行编排和真实案例验收规则。该模块只负责外部格式与内部数据模型之间的转换，不参与拓扑、几何、质量、碰撞、生长或停止传播决策。

对应开发路线图阶段 10。阶段 10 实施前，阶段 06、07 和 08 必须已经完成；阶段 09 的 Pyramid/Tetra 生成不是本阶段前置条件，但 VTK 写出器必须能够表达 `VolumeMesh` 已定义的全部体单元类型。

## 2. 已确认目标

阶段 10 必须完成：

1. 通过官方 `cgnslib` 读取 Pointwise 导出的 HDF5-CGNS 表面网格；
2. 通过同目录、同名 `.bc.txt` 将数字 Zone 映射为 Wall 或 Farfield；
3. 根据 CGNS 显式 Zone 连接合并跨 Zone 顶点，不按坐标距离猜测连接；
4. 将统一表面转换为现有 `SurfaceMesh`，再交给既有拓扑和生长流水线；
5. 从命令行接收输入路径和全局生长参数；
6. 使用 ASCII Legacy VTK 输出边界层体网格与最终远场边界；
7. 用 `2dot5_cf.cgns` 完成真实混合表面一层端到端验收；
8. 提供可显式运行的性能和峰值工作集基准。

## 3. 非目标

本阶段不实现：

- PLY 读取；
- 按坐标容差自动合并 Zone 顶点；
- CGNS Polygon、高阶面或体网格读取；
- Symmetry 边界映射；
- 在 IO 或 CLI 中实现任何网格算法；
- VTK Binary、XML VTU、`POINT_DATA` 或 `CELL_DATA`；
- 在远场边界 VTK 中保存边界条件；
- 为 Pyramid/Tetra 生成新的过渡算法。

## 4. 模块边界与依赖

新增独立构建目标 `BoundaryMesh::IO`：

```text
CGNS + .bc.txt
       │
       ▼
BoundaryMesh::IO
       │
       ▼
SurfaceMesh
       │
       ▼
Topology → Growth → Quality → Spatial → Coordination → Transition
       │
       ▼
RegularLayerGrowthResult
       │
       ├── VolumeMesh ─────────► boundary_layer.vtk
       └── farfield_boundary ──► farfield_boundary.vtk
```

依赖方向固定为：

```text
BoundaryMesh::IO → BoundaryMesh::Core + BoundaryMesh::Mesh
BoundaryMesh::IO → cgnslib（PRIVATE）
boundary_mesh_cli → BoundaryMesh::IO + BoundaryMesh::BoundaryLayer
Growth/Quality/Spatial/Transition 不依赖 IO、CGNS 或 HDF5
```

公共 IO 头文件不得包含 `cgnslib` 或 HDF5 头文件。CGNS 的句柄、枚举和原始错误码只能出现在 `src/io` 实现中。

## 5. 第三方依赖

standalone 工程固定以下官方依赖：

```text
third/cgns
third/hdf5
```

二者使用 Git submodule 固定版本。`cmake/Dependencies.cmake` 的发现顺序为：

```text
CGNS v4.5.2
HDF5 hdf5-1.14.6
```

1. 复用父工程已经提供的 CGNS/HDF5 CMake target；
2. 否则从 `third/hdf5` 和 `third/cgns` 构建；
3. 两者都不可用时给出明确配置错误。

提供选项：

```cmake
BOUNDARY_MESH_ENABLE_CGNS_IO=ON
```

该选项默认开启。关闭时不构建 `BoundaryMesh::IO`、CLI、CGNS 测试和真实案例 benchmark，Core、Mesh、Surface、Quality、Growth、Spatial 与 Transition 仍可独立构建。

## 6. 输入文件配对

调用者只传入 CGNS 路径：

```cpp
readCgnsSurface(
    const std::filesystem::path &cgns_path);
```

读取器从输入路径自动派生边界映射文件：

```text
case.cgns
case.bc.txt
```

例如传入 `C:\cases\2dot5_cf.cgns` 时，必须在同一目录找到 `C:\cases\2dot5_cf.bc.txt`。不提供第二个公共路径参数，也不搜索其他目录或其他文件名。

## 7. `.bc.txt` 语法

规范格式为：

```text
Far:
1
2

Wall:
3
4
```

解析规则：

- 只允许 `Far:` 和 `Wall:` 两个区段；
- 每行只允许一个十进制正整数 Zone 名称；
- 允许空行以及行首、行尾空白；
- Zone 名称必须能无损转换为 `std::uint32_t` 且大于零；
- 同一 Zone 必须出现且只能出现一次；
- CGNS 中的全部 Zone 必须出现在映射中；
- 映射不得引用 CGNS 中不存在的 Zone；
- 数字 Zone 名直接作为 `SurfaceBoundaryTag::region_id`；
- `Far` 映射为 `SurfaceBoundaryKind::Farfield`；
- `Wall` 映射为 `SurfaceBoundaryKind::Wall`；
- 当前阶段遇到 `Symmetry:` 或其他区段立即报错。

## 8. 支持的 CGNS 子集

读取器只接受：

- 一个 `CGNSBase_t`；
- `cell_dimension == 2` 且 `physical_dimension == 3`；
- `Unstructured` 表面 Zone；
- `CoordinateX`、`CoordinateY` 和 `CoordinateZ`；
- `TRI_3` 与 `QUAD_4` 元素段；
- PointList/PointListDonor 形式的跨 Zone 顶点连接。

以下输入必须返回错误：

- 多 Base 或空 Base；
- Structured Zone；
- 缺失坐标分量；
- NaN 或无穷坐标；
- Polygon、高阶面、线单元或体单元；
- 元素引用越界；
- 连接两侧长度不一致、索引越界或 donor Zone 不存在；
- 数字 Zone 名重复、为零、溢出或含非数字字符。

## 9. 确定性 Zone 合并

CGNS Zone 使用局部节点编号。统一 `SurfaceMesh` 按以下固定流程构建：

1. 将 Zone 名解析为 `std::uint32_t`，按数字 Zone ID 升序处理；
2. 在每个 Zone 内按 ElementRange 升序读取元素；
3. 为每个 `(zone_id, local_vertex_id)` 建立局部顶点键；
4. 读取所有显式 PointList/PointListDonor 对；
5. 使用并查集合并连接两侧的局部顶点键；
6. 不进行最近点搜索，也不使用距离容差；
7. 同一集合内的三个坐标分量必须逐值相等，否则返回连接坐标不一致错误；
8. 每个集合以最小 `(zone_id, local_vertex_id)` 作为规范代表；
9. 按规范代表升序生成连续全局 `VertexId`；
10. 将 Triangle/Quad 重映射到全局编号；
11. 面标签由所属 Zone 的 `.bc.txt` 映射产生；
12. 面输出顺序按 `(zone_id, ElementRange 中的元素 ID)` 固定。

坐标相同但没有 CGNS 显式连接的两个局部顶点保持分离。随后出现的开放边、重复面、非流形边或绕序冲突由 `SurfaceTopologyBuilder` 统一诊断。

## 10. IO 组件

### 10.1 `CgnsSurfaceReader`

职责：

- 派生并读取 `.bc.txt`；
- 调用官方 CGNS C API；
- 验证支持子集；
- 合并 Zone 顶点；
- 构造 `SurfaceMesh`。

它不构建拓扑，不反转面绕序，不修复输入，也不创建生长参数。

### 10.2 `BoundaryConditionMapReader`

该组件位于 IO 实现内部，独立解析 `.bc.txt` 并生成按 Zone ID 排序的映射。业务调用者不需要直接读取或修改映射对象。

### 10.3 `LegacyVtkWriter`

提供 `VolumeMesh` 与 `SurfaceMesh` 两个写出入口。两者均写 ASCII Legacy VTK `UNSTRUCTURED_GRID`，只包含：

- `POINTS`；
- `CELLS`；
- `CELL_TYPES`。

固定 VTK 单元类型：

| 内部类型 | VTK 类型 |
| --- | ---: |
| Triangle | 5 |
| Quad | 9 |
| Tetra | 10 |
| Hexa | 12 |
| Prism/Wedge | 13 |
| Pyramid | 14 |

坐标声明为 `double`，使用 `std::numeric_limits<double>::max_digits10` 精度。写出器不保存标签、region、源面、层号、停止原因或质量数据。

写出器先在目标目录完成临时文件，成功关闭后再替换同名旧文件。已有目标不要求 `--force`，默认直接覆盖。

## 11. 命令行接口

可执行程序命名为 `boundary_mesh_cli`。标准调用为：

```powershell
boundary_mesh_cli `
  --input "C:\cases\2dot5_cf.cgns" `
  --first-height 0.1 `
  --growth-ratio 1.0 `
  --layer-count 1 `
  --maximum-skewness 0.95 `
  --max-neighbor-layer-difference 1 `
  --output-prefix "D:\results\case01"
```

参数规则：

- `--input`、`--first-height`、`--growth-ratio` 和 `--layer-count` 必填；
- `first_height` 与 `growth_ratio` 必须是有限正数；
- `layer_count` 允许为 0；
- `maximum_skewness` 默认 0.95；
- `max_neighbor_layer_difference` 默认 1，允许为 0；
- `output_prefix` 缺省时使用输入 CGNS 所在目录与文件 stem；
- 全部 Wall 顶点获得独立但初值相同的 `VertexGrowthProfile`；
- CLI 不提供逐点 profile 文件。

输出文件固定为：

```text
<prefix>_boundary_layer.vtk
<prefix>_farfield_boundary.vtk
```

## 12. 输出和退出语义

局部质量停止、碰撞停止、顶点层数上限和邻接层数约束都是正常结果。只要完整流水线成功，CLI 返回 0 并写出两个 VTK。

成功诊断只输出聚合信息：

- 输入顶点、Triangle 和 Quad 数量；
- Wall/Farfield Zone 与面数量；
- 输出顶点和各体单元数量；
- 最终远场边界 Triangle/Quad 数量；
- 各停止原因对应的源面数量；
- 读取、拓扑、生长、写出和总耗时。

命令行参数错误、CGNS/边界映射错误、拓扑错误、生长程序错误和输出错误返回不同的非零类别。输入、拓扑或生长失败时不写 VTK。不得逐面向标准输出刷屏。

## 13. 错误模型

IO 使用现有 `Result<T, E>` 风格，不把可预期输入错误实现为业务异常。

错误按职责拆分：

```text
BoundaryMapError
    文件缺失、区段非法、Zone 重复/缺失/未知、数字非法

CgnsReadError
    文件/Base/Zone/元素/坐标/连接非法及 cgnslib 失败

VtkWriteError
    路径、引用、计数溢出、写入或替换失败

CliError
    参数解析及各模块错误的用户侧适配
```

所有新公共字段和枚举值继续添加中文 `//` 注释。错误对象保留足以定位问题的文件路径、Zone ID、元素 ID 或连接下标，但 CLI 只打印一条简明诊断。

## 14. 测试策略

### 14.1 边界映射单元测试

覆盖：

- 标准 `Far/Wall` 格式；
- 空行和空白；
- 重复 Zone；
- 缺失或未知 Zone；
- 非数字、零和溢出 Zone；
- 映射文件缺失；
- 未支持区段。

### 14.2 CGNS 读取单元测试

测试通过官方 cgnslib 创建小型临时 CGNS，覆盖：

- 单 Zone Triangle；
- Triangle/Quad 混合 Zone；
- 两个 Zone 共享点和共享边；
- 反转 Zone 创建顺序后输出不变；
- 非有限坐标、越界元素和错误 donor；
- 未支持元素类型；
- 连接坐标不一致。

### 14.3 VTK 与 CLI 集成测试

覆盖全部六种 VTK 单元类型、ASCII 段结构、double 精度、直接覆盖、临时文件清理、无附加数据，以及失败时不产生正式输出。

## 15. `2dot5_cf` 真实案例

真实输入由 IO/CLI 直接接收绝对文件路径，不通过 CMake 固定案例目录。文件配对为：

```text
2dot5_cf.cgns
2dot5_cf.bc.txt
```

映射为：

```text
Far:
1
2
Wall:
3
4
```

读取结果必须满足：

| 指标 | 数量 |
| --- | ---: |
| 合并前局部顶点 | 52,232 |
| 合并后全局顶点 | 52,010 |
| Triangle | 13,186 |
| Quad | 45,413 |
| Farfield 面 | 422 |
| Wall 面 | 58,177 |

端到端冒烟参数固定为：

```text
first_height = 0.1
growth_ratio = 1.0
layer_count = 1
maximum_skewness = 0.95
max_neighbor_layer_difference = 1
```

完整流程必须返回 0，并生成两个非空且结构可重新解析的 VTK。允许部分 Wall 面因质量或碰撞正常停止。

## 16. 性能和内存基准

真实性能测试由独立 benchmark 可执行程序接收 CGNS 路径，不加入日常快速 CTest。它输出：

- CGNS 读取时间；
- 拓扑构建时间；
- 一层生成时间；
- 两个 VTK 写出时间；
- 总时间；
- Windows 进程峰值工作集。

耗时只记录，不使用与机器强相关的固定秒数作为通过条件。`2dot5_cf` 的 58,599 面、一层案例峰值工作集不得超过 1 GiB。

## 17. 完成边界

阶段 10 只有在以下条件全部满足时才完成：

```text
官方 cgnslib/HDF5 依赖被隔离在 BoundaryMesh::IO
CGNS 与同名 .bc.txt 能确定性转换为 SurfaceMesh
跨 Zone 顶点只通过显式连接合并
全部 IO 错误具有明确 Result 诊断
CLI 能完成完整流水线并正确区分正常局部停止与程序错误
两个纯几何 ASCII Legacy VTK 正确写出并覆盖旧文件
小型单元/集成测试全部通过
2dot5_cf 计数、一层端到端和 VTK 重读通过
真实案例峰值工作集不超过 1 GiB
Debug 与 Release 全量 CTest 均为 0 failed
```
