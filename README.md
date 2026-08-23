# BoundaryMesh 使用说明

## 1. 项目用途

`BoundaryMesh` 是一个 C++17 边界层体网格生成库。程序从带边界分区的 CGNS 表面网格出发，在 `Wall` 表面沿前沿方向逐层生长：

- 三角形 Wall 面生成三棱柱（Prism）；
- 四边形 Wall 面生成六面体（Hexa）；
- 每一层生成前检查候选单元的有效性、skewness 和碰撞；
- 局部候选单元不合格时，对相应源面提前停止生长；
- 输出边界层体网格，以及供后续远场网格生成使用的外边界表面。

当前命令行程序支持 `Wall` 和 `Farfield` 两类边界。金字塔、四面体过渡单元以及复杂局部过渡尚未接入当前规则层生成流程。

## 2. 主程序在哪里

命令行主函数位于：

```text
apps/boundary_mesh_cli.cpp
```

`main()` 只负责把命令行参数转换为字符串数组，然后调用：

```cpp
boundary_mesh::runBoundaryMeshCommand(arguments, std::cout, std::cerr);
```

命令实现位于：

```text
src/cli/boundary_mesh_command.cpp
```

真正执行规则边界层生长的核心函数是：

```cpp
boundary_mesh::generateRegularLayers(...);
```

它声明在：

```text
include/boundary_mesh/growth/regular_layer_generator.hpp
```

完整调用链为：

```text
main()
  -> runBoundaryMeshCommand()
  -> readCgnsSurface()
  -> SurfaceTopologyBuilder::build()
  -> GrowthPatchBuilder::build()
  -> GrowthFrontBuilder::buildInitial()
  -> generateRegularLayers()
  -> writeLegacyVtk()
```

## 3. 编译环境

推荐环境：

- Windows 10/11；
- Visual Studio 2022，安装“使用 C++ 的桌面开发”；
- CMake 3.20 或更高版本；
- Git；
- C++17 编译器。

工程通过 Git submodule 提供 Eigen、tiger_geom、HDF5 和 CGNS。首次取得工程后执行：

```powershell
git submodule update --init --recursive
```

如果某个子模块目录为空，先执行上述命令，再配置 CMake。

## 4. 独立编译

在工程根目录打开 PowerShell：

```powershell
cmake -S . -B build
cmake --build build --config Release
```

生成的命令行程序为：

```text
build/Release/boundary_mesh_cli.exe
```

Debug 版本可这样构建：

```powershell
cmake --build build --config Debug
```

对应程序为：

```text
build/Debug/boundary_mesh_cli.exe
```

默认启用 CGNS 输入、VTK 输出和命令行程序。只想构建不依赖 CGNS 的核心算法库时，可以配置：

```powershell
cmake -S . -B build-no-io -DBOUNDARY_MESH_ENABLE_CGNS_IO=OFF
cmake --build build-no-io --config Release
```

关闭 CGNS IO 后不会生成 `boundary_mesh_cli.exe`。

## 5. 准备输入文件

### 5.1 CGNS 表面网格

输入文件必须满足以下条件：

- CGNS 中的 Zone 类型为 `Unstructured`；
- 坐标名称为 `CoordinateX`、`CoordinateY`、`CoordinateZ`；
- 表面单元为 `TRI_3` 或 `QUAD_4`；
- Zone 接口可以包含用于连接描述的 `BAR_2`；
- 坐标必须是有限数值，不能包含 `NaN` 或无穷值；
- 表面拓扑必须有效，不能存在越界顶点、重复顶点、非法共享边等问题；
- Wall 面顶点顺序决定面法向，遵循右手法则。

程序会根据 CGNS Zone 编号读取边界类型。Zone 名称可以是 `1`、`2`、`3` 等，但 `.bc.txt` 中填写的是 CGNS 的 Zone 编号，不是任意字符串标签。

### 5.2 边界条件映射文件

CGNS 文件旁必须有一个同名的 `.bc.txt` 文件。

例如：

```text
模型目录/
  2dot5_cf.cgns
  2dot5_cf.bc.txt
```

`2dot5_cf.bc.txt` 示例：

```text
Wall:
1
2
3

Far:
4
5
```

规则如下：

1. 段名只能写成 `Wall:` 或 `Far:`，区分大小写；
2. 每行填写一个正整数 Zone 编号；
3. CGNS 中的每个 Zone 都必须出现一次；
4. 同一个 Zone 不能重复归类；
5. `.bc.txt` 必须和 CGNS 文件位于同一目录；
6. 文件名必须是 `<CGNS文件名去掉扩展名>.bc.txt`。

当前 CLI 的 `.bc.txt` 解析器尚未提供 `Symmetry:` 段。

## 6. 运行命令行程序

### 6.1 基本格式

```powershell
.\build\Release\boundary_mesh_cli.exe `
    --input "输入文件.cgns" `
    --first-height 0.1 `
    --growth-ratio 1.2 `
    --layer-count 10
```

### 6.2 完整示例

```powershell
.\build\Release\boundary_mesh_cli.exe `
    --input "C:\Users\zpern\Desktop\todo\九院项目质量对标\test_case\2dot5_cf\2dot5_cf.cgns" `
    --first-height 0.1 `
    --growth-ratio 1.2 `
    --layer-count 10 `
    --maximum-skewness 0.95 `
    --max-neighbor-layer-difference 1 `
    --output-prefix ".\build\real_case\2dot5_cf_10_layers"
```

PowerShell 中行尾的反引号 `` ` `` 表示命令在下一行继续。也可以将所有参数写在同一行。

### 6.3 参数说明

| 参数 | 是否必填 | 说明 | 约束或默认值 |
|---|---:|---|---|
| `--input` | 是 | CGNS 表面网格路径 | 文件旁必须存在同名 `.bc.txt` |
| `--first-height` | 是 | 第一层生长步长 | 有限数且大于 0 |
| `--growth-ratio` | 是 | 相邻层步长增长倍率 | 有限数且大于 0 |
| `--layer-count` | 是 | 每个 Wall 顶点请求的最大生长层数 | 正整数 |
| `--maximum-skewness` | 否 | 候选体单元允许的最大 skewness | 默认 `0.95`，范围 `[0,1]` |
| `--max-neighbor-layer-difference` | 否 | 相邻源面最终层数允许相差的最大值 | 默认 `1`，允许为 `0` |
| `--output-prefix` | 否 | 两个 VTK 文件共用的输出前缀 | 默认使用输入 CGNS 的目录和文件名 |

命令行模式会把同一组 `first_height`、`growth_ratio` 和 `layer_count` 赋给所有 Wall 顶点。若不同顶点需要不同参数，应通过 C++ 库接口构造 `SourceVertexGrowthProfile`。

第 `k` 层未经平滑前的基准步长来自上一层实际步长乘以 `growth_ratio`。实际生长过程中还会对活动前沿的方向和步长进行平滑，因此最终局部步长可能和简单的等比数列略有差异。

### 6.4 `layer-count` 的含义

`--layer-count 1` 表示生成一层体单元。一个体单元层必须由底面和顶面两层表面顶点构成，因此在 ParaView 中会看到两张表面，但它仍然只是一层 Prism/Hexa 体单元。

判断实际生成层数应查看控制台：

```text
generate 1 boundarylayer
finish 1 boundarylayer. add 58126 cell
```

或者查看 VTK 中体单元的层次，不应把上下两张顶点面误认为两层体单元。

## 7. 输出文件

假设使用：

```text
--output-prefix .\build\real_case\case10
```

程序会直接覆盖并生成：

```text
build/real_case/case10_boundary_layer.vtk
build/real_case/case10_farfield_boundary.vtk
```

两个文件均为 ASCII Legacy VTK `UNSTRUCTURED_GRID`。

### 7.1 边界层体网格

```text
*_boundary_layer.vtk
```

包含实际成功提交的三棱柱和六面体体单元。因质量、碰撞或邻域层差限制而停止的面，不会继续向后生成单元。

### 7.2 远场边界表面

```text
*_farfield_boundary.vtk
```

包含原始 Farfield 表面和边界层最终外露接口，可供后续远场体网格生成使用。

如果没有提供 `--output-prefix`，输出前缀默认为输入文件路径。例如输入：

```text
D:\case\model.cgns
```

默认输出：

```text
D:\case\model_boundary_layer.vtk
D:\case\model_farfield_boundary.vtk
```

## 8. 控制台输出

每次实际尝试生成一层时会输出：

```text
generate 1 boundarylayer
finish 1 boundarylayer. add 58126 cell
```

运行结束后会输出网格统计信息：

```text
input_vertices=52010
input_faces=58599
volume_cells=58126
farfield_faces=58726
maximum_skewness=0.95
max_neighbor_layer_difference=1
```

停止原因统计含义如下：

| 输出字段 | 含义 |
|---|---|
| `stop_none` | 尚未记录完成或停止原因 |
| `stop_vertex_layer_limit` | 源面顶点达到外部请求层数 |
| `stop_degenerate_candidate` | 候选体单元退化 |
| `stop_reversed_candidate` | 候选体单元整体反转 |
| `stop_locally_inverted_candidate` | 候选体单元局部翻转 |
| `stop_skewness_exceeded` | 候选体单元 skewness 超过上限 |
| `stop_collision` | 候选单元发生非法几何接触或碰撞 |
| `stop_neighbor_layer_constraint` | 为限制相邻区域层数差而提前停止 |

`stop_vertex_layer_limit` 通常表示正常完成请求层数，不是错误。

## 9. 返回码

命令行程序返回：

| 返回码 | 含义 |
|---:|---|
| `0` | 成功生成并写出两个 VTK 文件 |
| `2` | 命令行参数缺失、重复或非法 |
| `3` | CGNS 或 `.bc.txt` 读取失败 |
| `4` | 表面拓扑构建失败 |
| `5` | GrowthPatch 或初始 GrowthFront 构建失败 |
| `6` | 边界层生成失败 |
| `7` | 输出目录创建或 VTK 写出失败 |

## 10. 运行测试

构建并运行 Debug 测试：

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

构建并运行 Release 测试：

```powershell
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

只运行某项测试时可以使用 `-R`：

```powershell
ctest --test-dir build -C Debug `
    -R "boundary_mesh_regular_layer_growth_pipeline_test" `
    --output-on-failure
```

## 11. 常见问题

### 12.1 提示 `failed to read CGNS surface`

检查：

- CGNS 路径是否正确；
- 同目录下是否存在同名 `.bc.txt`；
- `.bc.txt` 是否覆盖所有 Zone；
- 段名是否严格写成 `Wall:` 和 `Far:`；
- Zone 是否为非结构网格；
- 面单元是否为 `TRI_3` 或 `QUAD_4`。

### 12.2 `layer-count=1` 为什么看到两张表面

一层体单元由底面和顶面构成，所以会出现两张表面。检查 `volume_cells` 和逐层的 `add ... cell` 输出判断实际体单元层数。

### 12.3 为什么部分区域提前停止

查看 `stop_*` 统计。常见原因包括：

- 候选单元翻转或退化；
- skewness 超过阈值；
- 撞到原始表面、已提交边界层或同层其他候选；
- 相邻区域层数差超过允许值。

不要仅通过放宽 skewness 判断碰撞问题；应先根据对应停止原因定位。

### 12.4 为什么没有生成命令行程序

确认 CMake 配置中：

```text
BOUNDARY_MESH_ENABLE_CGNS_IO=ON
```

并确认 `third/hdf5`、`third/cgns` 子模块已经初始化。

## 12. 工程目录概览

```text
apps/            命令行主程序
benchmarks/      性能与真实数据流水线程序
cmake/           编译选项和依赖设置
docs/design/     架构和模块设计文档
docs/plans/      各阶段实施计划
include/         对外公开头文件
src/             模块实现
tests/unit/      单元测试
tests/integration/ 集成测试
third/           第三方子模块
```

建议初次阅读代码时按以下顺序进入：

```text
apps/boundary_mesh_cli.cpp
src/cli/boundary_mesh_command.cpp
include/boundary_mesh/growth/regular_layer_generator.hpp
src/growth/regular_layer_generator.cpp
```
