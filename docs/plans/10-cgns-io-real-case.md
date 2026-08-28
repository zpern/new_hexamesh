# CGNS IO, CLI, and Real-Case Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** 从数字 Zone 的 CGNS 表面和同名 `.bc.txt` 构造统一 `SurfaceMesh`，运行完整边界层流水线，并输出两个纯几何 ASCII Legacy VTK。

**Architecture:** `BoundaryMesh::IO` 私有适配官方 cgnslib/HDF5，内部解析边界映射并根据显式 Zone 连接确定性合并顶点；CLI 只编排既有拓扑和生长模块。VTK 写出器对 `SurfaceMesh` 与 `VolumeMesh` 提供无算法状态的重载。

**Tech Stack:** C++17、CMake 3.20+、CGNS v4.5.2、HDF5 1.14.6、Eigen、CTest、MSVC Debug/Release、ASCII Legacy VTK。

## Global Constraints

- 只由主代理内联执行，不使用子代理。
- 阶段 06 和 07 必须先完成，阶段 08、09 不在本轮实施范围。
- 使用 `third/cgns` 的 v4.5.2 与 `third/hdf5` 的 hdf5-1.14.6 固定 Git submodule。
- CGNS/HDF5 只能由 `BoundaryMesh::IO` 私有使用，Growth、Quality、Spatial 不得包含第三方 IO 头文件。
- 只读取一个三维非结构表面 Base、数字 Zone、`TRI_3`、`QUAD_4` 和显式 PointList/PointListDonor 连接。
- `.bc.txt` 只接受 `Far:`、`Wall:` 与逐行正整数 Zone。
- 跨 Zone 顶点只通过显式连接合并，不使用坐标容差或最近点。
- 两个输出均为纯几何 ASCII Legacy VTK，不写 `POINT_DATA` 或 `CELL_DATA`。
- CLI 全局参数复制为每个 Wall 顶点独立的 profile；不增加逐点 profile 文件。
- 已有输出文件默认直接覆盖。
- 新公共字段和枚举值必须带中文 `//` 注释。
- `.superpowers/` 永远不加入提交。
- 每项任务先验证 RED，再完成 GREEN、目标测试、全量回归和独立提交。

---

## File Structure

```text
.gitmodules                                      CGNS/HDF5 固定版本
third/cgns                                      官方 CGNS v4.5.2
third/hdf5                                      官方 HDF5 1.14.6
cmake/Dependencies.cmake                        父工程复用和 standalone 依赖
CMakeLists.txt                                   C、C++、IO、CLI target

include/boundary_mesh/io/boundary_condition_map_error.hpp 映射错误
include/boundary_mesh/io/cgns_surface_error.hpp           CGNS 读取错误
include/boundary_mesh/io/cgns_surface_reader.hpp          CGNS 公共入口
include/boundary_mesh/io/legacy_vtk_writer.hpp            两类 VTK 写出
include/boundary_mesh/io/vtk_write_error.hpp               VTK 错误

src/io/boundary_condition_map.hpp                内部 Zone 映射类型
src/io/boundary_condition_map.cpp                文本解析和完备性校验
src/io/cgns_file.hpp                             cgnslib 文件句柄 RAII
src/io/cgns_surface_reader.cpp                   Zone、元素、连接和合并
src/io/legacy_vtk_writer.cpp                     ASCII Legacy VTK

src/cli/boundary_mesh_command.hpp                可测试 CLI 入口
src/cli/boundary_mesh_command.cpp                参数、流水线和诊断
apps/boundary_mesh_cli.cpp                       main 薄适配

tests/helpers/cgns_fixture.hpp                   小型 CGNS 写入辅助
tests/helpers/cgns_fixture.cpp                   官方 cgnslib 测试数据
tests/unit/io/boundary_condition_map_test.cpp    `.bc.txt` 语法
tests/unit/io/cgns_surface_reader_test.cpp       单 Zone 与错误路径
tests/unit/io/cgns_zone_merge_test.cpp           跨 Zone 合并
tests/unit/io/legacy_vtk_writer_test.cpp          六种单元写出
tests/integration/cgns_cli_pipeline_test.cpp      小型端到端
tests/integration/cgns_real_case_test.cpp         外部路径真实案例
benchmarks/cgns_pipeline_benchmark.cpp            时间和峰值工作集
tests/CMakeLists.txt                              注册快速测试和工具
docs/design/roadmap.md                           阶段完成状态
docs/design/modules/cgns-io-and-real-case.md     最终接口同步
```

---

### Task 1: `.bc.txt` 边界映射解析

**Files:**
- Create: `include/boundary_mesh/io/boundary_condition_map_error.hpp`
- Create: `src/io/boundary_condition_map.hpp`
- Create: `src/io/boundary_condition_map.cpp`
- Modify: `CMakeLists.txt`
- Test: `tests/unit/io/boundary_condition_map_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `.bc.txt` 路径和 CGNS 中按数字升序排列的 Zone ID。
- Produces: `readBoundaryConditionMap(...)` 返回按 Zone ID 排序的 `BoundaryZoneMap`。

- [x] **Step 1: 写标准格式和完备性 RED 测试**

```cpp
const auto map = readBoundaryConditionMap(
    path,
    std::vector<std::uint32_t>{1, 2, 3, 4});
assert(map.hasValue());
assert(map.value().entries.size() == 4);
assert(map.value().find(1)->kind == SurfaceBoundaryKind::Farfield);
assert(map.value().find(3)->kind == SurfaceBoundaryKind::Wall);
assert(map.value().find(4)->region_id == 4);
```

测试文件内容必须使用已确认格式：

```text
Far:
1
2

Wall:
3
4
```

- [x] **Step 2: 写确定性错误 RED 测试**

分别覆盖文件缺失、未知区段、区段前出现 Zone、非数字、零、`uint32_t` 溢出、重复 Zone、缺失 Zone 和映射多余 Zone。断言 `BoundaryMapErrorCode`、行号和 Zone ID，而不是只断言 failure。

- [x] **Step 3: 运行 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_boundary_condition_map_test
```

Expected: FAIL，缺少 IO target 或 `boundary_condition_map.hpp`。

- [x] **Step 4: 定义错误和内部映射**

```cpp
enum class BoundaryMapErrorCode
{
    FileOpenFailure,      // 无法打开同名边界映射文件
    InvalidSection,       // 区段不是 Far 或 Wall
    ZoneOutsideSection,   // Zone 出现在任何合法区段以前
    InvalidZoneId,        // Zone 不是可表示的正 uint32_t
    DuplicateZone,        // 同一 Zone 被配置多次
    MissingZone,          // CGNS Zone 未出现在映射中
    UnknownZone           // 映射引用了 CGNS 中不存在的 Zone
};

struct BoundaryMapError
{
    BoundaryMapErrorCode code{BoundaryMapErrorCode::FileOpenFailure}; // 错误分类
    std::filesystem::path path; // 发生错误的映射文件
    std::size_t line{}; // 发生语法错误的 1-based 行号
    std::uint32_t zone_id{}; // 与错误相关的 Zone，未知时为 0
};
```

内部类型固定为：

```cpp
struct BoundaryZoneEntry
{
    std::uint32_t zone_id{};
    SurfaceBoundaryKind kind{SurfaceBoundaryKind::Farfield};
    std::uint32_t region_id{};
};

struct BoundaryZoneMap
{
    std::vector<BoundaryZoneEntry> entries;
    const BoundaryZoneEntry *find(std::uint32_t zone_id) const noexcept;
};

Result<BoundaryZoneMap, BoundaryMapError>
readBoundaryConditionMap(
    const std::filesystem::path &path,
    const std::vector<std::uint32_t> &available_zone_ids);
```

- [x] **Step 5: 实现严格逐行解析**

去除行首尾 ASCII 空白；空行跳过。`Far:` 和 `Wall:` 精确区分大小写，其余带冒号行返回 `InvalidSection`。Zone 使用 `std::from_chars` 完整消费，拒绝符号、尾随字符、零和溢出。最后排序输入 Zone、排序映射、线性比较完备性。

- [x] **Step 6: 运行目标测试与全量回归**

```powershell
cmake --build build --config Debug --target boundary_mesh_boundary_condition_map_test
ctest --test-dir build -C Debug -R boundary_mesh_boundary_condition_map_test --output-on-failure
ctest --test-dir build -C Debug --output-on-failure
```

- [x] **Step 7: 提交映射解析器**

```powershell
git add CMakeLists.txt include/boundary_mesh/io/boundary_condition_map_error.hpp src/io/boundary_condition_map.hpp src/io/boundary_condition_map.cpp tests/CMakeLists.txt tests/unit/io/boundary_condition_map_test.cpp
git diff --cached --check
git commit -m "feat: parse CGNS boundary zone maps"
```

---

### Task 2: 纯几何 ASCII Legacy VTK 写出

**Files:**
- Create: `include/boundary_mesh/io/vtk_write_error.hpp`
- Create: `include/boundary_mesh/io/legacy_vtk_writer.hpp`
- Create: `src/io/legacy_vtk_writer.cpp`
- Modify: `CMakeLists.txt`
- Test: `tests/unit/io/legacy_vtk_writer_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: 输出路径和 `SurfaceMesh` 或 `VolumeMesh`。
- Produces: `writeLegacyVtk(...)`，成功值为 `std::monostate`。

- [x] **Step 1: 写六种单元和纯几何 RED 测试**

构造同时包含 Tetra、Pyramid、Prism、Hexa 的 `VolumeMesh`，以及 Triangle/Quad `SurfaceMesh`：

```cpp
const auto volume_status = writeLegacyVtk(volume_path, volume_mesh);
const auto surface_status = writeLegacyVtk(surface_path, surface_mesh);
assert(volume_status.hasValue());
assert(surface_status.hasValue());
assert(readCellTypes(volume_path) ==
       std::vector<int>({10, 14, 13, 12}));
assert(readCellTypes(surface_path) ==
       std::vector<int>({5, 9}));
assert(fileText(volume_path).find("CELL_DATA") == std::string::npos);
assert(fileText(surface_path).find("POINT_DATA") == std::string::npos);
```

- [x] **Step 2: 写覆盖、精度和错误 RED 测试**

先写旧内容，再调用 writer，断言旧内容被替换。使用需要 17 位十进制区分的 double 坐标并重新解析，断言逐值相等。越界 VertexId、无法创建的目录、计数溢出注入点分别断言确定错误。

- [x] **Step 3: 运行 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_legacy_vtk_writer_test
```

Expected: FAIL，缺少 `legacy_vtk_writer.hpp`。

- [x] **Step 4: 定义写出错误和重载**

```cpp
enum class VtkWriteErrorCode
{
    FileOpenFailure,       // 无法创建临时输出文件
    InvalidVertexReference,// 单元引用了不存在的顶点
    CountOverflow,         // Legacy VTK 连接计数不可表示
    WriteFailure,          // 流写入或关闭失败
    ReplaceFailure         // 无法用完整临时文件替换目标
};

struct VtkWriteError
{
    VtkWriteErrorCode code{VtkWriteErrorCode::FileOpenFailure}; // 错误分类
    std::filesystem::path path; // 目标 VTK 路径
    std::size_t cell_index{}; // 相关单元下标，不适用时为 0
    VertexId vertex_id{}; // 相关顶点编号，不适用时为 0
};

using VtkWriteStatus = Result<std::monostate, VtkWriteError>;

VtkWriteStatus writeLegacyVtk(
    const std::filesystem::path &path,
    const SurfaceMesh &mesh);

VtkWriteStatus writeLegacyVtk(
    const std::filesystem::path &path,
    const VolumeMesh &mesh);
```

- [x] **Step 5: 实现验证、写临时文件和替换**

先完整验证所有引用和连接总长度，再创建 `<target>.tmp`。头部固定为：

```text
# vtk DataFile Version 3.0
BoundaryMesh
ASCII
DATASET UNSTRUCTURED_GRID
```

坐标输出 `double` 和 `max_digits10`。成功关闭后删除同名旧目标并重命名临时文件；所有失败路径清理临时文件。

- [x] **Step 6: 运行目标测试和回归**

```powershell
cmake --build build --config Debug --target boundary_mesh_legacy_vtk_writer_test
ctest --test-dir build -C Debug -R boundary_mesh_legacy_vtk_writer_test --output-on-failure
ctest --test-dir build -C Debug --output-on-failure
```

- [x] **Step 7: 提交 VTK writer**

```powershell
git add CMakeLists.txt include/boundary_mesh/io/vtk_write_error.hpp include/boundary_mesh/io/legacy_vtk_writer.hpp src/io/legacy_vtk_writer.cpp tests/CMakeLists.txt tests/unit/io/legacy_vtk_writer_test.cpp
git diff --cached --check
git commit -m "feat: write pure geometry legacy VTK"
```

---

### Task 3: CGNS/HDF5 依赖与单 Zone 读取

**Files:**
- Modify: `.gitmodules`
- Add submodule: `third/hdf5`
- Add submodule: `third/cgns`
- Modify: `cmake/Dependencies.cmake`
- Modify: `CMakeLists.txt`
- Create: `include/boundary_mesh/io/cgns_surface_error.hpp`
- Create: `include/boundary_mesh/io/cgns_surface_reader.hpp`
- Create: `src/io/cgns_file.hpp`
- Create: `src/io/cgns_surface_reader.cpp`
- Create: `tests/helpers/cgns_fixture.hpp`
- Create: `tests/helpers/cgns_fixture.cpp`
- Test: `tests/unit/io/cgns_surface_reader_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `.cgns` 路径；自动派生同目录同 stem `.bc.txt`。
- Produces: `readCgnsSurface(...)` 返回统一 `SurfaceMesh` 或 `CgnsSurfaceError`。

- [x] **Step 1: 写单 Zone Triangle/Quad RED 测试**

测试辅助用 cgnslib 创建数字 Zone `1` 与混合元素，再写 `case.bc.txt`：

```cpp
const auto mesh = readCgnsSurface(case_path);
assert(mesh.hasValue());
assert(mesh.value().vertices.size() == 5);
assert(mesh.value().faces.size() == 2);
assert(std::holds_alternative<Triangle>(mesh.value().faces[0]));
assert(std::holds_alternative<Quad>(mesh.value().faces[1]));
assert(mesh.value().face_tags[0].kind == SurfaceBoundaryKind::Wall);
assert(mesh.value().face_tags[0].region_id == 1);
```

- [x] **Step 2: 写 Base、Zone、元素和坐标错误 RED 测试**

覆盖：空文件、多 Base、`cell_dimension != 2`、`physical_dimension != 3`、Structured Zone、非数字/零/重复 Zone、缺坐标、NaN、Inf、线单元、Polygon、高阶面、体单元和越界连接。每例断言错误 code、Zone ID 与元素 ID。

- [x] **Step 3: 运行 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_cgns_surface_reader_test
```

Expected: FAIL，缺少 CGNS 依赖或 reader 头文件。

- [x] **Step 4: 固定子模块并配置最小静态依赖**

```powershell
git submodule add https://github.com/HDFGroup/hdf5.git third/hdf5
git -C third/hdf5 checkout hdf5-1.14.6
git submodule add https://github.com/CGNS/CGNS.git third/cgns
git -C third/cgns checkout v4.5.2
```

`.gitmodules` 不写跟踪 branch；外层仓库直接固定两个 tag 对应的 gitlink 提交。

根工程改为 `project(... LANGUAGES C CXX)`。增加：

```cmake
option(BOUNDARY_MESH_ENABLE_CGNS_IO
    "Build CGNS IO, CLI, and CGNS tests" ON)
```

standalone 配置关闭 HDF5 tools/examples/Fortran/C++/Java、CGNS tools/tests/Fortran/shared，构建 HDF5 静态库并让 CGNS v4.5.2 发现其 build-tree package。父工程若已有 `CGNS::cgns-static`、`CGNS::cgns-shared` 或规范化 `BoundaryMesh::CGNS`，不得重复添加子目录。建立唯一适配 target `BoundaryMesh::CGNS`。

- [x] **Step 5: 定义 CGNS 公共错误和入口**

```cpp
enum class CgnsSurfaceErrorCode
{
    FileOpenFailure,              // 无法打开 CGNS 文件
    BoundaryMapFailure,           // 同名 .bc.txt 解析失败
    InvalidBaseCount,             // Base 数量不是 1
    InvalidDimensions,            // 不是 cell dimension 2 / physical dimension 3
    InvalidZoneType,              // Zone 不是 Unstructured
    InvalidZoneId,                // Zone 名不是唯一正 uint32_t
    MissingCoordinate,            // 缺少 X、Y 或 Z
    NonFiniteCoordinate,          // 坐标包含 NaN 或无穷
    UnsupportedElementType,       // 元素不是 TRI_3 或 QUAD_4
    InvalidElementReference,      // 元素顶点索引越界
    InvalidConnectivity,          // Zone 连接信息不完整或越界
    ConnectivityCoordinateMismatch,// 显式连接两端坐标不相等
    VertexIdOverflow,             // 合并后顶点数超出 VertexId
    FaceIdOverflow,               // 面数超出 SurfaceFaceId
    CgnsLibraryFailure            // 官方 cgnslib 调用失败
};

struct CgnsSurfaceError
{
    CgnsSurfaceErrorCode code{CgnsSurfaceErrorCode::FileOpenFailure}; // 错误分类
    std::filesystem::path path; // CGNS 或映射文件
    std::uint32_t zone_id{}; // 相关数字 Zone，不适用时为 0
    std::uint64_t element_id{}; // 相关 CGNS 元素，不适用时为 0
    BoundaryMapError boundary_map_error{}; // BoundaryMapFailure 的具体原因
    std::string detail; // cgnslib 或维度诊断的稳定文本
};

using CgnsSurfaceResult = Result<SurfaceMesh, CgnsSurfaceError>;

CgnsSurfaceResult readCgnsSurface(
    const std::filesystem::path &cgns_path);
```

- [x] **Step 6: 实现 RAII 和单 Zone 读取**

`CgnsFile` 析构时只在已成功打开时调用 `cg_close`。reader 先读取 Base 和全部 Zone 名，排序并调用 Task 1 映射解析；再读取三坐标与 ElementSection。面顺序按 `(zone_id, element_id)`，CGNS 1-based 顶点转换为内部 0-based。

- [x] **Step 7: 运行目标测试和回归**

```powershell
cmake -S . -B build -DBOUNDARY_MESH_ENABLE_CGNS_IO=ON
cmake --build build --config Debug --target boundary_mesh_cgns_surface_reader_test
ctest --test-dir build -C Debug -R boundary_mesh_cgns_surface_reader_test --output-on-failure
ctest --test-dir build -C Debug --output-on-failure
```

再配置一个关闭 IO 的 build tree，确认核心仍可编译：

```powershell
cmake -S . -B build-no-io -DBOUNDARY_MESH_ENABLE_CGNS_IO=OFF
cmake --build build-no-io --config Debug --target boundary_mesh_boundary_layer
```

- [x] **Step 8: 提交依赖和 reader 基础**

```powershell
git add .gitmodules third/hdf5 third/cgns cmake/Dependencies.cmake CMakeLists.txt include/boundary_mesh/io/cgns_surface_error.hpp include/boundary_mesh/io/cgns_surface_reader.hpp src/io/cgns_file.hpp src/io/cgns_surface_reader.cpp tests/helpers/cgns_fixture.hpp tests/helpers/cgns_fixture.cpp tests/CMakeLists.txt tests/unit/io/cgns_surface_reader_test.cpp
git diff --cached --check
git commit -m "feat: read mixed CGNS surface zones"
```

---

### Task 4: 显式跨 Zone 连接与确定性合并

**Files:**
- Modify: `src/io/cgns_surface_reader.cpp`
- Modify: `tests/helpers/cgns_fixture.hpp`
- Modify: `tests/helpers/cgns_fixture.cpp`
- Test: `tests/unit/io/cgns_zone_merge_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: 数字 Zone 的局部坐标、元素和 PointList/PointListDonor 连接。
- Produces: 规范代表排序后的连续全局顶点及重映射面。

- [x] **Step 1: 写两 Zone 共享边 RED 测试**

Zone 1 与 Zone 2 各含一个 Quad，共享两个显式连接点：

```cpp
const auto mesh = readCgnsSurface(path);
assert(mesh.hasValue());
assert(mesh.value().vertices.size() == 6); // 4 + 4 - 2
assert(mesh.value().faces.size() == 2);
assert(sharedGlobalVertexCount(mesh.value().faces[0],
                               mesh.value().faces[1]) == 2);
```

- [x] **Step 2: 写顺序不变量和非法连接 RED 测试**

分别反转 Zone 创建顺序、connection 创建顺序、PointList 顺序，断言坐标、面、标签逐项相同。再覆盖 donor 不存在、两侧长度不同、索引越界、重复矛盾连接和显式连接坐标不一致。

- [x] **Step 3: 运行 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_cgns_zone_merge_test
```

Expected: FAIL，reader 尚未合并显式连接。

- [x] **Step 4: 实现局部键和并查集**

```cpp
struct LocalVertexKey
{
    std::uint32_t zone_id{};
    std::uint64_t local_vertex_id{}; // 0-based
};
```

所有局部键先按 `(zone_id, local_vertex_id)` 排序并分配并查集下标。每条连接无向规范化后排序去重，再按排序顺序 union。union 根不决定最终编号；最终对每个集合重新计算最小局部键作为规范代表。

- [x] **Step 5: 生成稳定全局编号并重映射**

逐集合验证坐标三个分量完全相等。规范代表排序后分配连续 `VertexId`。面按 `(zone_id, element_id)` 排序后重映射，标签的 `region_id` 等于数字 Zone。任何溢出在分配前返回错误。

- [x] **Step 6: 运行目标测试和回归**

```powershell
cmake --build build --config Debug --target boundary_mesh_cgns_zone_merge_test boundary_mesh_cgns_surface_reader_test
ctest --test-dir build -C Debug -R "boundary_mesh_cgns_(zone_merge|surface_reader)_test" --output-on-failure
ctest --test-dir build -C Debug --output-on-failure
```

- [x] **Step 7: 提交 Zone 合并**

```powershell
git add src/io/cgns_surface_reader.cpp tests/helpers/cgns_fixture.hpp tests/helpers/cgns_fixture.cpp tests/CMakeLists.txt tests/unit/io/cgns_zone_merge_test.cpp
git diff --cached --check
git commit -m "feat: merge explicit CGNS zone connections"
```

---

### Task 5: CLI 参数与小型完整流水线

**Files:**
- Create: `src/cli/boundary_mesh_command.hpp`
- Create: `src/cli/boundary_mesh_command.cpp`
- Create: `apps/boundary_mesh_cli.cpp`
- Modify: `CMakeLists.txt`
- Test: `tests/integration/cgns_cli_pipeline_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: CLI 字符串参数、stdout/stderr 流和阶段 07 完整库入口。
- Produces: `runBoundaryMeshCommand(...)` 返回进程退出码，并由 main 原样返回。

- [x] **Step 1: 写参数默认值和错误 RED 测试**

```cpp
std::ostringstream out;
std::ostringstream err;
const int code = runBoundaryMeshCommand(args, out, err);
assert(code == 0);
assert(out.str().find("maximum_skewness=0.95") != std::string::npos);
assert(out.str().find("max_layer_diff=1") != std::string::npos);
assert(err.str().empty());
```

缺少必填项、重复参数、未知参数、非法 double、负数、`layer_count` 溢出、`max_layer_diff` 溢出分别返回参数类非零退出码。

- [x] **Step 2: 写封闭小立方体一层 RED 测试**

测试辅助创建一个数字 Wall Zone 和一个 Far Zone，通过显式连接形成封闭、方向一致的混合表面。运行：

```text
--input cube.cgns
--first-height 0.1
--growth-ratio 1.0
--layer-count 1
--output-prefix result/cube
```

断言返回 0、生成两个 VTK、体网格包含预期 Prism/Hexa、远场边界可被测试解析器重新读取，且 stdout 只含汇总。

- [x] **Step 3: 运行 RED**

```powershell
cmake --build build --config Debug --target boundary_mesh_cgns_cli_pipeline_test
```

Expected: FAIL，缺少命令 target 或编排入口。

- [x] **Step 4: 实现可测试参数模型**

```cpp
struct BoundaryMeshCommandOptions
{
    std::filesystem::path input; // CGNS 输入路径
    Scalar first_height{}; // 全部 Wall 顶点的第一层高度
    Scalar growth_ratio{}; // 全部 Wall 顶点的增长率
    std::uint32_t layer_count{}; // 全部 Wall 顶点的请求层数
    Scalar maximum_skewness{0.95}; // 候选单元最大 skewness
    std::uint32_t max_layer_diff{1}; // 相邻源面最大层数差
    std::filesystem::path output_prefix; // 两个 VTK 的公共前缀
};

int runBoundaryMeshCommand(
    const std::vector<std::string> &arguments,
    std::ostream &output,
    std::ostream &error);
```

解析使用 `from_chars`/严格完整消费；double 若当前 MSVC `from_chars` 不完整则使用经典 locale 的 `istringstream` 并拒绝尾随字符。默认 prefix 为输入路径 `parent_path / stem`。

- [x] **Step 5: 按固定流水线编排**

```text
readCgnsSurface
SurfaceTopologyBuilder::build
GrowthPatchBuilder::build
GrowthFrontBuilder::buildInitial
为 patch.vertices() 创建独立 SourceVertexGrowthProfile
generateRegularLayers(surface, topology, patch, front, profiles, options)
writeLegacyVtk(boundary_layer)
writeLegacyVtk(farfield_boundary)
```

局部停止仍返回成功。输入、拓扑、生长程序错误或 writer 错误映射到不同非零退出类别。任何 writer 调用只在完整生长成功后发生。

- [x] **Step 6: 实现汇总和 main 薄层**

stdout 输出输入计数、Zone/边界计数、各体单元计数、远场表面计数、停止原因计数和阶段耗时。main 只转换 `argv[1..]` 并调用 `runBoundaryMeshCommand`，不包含算法分支。

- [x] **Step 7: 运行目标测试和全量回归**

```powershell
cmake --build build --config Debug --target boundary_mesh_cli boundary_mesh_cgns_cli_pipeline_test
ctest --test-dir build -C Debug -R boundary_mesh_cgns_cli_pipeline_test --output-on-failure
ctest --test-dir build -C Debug --output-on-failure
```

- [x] **Step 8: 提交 CLI**

```powershell
git add CMakeLists.txt src/cli/boundary_mesh_command.hpp src/cli/boundary_mesh_command.cpp apps/boundary_mesh_cli.cpp tests/CMakeLists.txt tests/integration/cgns_cli_pipeline_test.cpp
git diff --cached --check
git commit -m "feat: run CGNS boundary layer CLI"
```

---

### Task 6: `2dot5_cf` 真实案例与性能基准

**Files:**
- Create: `tests/integration/cgns_real_case_test.cpp`
- Create: `benchmarks/cgns_pipeline_benchmark.cpp`
- Modify: `tests/CMakeLists.txt`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: 命令行传入的真实 CGNS 路径；自动派生 `.bc.txt`。
- Produces: 固定计数验收、一层两个 VTK 和分阶段性能/峰值工作集报告。

- [x] **Step 1: 写真实输入计数验收程序**

程序只接受一个 CGNS 参数，读取后断言：

```cpp
assert(mesh.vertices.size() == 52010);
assert(countTriangles(mesh) == 13186);
assert(countQuads(mesh) == 45413);
assert(countKind(mesh, SurfaceBoundaryKind::Farfield) == 422);
assert(countKind(mesh, SurfaceBoundaryKind::Wall) == 58177);
```

缺少参数时打印 usage 并返回 2。该程序构建但不注册默认 CTest，因为真实数据路径由调用者提供。

- [x] **Step 2: 运行计数验收**

先确认同名映射存在。若目录中仍保留旧名 `2dot_cf.txt`，将其内容复制为规范文件 `2dot5_cf.bc.txt`，保留旧文件不删除：

```powershell
$caseDir = "C:\Users\zpern\Desktop\todo\九院项目质量对标\test_case\2dot5_cf"
if (-not (Test-Path -LiteralPath "$caseDir\2dot5_cf.bc.txt")) {
    Copy-Item -LiteralPath "$caseDir\2dot_cf.txt" `
              -Destination "$caseDir\2dot5_cf.bc.txt"
}
```

```powershell
cmake --build build --config Debug --target boundary_mesh_cgns_real_case_test
& .\build\tests\Debug\boundary_mesh_cgns_real_case_test.exe `
  "C:\Users\zpern\Desktop\todo\九院项目质量对标\test_case\2dot5_cf\2dot5_cf.cgns"
```

Expected: PASS 并打印六项固定计数。

- [x] **Step 3: 运行一层 CLI 真实案例**

```powershell
& .\build\Debug\boundary_mesh_cli.exe `
  --input "C:\Users\zpern\Desktop\todo\九院项目质量对标\test_case\2dot5_cf\2dot5_cf.cgns" `
  --first-height 0.1 `
  --growth-ratio 1.0 `
  --layer-count 1 `
  --maximum-skewness 0.95 `
  --max-neighbor-layer-difference 1 `
  --output-prefix ".\build\real_case\2dot5_cf"
```

Expected: exit 0；两个 VTK 非空，测试解析器可重读全部 POINTS/CELLS/CELL_TYPES；局部停止允许存在。

- [x] **Step 4: 实现 benchmark**

benchmark 接收同一 CGNS 路径和可选输出前缀，使用 `steady_clock` 分别测量读取、拓扑、一层生成、写出和总时间。Windows 用 `GetProcessMemoryInfo` 读取 `PeakWorkingSetSize`，非 Windows 打印 `peak_working_set=unavailable`。Windows 峰值大于 `1 GiB` 返回 3。

- [x] **Step 5: 构建并运行 Release benchmark**

```powershell
cmake --build build --config Release --target boundary_mesh_cgns_pipeline_benchmark
& .\build\Release\boundary_mesh_cgns_pipeline_benchmark.exe `
  "C:\Users\zpern\Desktop\todo\九院项目质量对标\test_case\2dot5_cf\2dot5_cf.cgns" `
  ".\build\real_case\benchmark_2dot5_cf"
```

Expected: exit 0，打印五类耗时和峰值工作集；Windows 峰值不超过 1 GiB。

- [x] **Step 6: 回归并提交真实案例工具**

```powershell
ctest --test-dir build -C Debug --output-on-failure
git add CMakeLists.txt tests/CMakeLists.txt tests/integration/cgns_real_case_test.cpp benchmarks/cgns_pipeline_benchmark.cpp
git diff --cached --check
git commit -m "test: verify 2dot5 CGNS workflow"
```

---

### Task 7: 确定性、双配置回归与文档收尾

**Files:**
- Modify: `tests/unit/io/cgns_zone_merge_test.cpp`
- Modify: `tests/integration/cgns_cli_pipeline_test.cpp`
- Modify: `docs/design/roadmap.md`
- Modify: `docs/design/modules/cgns-io-and-real-case.md`
- Modify: `docs/plans/10-cgns-io-real-case.md`

**Interfaces:**
- Consumes: 阶段 06、07、10 的最终实现。
- Produces: 顺序不变量、完整 Debug/Release 证据和路线图完成状态。

- [x] **Step 1: 增加完整顺序不变量测试**

同一 CGNS fixture 反转 Zone、Section、connection 和 PointList 创建顺序，断言：

```cpp
assert(forward.vertices == reversed.vertices);
assert(canonicalFaces(forward) == canonicalFaces(reversed));
assert(forward.face_tags == reversed.face_tags);
assert(fileText(forward_volume_vtk) == fileText(reversed_volume_vtk));
assert(fileText(forward_surface_vtk) == fileText(reversed_surface_vtk));
```

若 Eigen `Point3` 或标签没有 `operator==`，测试显式逐分量比较，不为测试扩展公共 API。

- [x] **Step 2: Debug 全量验证**

```powershell
cmake -S . -B build -DBOUNDARY_MESH_ENABLE_CGNS_IO=ON
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

Expected: 阶段 01–07 和 10 的全部快速测试通过，0 failed。

- [x] **Step 3: Release 全量验证**

```powershell
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Expected: 全部快速测试通过，0 failed。

- [x] **Step 4: 关闭 IO 验证**

```powershell
cmake -S . -B build-no-io -DBOUNDARY_MESH_ENABLE_CGNS_IO=OFF
cmake --build build-no-io --config Debug --target boundary_mesh_boundary_layer
```

Expected: 不构建 CGNS、HDF5、IO、CLI 或相关测试，核心库成功。

- [x] **Step 5: 更新设计和路线图**

将阶段 10 标记为“已完成”，同步实际 target、命令、依赖版本、真实案例输出和峰值工作集。阶段 08、09 继续保持“未开始”，不得因 VTK 支持其类型而误标完成。

- [x] **Step 6: 最终提交**

```powershell
git add tests/unit/io/cgns_zone_merge_test.cpp tests/integration/cgns_cli_pipeline_test.cpp docs/design/roadmap.md docs/design/modules/cgns-io-and-real-case.md docs/plans/10-cgns-io-real-case.md
git diff --cached --check
git commit -m "test: complete CGNS IO stage"
git status --short --branch
git log -16 --oneline --decorate
```

Expected: 仅 `.superpowers/` 未跟踪，阶段 06、07、10 的其他变更全部已提交。

---

## Final Verification

阶段 10 只有在以下条件全部满足时才可完成：

```text
CGNS v4.5.2 与 HDF5 1.14.6 固定且可由父工程复用
关闭 BOUNDARY_MESH_ENABLE_CGNS_IO 后核心仍可构建
同名 .bc.txt 严格映射全部数字 Zone
只支持 cell dimension 2 / physical dimension 3 的非结构表面
TRI_3/QUAD_4 按数字 Zone 和 element ID 确定排序
跨 Zone 顶点只由显式 PointList/Donor 合并
连接坐标逐值不等时明确失败
SurfaceMesh 和 VolumeMesh 的纯几何 ASCII VTK 正确
CLI 全局参数、默认值、退出类别和汇总正确
输入/拓扑/生长失败不写正式输出
2dot5_cf 六项输入计数完全匹配
2dot5_cf 一层 CLI 返回 0 且两个 VTK 可重读
Release benchmark 峰值工作集不超过 1 GiB
Debug/Release 快速 CTest 均为 0 failed
阶段 08、09 保持未开始
```

### 完成记录（2026-08-22）

- Debug：完整构建成功，CTest 45/45 通过；
- Release：完整构建成功，CTest 45/45 通过；
- IO-OFF：`boundary_mesh_boundary_layer` Debug 目标构建成功；
- `2dot5_cf`：局部顶点 52,232，合并后顶点 52,010，Triangle 13,186，
  Quad 45,413，Farfield 面 422，Wall 面 58,177；
- Release 一层流水线：总耗时 13.723 秒，峰值工作集 250,519,552 字节，
  输出 3,252 个体单元和 7,974 个最终外表面；
- 阶段 08、09仍为未开始，本阶段没有实现过渡区域或 Pyramid/Tetra 生成。
