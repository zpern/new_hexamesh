# 公共头文件命名与注释规范

## 1. 目标

公共头文件名必须清楚表达所属模块和数据职责，公共基础声明必须具有便于阅读与调试的行尾说明。本规范只改变文件组织和注释，不改变 C++ 类型名、数据布局、默认值或算法行为。

## 2. Mesh 头文件命名

Mesh 模块的数据头文件统一使用 `mesh_` 前缀：

```text
include/boundary_mesh/mesh/surface_mesh.hpp
    -> include/boundary_mesh/mesh/mesh_surface.hpp

include/boundary_mesh/mesh/volume_mesh.hpp
    -> include/boundary_mesh/mesh/mesh_volume.hpp

include/boundary_mesh/mesh/surface_topology.hpp
    -> include/boundary_mesh/mesh/mesh_surface_topology.hpp

include/boundary_mesh/mesh/surface_topology_builder.hpp
    -> include/boundary_mesh/mesh/mesh_surface_topology_builder.hpp

include/boundary_mesh/mesh/surface_topology_error.hpp
    -> include/boundary_mesh/mesh/mesh_surface_topology_error.hpp
```

旧头文件不保留转发兼容层。所有生产代码、测试和文档必须使用新 include 路径。

C++ 类型名保持不变：

```cpp
SurfaceMesh
VolumeMesh
SurfaceTopology
SurfaceTopologyBuilder
SurfaceTopologyError
```

文件名强调所属 Mesh 模块；类型名保持领域表达，避免出现 `MeshSurfaceMesh` 等冗余名称。

## 3. 公共声明行尾注释

范围固定为 `include/boundary_mesh` 下的全部公共头文件。

### 3.1 基础别名

每个公开 `using` 声明在完整声明结束处增加行尾说明：

```cpp
using Scalar = double; // 网格算法统一使用的浮点标量类型
using VertexId = std::uint32_t; // 顶点容器的稳定索引类型
```

跨多行的 variant 或 array 别名在结束的 `>;` 或 `;` 后注释，不在模板参数中间插入说明。

### 3.2 枚举值

每个枚举值说明其业务含义：

```cpp
enum class SurfaceBoundaryKind
{
    Farfield, // 远场边界
    Wall,     // 需要生成边界层的壁面
    Symmetry  // 对称面
};
```

最后一个枚举值不额外增加尾逗号，只为注释调整空格对齐。

### 3.3 公开数据成员

每个公开 struct 成员说明数值含义、索引对象或单位语义：

```cpp
struct Triangle
{
    std::array<VertexId, 3> vertex_ids{}; // 按面绕序保存的三个顶点编号
};
```

注释不得重复成员名，必须回答“它表示什么”或“它索引什么”。

### 3.4 不使用行尾注释的位置

- 函数形参：由函数上方的 `///` 说明输入约束和返回意义；
- 私有算法局部变量：只在算法意图不明显时使用独立注释；
- 类方法：继续使用方法上方的 `///`；
- include、namespace 和纯控制流语句。

## 4. 实施与提交边界

实施分为两个独立提交：

1. `refactor: align mesh header filenames`
2. `docs: annotate public declarations`

第一个提交只移动五个头文件并更新全部引用。第二个提交只补充公共声明注释，不改变可执行语句。

## 5. 验证标准

- 旧五个 include 路径在源码、测试和历史设计/计划中搜索为零；本迁移规范中的旧到新映射除外；
- 新五个头文件均存在且被正确引用；
- `include/boundary_mesh` 中公开 `using`、枚举值和 struct 数据成员具有行尾 `//`；
- CMake Debug 全量构建成功；
- 全部 CTest 通过；
- `git diff --check` 无输出；
- 两个提交均不改变运行行为。
