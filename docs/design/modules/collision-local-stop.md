# 空间查询、碰撞检测与局部停止设计

## 1. 文档目的

本文定义 BoundaryMesh 阶段 06 的空间查询、碰撞检测和逐源面局部停止模型。本阶段插入阶段 05 的“质量检查通过”和“候选单元提交”之间，检测候选 Prism/Hexa 是否接触原始障碍、历史外露边界或同层其他候选。

本文只描述模块边界、空间索引、接触语义、逐层事务和错误处理，不负责相邻面停止传播、层数协调及 Pyramid/Tetra 过渡。

## 2. 设计目标

本阶段必须满足以下要求：

1. 对候选 Prism/Hexa 的顶面和全部侧面进行碰撞检测，不检测底面；
2. 检测候选与原始 Wall/Farfield、已提交外露边界及同层其他候选之间的接触；
3. 合法拓扑连接不得误判，其他零距离接触、重叠和穿透均视为碰撞；
4. 同层两个候选相撞时，两个源面同时停止，结果不得依赖源面遍历顺序；
5. 碰撞后直接停止当前源面，不缩短步长，不重试；
6. 只记录发生碰撞的源面、目标层及 `FaceStopReason::Collision`；
7. 增量维护已提交体网格的外露边界；
8. 每层以事务方式提交，程序级错误不得留下孤立顶点或半提交单元；
9. 空间索引由整层共享，不为每个源面建立独立树；
10. Growth 业务代码不得直接依赖 `geom_func.h` 或旧 HexaMesh 类型。
11. 外露边界能够物化为边界层与剩余远场区域之间的接口表面。

## 3. 非目标

阶段 06 不负责：

- 计算或记录最大安全步长；
- 自动缩短碰撞层高后重试；
- 点在 Prism/Hexa 内部的包含检测；
- Symmetry 表面的碰撞检测和合法共面生长；
- 相邻源面停止传播或最大层数差协调；
- 停止后开口的 Pyramid/Tetra 填充；
- PLY、VTK、命令行或真实案例输入输出；
- 用户可配置的碰撞容差。

本阶段依据逐层连续推进假设：非法进入另一区域以前，候选顶面或侧面必然先发生接触。因此不增加完全包含检测。

## 4. 模块与依赖

### 4.1 BoundaryMesh::Spatial

新增 `BoundaryMesh::Spatial` 静态库，负责：

- AABB 表达、合并和闭区间重叠判断；
- 整理自 HexaMesh 思路的共享 `BinaryAabbTree`；
- 碰撞三角形插入和包围盒候选查询；
- `tiger_geom` 三角形相交适配；
- 基于拓扑归属的合法接触过滤；
- 返回需要停止的源面集合。

该模块不管理生长层数、不修改 `GrowthFront`、不提交 `VolumeMesh`。

### 4.2 tiger_geom 依赖

新工程固定引入 `third/geom`：

- 独立构建时，如果不存在 `tiger_geom`，通过 `add_subdirectory(third/geom)` 创建该目标；
- 嵌入 TiGER 时，如果父工程已经提供 `tiger_geom`，直接复用；
- 只有 Spatial 实现文件允许包含 `geom_func.h`；
- Growth、Surface 和 Quality 不得直接调用 `TiGER_GEOM_FUNC`。

精确检测优先复用：

```cpp
TiGER_GEOM_FUNC::tri_tri_overlap_test_3d(...);
```

`tri_tri_overlap_test_3d` 只负责给出基础接触结论。Spatial 内部还要结合 `orient2d/orient3d`、共面主平面投影和拓扑共享特征，将结果区分为分离、共点、共边、共面重叠和非共面穿透。该分类只服务合法接触过滤，不进入公共生成结果。

对于具有合法共享顶点或共享边的三角形，适配层必须继续检查非共享边、非共享顶点和共面投影区域，确认不存在合法共享特征以外的额外交叉。不能只根据共享顶点数量直接返回不碰撞。

旧 HexaMesh 的 `same_count` 分支、`HexaTag`、旧 `vec`、直接输出和逐面动态插入流程不复用。

### 4.3 Growth 侧组件

`ExposedBoundaryTracker` 增量维护已提交边界层的外露顶面和侧面，只管理拓扑与几何记录，不执行碰撞判断。

`LayerCollisionChecker` 接收一层中通过质量检查的所有候选，完成原始表面、历史外露边界和同层自碰撞检测，并输出碰撞停止的源面集合。

`FarfieldBoundaryBuilder` 在生成结束时读取原始 `SurfaceMesh` 和最终 `ExposedBoundaryTracker`，构造独立、紧凑编号的远场边界表面。该构建器不重新扫描全部体单元。

`RegularLayerStepper` 继续只负责预推出和质量检查。`RegularLayerGenerator` 在 Stepper 返回后、分配最终体网格顶点编号以前调用碰撞检查器。

## 5. 共享空间索引

一次生成过程维护三类索引：

```text
OriginalSurfaceTree   初始化一次，只包含 Wall 和 Farfield
ExposedBoundaryTree   每层开始时由当前外露边界集合建立
CandidateLayerTree    每层由静态和历史检查合格的候选建立
```

所有源面共享这些索引。单个源面只保存候选 Prism/Hexa、源面 ID 和构成候选边界的三角形。

三类对象分开建立索引，以便采用不同的拓扑过滤规则，并避免把静态原始表面与动态生长状态耦合在同一棵树中。

## 6. 碰撞图元与三角形化

碰撞图元至少保存：

```cpp
struct CollisionTriangle
{
    TrianglePoints points;                    // 三角形的三个空间坐标
    CollisionOwnerKind owner_kind{};          // 原始表面、历史外露边界或同层候选
    std::uint32_t owner_id{};                  // 内部使用的所属面或候选编号
    std::array<CollisionVertexKey, 3> vertices; // 用于判断合法拓扑共享关系
};
```

`owner_kind`、`owner_id` 和接触分类只在 Spatial/Growth 内部使用，不写入最终公共碰撞诊断。

Triangle 产生一个碰撞三角形。Quad 固定沿 `v0-v2` 拆成：

```text
(v0, v1, v2)
(v0, v2, v3)
```

候选 Prism/Hexa 生成顶面和全部侧面，不生成底面。暂时共享的侧面也参与静态和历史障碍检测，避免相邻候选后来停止后出现漏检。

## 7. 接触语义

几何检测和拓扑决策必须分离。

几何层判断三角形是否存在边界接触、共面重叠或非共面穿透。AABB 使用闭区间；精确谓词按正、负、零直接分类，不设置用户容差。

拓扑层只放行预期连接：

- 候选侧面从自己的源面边出发形成的共享边；
- 候选与当前底层 Front 的预期共享边；
- 拓扑相邻候选之间的预期共享侧面、共享边和共享点。

不能通过“坐标相等”判断合法连接。相同坐标但拓扑 ID 不同的点、边或面仍然属于碰撞。

合法共享特征以外再发生任何额外交叉或重叠时，仍然判为碰撞。两个没有合法拓扑关系的对象发生共点、共边、共面重叠或穿透时均判为碰撞。

## 8. 原始表面范围

`OriginalSurfaceTree` 包含：

- 全部 Wall 面；
- 全部 Farfield 面。

本阶段不插入 Symmetry 面。合法边界层侧面可能沿 Symmetry 面共面生长，而阶段 05 尚未正式接入对称约束；在对称生长设计完成前，将 Symmetry 当普通障碍会造成第一层误停。

原始源 Wall 面及其相邻面不能整面排除。只放行预期共享点、共享边或底层连接；候选在共享特征以外穿入相邻原始面时必须停止。

## 9. 外露边界增量维护

`ExposedBoundaryTracker` 初始为空。第 0 层 Wall 已存在于原始表面树，不在历史外露集合中重复保存。

提交候选单元时执行：

```text
当前底面存在于外露集合 → 删除
加入新顶面
侧面尚不存在           → 加入
相同侧面已经存在       → 两侧抵消并删除，成为内部面
```

因此：

- 连续生长时，上一层顶面被下一层底面覆盖后删除；
- 相邻单元的共享侧面自动成为内部面；
- 停止源面的最终顶面永久保留；
- 相邻层数不同时形成的台阶侧面永久保留；
- 每层只处理本层新增单元，不重复扫描全部历史体单元。

更新后的外露面集合在下一层开始时用于重建 `ExposedBoundaryTree`。

外露面还必须保留原绕序、源 Wall 面和源 Wall `region_id`，供最终远场边界物化。作为远场体域的内部边界输出时，边界层外露面的绕序必须反转，使法向从剩余远场体域指向边界层区域。

## 10. 单层数据流

生成入口调整为接收完整表面与拓扑：

```cpp
generateRegularLayers(
    const SurfaceMesh &surface_mesh,
    const SurfaceTopology &surface_topology,
    const GrowthPatch &patch,
    const GrowthFront &initial_front,
    const std::vector<SourceVertexGrowthProfile> &profiles,
    const RegularLayerGrowthOptions &options);
```

不保留旧入口兼容层。

每层严格执行：

1. `RegularLayerStepper` 根据层数、动态前沿和质量评价产生质量合格候选；
2. 为质量合格面构造临时 Prism/Hexa，不分配最终 `VolumeMesh` 顶点 ID；
3. 查询 `OriginalSurfaceTree`，标记碰撞候选；
4. 查询 `ExposedBoundaryTree`，标记碰撞候选；
5. 删除第 3、4 步停止的候选，它们不进入同层候选树；
6. 用剩余候选建立 `CandidateLayerTree`；
7. 每个候选对只精确检查一次，同层非法接触时双方同时停止；
8. 压缩最终 `GrowthFront`，移除停止面和无存活面引用的候选顶点；
9. 为最终顶点分配体网格 ID，原子提交顶点、Prism/Hexa、映射、面状态和外露边界；
10. 下一层根据更新后的外露集合重建历史空间树。

已经撞到原始表面或历史边界的候选不进入 `CandidateLayerTree`，不得作为幽灵障碍导致其他候选停止。

同层检测先收集全部非法候选对，再统一生成停止集合。空间树返回顺序和源面数组顺序不得改变最终停止集合。

## 11. 停止与错误语义

碰撞是正常局部停止：

```cpp
FaceStopEvent{
    previous_front_face_index,
    source_face_id,
    target_layer,
    FaceStopReason::Collision
};
```

每个停止源面最多生成一个碰撞停止事件。不记录撞到的对象、三角形或接触类型。

候选几何退化继续使用已有 `FaceStopReason::DegenerateCandidate`。以下状态属于程序级失败：

- 原始 SurfaceMesh、SurfaceTopology、GrowthPatch 与 Front 映射不一致；
- 原始碰撞三角形坐标无效或退化；
- 外露边界引用无效顶点；
- 空间索引 ID 溢出或内部状态不一致。

程序级失败返回 `RegularLayerGrowthError`，当前层零提交。普通碰撞只停止对应源面，其余候选正常提交。

## 12. 测试要求

### 12.1 Spatial 单元测试

- AABB 分离、重叠和边界相切；
- 三角形分离、非共面穿透、非拓扑共点、非拓扑共边、共面部分重叠和完全重合；
- 合法拓扑共点、共边和共享面放行；
- 相同坐标但不同拓扑 ID 判为碰撞；
- Triangle/Quad 三角形化；
- 非有限坐标和退化三角形拒绝进入索引。

### 12.2 外露边界测试

- 第一层提交后的顶面和侧面；
- 相邻 Prism/Hexa 共享侧面抵消；
- 上一层顶面被下一层覆盖；
- 停止源面的最终顶面保留；
- 不同层数产生台阶侧面；
- Triangle/Quad 混合邻接。

### 12.3 碰撞过滤测试

- 源 Wall 合法连接；
- 另一 Wall 和 Farfield 碰撞；
- Symmetry 不参与检测；
- 历史顶面和台阶侧面碰撞；
- 同层非相邻候选相撞时双方停止；
- 相邻候选合法共享侧面放行；
- 合法共享面以外的额外交叉仍停止；
- 已经撞到静态或历史障碍的候选不进入同层树；
- 交换源面顺序后停止集合不变；
- 顶面和所有侧面方向均有覆盖。

### 12.4 生成事务测试

- 碰撞停止不产生孤立顶点和对应体单元；
- 其他源面继续提交；
- `FaceGrowthRecord` 的接受层数、状态、停止层和原因正确；
- 空间索引错误导致当前层零提交；
- Triangle/Quad 混合生成；
- Debug、Release 完整 CTest 回归。

真实 CGNS 案例、规模性能和内存回归属于阶段 10。

### 12.5 远场边界物化测试

- 原始 Farfield 面及 `region_id` 保持不变；
- 最终顶面、台阶侧面和 Patch 开放侧面全部输出；
- 原始 Wall 底面和体单元内部共享面不输出；
- 边界层接口标记为 `BoundaryLayerInterface` 并继承源 Wall `region_id`；
- 接口绕序相对边界层外露面反转；
- 输出顶点使用独立紧凑编号且不存在未引用顶点。

## 13. 完成边界

阶段 06 完成后，规则生长流程能够在提交前确定性地拒绝发生碰撞的候选 Prism/Hexa，并保留停止区域形成的外露障碍。

阶段 06 不协调相邻源面的层数差，也不修补停止后产生的开口。上述职责分别留给阶段 07、08 和 09。

## 14. 实施结果

阶段 06 已实现以下公共入口：

- `BoundaryMesh::Spatial`、`Aabb` 与 `BinaryAabbTree`；
- `classifyTriangleContact(...)` 与合法分层拓扑接触过滤；
- `CollisionIndex` 与原始 Wall/Farfield 索引；
- `ExposedBoundaryTracker` 与 `buildFarfieldBoundary(...)`；
- `LayerCollisionChecker::filterAgainstObstacles(...)`；
- `LayerCollisionChecker::filterSelfCollisions(...)`；
- 显式接收完整表面及拓扑的 `generateRegularLayers(...)`。

最终实现保持以下依赖边界：`geom_func.h` 和
`TiGER_GEOM_FUNC` 只出现在 `src/spatial/triangle_contact.cpp`；
Growth、Surface、Quality 及全部公共头文件均不包含第三方几何头文件。

完成验证命令为：

```powershell
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Debug 与 Release 均为 36/36 通过。第三方 `tiger_geom` 在 MSVC
下仍报告其自身源码编码和既有返回路径警告；BoundaryMesh 目标无新增警告。
