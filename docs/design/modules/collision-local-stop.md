# 空间查询、碰撞检测与局部停止设计

## 1. 文档目的

本文定义当前 BoundaryMesh 的空间查询、碰撞检测和逐源面局部停止模型。检查发生在候选质量验证和正式提交之间，用于判断候选 Prism/Hexa 是否接触原始障碍、历史外露边界或同层其他候选。

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

阶段 07 接入后，该检查器保持两个独立入口：

```cpp
filterAgainstObstacles(...) // 原始表面和历史外露边界
filterSelfCollisions(...)   // 传播过滤后剩余的同层候选
```

两次调用之间由 Generator 执行停止传播和候选压缩，已经因质量、固定障碍
或邻接约束退出的候选不会成为同层幽灵障碍。

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

相邻候选的公共点、公共边和公共侧面通过分层 `CollisionVertexKey` 识别。
合法性针对两个三角形的实际交集判断，而不是要求两个三角形的全部顶点都属于
公共侧面。实际交集完全位于预期公共几何区域时放行；交集离开该区域时仍按非法
自碰撞停止双方。具体修正规则见第 15 节。

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

阶段 07 集成后又增加了合法相邻候选共享侧面的 RED/GREEN 覆盖，并确认
传播过滤发生在 `filterSelfCollisions(...)` 以前。集成回归的 Debug 与
Release 均为 40/40 通过；该结果不改变阶段 06 的完成边界。

## 15. 真实案例合法接触修正

### 15.1 问题证据

`2dot5_cf` 一层、`first_height=0.1` 的逐阶段诊断结果为：

```text
Wall 源面                         58,177
质量检查后候选                    58,137
原始/历史障碍过滤后               47,818
同层自碰撞过滤后                   3,252
```

最终停止原因中有 54,885 个 `Collision`，而 skewness、整体反转和局部翻转
合计只有 40 个。将首层高度降为 `0.001` 后仍只生成 4,710 个单元，将
`maximum_skewness` 放宽到 `1.0` 后只生成 3,254 个单元，因此主要问题不是
层高或质量阈值。

对同层非法接触对继续分类后，没有发现不共享源顶点的远程候选碰撞；命中全部
来自共享一个源顶点或一条源边的候选。当前实现要求两个碰撞三角形都完全由
公共侧面的四个分层顶点组成才放行，遗漏了顶面—侧面、公共竖边以及不同
Triangle/Quad 三角化组合产生的合法接触。

### 15.2 原 HexaMesh 判断方式

旧工程的 `IntersecChecker::checkIntersect(...)` 按碰撞三角形坐标相同的顶点数
`same_count` 分支：

- `same_count == 0`：调用 `tri_tri_overlap_test_3d(...)`；
- `same_count == 1`：忽略公共点本身，将两个三角形各自的对边与另一三角形调用
  `lin_tri_intersect3d(...)`，只捕获公共点以外的交叉；
- `same_count >= 2`：不再检测，直接将共享边或共享面视为合法。

旧工程还通过 `HexaTag` 管理外露面。候选侧面对应的邻居侧面已经存在时，该公共
侧面不加入本次查询；提交后相邻侧面互相抵消。因此已经成为内部面的公共侧面不会
继续参与候选之间的碰撞检测。

旧方法避免了连续前沿的大量合法邻接误判，但存在三个限制：用坐标相等代替拓扑
身份、`same_count >= 2` 无条件放行可能漏掉公共边以外的穿插、逐候选插入使结果
可能依赖遍历顺序。新实现只继承其“外露面和公共拓扑特征”语义，不照搬这些限制。

### 15.3 方案比较

1. **公共几何区域包含判定（采用）**：先从候选拓扑建立允许接触的点、线段或
   侧面区域，再判断实际三角形交集是否完全包含于该区域。能够保留合法邻接，
   同时继续发现越过公共区域的真实穿插。
2. **完全跳过拓扑相邻候选（拒绝）**：实现简单、速度最快，但会漏掉凹角处
   相邻 Prism/Hexa 穿过公共侧面后形成的真实重叠。
3. **只按 `TriangleContactKind` 放宽（拒绝）**：把相邻候选的共点、共边和
   共面接触一律视为合法，无法判断接触是否已经超出预期公共区域。

### 15.4 改良后的检测流程

同层检测先在候选单元层面建立拓扑关系，再进入三角形精确检测：

1. 公共侧面与旧工程一致，不作为两个相邻候选彼此的碰撞障碍；
2. 候选顶面及仍然外露的侧面继续参加检测；
3. 没有共享分层拓扑 key 时执行完整三角形相交检测；
4. 只有一个共享 key 时参考旧工程，忽略合法公共点，检查非共享对边是否进入
   对方三角形；
5. 共享两个或更多 key 时不采用旧工程的无条件放行，而是确认实际交集没有离开
   对应公共边或公共侧面；
6. 一层内先收集全部非法候选对，再同时停止双方，不按候选遍历顺序增量决定结果。

共享关系必须使用 `CollisionVertexKey`，不能使用坐标相等。坐标重合但 key 不同的
对象仍然没有合法拓扑关系。

### 15.5 允许接触区域

合法性由候选单元级拓扑关系决定，不再只由当前两个碰撞三角形的共享 key 数量
决定：

- 两个候选没有共享源顶点：不存在允许接触区域，任何接触均非法；
- 两个候选只共享一个源顶点：允许区域是该源顶点从当前层到底层下一层形成的
  公共竖直（实际为生长方向）线段；
- 两个候选共享一条源边：允许区域是该源边两端在当前层和下一层构成的公共
  侧面；非平面 Quad 按统一 key 顺序拆成两个允许三角形；
- 候选与原始源面或其拓扑邻面：只允许落在候选底层公共顶点、公共边或源面
  周界上的接触；候选进入原始面的内部仍然非法；
- 候选与历史外露边界：根据分层 key 建立同样的公共点、边或面，除此以外的
  接触均非法。

“允许”判断针对实际几何交集，而不是碰撞三角形的全部顶点。顶面三角形可以
包含公共侧面以外的顶点，只要它与另一三角形的交集完全位于允许区域内，就不应
停止候选。

### 15.6 精确交集证据

Spatial 层扩展三角形接触结果，使其除 `TriangleContactKind` 外还能提供用于
合法性判断的交集证据：

- 共点接触提供交点；
- 共边和非共面相交提供交线端点；
- 共面重叠提供投影裁剪多边形，并能判断该多边形是否完全位于允许侧面内。

所有包含判断继续采用项目现有的严格零判断，不增加用户容差。Growth 只构造
候选之间的允许区域并调用 Spatial 判定，不直接包含 `geom_func.h`。

实际实现补充了一条重要约束：不能把平面求交或二维裁剪得到的浮点交点重新构造
为三维坐标，再要求它严格满足 `dot == 0` 或 `cross == 0`。数学上共面、共线的
结果经过除法后可能不再得到机器意义上的精确零。在完整边界面 key 和对应坐标
一致时，局部共享 key 数用于选择常数时间分支：仅共点只放行 `VertexTouch`，
共享精确边只拒绝公共边以外的共面正面积重叠，完整公共面放行其三角化内部接触。
没有这些拓扑证据时仍使用完整交集证据判断。整个过程没有引入几何容差。

### 15.7 保留真实碰撞

拓扑相邻不是无条件豁免。以下情况仍必须同时停止相关候选：

- 交点或交线离开公共竖边、公共底边或公共侧面；
- 共面重叠面积延伸到公共侧面以外；
- 相邻候选在凹角处相互穿透；
- 没有拓扑关系的候选发生共点、共边、重叠或穿透；
- 候选撞到非关联原始 Wall、Farfield 或历史外露边界。

### 15.8 诊断与测试

CLI 汇总增加各 `FaceStopReason` 的源面数量，使真实案例无需修改库代码即可区分
质量停止、碰撞停止和邻层约束停止。

实现必须按以下顺序完成 RED/GREEN：

1. 相邻 Triangle/Triangle、Triangle/Quad、Quad/Quad 的公共侧面合法接触；
2. 只共享源顶点的公共竖边合法接触；
3. 顶面—侧面和不同 Quad 三角化组合的合法接触；
4. 接触超出公共点、边或侧面的相邻候选仍停止；
5. 原始表面及历史边界的合法底层连接与非法穿入；
6. 非相邻候选真实碰撞仍同时停止；
7. 交换源面顺序后结果不变；
8. `2dot5_cf` Release 一层回归、Debug/Release 全量 CTest 和 IO-OFF 构建。

### 15.9 修正结果

`2dot5_cf` 使用 `first_height=0.1`、`growth_ratio=1.0`、一层和
`maximum_skewness=0.95` 的 Release 结果为：

```text
volume_cells=41453
farfield_faces=58800
stop_vertex_layer_limit=41453
stop_reversed_candidate=1
stop_locally_inverted_candidate=7
stop_skewness_exceeded=32
stop_collision=16684
growth_seconds=19.9299
total_seconds=20.7472
peak_working_set_bytes=414887936
```

相较修正前的 3,252 个体单元和 54,885 个碰撞停止面，体单元增至约
12.7 倍，碰撞停止下降约 69.6%。峰值工作集约 395.7 MiB，低于 1 GiB。
两个 ASCII legacy VTK 均成功生成且非空：边界层体网格 7,511,825 字节，
远场边界 5,054,139 字节。

大量候选恢复后，原 `ExposedBoundaryTracker` 基于 `vector` 的逐面线性切换暴露出
O(n²) 路径。实现改为以规范 `BoundaryFaceKey` 为键的有序容器，`prepare/apply`
降为 O(n log n)，并在导出 `faces()` 时保持稳定规范顺序。

最终验证结果：Debug 45/45、Release 45/45，均为 0 failed；关闭
`BOUNDARY_MESH_ENABLE_CGNS_IO` 后 `boundary_mesh_boundary_layer` Debug 构建通过。
层数协调和过渡处理由 `transition` 与 `boundary_layer` 模块负责，具体行为以当前实现和对应测试为准。
