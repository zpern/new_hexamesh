# 逐层生长过渡区滑移面联合相交检测设计

## 目标

在现有常规边界层对 `Symmetry`/`Internal` 的相交检测基础上，把同一检测加入逐层生长产生的层差过渡区联合碰撞判断。多法向预处理及多法向过渡单元生成阶段明确不执行该检测。

最终行为必须满足：

- 常规 Prism/Hexa 候选不得非法穿越 `Symmetry` 或 `Internal`；
- 层差过渡候选参与联合检测时，同样不得非法穿越这两类滑移面；
- 合法的共享顶点、真实物理边和完整侧面接触继续放行；
- 多法向阶段不因滑移面相交检测发生停止或回退；
- 完整生成结果保留必要的层差过渡单元，不再以仅含规则层的测试输出代替完整结果。

## 阶段边界

完整生成顺序保持现状：

1. `generateMultiNormalTransition()` 完成多法向拆点及其过渡网格；该阶段不构建、不接收也不查询 `SlidingIntersectionIndex`。
2. `generateIncrementalBoundaryLayers()` 执行逐层生长；在该阶段构建或接收只读的静态滑移面索引。
3. 常规候选经 `LayerCollisionChecker` 检查普通障碍、历史边界、同层候选和滑移面。
4. 层差过渡候选经 `TransitionBoundaryChecker` 做联合检测，其中加入同一个静态滑移面索引。
5. 逐层结果稳定后，再与第一步的多法向结果合并；合并阶段不追溯审计多法向单元。

“多法向过程中跳过”只指步骤 1，不表示步骤 2 中与多法向变换后的前沿相邻的常规或层差候选可以跳过检测。只要候选属于逐层生长流程，就执行滑移面检测。

## 架构

继续复用现有 `SlidingIntersectionIndex` 和 `SlidingContactPermission`，不建立第二套几何算法。

`TransitionBoundaryInput` 增加逐层联合检测所需的只读上下文：

- `const SlidingIntersectionIndex *sliding_surface`；
- 候选三角形三个节点的滑移 region 归属；
- 真实物理边掩码；
- 完整侧四边形豁免 region；
- 必要时用于自身 region 同侧判断的同一体单元列信息。

候选信息应在过渡模板构造边界三角形时生成，不能仅从三角剖分后的几何反推。这样可以区分真实四边形边和人工对角线，并保留源点到滑移 region 的拓扑授权。

`TransitionBoundaryChecker::findRollbackFaces()` 在现有三类检测后增加滑移面检查：

1. 原始 Wall/Farfield 障碍；
2. 历史外露边界；
3. 同批候选之间的非法接触；
4. `Symmetry`/`Internal` 静态滑移面。

任一过渡边界三角形产生非法滑移命中时，沿用现有 `LayerBoundaryOwner::rollback_high_faces` 回退相关高层源面，使层协调循环重新构造候选，直到联合检测稳定。

## 接触权限

过渡候选沿用常规候选的授权规则：

- 三角形顶点属于某 region 时，该顶点接触可授权；
- 只有模板声明为真实物理边且两端同属某 region 时，该边接触可授权；
- 人工三角剖分对角线不授权；
- 只有完整侧四边形的四个上下节点同属同一 region 时，两个剖分三角形才能获得完整面豁免；
- 顶盖不获得完整面豁免；
- 权限按 `region_id` 隔离，`Symmetry` 与 `Internal` 使用相同算法。

自身 region 命中仍采用单元级同侧判断：只有非关联列的上下点都保持在同一侧时才忽略该 region，并继续查询其他 region。数据不足、非有限或局部法向不可判定时保守回退。

## 错误处理

- 静态滑移索引初始化失败，映射为逐层生成初始化失败，不提交部分结果。
- 过渡候选权限数据长度不一致、节点映射缺失或几何退化，返回 `TransitionBoundaryError`，由增量生成器保持事务失败。
- 合法查询无命中不产生回退。
- 同一候选命中多个 region 时，只要存在任一未授权命中即回退；忽略自身 region 后仍须继续查询其他 region。

## 测试

### 单元测试

扩展 `transition_boundary_checker_test`：

- 过渡侧面合法贴合 `Symmetry` 时不回退；
- 过渡三角形穿越 `Symmetry` 时回退其 `rollback_high_faces`；
- 同一几何改为 `Internal` 时结果一致；
- 人工对角线接触不被误授权；
- 忽略合法自身 region 后，命中另一滑移 region 仍回退。

### 流水线测试

扩展增量层过渡流水线，构造常规层发生差异并生成过渡模板的案例，断言：

- 未接入滑移联合检测时会接受穿越滑移面的过渡候选；
- 接入后相关高层面回退；
- 稳定结果仍含 `CellRole::LayerTransition` 单元；
- 接受的常规单元和层差过渡单元均无非法滑移相交。

另加多法向范围测试，确认 `generateMultiNormalTransition()` 的输入、输出和停止行为不因滑移索引改变，且该模块不依赖 Spatial 滑移相交接口。

### 真实案例

继续使用原始 `tests/data/symm_intersection/BoundaryMeshing.txt` 和相同参数。回归改走完整边界层入口，而不是直接调用 `generateRegularLayers()`：

- face 3 分别作为 `Symmetry` 和 `Internal`；
- `nLN=20`、`dLen=0.10000000100000001`、`dRto=1.19999999999999996`；
- 质量及停止参数沿用输入中当前 API 可表达的值；
- `b_use_multiple_normals=0`；
- 两种边界类型的逐面决策一致；
- 最终接受网格无非法滑移面相交；
- 输出中存在 `CellRole::LayerTransition`；
- 分别写出 Symmetry/Internal 完整体网格 VTK，供 ParaView 检查。

## 非目标

- 不在多法向预处理或 `generateMultiNormalTransition()` 中新增滑移相交检测；
- 不改变多法向拆点、模板或合并算法；
- 不把 `Symmetry`/`Internal` 塞入普通 `CollisionIndex`；
- 不修改既有接触分类的几何容差语义；
- 不对最终合并网格执行会回滚多法向单元的全局后处理。
