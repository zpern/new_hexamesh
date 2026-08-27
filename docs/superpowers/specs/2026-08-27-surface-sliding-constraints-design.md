# Symmetry 与 Internal 统一滑移约束设计

## 目标

将现有仅针对 Symmetry 面的附面层顶点约束泛化为 surface sliding constraint，使与 Internal 面相接的 Wall 顶点也沿对应几何平面滑移。

Growth front 的面选择保持不变：只有 `SurfaceBoundaryKind::Wall` 面进入 `GrowthPatch` 和 `GrowthFront`。Symmetry 与 Internal 面只提供 Wall 顶点的滑移边界条件，不进入 front。

## Region 语义

`SurfaceFaceId` 标识一个离散三角形或四边形；`SurfaceBoundaryTag::region_id` 标识一组共同组成同一物理边界区域的离散面。

本设计要求：

- Symmetry 与 Internal 的 `region_id` 在整个 `SurfaceMesh` 中全局不重复；
- 同一滑移 region 下的全部面必须共面；
- 不同几何平面必须使用不同的 `region_id`。

多个共面离散面只生成一个滑移平面。位于两个不同 region 交线上的 Wall 顶点同时继承两个滑移平面。

## 数据模型泛化

将只表达 Symmetry 的公开名称泛化为 Sliding：

```text
PatchVertex::symmetry_region_ids
    -> PatchVertex::sliding_region_ids

FrontVertexBoundary::symmetry_region_ids
    -> FrontVertexBoundary::sliding_region_ids

SymmetryPlane
    -> SlidingPlane

VertexSymmetryConstraint
    -> VertexSlidingConstraint

SymmetryConstraints
    -> SlidingConstraints

SymmetryConstraintBuilder
    -> SlidingConstraintBuilder
```

对应公开头文件、源文件、CMake 源清单、调用方和测试文件同步重命名。不保留含义错误的旧字段或旧类型兼容别名。

错误类型同步泛化：

```text
InvalidSymmetrySurface -> InvalidSlidingSurface
SymmetryInputMismatch  -> SlidingInputMismatch
```

`UndefinedConstrainedDirection` 与 `OverConstrainedGrowthVertex` 已是通用名称，保持不变。

## Patch 与 Front 构建

`GrowthPatchBuilder` 继续只扫描 Wall 面来生成 `source_face_ids` 和选中顶点。

对每个已选中的 Wall 顶点，遍历 `SurfaceTopology::vertexFaces()`。当关联面标签为以下任一种时，记录其 `region_id`：

```cpp
tag.kind == SurfaceBoundaryKind::Symmetry ||
tag.kind == SurfaceBoundaryKind::Internal
```

收集结果按数值排序并去重，写入 `PatchVertex::sliding_region_ids`。其他面类型不增加滑移约束。

`GrowthFrontBuilder` 将这些 region 原样复制到 `GrowthFrontVertex::boundary.sliding_region_ids`。Internal 面不会因此进入 `GrowthFront::faces` 或 `source_face_ids`。

## 滑移平面构建

`SlidingConstraintBuilder` 从 front 顶点收集所有被引用的 `sliding_region_ids`。对每个 region，在完整 `SurfaceMesh` 中查找：

```cpp
(tag.kind == SurfaceBoundaryKind::Symmetry ||
 tag.kind == SurfaceBoundaryKind::Internal) &&
tag.region_id == region_id
```

由于 region ID 全局不重复，一个 region 只对应一种边界类别，无需将 `SurfaceBoundaryKind` 加入约束键。

构建器复用现有面求值、法向平行检查和点到平面距离检查。相同 region 的任意面不共面时返回 `InvalidSlidingSurface`；找不到被引用 region 时返回 `SlidingInputMismatch`。

## 方向约束

滑移方向规则保持现有数学行为：

- 零个独立滑移平面：使用归一化后的原始方向；
- 一个独立滑移平面：去除法向分量，使方向位于平面内；
- 两个独立滑移平面：方向沿两平面的交线，并选择与原始方向同向的一侧；
- 三个独立滑移平面：返回 `OverConstrainedGrowthVertex`；
- 投影后方向退化：返回 `UndefinedConstrainedDirection`。

共面或法向线性相关的重复 region 不重复增加独立约束。

## 逐层传播

当前 front 顶点边界信息会随规则层生长传递。字段泛化后继续原样传播 `sliding_region_ids`，确保 Internal 滑移约束在每一层生长方向计算中生效，而不是只作用于第 0 层。

## 测试

测试至少覆盖：

1. 仅 Wall 面仍正常生成 patch/front；
2. Wall 与 Symmetry 共点时继承 Symmetry sliding region；
3. Wall 与 Internal 共点时继承 Internal sliding region，但 Internal 面不进入 patch/front；
4. 同一顶点关联 Symmetry 与 Internal 时，两个 region 均被排序去重保存；
5. 单 Internal 平面把方向投影到平面内；
6. Internal 与 Symmetry 两个独立平面把方向限制到交线；
7. 同 region 的非共面 Internal 面返回 `InvalidSlidingSurface`；
8. 三个独立滑移 region 返回 `OverConstrainedGrowthVertex`；
9. 缺失 region 返回 `SlidingInputMismatch`；
10. 原有 Symmetry 单平面、双平面、冗余平面、错误输入和逐层传播测试保持通过。

## 兼容性

本设计有意修改公开 C++ API 名称。仓库内调用方将同步迁移；仓库外使用旧 Symmetry 类型、头文件或字段名的代码需要更新后重新编译。

本设计不改变：

- `SurfaceBoundaryKind` 枚举值；
- `SurfaceBoundaryTag` 布局；
- Wall-only 的 front 面选择；
- `SurfaceTopology::vertexFaces()` 的全量 point-to-face 语义；
- 滑移投影的数学算法和错误触发阈值。
