# 零层 Wall 面输出回退设计

## 1. 问题

当前 `ExposedBoundaryTracker` 初始为空，只在体单元被接受后记录新顶面和侧面。
如果某个 Wall 源面在第一层候选阶段就因质量、碰撞或约束停止，该面不会进入
tracker，最终 `farfield_boundary` 中既没有新顶面，也没有原始 Wall 面，形成空洞。

## 2. 输出不变量

每个 Wall 源面在最终边界层接口中必须恰好拥有一个外露状态：

- `accepted_layer_count == 0`：使用该源面的初始 Triangle 或 Quad；
- `accepted_layer_count >= 1`：使用最后一个成功层已有的外露顶面；
- 原始 Farfield 面继续原样保留；
- 最终输出不得因为某个源面零层生长而产生空洞。

## 3. 采用方案

在最终远场边界构建阶段补回零层 Wall 面，不改变
`ExposedBoundaryTracker` 的碰撞职责。

`RegularLayerGenerator` 在全部层结束后，从 `FaceGrowthRecord` 中提取
`accepted_layer_count == 0` 的 `source_face_id`，传给
`buildFarfieldBoundary`。构建器按这些编号从原始 `SurfaceMesh` 复制对应
Wall 面，并加入输出接口。

不在 tracker 初始化时放入全部 Wall 面，避免原始 Wall 同时进入原始表面碰撞树
和历史外露边界树，进而改变碰撞判定或增加重复查询。

## 4. 面属性与绕向

补回的初始面遵循现有边界层接口约定：

- Triangle 和 Quad 使用同一处理路径；
- 顶点顺序相对输入 Wall 面反转，使接口法向朝向后续远场区域；
- `region_id` 继承原始 Wall 面；
- `kind` 改为 `SurfaceBoundaryKind::BoundaryLayerInterface`；
- 使用层号为 `0` 的源顶点键，与原始 Farfield 顶点去重规则保持一致。

## 5. 数据校验与错误

构建器在补回面之前验证：

- `source_face_id` 位于原始面数组范围内；
- 对应面标签确实为 Wall；
- 输入编号不存在重复；
- 面引用的顶点有效。

任何状态不一致继续使用 `SpatialError::InvalidTopologyReference` 返回失败，不静默
忽略错误。

## 6. 测试

采用测试先行：

1. 单元测试先构造空 tracker 和零层 Triangle/Quad，验证测试在当前实现下失败；
2. 验证补回面数量、类型、反向绕向、`BoundaryLayerInterface` 标签和 `region_id`；
3. 验证已有成功生长顶面不被原始 Wall 面重复补回；
4. 集成测试制造首层候选全部失败，验证体单元为零但输出接口仍包含初始 Wall 面；
5. 运行完整 Debug 回归。

## 7. 非目标

本修改不改变质量阈值、碰撞算法、停止传播、层数统计、体单元生成和原始
Farfield 面输出规则。
