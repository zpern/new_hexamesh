# 0 高邻、两个相邻继续角点过渡模板设计

## 目标与范围

在保留两层过渡阶段增加一种协调状态：当前 Triangle 或 Quad 没有真实高邻边，
但恰好有两个相邻角点的一环中存在 `trial_layers` 更高的其他 front 面。该状态复用
已有试生长层点生成局部封口单元，避免两个仍被邻近体单元使用的下一层点悬空。

本次不改变层差传播、停止原因、规则层生长、多法向 front 变换和已有高邻模板。
Quad 的两个相对继续角点，以及继续角点数量不是 2 的无高邻状态，继续走现有无
高邻模板。

## 概念分离

协调阶段独立计算两类信息：

- 高邻边：共享边邻面的 `occupied_layers` 比当前面严格高 1；它表示真实台阶。
- 继续角点：该角点连接的其他 front 面中，至少一个面的 `trial_layers` 比当前面
  严格高；它表示当前面的下一层角点仍被周围单元使用。

两者可以复用角点/边使用表和扫描辅助函数，但不能复用状态字段。新增：

```cpp
std::optional<std::size_t> continuing_edge_local_index;
```

该字段仅表示无高邻时两个相邻继续角点组成的局部边；不得写入
`high_edge_local_index`，以免下游误认为共享边邻面高一层。

## 协调判定

`coordinateFront()` 保持先检查共享边层差和高邻边，再分类继续角点：

1. 对每条恰好被两个 front 面使用的共享边，要求两面的 `occupied_layers` 差不
   超过 1。
2. 邻面比当前面高 1 时，记录当前面的高邻边。
3. 有一条高邻边的 Quad 只扫描高边之外两个角点：0 个额外继续点进入现有
   `1 Pyramid + 1 Tetra`；1 个记录 `third_continuing_vertex_id` 并进入现有
   `2 Pyramid + 2 Tetra`；2 个返回 `TransitionCornerLayerViolation`。
4. 没有高邻边且 `trial_layers >= 1` 时扫描当前面的全部角点。若恰好两个继续
   角点且局部编号相邻，记录它们组成的 `continuing_edge_local_index`。
5. `trial_layers == 0` 时不建立继续边。0 层没有下一层点，且既有规则明确不做
   非接触角点压低；共享边层差检查仍照常执行。
6. 无高邻 Quad 的两个相对继续角点不进入新模板；其他继续点数量也不进入。

继续角点的公共判断为：

```cpp
incident != current &&
coordinated[incident].layers.trial_layers >
    coordinated[current].layers.trial_layers
```

同一角点连接多个更高面时仍只计为一个继续角点。

## Triangle 模板

Triangle 输入增加 `continuing_edge_local_index`。该字段与
`high_edge_local_index` 互斥，局部编号必须小于 3，且 `trial_layers >= 1`。

设继续边为 `a-b`，当前模板低层为 `low = layer_vertex_ids[occupied]`，下一层为
`high = layer_vertex_ids[occupied + 1]`：

```text
a = low[edge]       b = low[edge+1]       c = low[edge+2]
e = high[edge]      f = high[edge+1]

Pyramid [a,e,f,b,c]
```

在该 Pyramid 之前仍生成现有的 `occupied` 个规则 Prism。Pyramid 标记为
`ReservedLayerTransition`，层号为 `occupied + 1`。外露顶面是 Pyramid 除底面
`[a,e,f,b]` 以外的四个三角侧面；与邻近模板重合的三角面由全局顶面去重消除。

## Quad 模板

Quad 输入同样增加 `continuing_edge_local_index`，并要求它与全部高邻/第三继续
字段互斥、局部编号小于 4、`trial_layers >= 1`。

无高邻两个相邻继续点仍使用已有公共基础块和它已经确定的底面对角线：

- `trial_layers == 1` 时没有公共基础块，直接对第 0/1 层应用新侧向模板；底面
  对角线由 `chooseQuadDiagonal()` 选择。
- `trial_layers >= 2` 时先生成 `regular_layers` 个 Hexa 和 `5 Pyramid + 2 Tetra`
  公共基础块；新侧向模板使用公共基础块选定的顶面（即侧向模板底面）对角线，
  不得重新选择。

把继续边旋转为 `a-b`，循环低层点为 `a,b,c,d`，下一层点为 `e,f,g,h`。

公共基础块沿 `b-c` 方向切割时固定生成：

```text
Pyramid [a,e,f,b,c]
Tetra   [b,c,f,d]
```

镜像的 `a-d` 方向固定生成：

```text
Pyramid [b,f,e,a,d]
Tetra   [a,d,e,c]
```

前两个单元由底面对角线强制决定。剩余空腔的两个 Tetra 只在两种合法顶面对角线
之间选择；不得反过来改变 Pyramid 和第一个 Tetra 的底面连接。

对于 `b-c` 底面方向，剩余两个 Tetra 的公共低层顶点（`apex`）为 `c`：

```text
顶面对角线 e-g：
Tetra [e,g,h,c]
Tetra [e,g,f,c]

顶面对角线 f-h：
Tetra [f,h,e,c]
Tetra [f,h,g,c]
```

对于镜像的 `a-d` 底面方向，采用同样的循环旋转规则，`apex` 为 `d`；两套候选
仍分别沿对应旋转后的 `e-g` 与 `f-h` 顶面对角线构造。

每种候选的分数是两个 Tetra 全部八个三角面的最大等角 skewness：

```text
score = max(all triangular-face skewness of both tetrahedra)
```

选择 `score` 更小的候选。沿用当前四单元模板的确定性行为：仅当方案一严格更小
时选择方案一；分数相等时选择方案二。一个候选质量计算失败则选择另一个成功候选；
两者都失败则返回 `FaceEvaluationError`。

新 Quad 分支最终生成 `1 Pyramid + 3 Tetra`。`top_faces` 取这四个单元边界中不
属于模板底面且没有在四个单元之间成对抵消的全部三角面，再交给全局顶面去重。
新增单元均标记为 `ReservedLayerTransition`，层号与同一高度上的现有侧向模板一致。

## 类型矩阵

| 面 | 高邻边 | 继续角点附加状态 | 处理 |
|---|---:|---|---|
| Triangle | 0 | 恰好两个，`trial>=1` | 新增 `1 Pyramid` |
| Triangle | 0 | 其他 | 现有无高邻模板 |
| Triangle | 1 | — | 现有单高邻模板 |
| Triangle | >1 | — | 拒绝 |
| Quad | 0 | 恰好两个相邻，`trial>=1` | 新增 `1 Pyramid + 3 Tetra` |
| Quad | 0 | 两个相对或其他数量 | 现有无高邻模板 |
| Quad | 1 | 无额外继续点 | 现有 `1 Pyramid + 1 Tetra` |
| Quad | 1 | 一个额外继续点 | 现有 `2 Pyramid + 2 Tetra` |
| Quad | 1 | 两个额外继续点 | `TransitionCornerLayerViolation` |
| Quad | 2 | 两条相邻 | 现有双高模板 |
| Quad | 2 | 两条相对 | 拒绝 |
| Quad | >2 | — | 拒绝 |

## 验证

测试按 TDD 增加以下覆盖：

1. 协调器能识别 0 高邻 Triangle 的两个继续点，并记录继续边。
2. 协调器能识别 0 高邻 Quad 的两个相邻继续点；两个相对点不触发。
3. `trial_layers == 0` 不记录继续边，但共享边层差检查不变。
4. Triangle 新模板的 Pyramid 连接、顶面、元数据和三个旋转编号正确。
5. Quad 两种底面对角线分别固定正确的 Pyramid 与第一个 Tetra。
6. Quad 两套顶面对角线构造可通过专门几何分别选中，并验证选择的是最坏
   skewness 更小的方案。
7. 新状态与高邻字段同时存在、编号越界或 0 层携带继续边时返回非法输入。
8. 现有单高、三继续、双高、无高模板测试保持通过。
9. 完整 Release 构建与全部测试通过，随后更新 `docs/reserved_layer_transition.md`
   只记录最终实际实现的规则。
