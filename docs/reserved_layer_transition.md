# 保留两层附面层过渡算法

## 1. 文档范围

本文只描述当前代码已经实现并可执行的保留层过渡规则。Pointwise 等商业软件的
大层差 Prism 过渡带、全局混合空腔以及其他尚未实现的方案不属于本文范围。

当前实现先试生长规则 Prism/Hexa，再删除最后两层对应的规则体，复用试生长点
生成局部 Pyramid/Tetra 过渡模板。其目标是让 Triangle/Quad 源面生成的附面层
在局部停止或出现一层高度差时仍保持共形，并输出完全三角化的附面层顶面。

主要实现位置：

- `reserved_layer_growth.cpp`：预留层数和三种层数定义；
- `reserved_layer_transition.cpp::coordinateFront()`：统一的 front 协调；
- `triangle_transition_template.cpp`：Triangle 模板；
- `quad_transition_template.cpp`：Quad 模板和对角线选择。

## 2. 总体流程

```text
用户请求逐点层数
    ↓ 每个请求增加 2 层
规则层试生长
    ↓ 得到每个源面的 accepted_layer_count
协调相邻面的有效占用层数
    ↓ 确定高邻边、第三继续角点和无高邻继续边
按 Triangle/Quad 选择局部模板
    ↓ 重新生成规则体和保留层过渡体
收集所有模板顶面 Triangle
    ↓ 删除成对出现的内部 Triangle
输出 boundary_layer_top 和 farfield_boundary
```

试生长调用强制设置 `enforce_single_high_edge = true`。生长阶段负责碰撞、质量、
各向同性高度、点层数上限和邻层差传播；过渡阶段不重新决定这些停止原因，只消费
最终接受层数并验证它们能否由现有模板协调。

## 3. 层数定义与短层数特例

常量 `ReservedTransitionLayerCount` 当前为 2。对一个源面：

```cpp
trial_layers    = accepted_layer_count;
occupied_layers = trial_layers > 0 ? trial_layers - 1 : 0;
regular_layers  = trial_layers > 2 ? trial_layers - 2 : 0;
```

下文单元连接公式中的简写 `regular` 与这里的 `regular_layers` 表示同一个值。

三种层数含义不同：

| 名称 | 含义 | 用途 |
|---|---|---|
| `trial_layers` | 试生长实际接受的体层数 | 建立逐层点表、比较角点是否继续 |
| `occupied_layers` | 过渡协调使用的有效高度 | 判断共享边层差和高邻边 |
| `regular_layers` | 最终重新保留的完整规则层数 | 生成最终 Prism/Hexa |

短试生长时的数值为：

| `trial_layers` | `occupied_layers` | `regular_layers` | 含义 |
|---:|---:|---:|---|
| 0 | 0 | 0 | 没有试生长体，只保留源面作为顶面候选 |
| 1 | 0 | 0 | 有第一层点，但不保留完整规则体 |
| 2 | 1 | 0 | 没有完整规则体，已有一层有效占用高度 |
| 3 | 2 | 1 | 保留一层规则体，其余点用于过渡 |
| N，N > 2 | N - 1 | N - 2 | 常规情况 |

因此“删除两层”不是从最终网格中删除已有单元，而是最终重建时只生成
`regular_layers` 个规则体。最后两层试生长点仍保存在 `LayerVertexTable` 中，供
Pyramid/Tetra 模板复用。

Triangle 模板存在一个值得注意的实现差异：它生成 `occupied_layers` 个 Prism，
而 Quad 模板只生成 `regular_layers` 个 Hexa，再显式生成中心点过渡块。后续章节
分别按真实代码说明，不能把两者统一理解为相同的单元计数。

## 4. 协调规则

### 4.1 高邻边定义

对共享一条边的两个参与生长面，低侧面满足：

```text
neighbor.occupied_layers == face.occupied_layers + 1
```

时，该共享边是低侧面的高邻边。高侧面不会把同一条边记录为高邻边。

任意共享边必须满足：

```text
abs(first.occupied_layers - second.occupied_layers) <= 1
```

否则返回 `UncoordinatedTransitionLayerDifference`。所以当前过渡模板只处理有效
占用层数差 0 或 1；更大的差值必须在生长阶段通过邻层差传播消除。

### 4.2 统一的 front 协调

`generateReservedLayerTransition()` 只保留带 `MultiNormalOptions` 的统一入口。
无论 `multi_normal_options.enabled` 是 true 还是 false，过渡判定都针对
`transformed_front` 重新建立：

- 边使用表 `edge_uses`；
- 点使用表 `vertex_uses`；
- front 面到原始 `source_face_id` 的映射。

Quad 最多允许两条高邻边；若有两条，它们必须相邻。Triangle 最多允许一条。
单高邻 Quad 的非接触角点中，恰好一个角点连接更高 front 面时，记录第三继续
角点；两个角点都违反则拒绝。

没有高邻边且 `trial_layers >= 1` 时，协调器扫描当前面的全部角点。角点连接的
其他 front 面只要满足 `incident.trial_layers > current.trial_layers`，该角点即
继续生长；同一角点连接多个更高面仍只计一次。恰好两个继续角点且它们在当前面
中相邻时，记录独立的 `continuing_edge_local_index`。它不是高邻边：前者来自
角点一环 `trial_layers`，后者来自共享边两侧 `occupied_layers`。

`coordinateFront()` 只把恰好被两个 front 面使用的边当作内部共享边。只被一个
面使用的边和使用次数不是 2 的边不会参加高低比较；当前代码不会在这里额外报告
非流形边错误。

关闭多法向时，`transformed_front` 保持原始 front，但仍使用同一套
`coordinateFront()` 规则；开启且实际发生多法向分裂或 Quad 三角化后，高邻边
局部编号可能与原始源面观察结果不同。

### 4.3 0 层面的处理

`trial_layers == 0` 的面不会保留任何体单元，也不允许模板接收高邻边。Triangle
和 Quad 模板都会拒绝“0 层且存在高邻边”的输入。

0 层面也不会建立或接收 `continuing_edge_local_index`，因为它没有可供模板使用
的下一层点。

生长阶段仍保留共享边层差传播，使 0 层面的直接邻面不能形成模板无法处理的有效
层差。0 层停止本身不应通过非接触角点规则把整圈角点一环压到 0；角点检查只在
协调结果真正出现单高邻边时有意义。

## 5. 支持性总表

`T` 表示 Triangle，`Q` 表示 Quad。

| 源面 | 高邻状态 | 附加状态 | 模板结果 | 统一入口 |
|---|---|---|---|---|
| T | 无 | 任意合法层数 | Prism 规则层，输出一个 Triangle 顶面 | 支持 |
| T | 无 | 恰好两个相邻继续点，`trial_layers >= 1` | Prism 规则层 + 1 Pyramid | 支持 |
| T | 一条 | `trial_layers >= 1` | Prism 规则层 + 1 Pyramid | 支持 |
| T | 两条及以上 | — | 拒绝 | 拒绝 |
| Q | 无 | `trial_layers < 2` | 不生成体，源 Quad 按质量切成 2 Triangle | 支持 |
| Q | 无 | `trial_layers >= 2` | `regular_layers` Hexa + 5 Pyramid + 2 Tetra | 支持 |
| Q | 无 | 恰好两个相邻继续点，`trial_layers >= 1` | 基础块 + 1 Pyramid + 3 Tetra | 支持 |
| Q | 一条 | 只有高边两个端点继续 | 基础块 + 1 Pyramid + 1 Tetra | 支持 |
| Q | 一条 | 另有一个非接触角点继续 | 基础块 + 2 Pyramid + 2 Tetra | 支持 |
| Q | 相邻两条 | — | 基础块 + 2 Pyramid + 2 Tetra | 支持 |
| Q | 相对两条 | — | 非法输入 | 拒绝 |
| Q | 三条或四条 | — | 非法输入 | 拒绝 |
| 任意 | 共享边有效层差 > 1 | — | 无对应模板 | 拒绝 |
| 任意 | 0 层且记录了高邻边 | — | 非法输入 | 拒绝 |

表中的“基础块”只在 `trial_layers >= 2` 时存在，指 Quad 的
`regular_layers` 个完整规则 Hexa 加中心点过渡块。`trial_layers == 1` 时直接
生成侧向模板，不创建中心点基础块。

### 5.1 短层数快速查询

下表给出最常调试的 0～3 层。`P`、`Y`、`T`、`H` 分别表示 Prism、Pyramid、
Tetra、Hexa；“基础”表示 Quad 的 `5Y + 2T` 中心点过渡块。

| 源面与状态 | trial=0 | trial=1 | trial=2 | trial=3 |
|---|---|---|---|---|
| Triangle，无高邻 | 0P，源面顶面 | 0P，源面顶面 | 1P，第1层顶面 | 2P，第2层顶面 |
| Triangle，0 高邻两相邻继续点 | 不触发 | 1Y | 1P+1Y | 2P+1Y |
| Triangle，单高邻 | 非法 | 0P+1Y | 1P+1Y | 2P+1Y |
| Quad，无高邻 | 0体，源面切2三角 | 0体，源面切2三角 | 0H+基础 | 1H+基础 |
| Quad，0 高邻两相邻继续点 | 不触发 | 1Y+3T | 基础+1Y+3T | 1H+基础+1Y+3T |
| Quad，单高邻两端继续 | 非法 | 1Y+1T | 基础+1Y+1T | 1H+基础+1Y+1T |
| Quad，三个角点继续 | 非法 | 2Y+2T | 基础+2Y+2T | 1H+基础+2Y+2T |
| Quad，相邻双高边 | 非法 | 2Y+2T | 基础+2Y+2T | 1H+基础+2Y+2T |

相邻双高边一行无论多法向是否开启都适用；前提是两条高邻边在当前 front 中相邻。

## 6. Triangle 模板

设源 Triangle 的循环顶点为：

```text
v0 ----- v1
 \       /
  \     /
    v2
```

### 6.1 无高邻边

模板生成 `occupied_layers` 个完整 Prism：

```text
第 layer-1 层 Triangle + 第 layer 层 Triangle → Prism
```

每个 Prism 的元数据为：

```text
role           = RegularLayer
source_face_id = 当前源面
layer          = 1 ... occupied_layers
```

最终输出第 `occupied_layers` 层的一个 Triangle 作为顶面。具体短层数结果：

| `trial_layers` | Prism 数量 | 顶面 |
|---:|---:|---|
| 0 | 0 | 源 Triangle |
| 1 | 0 | 源 Triangle |
| 2 | 1 | 第 1 层 Triangle |
| N | N - 1 | 第 N - 1 层 Triangle |

### 6.2 单高邻边

设高邻边局部端点为 `first`、`second`，第三点为 `apex`。低层点记为 `low`，
再上一层点记为 `high`。在已有 `occupied_layers` 个 Prism 后追加：

```text
Pyramid [low[first], low[second],
         high[second], high[first], low[apex]]
```

它把高邻边抬高一层，未接触高边的角点保持在低层。输出三个 Triangle：

```text
[high[first], high[second], low[apex]]
[low[first],  high[first],  low[apex]]
[high[second], low[second], low[apex]]
```

该 Pyramid 的角色是 `ReservedLayerTransition`，层号为
`occupied_layers + 1`。Triangle 没有第三继续角点模板，也没有顶面 Quad
对角线选择。

### 6.3 0 高邻、两个相邻继续角点

该状态使用独立的 `continuing_edge_local_index`。设继续边为 `a-b`，下一层对应点
为 `e-f`，第三个低层点为 `c`：

```text
Pyramid [a,e,f,b,c]
```

它与已有 `occupied_layers` 个 Prism 兼容。Pyramid 的四边形模板接口和下方
`[a,b,c]` 三角界面不输出，其余三个三角侧面作为顶面候选。

## 7. Quad 的公共基础块

### 7.1 `trial_layers < 2`

此时不生成 Hexa、中心点或五 Pyramid 基础块：

- 无高邻边：直接把源 Quad 按质量切成两个顶面 Triangle；
- 单高邻边：直接在第 0 层和第 1 层点之间生成侧向模板；
- 相邻双高边或三个继续角点：直接生成四单元侧向模板；
- 0 层且有高邻边：非法。

### 7.2 `trial_layers >= 2`

首先生成 `regular_layers` 个完整 Hexa。然后取：

```text
bottom = layer_vertex_ids[regular]
top    = layer_vertex_ids[regular + 1]
```

在 `bottom` 和 `top` 共八个点的几何平均位置创建中心点 `c`。基础过渡块由五个
Pyramid 组成：

```text
1 个底面 Pyramid：[bottom0,bottom1,bottom2,bottom3,c]
4 个侧面 Pyramid：[bottomi,topi,top(i+1),bottom(i+1),c]
```

随后将 `top` Quad 沿选定对角线切分，并生成两个 Tetra：

```text
对角线 0-2：
Tetra [top0,top1,top2,c]
Tetra [top0,top2,top3,c]

对角线 1-3：
Tetra [top1,top2,top3,c]
Tetra [top1,top3,top0,c]
```

五个 Pyramid 和两个 Tetra 都标记为 `ReservedLayerTransition`，层号为
`regular + 1`。如果存在高邻边，后面还会继续追加侧向模板。

## 8. Quad 无高邻边

### 8.1 短层数

`trial_layers` 为 0 或 1 时，不生成体单元。模板直接比较 Quad 的两条对角线，
输出质量较好方案对应的两个 Triangle。

### 8.2 常规层数

`trial_layers >= 2` 时，仅生成第 7 节的公共基础块：

```text
regular 个 Hexa
+ 5 个 Pyramid
+ 2 个 Tetra
```

顶面为 `top` Quad 沿同一条已选对角线形成的两个 Triangle。这里没有额外
`side_cells`。

### 8.3 两个相邻继续角点

当无高邻边、`trial_layers >= 1`，且恰好两个相邻角点继续生长时，使用
`continuing_edge_local_index` 进入 `1 Pyramid + 3 Tetra` 模板。设局部布局为：

```text
低层：a,b,d,c（循环顺序）
高层：e,f,h,g（对应循环顺序）
继续边：a-b
```

公共基础块选中 `b-c` 对角线时，前两个单元固定为：

```text
Pyramid [a,e,f,b,c]
Tetra   [b,c,f,d]
```

选中镜像的 `a-d` 对角线时固定为：

```text
Pyramid [b,f,e,a,d]
Tetra   [a,d,e,c]
```

底面对角线由普通 Quad 质量选择或已生成的公共基础块继承，不在侧向模板中重新
选择。后两个 Tetra 分别比较顶面对角线 `e-h` 和 `f-g`；每套候选取两个 Tetra
全部三角面的最大 skewness，选择该最大值更小者。分数相同沿用现有确定性规则，
选择第二套。最终输出八个外露三角面候选。

两个相对继续角点或继续角点数量不是 2 时，不触发本模板，仍走普通无高邻分支。

## 9. Quad 单高邻边：只有两个端点继续

令唯一高邻边为局部边 `edge → edge+1`，按循环顺序定义：

```text
低层：a=low[edge], b=low[edge+1],
      d=low[edge+2], c=low[edge+3]
高层：e=high[edge], f=high[edge+1]
```

这里“只有两个端点继续”表示除高邻边端点外，没有记录
`third_continuing_vertex_local_index`。

模板根据该 Quad 已选中的对角线选择一种定向，追加：

```text
1 个 Pyramid
1 个 Tetra
```

若对角线对应 `a-d` 定向：

```text
Pyramid [e,f,b,a,d]
Tetra   [a,c,d,e]
```

否则：

```text
Pyramid [f,e,a,b,c]
Tetra   [b,d,c,f]
```

两种定向都输出四个 Triangle 顶面。对角线在没有额外拓扑约束时由 Quad 质量
选择，因此局部高边编号旋转后单元连接也随之旋转。

短层数和常规层数的差别仅在于公共基础块：

| `trial_layers` | 公共基础块 | 侧向追加 |
|---:|---|---|
| 1 | 无 | 1 Pyramid + 1 Tetra |
| >= 2 | `regular_layers` Hexa + 5 Pyramid + 2 Tetra | 1 Pyramid + 1 Tetra |

## 10. Quad 单高邻边：三个角点继续

这是单高邻边的特殊四单元模板。高邻边两个端点之外，恰有一个非接触角点还
连接到试生长层数更高的面；另一个非接触角点是唯一未继续角点。

设协调器计算出的公共定向角点为 `common`，循环定义：

```text
低层：a=low[common], b=low[common+1],
      d=low[common+2], c=low[common+3]
高层：e=high[common], f=high[common+1],
      h=high[common+2], g=high[common+3]
```

无论第三继续角点位于高邻边的哪一侧，按上述旋转后 `d` 始终是唯一未继续的
低层角点。`common` 的计算是：

```text
high edge = edge → edge+1

第三继续角点 = edge+2：common = edge+1，唯一未继续点 = edge+3 = d
第三继续角点 = edge+3：common = edge，  唯一未继续点 = edge+2 = d
```

虽然为书写完整单元连接定义了四个高层点 `e/f/g/h`，这不表示四个底角都继续；
“三个角点继续”仍然只指高边两端加一个非接触角点。

追加的两个 Pyramid 固定为：

```text
Pyramid [a,b,f,e,d]
Pyramid [a,e,g,c,d]
```

底层对角线由该共形拓扑强制确定，不再允许按单个 Quad 的局部质量自由改变。
随后在两种合法的顶层 Tetra 切法中选择：

```text
方案一：
Tetra [e,g,f,d]
Tetra [g,f,h,d]

方案二：
Tetra [e,g,h,d]
Tetra [e,f,h,d]
```

最终追加总数为：

```text
2 个 Pyramid + 2 个 Tetra
```

两种 Tetra 切法都会输出六个顶面 Triangle；具体连接随所选方案变化。

该模板复用相邻双高边的四单元实现，但 `common` 的来源不同：它由唯一高邻边
与第三继续角点的位置共同确定。

## 11. Quad 相邻双高边

两条高邻边必须相邻。它们的公共角点确定 `common`，随后使用与第 10 节相同的
两个 Pyramid、两个候选 Tetra 方案和六个顶面 Triangle。

统一协调器允许恰好两条相邻高邻边并进入本模板；两条相对高邻边仍然返回
`MultipleTransitionHighEdges`。这条规则与多法向开关无关，多法向开关只决定
协调前的 front 是否实际发生拓扑变换。

## 12. 对角线与质量规则

### 12.1 普通 Quad 对角线

`chooseQuadDiagonal()` 比较 Quad 的两条候选对角线。每种方案产生两个 Triangle，
以其中较差的三角形质量作为方案分数，选择最大 skewness 更小的方案。分数相同
时，分别将两条对角线的两个端点编号升序排列，选择端点编号对字典序更小的
对角线。该规则保证相同输入重复运行得到相同结果。

### 12.2 四单元模板的顶面对角线

三个继续角点和相邻双高边场景有两种合法 Tetra 对。代码对每个 Tetra 的四个
三角面计算等角 skewness，再取两个 Tetra 所有面的最大值作为该方案分数：

```text
score = max(两个 Tetra 的全部三角面 skewness)
```

选择分数更小的方案，即选择“最差三角面仍然更好”的切法。若一个方案质量计算
失败而另一个成功，选择成功方案；两者都失败则返回 `FaceEvaluationError`。
当前严格使用 `<` 比较，分数相等时落到方案二。

### 12.3 拓扑优先级

当三个继续角点或相邻双高边已经规定底层对角线时，共形拓扑优先于局部 Quad
质量。质量比较只发生在该拓扑允许的两种顶层 Tetra 切法之间，不能改变底层
强制对角线。

普通无高邻 Quad 的顶面是各自独立的外露面，不存在跨源面的“整块公共 Quad
对角线传播”。只有模板之间实际共享同一个多边形接口时才要求双方三角剖分一致；
三个继续角点和双高边分支通过强制拓扑满足这一要求。

## 13. 输出顶面、内部面消除与远场边界

每个局部模板产生 `top_faces`，全部为 Triangle。生成器先汇总所有候选顶面，
再通过规范化顶点集合统计重复：

- 只出现一次：保留为外露 `boundary_layer_top`；
- 出现两次：视为相邻模板内部公共面，不输出；
- 模板必须保证共享界面的三角剖分一致，否则会留下裂缝或重叠面。

最终 `farfield_boundary` 由以下部分组合：

```text
原始 Farfield 面
+ 去重后的 boundary_layer_top
```

多法向路径还会先把顶面点号映射到合并后的体网格点号，再合并
`MultiNormalTransition` 单元与规则/保留层单元。

## 14. 单元元数据

写入体网格的每个单元都对应一个 `CellMetadata`：

| 字段 | 含义 |
|---|---|
| `source_face_id` | 产生该单元的原始源面编号；多法向 front 面仍映射回原始源面 |
| `layer` | 模板赋予的层号，不是 ParaView 的 Cell ID |
| `cell_role` | `RegularLayer`、`ReservedLayerTransition` 或 `MultiNormalTransition` |

规则 Prism/Hexa 的 `layer` 从 1 递增。各分支的过渡层号为：

| 单元来源 | `layer` |
|---|---:|
| Quad 五 Pyramid + 两 Tetra 基础块 | `regular + 1` |
| Quad 单高邻追加的 1 Pyramid + 1 Tetra | `regular + 2` |
| Quad 三继续角点/双高边追加的 2 Pyramid + 2 Tetra | `regular + 2` |
| `trial_layers == 1` 的 Quad 侧向模板 | 1 |
| Triangle 单高邻 Pyramid | `occupied + 1` |
| Multi-normal 入口预先生成的 Tetra | 0 |

同一个局部模板追加的多个单元使用相同层号，不会在 Pyramid/Tetra 之间逐个递增。
Triangle 的第 `occupied` 个 Prism 仍标记为 `RegularLayer`；这是当前实现行为，
即使它的数量不等于 `regularLayerCount()` 也不要在调试时重新解释其角色。

Legacy VTK 输出中这些字段写成 Cell Data：

```text
source_face_id
layer
cell_role
```

调试某个体单元时，应先读取这三个字段，再根据本文件的支持性总表定位模板；
不要把 ParaView 的 0-based Cell ID 当作 `source_face_id`。

## 15. 错误与拒绝条件

### 15.1 协调阶段

| 错误 | 触发条件 |
|---|---|
| `MissingTransitionFaceState` | front、增长记录或源面状态缺失 |
| `UncoordinatedTransitionLayerDifference` | 共享边有效占用层数差大于 1 |
| `MultipleTransitionHighEdges` | Triangle 超过一条高邻边、Quad 超过两条，或 Quad 两条高邻边相对 |
| `TransitionCornerLayerViolation` | 单高邻边的非接触角点存在当前模板不能表达的更高一环 |

### 15.2 模板阶段

`InvalidTransitionTemplateInput` 包括但不限于：

- 层点数组数量不等于 `trial_layers + 1`；
- 高邻边局部编号越界；
- 第二高邻边存在但第一高邻边缺失；
- 第三继续角点没有唯一高邻边，或同时存在第二高邻边；
- 第三继续角点不是高邻边之外的合法 Quad 角点；
- 继续边与任一高邻/第三继续字段同时存在；
- 继续边局部编号越界；
- 0 层面携带继续边；
- 两条高邻边不相邻；
- 0 层面携带高邻边；
- Quad 点号超出网格顶点数组；
- 中心点编号或 front 到源面的映射无效。

几何质量计算失败则返回 `FaceEvaluationError`，不会静默改用未经验证的切法。

## 16. 共形与有效性要求

所有当前模板共同依赖以下不变量：

1. 共享边两侧的 `occupied_layers` 差不超过 1；
2. 相邻模板对任何实际共享的多边形接口使用相同三角剖分；
3. 任一内部 Triangle 恰有两个体单元拥有者；
4. 任一外露顶面 Triangle 恰有一个体单元拥有者；
5. 体单元不反转、不退化，不产生正体积交叉；
6. `mesh.cells.size() == mesh.metadata.size()`；
7. 每个过渡单元保留可追踪的 `source_face_id`、`layer` 和 `cell_role`。

当前算法是局部、确定性的两层预留模板，不包含任意大层差模板。如果这些不变量
无法满足，正确行为是协调或模板生成失败，而不是生成一个局部看似闭合但全局不
共形的网格。
