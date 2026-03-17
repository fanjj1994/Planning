
# Decision Making（路径决策）

本文档说明：
1) 在一般 ADAS/自动驾驶系统中，“决策模块”是什么、为什么需要路径决策，以及常见决策方法分类；
2) 本项目中 `DecisionCenter` 如何基于参考线、可通行区域与障碍物生成“粗解决策点”（SLPoint），并与后端局部路径规划衔接。

> 说明：本文以工程实现为主，描述与代码保持一致；涉及的关键实现位于：
> - [DecisionCenter 头文件](../src/planning_core/src/decision_center/decision_center.h)
> - [DecisionCenter实现](../src/planning_core/src/decision_center/decision_center.cpp)
> - [决策参数配置](../src/planning_core/config/planning_static_tps_config.yaml)
> - [决策输出如何被局部路径规划使用](../src/planning_core/src/local_planner/local_path/local_path_planner.cpp)

---

## 1. 简介：ADAS 中的决策模块与路径决策

在典型的 ADAS/自动驾驶软件架构中（简化描述）：

- **感知（Perception）**：输出障碍物、车道线/可通行区域、交通要素等。
- **预测（Prediction）**：输出动态目标未来轨迹或意图（静态场景可省略或弱化）。
- **决策（Decision / Behavior Planning）**：在规则、交通约束、安全约束下选择**行为/策略**（例如保持车道、超车、停车、绕行等），并产出后端规划可消化的**目标/约束**。
- **规划（Planning / Trajectory Planning）**：在给定的目标与约束下生成连续、可控、平滑的轨迹（路径 + 速度 + 时间）。
- **控制（Control）**：跟踪轨迹。

### 1.1 决策模块的意义

决策模块的核心价值在于：

1. **把“离散选择”前置**：很多驾驶行为天然是离散的（左超、右超、停），后端规划更擅长做连续优化（曲线/速度的平滑与可控）。
2. **降低后端求解难度**：如果不做路径决策，后端往往需要在更大的空间里同时搜索“该绕哪边/是否停车”，计算量和约束复杂度都会上升。
3. **显式注入安全与规则**：例如安全距离、道路边界、优先级规则（左>右>停）等，可以在决策层以可审计的方式表达。
4. **提高工程可控性**：当感知/预测不完美时，决策层可用保守策略兜底（例如停车），提升系统鲁棒性。

### 1.2 为什么需要“路径决策”

路径决策更偏向“几何空间上的行为选择”，典型输出形式是：

- 目标侧向偏移（例如绕障选择左侧通行并给出目标 $l$）；
- 目标纵向行为（例如设置停车位置 $s$）；
- 或者输出一组关键“锚点/关键帧”，供后端生成连续曲线。

在本项目中，`DecisionCenter` 产生一组 Frenet 坐标系下的关键点（`SLPoint`），作为局部路径规划（LocalPathPlanner）的“粗解/锚点”。

---

## 2. 经典决策方法分类（工程视角）

为了便于对比，这里用“重决策 / 轻决策 / 大模型决策”来做工程分类（并非学术严格分类）。

| 对比维度 | 重决策（规则/离散策略） | 轻决策（代价/搜索/约束驱动） | 大模型决策（数据驱动/端到端） |
|---|---|---|---|
| 核心思想 | 把行为选择写成明确规则与优先级，按条件触发离散动作（超车/停车/绕行等） | 用代价函数与约束描述“好坏”，通过搜索/优化得到动作或粗轨迹（决策隐含在求解过程） | 从数据学习策略：输入感知/地图/历史，直接输出行为或轨迹（或输出高层意图再与规划融合） |
| 典型方法/代表性算法 | 有限状态机（FSM）、决策树、基于规则的 if-else / 优先级仲裁、行为脚本 | A* / Dijkstra、动态规划（DP）、采样 + 评估（lattice）、最短路径/最小代价、（行为层）MPC/约束优化等 | 端到端神经网络（BC/IL/RL）、Transformer/多模态融合策略网络、（学术代表）Transfuser、UniAD、VAD 等；LLM/VLM 决策融合（作为高层意图生成/工具调用） |
| 可解释性 | **强**：规则透明、可追溯、易做功能安全分析 | **中**：解释依赖代价项设计与约束边界；求解过程相对复杂 | **弱**：难以给出逐条规则解释，需要可解释性与可验证性方案（可视化、对抗测试、形式化约束） |
| 泛化能力 | **弱-中**：对未覆盖场景易失效，需要持续补规则 | **中**：可通过代价与约束覆盖更多场景，但建模质量决定上限 | **强（潜力）**：可从海量数据学习更丰富场景，但需数据清洗、分布外检测、避免数据污染 |
| 实时性 | **强**：计算量小、确定性强 | **中**：计算量更大，需权衡分辨率与时延 | **中-弱**：推理依赖 GPU/专用加速；需严格时延预算 |
| 对感知/建模依赖 | **中-低**：对环境模型要求相对较低，但依赖“关键特征”正确（边界/相对距离） | **高**：需要更准确的环境建模（可通行区域、障碍物占据、约束边界） | **极高**：需要高质量感知/语义输入与训练数据闭环 |
| 适用场景（典型） | 封闭园区、高速/快速路、结构化道路、需求强调确定性与可审计 | 城市道路、交互更复杂的场景，且允许更高算力与更复杂模型 | 面向更广泛场景的通用能力探索，或作为规则/优化的补充（混合架构） |

> 注：上表中的“代表性网络名称”来自公开论文/开源社区常见方法，工业落地实现会随公司与版本迭代变化；工程上更常见的是**混合架构**（规则兜底 + 搜索/优化 + 学习模块）。

---

## 3. 本项目中的应用：`DecisionCenter`（重决策）

本项目当前路径决策采用 **重决策（规则优先级）**：

- 主要面向**准静态障碍物**（低速/无明显横向运动）；
- 在可通行走廊内，按优先级尝试：**左侧绕行 > 右侧绕行 > 停车**；
- 输出一组 `SLPoint` 作为局部路径规划的“关键帧”。

### 3.1 决策在系统中的位置（数据流）

规划主流程中，决策发生在参考线创建与 Frenet 投影之后：

1. 生成参考线（ReferenceLineCreator）。
2. 将 ego 与 TP（Traffic Participant）投影到参考线，获得 Frenet 状态。
3. TP 按 `s` 排序（从近到远/从后到前）。
4. `DecisionCenter::makePathDecision()` 生成 `pathDecisionPoints`。
5. `LocalPathPlanner::generateLocalPath()` 使用这些决策点生成连续局部路径。

对应代码：

- 规划主流程调用点：../src/planning_core/src/planning_process/planning_process.cpp
- 决策实现：../src/planning_core/src/decision_center/decision_center.cpp
- 局部路径使用决策点：../src/planning_core/src/local_planner/local_path/local_path_planner.cpp

### 3.2 输入/输出定义

#### 输入

`DecisionCenter::makePathDecision(egoCarInfo, tpInfoList)`：

- `egoCarInfo`：ego 车辆当前 Frenet 状态与尺寸（`s, l, ds/dt, dl/dt, width...`）。
- `tpInfoList`：交通参与者列表（每个 TP 提供 Frenet 状态与尺寸、ID）。
- 配置参数：由 `ConfigReader::readDecisionConfig()` 从 YAML 加载（例如安全距离、道路宽度、参考线长度）。

关键配置来源：../src/planning_core/config/planning_static_tps_config.yaml

- `decision.lat_safe_margin`：侧向安全裕度（左右各一份）。
- `decision.long_safe_margin`：纵向安全裕度（停车点提前量）。
- `pnc_map.road_half_width`：道路半宽。
- `reference_line.front_size` 与 `pnc_map.segment_len`：参考线前向长度。
- `local_path.path_size`：局部路径点数，用于推导决策视野。

#### 输出

输出为内部成员 `pathDecisionPoints`（`std::vector<SLPoint>`），通过 `getPathDecisionPoints()` 供下游读取。

`SLPoint`（见 ../src/planning_core/src/decision_center/decision_center.h ）字段含义：

- `s`：沿参考线的纵向位置。
- `l`：相对参考线的侧向偏移。
- `type`：决策类型（见 `SLPointType`）。
- `speed_limit`：从该点起的速度上限（当前实现中 **未赋值/未使用**，默认初始化为 0）。

`SLPointType` 关键枚举：

- `DECISION_LEFT_OVERTAKE`：左绕行。
- `DECISION_RIGHT_OVERTAKE`：右绕行。
- `DECISION_STOP`：停车。
- `DECISION_START / DECISION_END`：标记决策影响区间的开始/结束，用于下游规划做平滑“引入/退出”。

### 3.3 决策逻辑原理与代码对应

下面按实际实现顺序描述 `DecisionCenter::makePathDecision()` 的核心逻辑（见 ../src/planning_core/src/decision_center/decision_center.cpp ）。

#### Step 0：空输入直接返回

- 若 `tpInfoList` 为空，直接返回，不产生决策点。

#### Step 1：清理上一周期决策

- 调用 `pathDecisionInitialize()` 清空 `pathDecisionPoints`，避免历史决策污染当前周期。

#### Step 2：计算本周期决策视野（纵向）

实现中把“决策视野”抽象为 `decisionMakingLeastDistance`，并用于：

- 过滤 ego 车后方过远 TP（认为已通过/不需再决策）；
- 作为 `DECISION_START/END` 的 lead-in/lead-out 缓冲距离。

计算方式：

1) 先得到 `decisionMakingLeadPoint`（与局部路径点数相关，且被 clamp 到 `[30, 40]`）：

- `decisionMakingLeadPoint = clamp(local_path.path_size - 50, 30, 40)`

2) 再按 ego 速度进行缩放，并设置最小值：

- `decisionMakingLeastDistance = max(ego_dsDt * decisionMakingLeadPoint, 30.0)`

对应代码变量：`decisionMakingLeadPoint`、`decisionMakingLeastDistance`。

#### Step 3：定义可通行走廊（道路边界）

实现里使用 `road_half_width` 推导左右边界阈值：

- `leftBoundaryDistance = 1.5 * road_half_width`（正值，位于参考线左侧）
- `rightBoundaryDistance = -(0.5 * road_half_width)`（负值，位于参考线右侧）

并以 `rightBoundaryDistance < tp.l < leftBoundaryDistance` 判定 TP 是否位于"道路走廊"内。

> 注：Frenet 坐标系中 `l` 轴正方向朝左，因此左边界为正值（如 `+6.0m`），右边界为负值（如 `-2.0m`）。走廊范围即 `(rightBoundaryDistance, leftBoundaryDistance)` = `(-2.0, 6.0)`。

#### Step 4：TP 纵向范围过滤（参考线长度 + 后方阈值）

对每个 TP，先计算相对纵向距离：

- `distanceToEgoCar = tp.s - ego.s`

并过滤：

- 若 `distanceToEgoCar > referenceLineEndDistance`：TP 在参考线规划范围之外（过远前方）。
- 若 `distanceToEgoCar < -decisionMakingLeastDistance`：TP 在 ego 后方过远（认为已通过）。

其中：

- `referenceLineEndDistance = reference_line.front_size * pnc_map.segment_len`

#### Step 5：准静态障碍物判定

仅对“准静态”TP做当前决策（动态 TP 分支为 TODO）：

- 横向速度阈值：`fabs(tp.dl/dt) < 0.03`（常量 `MINSPEED`）
- 纵向速度阈值：`tp.ds/dt < ego.ds/dt / 2`

直观含义：TP 横向几乎不动、纵向明显慢于 ego，存在追尾/占道风险，因此需要绕行或停车。

#### Step 6：预测相遇点（纵向 s）

为把决策点放在“将发生交互的位置”，实现估计相对追赶时间：

- `relativeLongSpeed = ego.ds/dt - tp.ds/dt`
- `approachTime = distanceToEgoCar / relativeLongSpeed`（若分母为 0 则设为 0）
- `p.s = tp.s + tp.ds/dt * approachTime`

该 `p.s` 作为绕行/停车决策点的纵向位置基础。

#### Step 7：计算障碍物侧向占据与到边界的净空

用 TP 宽度构建侧向包络：

- `tpHalfWidth = tp.width / 2`
- `tpLeftEdge = tp.l + tpHalfWidth`
- `tpRightEdge = tp.l - tpHalfWidth`

与道路边界的净空：

- 左侧净空：`gapLeft = leftBoundaryDistance - tpLeftEdge`
- 右侧净空：`gapRight = tpRightEdge - rightBoundaryDistance`

#### Step 7.1：避障时的“碰撞检测”逻辑（当前实现：间隙可行性判定）

本项目当前在路径决策阶段采用的是一种**轻量级的碰撞风险判定**：

- 将障碍物在 Frenet 坐标系的横向占据近似为一个区间 $[l_{tp}-w_{tp}/2,\ l_{tp}+w_{tp}/2]$（即上一步的 `tpRightEdge` 到 `tpLeftEdge`）。
- 将道路边界视为“硬约束边界”（不可穿越），并计算障碍物到道路边界的可通行净空（`gapLeft/gapRight`）。
- 用净空是否足够容纳“ego 车辆宽度 + 两侧安全裕度”来判断绕行是否会产生横向碰撞风险。
- 当左右两侧都不满足净空要求时，采用**纵向安全缓冲**触发停车：停车点的纵向位置取 $s_{stop}=s_{meet}-d_{lon}$，其中 $s_{meet}$ 来自 Step 6 的相遇点预测（变量 `p.s`），$d_{lon}$ 来自配置 `decision.long_safe_margin`。

具体到代码（见 [decision_center.cpp](../src/planning_core/src/decision_center/decision_center.cpp)），判定阈值为：

$$
\mathrm{requiredWidth}=w_{ego}+2\cdot d_{lat}
$$

其中：

- $w_{ego}$ 对应 `egoCarInfo->getVehicleWidth()`
- $d_{lat}$ 对应配置 `decision.lat_safe_margin`

左右绕行可行性的判定即：

- 若 `gapLeft > requiredWidth`：认为左侧存在一条不与障碍物/边界发生横向侵入的可行通道（触发左绕行决策）。
- 否则若 `gapRight > requiredWidth`：认为右侧存在可行通道（触发右绕行决策）。
- 否则：认为左右都无法在侧向上满足“车宽+安全裕度”，因此绕行将产生碰撞/剐蹭风险，触发停车决策。

> 重要说明（与当前实现一致）：
> 1) 该“碰撞检测”本质是**横向间隙检查**，并不是对生成的完整轨迹做 2D 采样碰撞检测；
> 2) 判定时只显式使用了“车辆宽度”，未引入车辆长度、姿态（yaw）、曲线过渡过程中的扫掠体积；
> 3) 动态障碍物的时域碰撞预测尚未实现（moving TP 分支为 TODO）。
> 
> 因此它更适合作为**决策层快速兜底/粗筛**：先决定“左/右/停”，再由后端路径规划生成连续曲线（并可在后续迭代中加入更严格的轨迹碰撞检测）。

#### Step 8：按优先级决策（左绕行 > 右绕行 > 停车）

判定阈值采用“ego 宽度 + 双侧安全裕度”：

- 可绕行条件：`gap > ego.width + 2 * lat_safe_margin`

1) **左绕行（优先）**

- 若 `gapLeft` 满足阈值：
	- `p.type = DECISION_LEFT_OVERTAKE`
	- `p.l` 取“边界与障碍物边缘的中间位置”作为目标横向偏移。

2) **右绕行**

- 否则若 `gapRight` 满足阈值：
	- `p.type = DECISION_RIGHT_OVERTAKE`
	- `p.l` 同样取中间位置。

3) **停车**

- 否则：
	- `p.type = DECISION_STOP`
	- `p.l = 0`
	- `p.s = p.s - long_safe_margin`（在预测相遇点前方预留纵向安全距离；实现为减法，表示“提前停车”）
	- 并 `break`：停车决策优先级最高，直接终止后续 TP 处理。

> 实现细节提示：代码中的做法是“取中点”:
> - 左绕：`p.l = (leftBoundaryDistance + tpLeftEdge) / 2.0`
> - 右绕：`p.l = (rightBoundaryDistance + tpRightEdge) / 2.0`


#### Step 9：补齐决策影响区间的 START/END

若本周期生成了至少一个决策点：

- 在首个决策点前插入 `DECISION_START`：
	- `start.s = first.s - decisionMakingLeastDistance`
	- `start.l = 0`
- 若最后一个决策不是停车，则追加 `DECISION_END`：
	- `end.s = last.s + decisionMakingLeastDistance`
	- `end.l = 0`

这样下游规划在进入/退出绕行时有更平滑的“引入/退出”区间。

### 3.4 示例场景：按实现流程走一遍

下面给出一个“静态障碍物”的数值示例，帮助把上述 Step 0~9 串起来。示例参数尽量采用本项目默认配置（见 [planning_static_tps_config.yaml](../src/planning_core/config/planning_static_tps_config.yaml)），并以当前代码实现为准（见 [decision_center.cpp](../src/planning_core/src/decision_center/decision_center.cpp)）。

#### 示例公共配置/假设

- 地图/道路：`road_half_width = 4.0m`
	- `leftBoundaryDistance = 1.5 * road_half_width = 6.0m`
	- `rightBoundaryDistance = -(0.5 * road_half_width) = -2.0m`（右边界在参考线右侧，Frenet 坐标为负值）
- 安全距离：`lat_safe_margin = 0.5m`，`long_safe_margin = 10.0m`
- 局部路径点数：`local_path.path_size = 80`
	- `decisionMakingLeadPoint = clamp(80 - 50, 30, 40) = 30`
- Ego 状态：`ego.s = 50.0m`，`ego.ds/dt = 1.0m/s`，`ego.width = 1.5m`
	- `decisionMakingLeastDistance = max(ego.ds/dt * decisionMakingLeadPoint, 30.0) = max(30, 30) = 30m`

#### 场景 A：左侧绕行可行（输出 LEFT_OVERTAKE + START/END）

输入一个准静态 TP：

- `tp.s = 60.0m`（在 ego 前方 10m）
- `tp.l = 2.3m`（满足走廊：`-2.0 < 2.3 < 6.0`）
- `tp.ds/dt = 0.0m/s`，`tp.dl/dt = 0.0m/s`（准静态）
- `tp.width = 1.6m`（`tpHalfWidth = 0.8m`）

按代码流程计算：

1) **纵向范围过滤**：`distanceToEgoCar = 60 - 50 = 10m`，满足范围。

2) **准静态判定**：

- `fabs(tp.dl/dt) = 0 < 0.03` 成立
- `tp.ds/dt = 0 < ego.ds/dt / 2 = 0.5` 成立

3) **预测相遇点 s**：

- `relativeLongSpeed = 1.0 - 0.0 = 1.0`
- `approachTime = 10 / 1.0 = 10s`
- `p.s = tp.s + tp.ds/dt * approachTime = 60 + 0 * 10 = 60m`

4) **计算净空**：

- `tpLeftEdge = tp.l + tpHalfWidth = 2.3 + 0.8 = 3.1m`
- `gapLeft = leftBoundaryDistance - tpLeftEdge = 6.0 - 3.1 = 2.9m`
- 左侧绕行阈值：`ego.width + 2 * lat_safe_margin = 1.5 + 1.0 = 2.5m`
- `gapLeft = 2.9 > 2.5`，因此触发 **左绕行**。

5) **生成决策点**：

- 生成一个 `DECISION_LEFT_OVERTAKE`，纵向位置 `s = 60m`。
- 横向目标 `l` 的表达式当前实现为：`l = leftBoundaryDistance + tpLeftEdge / 2`。
	- 注意：这并不是严格的中点 $(leftBoundaryDistance + tpLeftEdge)/2$（详见上一节的“实现细节提示”）。

6) **插入 START/END**：

- `DECISION_START.s = 60 - 30 = 30m`，`l = 0`
- 因为最后一个决策不是 STOP，所以追加 `DECISION_END.s = 60 + 30 = 90m`，`l = 0`

最终输出 `pathDecisionPoints`（按顺序）：

1. `DECISION_START (s=30, l=0)`
2. `DECISION_LEFT_OVERTAKE (s=60, l=按实现公式计算)`
3. `DECISION_END (s=90, l=0)`

#### 场景 B：左右都不可绕行（输出 STOP + START，无 END）

仍使用同样 ego/配置，只把 TP 的横向位置改为更“居中”以导致净空不足，例如：

- `tp.s = 60.0m`，`tp.l = 3.5m`，`tp.ds/dt = 0.0m/s`，`tp.dl/dt = 0.0m/s`，`tp.width = 1.6m`

净空变为：

- `tpLeftEdge = 3.5 + 0.8 = 4.3m`，`gapLeft = 6.0 - 4.3 = 1.7m`（小于 2.5m）
- `tpRightEdge = 3.5 - 0.8 = 2.7m`，`gapRight = 2.7 - 2.0 = 0.7m`（小于 2.5m）

因此走“停车”分支：

- `p.type = DECISION_STOP`
- `p.s = predicted_meet_s - long_safe_margin = 60 - 10 = 50m`
- 插入 `DECISION_START.s = 50 - 30 = 20m`
- 因为最后一个决策是 STOP，所以**不追加** `DECISION_END`

最终输出 `pathDecisionPoints`（按顺序）：

1. `DECISION_START (s=20, l=0)`
2. `DECISION_STOP (s=50, l=0)`

> 额外说明：当前实现中一旦产生 STOP，会 `break` 退出 TP 循环，因此不会再对更远处的其它 TP 继续生成绕行点。

### 3.5 下游如何使用这些决策点生成连续局部路径

`LocalPathPlanner::generateLocalPath()`（../src/planning_core/src/local_planner/local_path/local_path_planner.cpp）对 `pathDecisionPoints` 的使用方式可以概括为：

1. 在 $s$ 轴上采样局部路径点（按 ego 当前 `s` 往前推进）。
2. 对每个采样点 `wayPoint_s`，找到它落在哪两个相邻决策点 `[j, j+1]` 之间。
3. 以这两个决策点为边界条件，使用 5 次多项式生成该区间的 $l(s)$（并求导得到 `dl/ds`、`ddl/ds`），从而把离散决策点变成连续曲线。
4. 对生成的 Frenet 曲线再做平滑（LocalPathSmoother）。
5. 用参考线上的投影参数把 Frenet 局部路径转换到笛卡尔坐标系。

因此，从系统设计角度看：

- `DecisionCenter` 负责把“绕哪边/停不停”的离散策略变成**少量关键 SLPoint**；
- `LocalPathPlanner` 负责把关键点插值/平滑成**连续、可控、密集采样**的局部路径点序列。

---

## 4. 当前实现的边界与后续扩展方向（与代码一致）

1. **动态障碍物未处理**：TP 速度较高或存在明显横向运动时，分支为 TODO（未产生决策）。
2. **`speed_limit` 字段未使用**：`SLPoint::speed_limit` 当前未在 `DecisionCenter` 里赋值，速度决策尚未实现。
3. **走廊与坐标约定依赖工程假设**：左右边界距离的推导基于项目对参考线/车道的假设，理解时以 `l` 的比较条件为准。
