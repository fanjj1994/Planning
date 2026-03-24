# Projected Point（投影点）计算说明

本文用于解释本项目中“投影点（Projected Point）”的计算方式与代码对应关系，并说明：

1) 严肃的投影点定义是什么；
2) 一般严格求投影点会用到哪些算法；
3) 本项目为什么采用近似做法，以及这种近似在当前模块为何足够。

核心结论先说清楚：

- **严格定义的投影点**：给定点 $P$ 与曲线集合 $\Gamma$（连续曲线或折线），投影点是

$$
\operatorname{proj}_{\Gamma}(P) \in \arg\min_{Q\in \Gamma} \|Q-P\|
$$

也就是 $\Gamma$ 上到 $P$ 欧氏距离最小的点（可能不唯一）。

- **严格求投影点的常见工程算法**：
	- 对折线（polyline）：对每个线段做“点到线段的垂足投影”（用向量点积求参数 $t$ 并夹紧到 $[0,1]$），取全局最小距离。
	- 对光滑参数曲线：最小化 $f(s)=\|r(s)-P\|^2$，解 $f'(s)=0$（通常用迭代法如 Newton/梯度法，或先粗采样再局部迭代）。

- **本项目中的投影点实现**采用常见近似：把参考线/LocalPath 看成“离散采样点序列”，投影点近似为**距离目标点最近的离散点（match point）**，不做线段内插值求精确垂足。
	- 主要理由是：实现简单、计算量更低；在参考线/LocalPath 足够稠密、且当前模块只需要稳定的 Frenet↔Cartesian 转换时，这个近似通常已经够用。

相关代码主要在：

- [src/planning_core/src/common/math/curve.h](../../src/planning_core/src/common/math/curve.h)
- [src/planning_core/src/common/math/curve.cpp](../../src/planning_core/src/common/math/curve.cpp)

---

## 1. 简介：什么是投影点？为什么需要投影点？

### 1.1 什么是投影点（严格定义）

严格来说，投影点是一个“最优化意义下的最近点”。当 $\Gamma$ 是一条足够光滑的曲线（至少可导）且最近点位于曲线内部（不在端点），在最优点 $Q$ 处往往满足“正交条件”：

$$
(P-Q)\cdot T(Q)=0
$$

其中 $T(Q)$ 是曲线在 $Q$ 处的切向量。直观上就是：从投影点指向 $P$ 的向量与曲线切线正交（“垂足”）。

对于折线（由线段组成）也类似：若最近点落在线段内部，则 $P-Q$ 与该线段方向向量正交；若最近点落在端点，则由端点成为最近点。

### 1.2 严格求投影点的一般算法（工程上怎么做）

本节只给出本项目相关、且工程里最常用的两类算法概览。

#### 1) 点到折线（polyline）的严格投影：点积 + 线段夹紧

给定目标点 $P$，以及折线上的一条线段 $A\rightarrow B$。

令：

- $\mathbf{u}=B-A$（线段方向向量）
- $\mathbf{v}=P-A$

把 $P$ 投影到直线 $AB$ 上的参数为：

$$
t=\frac{\mathbf{v}\cdot \mathbf{u}}{\|\mathbf{u}\|^2}
$$

当 $\|\mathbf{u}\|$ 很小（退化线段，$A\approx B$）时，通常直接把 $Q$ 取为 $A$（或 $B$），以避免数值不稳定。

然后把 $t$ 夹紧到线段范围：$t\leftarrow\operatorname{clip}(t,0,1)$。

线段上的最近点（严格意义的点到线段投影点）为：

$$
Q=A+t\,\mathbf{u}
$$

对折线的每一条线段都算一次 $Q$ 与距离 $\|Q-P\|$，取全局最小者就是严格投影点。

> 这也是“严格投影点”的一个典型实现：它比“找最近离散点”更精确，但计算量更大（需要遍历线段并做点积/夹紧/插值），同时还需要在 $Q$ 处插值出 $\theta,\kappa$ 等几何量。

#### 2) 点到光滑曲线的严格投影：极值条件 + 迭代求解

若曲线用参数形式表示为 $r(s)$，严格投影可写为：

$$
\min_{s}\; f(s)=\|r(s)-P\|^2
$$

其一阶必要条件为：

$$
f'(s)=2\,(r(s)-P)\cdot r'(s)=0
$$

这对应“位移向量与切向量正交”。实际工程里常见做法是：先粗采样得到一个接近的初值 $s_0$，再用 Newton 法或梯度下降在附近收敛到更精确的投影点。

### 1.3 什么是投影点（在本项目语境下）

给定：

- 一条“参考曲线”（参考线 Referline 或局部路径 LocalPath），由一系列点按顺序组成。
- 一个待处理的目标点（例如车辆当前位置、轨迹点），记为 $P(x, y)$。

投影点（Projected Point）是参考曲线上与 $P$ **最接近**的点（或近似最接近的点）。本项目中采用的是“最近离散点”的近似投影。

在本项目中，投影点最终会被组织为 [src/planning_core/src/common/math/curve.h](../../src/planning_core/src/common/math/curve.h) 中的 `ProjectedPointInfo`：

- `rs`：参考曲线上的弧长坐标（沿曲线累计距离）
- `rx, ry`：投影点在全局坐标系下的位置
- `rtheta`：投影点处参考曲线的切向航向角
- `rkappa`：投影点处参考曲线曲率
- `rdkappa`：投影点处曲率对弧长的导数（曲率变化率）

### 1.4 为什么规划里要计算投影点？

投影点是 Cartesian 与 Frenet 坐标系之间转换的关键中间量。

- 例如：`Curve::CartesianToFrenet()` 需要已知目标点在参考线上的投影点（含 $r_s, r\theta, r\kappa, r\kappa'$），才能计算 $s,l$ 以及导数。
- 例如：`Curve::FrenetToCartesian()` 需要已知 Frenet 状态对应的投影点，才能恢复到全局坐标系下的 $(x,y,\theta,\kappa,v,a)$。

因此“投影点模块”往往处在：

- 参考线/局部路径处理（给曲线点计算几何参数）
- 决策/规划中对目标点的坐标变换

这两者之间。

---

## 2. 项目中投影点计算的实现逻辑（原理 + 流程 + 函数对应）

为了讲清楚“已知什么 → 要算什么 → 具体怎么做”，本节按三个阶段组织：

1) **预处理（对曲线点）**：为 Referline/LocalPath 的每个离散点计算 `rs/rtheta/rkappa/rdkappa`。
2) **匹配（对目标点）**：根据目标点的位置或给定的 `rs`，找出最合适的离散点索引 `matchPointIndex`。
3) **组装（输出投影点）**：把曲线点上的参数拷贝到 `ProjectedPointInfo`。

### 2.1 已知/未知变量：输入是什么？输出是什么？

#### 输入（已知变量）

- 曲线：
	- Referline：`base_msgs::msg::Referline`，点序列在 `referenceline.refer_line`。
	- LocalPath：`base_msgs::msg::LocalPath`，点序列在 `localPath.local_path`。
- 目标点：`geometry_msgs::msg::PoseStamped targetPoint`（主要使用其中的 `position.x/y`）。
- 或者：查询弧长 `rs`（用于基于弧长的一维匹配）。

#### 输出（要计算/要得到的变量）

- `matchPointIndex`：目标点匹配到的曲线离散点下标。
- `ProjectedPointInfo projectedPoint`：投影点参数（`rs/rx/ry/rtheta/rkappa/rdkappa`）。

### 2.2 预处理：计算每个曲线点的 `rs/rtheta/rkappa/rdkappa`

对应函数：

- Referline：`Curve::calculateProjectedPointParameters(base_msgs::msg::Referline&)`
- LocalPath：`Curve::calculateProjectedPointParameters(base_msgs::msg::LocalPath&)`

#### 2.2.1 计算 `rs`（弧长累计）

已知：第 $i$ 个点的坐标 $(x_i, y_i)$。

要算：每个点的弧长坐标 $r_s(i)$。

实现：

- 令 $r_s(0)=0$
- 对 $i\ge 1$：

$$
r_s(i)=r_s(i-1)+\sqrt{(x_i-x_{i-1})^2 + (y_i-y_{i-1})^2}
$$

实现中通常用欧氏距离累加；代码里对应为 `std::hypot(dy, dx)`，其数学含义就是 $\sqrt{dx^2+dy^2}$（注意传参顺序不影响结果）。

#### 2.2.2 计算 `rtheta`（航向角）

要算：每个点处参考曲线的切向角 $r\theta(i)$。

实现：

- 对于非末点：用前向差分

$$
r\theta(i)=\operatorname{atan2}(y_{i+1}-y_i,\; x_{i+1}-x_i)
$$

- 对于末点：用后向差分

$$
r\theta(N-1)=\operatorname{atan2}(y_{N-1}-y_{N-2},\; x_{N-1}-x_{N-2})
$$

#### 2.2.3 计算 `rkappa`（曲率）

本项目对 Referline 的曲率采用一个简单定义：

$$
\kappa \approx \frac{d\theta}{ds}
$$

实现上用相邻点做差分（并用 `EPSILON` 防止除零）：

- 非末点：

$$
rkappa(i)=\frac{r\theta(i+1)-r\theta(i)}{\sqrt{(x_{i+1}-x_i)^2 + (y_{i+1}-y_i)^2}}
$$

- 末点：

$$
rkappa(N-1)=\frac{r\theta(N-1)-r\theta(N-2)}{\sqrt{(x_{N-1}-x_{N-2})^2 + (y_{N-1}-y_{N-2})^2}}
$$

LocalPath 的处理略有不同：

- `rtheta` 直接取 `localPathPoint.theta`
- `rkappa` 直接取 `localPathPoint.kappa`

也就是说 LocalPath 认为这些几何量在上游已经计算好了，这里只做“投影所需字段”的整理。

#### 2.2.4 计算 `rdkappa`（曲率变化率）

Referline 与 LocalPath 都使用相同的差分方式计算：

$$
rdkappa \approx \frac{d\kappa}{ds}
$$

- 非末点：

$$
rdkappa(i)=\frac{rkappa(i+1)-rkappa(i)}{\sqrt{(x_{i+1}-x_i)^2 + (y_{i+1}-y_i)^2}}
$$

- 末点：

$$
rdkappa(N-1)=\frac{rkappa(N-1)-rkappa(N-2)}{\sqrt{(x_{N-1}-x_{N-2})^2 + (y_{N-1}-y_{N-2})^2}}
$$

> 注：实现中对 `rtheta` 的差分没有做角度 wrap（例如从 $\pi$ 跳到 $-\pi$ 的情况），如果参考线存在这种跨越，`rkappa` 可能会出现异常尖峰；是否需要做 `NormalizeAngle` 取决于上游参考线生成的角度连续性。

### 2.3 匹配：根据目标点（或 `rs`）找到 `matchPointIndex`

对应函数（重载）：

- `Curve::findMatchPointIndex(const nav_msgs::msg::Path&, const int16 lastMatchPointIndex, const geometry_msgs::msg::PoseStamped&)`
- `Curve::findMatchPointIndex(const base_msgs::msg::Referline&, const geometry_msgs::msg::PoseStamped&)`
- `Curve::findMatchPointIndex(const base_msgs::msg::Referline&, const float64 rs)`
- `Curve::findMatchPointIndex(const base_msgs::msg::LocalPath&, const geometry_msgs::msg::PoseStamped&)`

#### 2.3.1 基于几何距离的匹配（最常用）

已知：

- 目标点 $P(x,y)$
- 曲线离散点序列 $(x_i, y_i)$

要算：

- `matchPointIndex = argmin_i \sqrt{(x_i-x)^2 + (y_i-y)^2}`

实现：遍历所有点，取欧氏距离最小的索引。

> 说明：这里的“最近点”是最近**离散点**（vertex），不是最近**线段点**。严格投影若按 1.2 的线段投影做，需要对每个线段计算垂足点并比较距离，同时还要在垂足点处插值/计算 $r\theta,r\kappa,r\kappa'$。

nav_msgs::Path 的版本额外引入了：

- `lastMatchPointIndex`
- `MAX_JUMP`

当候选索引与 `lastMatchPointIndex` 相差过大时会跳过该候选，用于抑制“匹配点突然跳到很远处”的异常情况。

#### 2.3.2 基于弧长 `rs` 的匹配（一维匹配）

已知：查询 `rs` 与曲线上每个点预先计算好的 `refer_line[i].rs`。

要算：

- `matchPointIndex = argmin_i |refer_line[i].rs - rs|`

这个版本不看目标点的 $(x,y)$，只在 Frenet 的纵向一维上找最近。

### 2.4 组装：输出 `ProjectedPointInfo`

对应函数：

- Referline：`Curve::figureOutProjectedPoint(const base_msgs::msg::Referline&, const geometry_msgs::msg::PoseStamped&, ProjectedPointInfo&)`
- LocalPath：`Curve::figureOutProjectedPoint(const base_msgs::msg::LocalPath&, const geometry_msgs::msg::PoseStamped&, ProjectedPointInfo&)`

已知：

- `matchPointIndex`

要算：

- `ProjectedPointInfo` 的 6 个字段

实现：直接从对应离散点把字段拷贝出来：

- `rs = curve[i].rs`
- `rx/ry = curve[i].pose.pose.position.(x/y)`
- `rtheta/rkappa/rdkappa = curve[i].(rtheta/rkappa/rdkappa)`

并且如 1.1 所述：这里是“离散最近点”的近似投影，不做线段内插值。

### 2.5 投影点在坐标变换中的使用（与曲线模块的关系）

投影点计算本身只负责提供 `ProjectedPointInfo`。它的主要使用者是：

- `Curve::CartesianToFrenet(const CartesianState&, const ProjectedPointInfo&, FrenetState&)`
- `Curve::FrenetToCartesian(const FrenetState&, const ProjectedPointInfo&, CartesianState&)`

其中 `FrenetToCartesian()` 里还会检查 `|projectedPoint.rs - frenet.s|`，若超过阈值 `DELTASMIN` 则报错返回，避免“用错投影点”造成的几何不一致。

---

## 3. 推荐的调用顺序（实践角度）

1) 当你拿到一条新的 Referline / LocalPath 点序列后，先调用：

- `Curve::calculateProjectedPointParameters(referenceline)` 或
- `Curve::calculateProjectedPointParameters(localPath)`

为每个点填好 `rs/rtheta/rkappa/rdkappa`。

2) 对于每一个需要转换坐标或计算 Frenet 状态的目标点：

- `Curve::figureOutProjectedPoint(curve, targetPoint, projectedPoint)`
- 然后把 `projectedPoint` 传入 `CartesianToFrenet()` / `FrenetToCartesian()`。

---

## 4. 局限与注意事项

- **离散近似误差**：不做线段内精确垂足投影，误差与曲线点密度相关；点越稠密，误差通常越小。
- **计算量**：距离匹配是 $O(N)$ 全扫描；对 nav_msgs::Path 提供了基于 `lastMatchPointIndex + MAX_JUMP` 的简单约束，但 Referline/LocalPath 的重载目前仍是全扫描。
- **最小点数要求**：`calculateProjectedPointParameters()` 对曲线点数有下限（参考线/LocalPath 小于 3 个点直接返回）。
- **角度差分**：参考线的 `rkappa = d\theta/ds` 依赖 `rtheta` 的连续性；上游若产生 $\pi$ 附近跳变，建议在上游或此处做角度归一化处理。

