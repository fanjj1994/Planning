
# Frenet Coordinate

本文介绍 Frenet（Frénet–Serret）坐标系在自动驾驶/ADAS 规划中的常见用法，并给出 Frenet 与 Cartesian（直角坐标系）之间的转换公式与推导过程。

---

## 1. Introduction

### 1.1 Frenet 坐标系是什么？

Frenet 坐标系是**相对于一条参考曲线**（Reference Line）来定义的局部坐标系。对于参考线上的某个弧长参数 $s$：

- 参考线点的位置为 $\mathbf{r}(s) = [x_r(s),\; y_r(s)]^\top$
- 参考线在该点的切向（单位向量）为 $\mathbf{t}(s)$，法向（单位向量）为 $\mathbf{n}(s)$
- Frenet 坐标用 $(s, l)$ 描述任意点：
	- $s$：沿参考线切向的“前进距离”（通常取参考线的弧长参数）
	- $l$：相对于参考线的横向偏移（沿法向），一般约定**向左为正**
**Frenet 坐标系的产生原理——轨迹向参考线的投影**

Frenet 坐标系的核心思想是：给定一条光滑的参考曲线（通常为道路中心线），空间中任意一点都可以通过**向参考线投影**来获得其 Frenet 坐标。具体过程如下：

1. **确定投影点**：对于待描述的点 $P=(x,y)$（例如自车位置或轨迹上某一点），在参考线上找到距离 $P$ 最近的点 $R$，称为**投影点**（也叫最近点或匹配点）。几何上，这等价于从 $P$ 向参考线做垂线，垂足即为 $R$（因为最近点处切线必然与连线垂直——否则沿切向微移即可找到更近的点）。投影点 $R$ 在参考线上对应的弧长参数即为纵向坐标 $s$。

2. **建立局部正交基**：在投影点 $R$ 处，利用参考线的微分几何性质建立一组正交基：
	- **切向量** $\mathbf{t}(s) = (\cos\theta_r,\;\sin\theta_r)^\top$：沿参考线前进方向，其中 $\theta_r$ 是参考线在该点的航向角。
	- **法向量** $\mathbf{n}(s) = (-\sin\theta_r,\;\cos\theta_r)^\top$：将切向量逆时针旋转 90° 得到，指向参考线左侧。
	- 这组 $(\mathbf{t}, \mathbf{n})$ 基随 $s$ 沿参考线移动而连续变化，构成了一个**沿曲线滑动的局部坐标系**——这正是 Frenet 标架（Frenet frame）。

3. **计算横向偏移**：点 $P$ 到投影点 $R$ 的位移向量 $\vec{d} = P - R$ 在法向 $\mathbf{n}(s)$ 上的投影即为横向坐标 $l$。当 $P$ 在参考线左侧时 $l > 0$，在右侧时 $l < 0$。

用向量语言，任意点的位置可表达为：
$$
\mathbf{p} = \mathbf{r}(s) + l\,\mathbf{n}(s)
$$
其中 $\mathbf{r}(s)$ 为参考线上弧长 $s$ 对应的点。这个关系是 Frenet 坐标系一切推导的出发点——轨迹上的点被"分解"为沿参考线的纵向分量 $s$ 和垂直于参考线的横向分量 $l$。

> 该投影关系成立需要两个基本假设：
> - **投影唯一性**：点 $P$ 距离参考线足够近，使得最近点唯一。当参考线曲率很大或点离参考线很远时，可能出现多个等距投影点，导致 $(s,l)$ 不唯一。
> - **法向不退化**：$1-\kappa_r l \neq 0$，否则坐标变换的 Jacobian 退化，微分关系失效（参见 1.3 节缺点部分的讨论）。
直观理解：

- $s$ 类似“沿车道中心线走了多远”
- $l$ 类似“偏离中心线多少米”

在本项目中，参考线点通常包含（或可计算出）如下信息：

- $r_s$：参考线弧长（`rs`）
- $\theta_r$：参考线航向角（`rtheta`）
- $\kappa_r$：参考线曲率（`rkappa`）
- $\kappa'_r = d\kappa_r/ds$：参考线曲率对弧长的导数（`rdkappa`）

这些量在参考线点序列上可由离散几何近似计算得到（本项目也提供了对应的计算函数）。

### 1.2 在 ADAS/自动驾驶规划中的应用

Frenet 坐标在规划模块中非常常见，典型用途包括：

- **纵向/横向解耦**：把“沿道路前进（纵向）”与“横向偏移/换道（横向）”分开处理。
- **多项式轨迹生成**：常用 $l(s)$（横向偏移随弧长变化）或 $s(t)$（纵向随时间变化）用多项式描述，便于满足边界条件与光滑性约束。
- **约束表达更自然**：车道边界、横向安全距离、沿道路的限速/坡度等，在 Frenet 域往往比在 $(x,y)$ 上更易表达。
- **参考线投影**：感知/预测目标（或自车）先投影到参考线得到 $(s,l)$，再在 Frenet 域做决策和轨迹搜索。

### 1.3 优势与缺点

**优势**

- 变量语义强：$s$ 表示沿道路前进，$l$ 表示横向偏移。
- 更易做轨迹生成与优化：常见约束（车道宽度、目标车在前方多少米）更直观。
- 在“沿道路行驶”的场景，局部近似效果好，计算与搜索更高效。

**缺点 / 注意点**

- **强依赖参考线质量**：参考线不连续、抖动或曲率估计噪声大，会直接影响 Frenet 转换与规划稳定性。
- **存在奇异/退化情况**：当 $1-\kappa_r l \approx 0$ 时，坐标变换会出现数值问题（几何上对应偏移过大导致“法向偏移曲线”局部退化）。
- **不是全局坐标**：离参考线过远、道路曲率很大或参考线不唯一时，$(s,l)$ 的意义会变弱，甚至出现投影不唯一。

---

## 2. Frenet 坐标系的表示

规划与控制中常用的不仅是 $(s,l)$，还需要其一阶/二阶导数。该项目的转换接口同时使用了：

- 对时间的导数（例如 $ds/dt$）
- 对弧长的导数（例如 $dl/ds$）

### 2.1 Cartesian 坐标变量与参考线投影点变量

在进行 Frenet↔Cartesian 转换时，需要用到待转换点的 **Cartesian 状态**以及该点在参考线上的**投影点信息**。下表整理了这两类变量：

**待转换点的 Cartesian 状态变量**

| 符号 | 变量名（本项目接口） | 含义 | 常见单位 |
|---|---|---|---|
| $x$ | `x` | 点在全局坐标系下的 x 坐标 | m |
| $y$ | `y` | 点在全局坐标系下的 y 坐标 | m |
| $\theta$ | `theta` | 航向角（速度方向与 x 轴正方向的夹角） | rad |
| $v$ | `speed` | 速度标量 | m/s |
| $a$ | `acceleration` | 加速度标量（$\dot{v}$） | m/s² |
| $\kappa$ | `curvature` | 轨迹曲率 | 1/m |

**参考线投影点的变量（以 `r` 前缀命名）**

| 符号 | 变量名（本项目接口） | 含义 | 常见单位 | 备注 |
|---|---|---|---|---|
| $r_s$ | `rs` | 投影点在参考线上的弧长 | m | 即待转换点对应的 $s$ |
| $x_r$ | `rx` | 投影点的 x 坐标 | m | |
| $y_r$ | `ry` | 投影点的 y 坐标 | m | |
| $\theta_r$ | `rtheta` | 投影点处参考线的航向角 | rad | $\theta_r = \text{atan2}(\Delta y, \Delta x)$ |
| $\kappa_r$ | `rkappa` | 投影点处参考线的曲率 | 1/m | $\kappa_r = d\theta_r / ds$ |
| $\kappa'_r$ | `rdkappa` | 投影点处参考线曲率对弧长的导数 | 1/m² | $\kappa'_r = d\kappa_r / ds$ |

### 2.2 Frenet 坐标变量

下表整理了常见 Frenet 变量（符号）与本项目接口变量名（如 `ds_dt`）的对应关系。

| 符号 | 变量名（本项目接口） | 含义 | 常见单位 | 备注 |
|---|---|---|---|---|
| $s$ | `s` | 沿参考线的弧长坐标 | m | 常取投影点参考线弧长 |
| $\dot{s}$ | `ds_dt` | $ds/dt$，纵向速度（沿参考线参数的变化率） | m/s | 与实际车速不完全相同 |
| $\ddot{s}$ | `dds_dt` | $d^2s/dt^2$ | m/s² | 用于与加速度/约束关联 |
| $l$ | `l` | 横向偏移（相对参考线法向） | m | 常约定左正右负 |
| $l'$ | `dl_ds` | $dl/ds$，横向偏移随弧长的变化率 | 1（无量纲） | 决定航向相对参考线的偏差 |
| $\dot{l}$ | `dl_dt` | $dl/dt$，横向速度 | m/s | 与 `dl_ds * ds_dt` 等价（当 $l=l(s)$） |
| $l''$ | `ddl_ds` | $d^2l/ds^2$ | 1/m | 影响曲率与光滑性 |
| $\ddot{l}$ | `ddl_dt` | $d^2l/dt^2$，横向加速度 | m/s² | 可由链式法则从 $l',l'',\dot{s},\ddot{s}$ 得到 |

此外，进行坐标变换还需要“参考线投影点”的 Cartesian 参数（本项目在转换函数参数中以 `r*` 前缀出现）：

- $r_s$（`rs`）：投影点弧长
- $(x_r, y_r)$（`rx, ry`）：投影点位置
- $\theta_r$（`rtheta`）：投影点航向
- $\kappa_r$（`rkappa`）：投影点曲率
- $\kappa'_r$（`rdkappa`）：投影点曲率导数

---

## 3. Frenet 与 Cartesian 坐标系之间的转换公式

下面给出常用的二维（忽略高度）转换关系。为避免符号混乱，先统一基本定义：

- 参考线在弧长 $s$ 处的航向角 $\theta_r(s)$
- 切向、法向单位向量：
	$$
	\mathbf{t}(s) = \begin{bmatrix}\cos\theta_r \\ \sin\theta_r\end{bmatrix},\qquad
	\mathbf{n}(s) = \begin{bmatrix}-\sin\theta_r \\ \cos\theta_r\end{bmatrix}
	$$
- 定义
	$$A \triangleq 1 - \kappa_r\,l$$
	其中 $\kappa_r$ 为参考线曲率。

### 3.1 Frenet → Cartesian（由 $s,l$ 得到 $x,y,\theta,v,a,\kappa$）

已知 Frenet 状态：
$$s,\; \dot{s},\; \ddot{s},\; l,\; l' = \frac{dl}{ds},\; l'' = \frac{d^2l}{ds^2}$$
以及投影点参考线信息：$x_r,y_r,\theta_r,\kappa_r,\kappa'_r$。

**(1) 位置**

$$
\begin{aligned}
x &= x_r - l\sin\theta_r \\
y &= y_r + l\cos\theta_r
\end{aligned}
$$

**推导**：点在参考线法向方向偏移 $l$，即 $\mathbf{p} = \mathbf{r}(s) + l\,\mathbf{n}(s)$，代入 $\mathbf{n}(s)$ 即得。

**(2) 航向角**

先定义航向差（相对参考线切向的偏差角）
$$
\Delta\theta \triangleq \arctan(\frac{l'}{A})
$$
则
$$
	\theta = \theta_r + \Delta\theta = \theta_r + \arctan(\frac{l'}{A}) = \theta_r +\arctan(\frac{l'}{1 - \kappa_r\,l})
$$

**推导核心**：

轨迹在 Cartesian 中可写为 $\mathbf{p}(s)=\mathbf{r}(s)+l(s)\mathbf{n}(s)$，对 $s$ 求导：

$$
\mathbf{p}'(s)=\frac{d\mathbf{p}}{ds} = A\,\mathbf{t}(s) + l'\,\mathbf{n}(s)
$$

因此轨迹切向与参考线切向夹角满足：
$$
	\tan(\Delta\theta)=\frac{l'}{A}
$$

**(3) 速度**

若假设 $l=l(s)$（常见于路径规划），则链式法则：
$$
\dot{l} = l'\,\dot{s}
$$

速度分解（在参考线切-法坐标下），将 $\mathbf{p}'(s)$ 乘以 $\dot{s}$ 得到速度向量：

$$
\mathbf{\dot{p}} = \dot{s}\,\mathbf{p}'(s) = \underbrace{\dot{s}(1-\kappa_r l)}_{\text{切向分量}}\,\mathbf{t} + \underbrace{\dot{s}\,l'}_{\text{法向分量}}\,\mathbf{n}
$$

由于 $\mathbf{t}$ 和 $\mathbf{n}$ 正交，速度标量为两分量的平方和开根号：

$$
v = \sqrt{[\dot{s}(1-\kappa_r l)]^2 + (\dot{s}\,l')^2}
$$

提取公因子 $\dot{s}$ 可得等价紧凑形式 $v = \dot{s}\sqrt{A^2+(l')^2}$。

进一步，利用航向差关系 $\cos\Delta\theta = \dfrac{A}{\sqrt{A^2+(l')^2}}$（来自 (2) 的定义），可得另一常用形式：

$$
v = \frac{\dot{s}(1-\kappa_r l)}{\cos\Delta\theta}
$$

该形式在推导加速度时尤为方便（见 (5)）。

**(4) 曲率**

曲率 $\kappa$ 定义为航向角对轨迹自身弧长的变化率。设轨迹弧长微元为 $d\sigma$，参考线弧长微元为 $ds$，二者关系为：

$$
d\sigma = |\mathbf{p}'|\,ds = \frac{1-\kappa_r l}{\cos\Delta\theta}\,ds
$$

因此

$$
\kappa = \frac{d\theta}{d\sigma} = \frac{d\theta/ds}{|\mathbf{p}'|} = \frac{\cos\Delta\theta}{1-\kappa_r l}\,\frac{d\theta}{ds}
$$

**计算 $d\theta/ds$：** 由 $\theta = \theta_r + \Delta\theta$ 对 $s$ 求导：

$$
\frac{d\theta}{ds} = \kappa_r + \frac{d\Delta\theta}{ds}
$$

其中 $\Delta\theta = \arctan\!\left(\dfrac{l'}{1-\kappa_r l}\right)$，利用反正切的导数公式：

$$
\frac{d\Delta\theta}{ds} = \frac{1}{1+\left(\frac{l'}{A}\right)^2}\cdot\frac{l''\,A - l'\,A'}{A^2} = \frac{l''\,A + l'(\kappa'_r l + \kappa_r l')}{A^2+(l')^2}
$$

这里用了 $A' = -\kappa'_r l - \kappa_r l'$，因此 $-l'A' = l'(\kappa'_r l + \kappa_r l')$。

再利用 $l' = A\tan\Delta\theta$ 以及 $A^2+(l')^2 = A^2/\cos^2\Delta\theta$ 化简：

$$
\frac{d\Delta\theta}{ds} = \frac{\left[l'' + (\kappa'_r l + \kappa_r l')\tan\Delta\theta\right]\cos^2\Delta\theta}{1-\kappa_r l}
$$

代入 $\kappa$ 的表达式，得到：

$$
\kappa = \left(\left(l'' + (\kappa'_r l + \kappa_r l')\tan\Delta\theta\right)\frac{\cos^2\Delta\theta}{1-\kappa_r l} + \kappa_r\right)\frac{\cos\Delta\theta}{1-\kappa_r l}
$$

> **等价形式验证**：将 $\cos\Delta\theta = A/\sqrt{A^2+(l')^2}$、$\tan\Delta\theta = l'/A$ 代回上式，可还原为不含 $\Delta\theta$ 的经典形式：
> $$\kappa = \frac{\kappa_r A^2 + Al'' + \kappa'_r l l' + 2\kappa_r(l')^2}{(A^2+(l')^2)^{3/2}}$$
> 两种形式完全等价；$\Delta\theta$ 形式在工程实现中更常用，因为 $\Delta\theta$ 通常已在航向角计算中获得。

**(5) 加速度（标量）**

将加速度理解为速度标量的时间导数 $a = \dot{v}$。利用 (3) 中的速度表达 $v = \dfrac{\dot{s}(1-\kappa_r l)}{\cos\Delta\theta}$，对时间 $t$ 求导：

$$
a = \frac{d}{dt}\!\left(\frac{\dot{s}\,A}{\cos\Delta\theta}\right) = \frac{\ddot{s}\,A}{\cos\Delta\theta} + \dot{s}\,\frac{d}{dt}\!\left(\frac{A}{\cos\Delta\theta}\right)
$$

**计算 $\dfrac{d}{dt}\!\left(\dfrac{A}{\cos\Delta\theta}\right)$：** 利用商法则展开：

$$
\frac{d}{dt}\!\left(\frac{A}{\cos\Delta\theta}\right) = \frac{\dot{A}\cos\Delta\theta + A\sin\Delta\theta\cdot\dot{\Delta\theta}}{\cos^2\Delta\theta}
$$

其中各项由链式法则得到：

- $\dot{A} = \dfrac{dA}{ds}\,\dot{s} = -(\kappa'_r l + \kappa_r l')\,\dot{s}$
- $\dot{\theta} = \kappa\,v$（曲率与角速度的关系），$\dot{\theta}_r = \kappa_r\,\dot{s}$
- $\dot{\Delta\theta} = \dot{\theta} - \dot{\theta}_r = \kappa\,v - \kappa_r\,\dot{s} = \dot{s}\!\left(\dfrac{\kappa(1-\kappa_r l)}{\cos\Delta\theta} - \kappa_r\right)$

注意这里的 $\kappa$ 为 (4) 中已求得的**轨迹曲率**。将 $A\sin\Delta\theta / \cos\Delta\theta = A\tan\Delta\theta = l'$ 代入并化简：

$$
\frac{d}{dt}\!\left(\frac{A}{\cos\Delta\theta}\right) = \frac{\dot{s}}{\cos\Delta\theta}\!\left[l'\!\left(\frac{\kappa(1-\kappa_r l)}{\cos\Delta\theta} - \kappa_r\right) - (\kappa'_r l + \kappa_r l')\right]
$$

因此最终得到加速度公式：

$$
a = \ddot{s}\,\frac{1-\kappa_r l}{\cos\Delta\theta} + \frac{\dot{s}^2}{\cos\Delta\theta}\!\left[l'\!\left(\kappa\,\frac{1-\kappa_r l}{\cos\Delta\theta} - \kappa_r\right) - (\kappa'_r l + \kappa_r l')\right]
$$

> 说明：该公式依赖轨迹曲率 $\kappa$（已由 (4) 求得）。Frenet→Cartesian 的转换流程按"位置→航向→曲率→速度→加速度"的顺序逐步计算。

---

### 3.2 Cartesian → Frenet（由 $x,y,\theta,v,a,\kappa$ 得到 $s,l,\dot{s},\ddot{s},l',l''$ 等）

已知点的 Cartesian 状态：
$$x,\;y,\;\theta,\;v,\;a,\;\kappa$$
以及其在参考线上的投影点参数：$r_s,x_r,y_r,\theta_r,\kappa_r,\kappa'_r$。

**(1) $s$ 与 $l$**

通常取
$$s = r_s$$

**横向偏移 $l$ 的计算**

横向偏移 $l$ 表示点 $P=(x,y)$ 相对投影点 $R=(x_r,y_r)$ 在参考线法向方向上的带符号距离：

$$
l = \mathrm{sign}\!\left((y-y_r)\cos\theta_r - (x-x_r)\sin\theta_r\right) \cdot \sqrt{(x-x_r)^2 + (y-y_r)^2}
$$

**推导与解释**

该公式将 $l$ 的计算分解为"**距离大小**"和"**方向符号**"两部分：

**① 距离部分** $\sqrt{(x-x_r)^2+(y-y_r)^2}$

这是点 $P$ 到投影点 $R$ 的欧几里得距离 $|\vec{d}|$。由于 $R$ 是参考线上距 $P$ 最近的点，根据最近点的几何性质，位移向量 $\vec{d}=(x-x_r,\,y-y_r)$ 必然**垂直于参考线切向** $\mathbf{t}$——否则沿切向微移即可找到更近的点。因此 $\vec{d}$ 完全在法向方向上，即 $\vec{d} = l\,\mathbf{n}(s)$，从而：

$$
|\vec{d}| = |l|
$$

**② 符号部分** $\mathrm{sign}((y-y_r)\cos\theta_r-(x-x_r)\sin\theta_r)$——**二维叉积判定左右侧**

括号内的表达式本质上是参考线**切向量** $\mathbf{t}$ 与位移向量 $\vec{d}$ 的**二维叉积**（标量结果）：

$$
\mathbf{t}\times\vec{d} = \cos\theta_r\cdot(y-y_r) - \sin\theta_r\cdot(x-x_r)
$$

> **回顾二维叉积**：对于两个二维向量 $\mathbf{a}=(a_x,a_y)$、$\mathbf{b}=(b_x,b_y)$，其叉积定义为标量
> $$\mathbf{a}\times\mathbf{b} = a_x b_y - a_y b_x = |\mathbf{a}||\mathbf{b}|\sin\alpha$$
> 其中 $\alpha$ 是从 $\mathbf{a}$ 逆时针转到 $\mathbf{b}$ 的有向角。因此叉积的**符号直接反映 $\mathbf{b}$ 在 $\mathbf{a}$ 的哪一侧**：
> - $\mathbf{a}\times\mathbf{b} > 0$：$\mathbf{b}$ 在 $\mathbf{a}$ 的**左侧**（逆时针方向）
> - $\mathbf{a}\times\mathbf{b} < 0$：$\mathbf{b}$ 在 $\mathbf{a}$ 的**右侧**（顺时针方向）
> - $\mathbf{a}\times\mathbf{b} = 0$：$\mathbf{b}$ 与 $\mathbf{a}$ **共线**

代入 $\mathbf{t}=(\cos\theta_r,\sin\theta_r)$ 和 $\vec{d}=(x-x_r,\,y-y_r)$：

$$
\mathbf{t}\times\vec{d} = \cos\theta_r\,(y-y_r) - \sin\theta_r\,(x-x_r)
$$

这正是公式中 $\mathrm{sign}$ 内的表达式。其几何含义非常直观：**$\vec{d}$ 在切向 $\mathbf{t}$ 的左侧（叉积 > 0）则 $l > 0$，在右侧（叉积 < 0）则 $l < 0$**。

> **叉积与法向点积的等价性**：二维叉积 $\mathbf{t}\times\vec{d}$ 在数值上等于 $\vec{d}$ 与法向 $\mathbf{n}$ 的点积：
> $$\mathbf{t}\times\vec{d} = \vec{d}\cdot\mathbf{n}$$
> 这是因为法向 $\mathbf{n}$ 是将 $\mathbf{t}$ 逆时针旋转 90° 得到的，而二维叉积 $\mathbf{a}\times\mathbf{b}$ 恰好等于"$\mathbf{a}$ 旋转 90° 后与 $\mathbf{b}$ 的点积"。两种理解方式完全等价，但叉积的"左/右判定"语义更直接。

**③ 等价性证明**

在投影点处 $\vec{d}\perp\mathbf{t}$，因此 $\vec{d}=l\,\mathbf{n}$，于是：

$$
\mathrm{sign}(\mathbf{t}\times\vec{d})\cdot|\vec{d}| = \mathrm{sign}(l)\cdot|l| = l
$$

即 $\mathrm{sign}$ 形式与直接法向投影形式完全等价。

> **与法向投影形式的关系**：直接计算 $\vec{d}\cdot\mathbf{n}$（等价于 $\mathbf{t}\times\vec{d}$）也可得到 $l$：
> $$l = (x-x_r)(-\sin\theta_r) + (y-y_r)\cos\theta_r$$
> 两种形式数学上完全等价。使用 $\mathrm{sign}\cdot\sqrt{\cdot}$ 的形式有两点好处：(1) 几何含义更清晰——$|l|$ 就是点到参考线的欧氏距离，叉积符号表示点在参考线的左/右侧；(2) 在浮点计算中，先求欧氏距离再加符号，可避免当两项几乎相消时的数值精度损失。

**(2) 航向差与 $l'$**

$$
\Delta\theta = \mathrm{NormalizeAngle}(\theta - \theta_r)
$$

利用 $\tan(\Delta\theta)=\frac{l'}{A}$，得到
$$
l' = A\tan(\Delta\theta)
$$

**(3) $\dot{s}$ 与 $\dot{l}$**

速度在切-法坐标下的分解满足：
$$
\dot{s} = \frac{v\cos(\Delta\theta)}{A},\qquad
\dot{l} = v\sin(\Delta\theta)
$$

并且当 $l=l(s)$ 时，仍有一致关系：
$$\dot{l} = l'\dot{s}$$

**(4) 由曲率求 $l''$（$\Delta\theta$ 形式）**

从 3.1 节 (4) 给出的 $\Delta\theta$ 形式的曲率公式出发：

$$
\kappa = \left(\left(l'' + (\kappa'_r l + \kappa_r l')\tan\Delta\theta\right)\frac{\cos^2\Delta\theta}{1-\kappa_r l} + \kappa_r\right)\frac{\cos\Delta\theta}{1-\kappa_r l}
$$

反解 $l''$：

**Step 1**：两侧乘以 $\dfrac{1-\kappa_r l}{\cos\Delta\theta}$，消去最外层的因子：

$$
\frac{\kappa(1-\kappa_r l)}{\cos\Delta\theta} = \left(l'' + (\kappa'_r l + \kappa_r l')\tan\Delta\theta\right)\frac{\cos^2\Delta\theta}{1-\kappa_r l} + \kappa_r
$$

**Step 2**：移项，将 $\kappa_r$ 移至左侧：

$$
\frac{\kappa(1-\kappa_r l)}{\cos\Delta\theta} - \kappa_r = \left(l'' + (\kappa'_r l + \kappa_r l')\tan\Delta\theta\right)\frac{\cos^2\Delta\theta}{1-\kappa_r l}
$$

**Step 3**：两侧乘以 $\dfrac{1-\kappa_r l}{\cos^2\Delta\theta}$：

$$
\frac{1-\kappa_r l}{\cos^2\Delta\theta}\left(\frac{\kappa(1-\kappa_r l)}{\cos\Delta\theta} - \kappa_r\right) = l'' + (\kappa'_r l + \kappa_r l')\tan\Delta\theta
$$

**Step 4**：解出 $l''$：

$$
\boxed{l'' = -(\kappa'_r l + \kappa_r l')\tan\Delta\theta + \frac{1-\kappa_r l}{\cos^2\Delta\theta}\left(\kappa\,\frac{1-\kappa_r l}{\cos\Delta\theta} - \kappa_r\right)}
$$

该式在 $\cos\Delta\theta \neq 0$（即 $|\Delta\theta| \neq \pi/2$）且 $1-\kappa_r l \neq 0$ 时有效。

> **等价形式验证**：将 $\cos\Delta\theta = A/\sqrt{A^2+(l')^2}$、$\tan\Delta\theta = l'/A$ 代入上式并化简，可还原为不含 $\Delta\theta$ 的经典形式：
> $$l'' = \frac{\kappa\left(A^2+(l')^2\right)^{3/2} - \kappa_r A^2 - \kappa'_r l l' - 2\kappa_r(l')^2}{A}$$
> 两种形式数学等价。$\Delta\theta$ 形式在工程实现中更常用，因为 $\Delta\theta$ 和 $\kappa$ 在前置步骤 (2)、(3) 中已经获得，可直接代入。

**(5) $\ddot{l}$（横向加速度）**

**方法一：链式法则**

若已知 $l', l'', \dot{s}, \ddot{s}$，由 $\dot{l} = l'\dot{s}$ 对 $t$ 求导可得：
$$
\ddot{l} = l''\,\dot{s}^2 + l'\,\ddot{s}
$$

**方法二：从速度分解直接推导**

从 (3) 中的精确关系 $\dot{l} = v\sin\Delta\theta$ 出发，对时间 $t$ 求导（乘积法则）：

$$
\ddot{l} = \frac{d}{dt}(v\sin\Delta\theta) = \dot{v}\sin\Delta\theta + v\cos\Delta\theta\cdot\dot{\Delta\theta}
$$

即

$$
\ddot{l} = a\sin\Delta\theta + v\cos\Delta\theta\cdot\dot{\Delta\theta}
$$

其中 $\dot{\Delta\theta} = \dot{\theta} - \dot{\theta}_r$。利用曲率-角速度关系 $\dot{\theta}=\kappa v$ 和 $\dot{\theta}_r = \kappa_r\dot{s}$：

$$
\dot{\Delta\theta} = \kappa\,v - \kappa_r\,\dot{s}
$$

**近似形式**：在实际驾驶中，车辆沿参考线行驶时航向差 $\Delta\theta$ 通常较小且变化缓慢。当 $\dot{\Delta\theta}\approx 0$（即车辆航向变化率近似等于参考线航向变化率 $\kappa v \approx \kappa_r\dot{s}$）时，第二项可忽略，得到简化公式：

$$
\boxed{\ddot{l} \approx a\sin\Delta\theta}
$$

> **该近似成立的物理条件**：$\kappa v = \kappa_r\dot{s}$ 等价于 $\kappa = \kappa_r\cos\Delta\theta/(1-\kappa_r l)$，几何含义是车辆当前的瞬时曲率恰好等于在横向偏移 $l$ 处与参考线等距的曲线的曲率。这在车辆大致沿车道行驶、横向偏移缓慢变化时天然近似满足。该公式直观含义也很清晰：纵向加速度 $a$ 通过 $\sin\Delta\theta$ "投影"到法向方向，即为横向加速度。

**(6) 由加速度求 $\ddot{s}$（与 3.1 的加速度定义配套）**

若仍采用 $a=\dot{v}$ 的标量定义，由 3.1 节 (5) 的加速度公式：
$$a = \ddot{s}\,\frac{A}{\cos\Delta\theta} + \frac{\dot{s}^2}{\cos\Delta\theta}\!\left[l'\!\left(\kappa\,\frac{A}{\cos\Delta\theta} - \kappa_r\right) - (\kappa'_r l + \kappa_r l')\right]$$
可反解
$$
\ddot{s} = \frac{a\cos\Delta\theta - \dot{s}^2\!\left[l'\!\left(\kappa\,\dfrac{1-\kappa_r l}{\cos\Delta\theta} - \kappa_r\right) - (\kappa'_r l + \kappa_r l')\right]}{1-\kappa_r l}
$$

其中 $A = 1-\kappa_r l$，$\kappa$ 为轨迹曲率（已由 3.1 节 (4) 或 3.2 节 (4) 求得）。

---

## 小结

### Frenet → Cartesian（自然坐标系 → 笛卡尔坐标系）

**已知量**：

- 参考线投影点信息：$[x_r,\; y_r,\; \theta_r,\; \kappa_r,\; \kappa'_r]$
- Frenet 状态：$[s,\; \dot{s},\; \ddot{s},\; l,\; \dot{l},\; \ddot{l},\; l',\; l'']$

**求解量**：$[x,\; y,\; \theta,\; \kappa,\; v,\; a]$

| 序号 | 求解量 | 公式 |
|:---:|:---:|:---|
| (1) | $x$ | $x = x_r - l\sin\theta_r$ |
| (2) | $y$ | $y = y_r + l\cos\theta_r$ |
| (3) | $\theta$ | $\theta = \theta_r + \arctan\!\left(\dfrac{l'}{1-\kappa_r l}\right)$ |
| (4) | $\kappa$ | $\kappa = \left(\left(l'' + (\kappa'_r l + \kappa_r l')\tan\Delta\theta\right)\dfrac{\cos^2\Delta\theta}{1-\kappa_r l} + \kappa_r\right)\dfrac{\cos\Delta\theta}{1-\kappa_r l}$ |
| (5) | $v$ | $v = \sqrt{[\dot{s}(1-\kappa_r l)]^2 + (\dot{s}\,l')^2}$ |
| (6) | $a$ | $a = \ddot{s}\,\dfrac{1-\kappa_r l}{\cos\Delta\theta} + \dfrac{\dot{s}^2}{\cos\Delta\theta}\!\left[l'\!\left(\kappa\,\dfrac{1-\kappa_r l}{\cos\Delta\theta} - \kappa_r\right) - (\kappa'_r l + \kappa_r l')\right]$ |

其中 $\Delta\theta = \theta - \theta_r = \arctan\!\left(\dfrac{l'}{1-\kappa_r l}\right)$。

**求解顺序**：位置 $(x,y)$ → 航向 $\theta$ → 曲率 $\kappa$ → 速度 $v$ → 加速度 $a$。

---

### Cartesian → Frenet（笛卡尔坐标系 → 自然坐标系）

**已知量**：

- 参考线投影点信息：$[x_r,\; y_r,\; \theta_r,\; \kappa_r,\; \kappa'_r,\; r_s]$
- Cartesian 状态：$[x,\; y,\; \theta,\; \kappa,\; v,\; a]$

**求解量**：$[s,\; \dot{s},\; \ddot{s},\; l,\; \dot{l},\; \ddot{l},\; l',\; l'']$

| 序号 | 求解量 | 公式 |
|:---:|:---:|:---|
| (1) | $s$ | $s = r_s$ |
| (2) | $l$ | $l = \mathrm{sign}\!\left((y-y_r)\cos\theta_r - (x-x_r)\sin\theta_r\right)\sqrt{(x-x_r)^2+(y-y_r)^2}$ |
| (3) | $l'$ | $l' = (1-\kappa_r l)\tan\Delta\theta$ |
| (4) | $\dot{s}$ | $\dot{s} = \dfrac{v\cos\Delta\theta}{1-\kappa_r l}$ |
| (5) | $\dot{l}$ | $\dot{l} = v\sin\Delta\theta$ |
| (6) | $l''$ | $l'' = -(\kappa'_r l + \kappa_r l')\tan\Delta\theta + \dfrac{1-\kappa_r l}{\cos^2\Delta\theta}\!\left(\kappa\,\dfrac{1-\kappa_r l}{\cos\Delta\theta} - \kappa_r\right)$ |
| (7) | $\ddot{s}$ | $\ddot{s} = \dfrac{a\cos\Delta\theta - \dot{s}^2\!\left[l'\!\left(\kappa\,\dfrac{1-\kappa_r l}{\cos\Delta\theta} - \kappa_r\right) - (\kappa'_r l + \kappa_r l')\right]}{1-\kappa_r l}$ |
| (8) | $\ddot{l}$ | $\ddot{l} = l''\,\dot{s}^2 + l'\,\ddot{s}$（精确）；$\ddot{l} \approx a\sin\Delta\theta$（近似） |

其中 $\Delta\theta = \mathrm{NormalizeAngle}(\theta - \theta_r)$。

**求解顺序**：$s,l$ → $l'$ → $\dot{s},\dot{l}$ → $l''$ → $\ddot{s}$ → $\ddot{l}$。

---

**关键提示**：
- 所有公式中 $A \triangleq 1-\kappa_r l$，当 $A \approx 0$ 时坐标变换退化，需做数值保护。
- 关键几何关系来自 $\mathbf{p}'(s) = A\,\mathbf{t} + l'\,\mathbf{n}$，由此推导出航向差、速度分解与曲率公式。