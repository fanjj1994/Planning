
# Reference Line Smoothing

本文介绍本项目中参考线平滑（Reference Line Smoothing）的方法。该方法基于 Apollo 的参考线平滑思路，构建包含**平滑代价、均匀代价和几何相似代价**的目标函数，将问题转化为带约束的二次规划（QP）问题，并使用 OSQP 求解器求解。

对应代码文件：
- [reference line smoother header file](../src/planning_core/src/reference_line/reference_line_smoother.h)
- [reference line smoother cpp file](../src/planning_core/src/reference_line/reference_line_smoother.cpp)

---

## 1. Introduction

### 1.1 为什么需要参考线平滑？

在 EM-Planner 的规划流程中，参考线（Reference Line）是从全局路径中截取的一段有限长度的路径，是后续局部规划（Local Path、Local Speeds）以及 Frenet 坐标变换的基础。然而，原始参考线通常具备以下问题：

1. **离散跳变**：全局路径由离散点组成，相邻点之间的航向和曲率可能存在突变，导致 Frenet 坐标变换中 $\kappa_r$、$\mathrm{d}\kappa_r / \mathrm{d}s$ 不连续，影响后续规划质量。
2. **噪声干扰**：路径点可能来自地图数据或上游模块的计算结果，包含微小的定位噪声，这些噪声会在求曲率（二阶导数）时被放大。
3. **曲率不平滑**：即使路径本身看起来"平滑"，其曲率分布也可能不均匀，导致车辆转向不平顺或横向加速度突变。

因此，在使用参考线前需要对其进行平滑处理，使其同时满足：
- **几何相似**：平滑后的参考线不能偏离原始路径太远，否则会导致车辆偏离道路。
- **平滑性**：曲率变化尽量连续，减少二阶导数的突变。
- **均匀性**：相邻点之间的间距变化尽量均匀，避免局部稀疏或密集。

### 1.2 平滑方法概述

本项目采用**基于二次规划（QP）的离散点平滑方法**，其核心思想是：

> 将参考线上每个点的坐标 $(x_i, y_i)$ 作为优化变量，构建一个以"平滑性 + 均匀性 + 几何相似性"为目标的二次代价函数，加上位置偏移的上下界约束，形成标准 QP 问题后使用 OSQP 求解器求解。

---

## 2. 二次规划（QP）问题

### 2.1 什么是二次规划？

**二次规划（Quadratic Programming, QP）** 是一类目标函数为二次、约束条件为线性的优化问题。其标准形式为：

$$
\min_{\mathbf{x}} \quad \frac{1}{2}\,\mathbf{x}^\top P\,\mathbf{x} + \mathbf{q}^\top \mathbf{x}
$$

$$
\text{s.t.} \quad \mathbf{l} \le A\,\mathbf{x} \le \mathbf{u}
$$

其中：
- $\mathbf{x} \in \mathbb{R}^n$ 为优化变量
- $P \in \mathbb{R}^{n \times n}$ 为对称半正定矩阵（Hessian 矩阵）
- $\mathbf{q} \in \mathbb{R}^n$ 为一次项系数（梯度向量）
- $A \in \mathbb{R}^{m \times n}$ 为线性约束矩阵
- $\mathbf{l},\, \mathbf{u} \in \mathbb{R}^m$ 分别为约束的下界和上界

当 $P$ 半正定时，QP 是一个**凸优化**问题，任意局部最优解即为全局最优解。

### 2.2 OSQP 求解器

本项目使用 [OSQP](https://osqp.org/)（Operator Splitting Quadratic Program）求解器，它是一个高效的稀疏 QP 求解器，在自动驾驶和机器人规划中被广泛使用。代码中通过 [OsqpEigen](https://github.com/robotology/osqp-eigen) 库的 C++ 接口调用。

### 2.3 海塞矩阵（Hessian Matrix）

**海塞矩阵（Hessian Matrix）** 是标量函数对其多维自变量的二阶偏导数矩阵。对于函数 $f(\mathbf{x})$，$\mathbf{x} = [x_1, x_2, \ldots, x_n]^\top$，海塞矩阵定义为：

$$
H = \nabla^2 f = \begin{bmatrix}
\dfrac{\partial^2 f}{\partial x_1^2} & \dfrac{\partial^2 f}{\partial x_1 \partial x_2} & \cdots & \dfrac{\partial^2 f}{\partial x_1 \partial x_n} \\[6pt]
\dfrac{\partial^2 f}{\partial x_2 \partial x_1} & \dfrac{\partial^2 f}{\partial x_2^2} & \cdots & \dfrac{\partial^2 f}{\partial x_2 \partial x_n} \\[6pt]
\vdots & \vdots & \ddots & \vdots \\[6pt]
\dfrac{\partial^2 f}{\partial x_n \partial x_1} & \dfrac{\partial^2 f}{\partial x_n \partial x_2} & \cdots & \dfrac{\partial^2 f}{\partial x_n^2}
\end{bmatrix}
$$

**海塞矩阵与 QP 的关系**：对于二次型 $f(\mathbf{x}) = \mathbf{x}^\top P\,\mathbf{x}$（$P$ 对称），其海塞矩阵为 $H = 2P$，即 $P = \frac{1}{2} H$。

因此，后续在推导代价函数的 $P$ 矩阵时，可以用两种方法互相验证：

- **方法一（矩阵乘法）**：将代价函数写成 $\mathbf{x}^\top (A\,A^\top)\,\mathbf{x}$ 的形式，直接计算 $P = A\,A^\top$。
- **方法二（海塞矩阵）**：对代价函数求所有二阶偏导数，得到海塞矩阵 $H$，然后 $P = \frac{1}{2}\,H$。

---

## 3. 参考线平滑的 QP 建模与推导

### 3.1 优化变量定义

设参考线包含 $n$ 个离散点，每个点为二维坐标 $\mathbf{p}_i = (x_i,\, y_i)^\top$，$i = 0, 1, \ldots, n-1$。

将所有点坐标堆叠为一个 $2n \times 1$ 的列向量：

$$
\mathbf{X} = [x_0,\, y_0,\, x_1,\, y_1,\, \ldots,\, x_{n-1},\, y_{n-1}]^\top \in \mathbb{R}^{2n}
$$

原始参考线对应的向量记为 $\mathbf{X}_{\mathrm{ref}}$。

### 3.2 三个代价函数

下面以**三个点**为例，分别推导平滑、均匀和几何相似三个代价函数的矩阵形式，再推广到 $n$ 个点的一般情形。

#### 3.2.1 平滑代价（Smoothness Cost）

##### 三点情形

设路径上依次有三个点 $p_0(x_0, y_0)$、$p_1(x_1, y_1)$、$p_2(x_2, y_2)$。

**核心思想**：对于中间点 $p_1$，定义其**离散二阶差分向量（second-order finite difference）**：

$$
\Delta^2 p_1 \;=\; p_0 - 2\,p_1 + p_2 \;=\; (x_0 + x_2 - 2x_1,\;\; y_0 + y_2 - 2y_1)
$$

平滑代价即该向量模的平方：

$$
\mathrm{Cost}_{S,3} = \|\Delta^2 p_1\|^2 = (x_0 + x_2 - 2x_1)^2 + (y_0 + y_2 - 2y_1)^2
$$

**为什么二阶差分能度量平滑性？** 从三个角度理解：

1) **几何直觉——中点偏移**

$\Delta^2 p_1$ 可以改写为 $2\bigl(\tfrac{p_0 + p_2}{2} - p_1\bigr)$，即 $p_1$ 到其两侧邻点中点的偏移向量（乘以 2）：

```text
p0 ---------- M ---------- p2       M = (p0 + p2) / 2
              |
              | ← Δ²p₁/2 (中点偏移)
              |
              p1

**矩阵表示**：令

$$
\mathbf{f}_S = \begin{bmatrix} x_0 + x_2 - 2x_1 \\ y_0 + y_2 - 2y_1 \end{bmatrix}^\top
\quad (1 \times 2 \text{ 行向量})
$$

写成 $\mathbf{f}_S = \mathbf{X}^\top A_S$ 的形式，其中 $\mathbf{X} = [x_0, y_0, x_1, y_1, x_2, y_2]^\top \in \mathbb{R}^{6}$，则 $A_S$ 为 $6 \times 2$ 矩阵：

$$
A_S = \begin{bmatrix}
 1 &  0 \\
 0 &  1 \\
-2 &  0 \\
 0 & -2 \\
 1 &  0 \\
 0 &  1
\end{bmatrix}
\quad (6 \times 2)
$$

验证：$\mathbf{X}^\top A_S = [x_0 \!\cdot\! 1 + x_1 \!\cdot\! (-2) + x_2 \!\cdot\! 1,\; y_0 \!\cdot\! 1 + y_1 \!\cdot\! (-2) + y_2 \!\cdot\! 1] = \mathbf{f}_S$ ✓

代价函数可以写成二次型：

$$
\mathrm{Cost}_{S,3} = \mathbf{f}_S \, \mathbf{f}_S^\top
= (\mathbf{X}^\top A_S)(\mathbf{X}^\top A_S)^\top
= \mathbf{X}^\top \underbrace{A_S \, A_S^\top}_{P_S} \mathbf{X}
$$

$$
\boxed{\mathrm{Cost}_{S,3} = \mathbf{X}^\top P_S \, \mathbf{X}, \quad P_S = A_S \, A_S^\top \;\in \mathbb{R}^{6 \times 6}}
$$

**计算 $P_S$**：注意 $A_S$ 的每一行 $\mathbf{r}_i$ 都是 $1 \times 2$ 行向量，$P_S$ 的第 $(i,j)$ 元素就是 $\mathbf{r}_i \cdot \mathbf{r}_j$（行向量的内积）。直接展开：

$$
P_S = A_S \, A_S^\top = \begin{bmatrix}
 1 &  0 & -2 &  0 &  1 &  0 \\
 0 &  1 &  0 & -2 &  0 &  1 \\
-2 &  0 &  4 &  0 & -2 &  0 \\
 0 & -2 &  0 &  4 &  0 & -2 \\
 1 &  0 & -2 &  0 &  1 &  0 \\
 0 &  1 &  0 & -2 &  0 &  1
\end{bmatrix}
\quad (6 \times 6)
$$

将 $P_S$ 按 $2 \times 2$ 分块，记 $I_2 = \bigl[\begin{smallmatrix}1&0\\0&1\end{smallmatrix}\bigr]$，可以清晰地看出块结构：

$$
P_S = \begin{bmatrix}
 I_2  & -2I_2 &  I_2 \\
-2I_2 &  4I_2 & -2I_2 \\
 I_2  & -2I_2 &  I_2
\end{bmatrix}
\quad (3 \times 3 \text{ 个 } 2\!\times\!2 \text{ 块})
$$

提取每个块中 $I_2$ 前面的标量系数，得到 **标量系数矩阵**（后面会多次使用这种表示）：

$$
\alpha_S^{(3)} = \begin{bmatrix} 1 & -2 & 1 \\ -2 & 4 & -2 \\ 1 & -2 & 1 \end{bmatrix}
\quad (3 \times 3)
$$

**海塞矩阵验证**（见 2.3 节）：对 $\mathrm{Cost}_{S,3}$ 求二阶偏导即可验证。例如：

$$
\frac{\partial^2 \mathrm{Cost}_{S,3}}{\partial x_0^2} = 2, \quad
\frac{\partial^2 \mathrm{Cost}_{S,3}}{\partial x_0 \, \partial x_1} = -4, \quad
\frac{\partial^2 \mathrm{Cost}_{S,3}}{\partial x_1^2} = 8
$$

于是海塞矩阵满足 $H_S = 2\,P_S$，即 $P_S = \frac{1}{2} H_S$，与矩阵乘法的结果一致。 ✓

##### 推广到 $n$ 个点

对于 $n$ 个点 $p_0, p_1, \ldots, p_{n-1}$，共有 $n - 2$ 个内部点 $(i = 1, 2, \ldots, n\!-\!2)$，每个内部点 $p_i$ 产生一个平滑条件：

$$
\mathbf{v}_i = p_{i-1} - 2\,p_i + p_{i+1}
$$

对应 $\mathbf{f}_{S,i} = \mathbf{X}^\top A_{S,i}$，其中 $A_{S,i} \in \mathbb{R}^{2n \times 2}$ 的结构与三点情形的 $A_S$ 相同，只是嵌入到更大的向量空间中（仅在第 $i\!-\!1,\, i,\, i\!+\!1$ 个点对应的行有非零值，其余全零）。

总代价为所有内部点的贡献之和：

$$
\mathrm{Cost}_S = \sum_{i=1}^{n-2} \mathbf{X}^\top (A_{S,i}\, A_{S,i}^\top) \, \mathbf{X} = \mathbf{X}^\top P_S \, \mathbf{X}
$$

$$
P_S = \sum_{i=1}^{n-2} A_{S,i}\, A_{S,i}^\top \;\in \mathbb{R}^{2n \times 2n}
$$

由于每个 $A_{S,i} A_{S,i}^\top$ 只在以第 $i$ 个点为中心的 $3 \times 3$ 块中贡献非零值，所有条件叠加后形成**五对角带状**的块矩阵。其标量系数矩阵（$n \times n$，各元素为对应 $I_2$ 块的系数）为：

$$
\alpha_S = \begin{bmatrix}
1 & -2 & 1 & & & \\
-2 & 5 & -4 & 1 & & \\
1 & -4 & 6 & -4 & 1 & \\
  & \ddots & \ddots & \ddots & \ddots & \ddots \\
  & & 1 & -4 & 5 & -2 \\
  & & & 1 & -2 & 1
\end{bmatrix}
\quad (n \times n, \; n \ge 4)
$$

各对角线上的系数规律如下：

| 位置 | 主对角线 | 次对角线 $(\pm 1)$ | 次次对角线 $(\pm 2)$ |
|:---|:---:|:---:|:---:|
| 首 / 末行 $(i=0,\,n\!-\!1)$ | $1$ | $-2$ | $1$ |
| 第 $2$ / 倒数第 $2$ 行 $(i=1,\,n\!-\!2)$ | $5$ | $-4$ | $1$ |
| 中间行 $(2 \le i \le n\!-\!3)$ | $6$ | $-4$ | $1$ |

> **特例 $n = 3$**：只有 1 个内部点，中间行对角值为 $4$（而非 $6$），退化回三点情形的 $\alpha_S^{(3)}$。

#### 3.2.2 均匀代价（Uniformity Cost）

##### 三点情形

均匀代价衡量的是相邻点之间的距离——最小化所有线段长度的平方和，使路径点分布均匀、紧凑。

三个点有 2 条线段：$p_0 p_1$ 和 $p_1 p_2$。定义：

$$
\mathbf{d}_1 = p_1 - p_0 = (x_1 - x_0,\; y_1 - y_0), \quad
\mathbf{d}_2 = p_2 - p_1 = (x_2 - x_1,\; y_2 - y_1)
$$

$$
\mathrm{Cost}_{L,3} = \|\mathbf{d}_1\|^2 + \|\mathbf{d}_2\|^2
$$

采用与平滑代价相同的推导方法。对于每条线段 $j$，写出 $\mathbf{f}_{L,j} = \mathbf{X}^\top A_{L,j}$：

- 线段 1：$\mathbf{f}_{L,1} = [x_1\!-\!x_0,\; y_1\!-\!y_0]$

$$
A_{L,1} = \begin{bmatrix}
-1 &  0 \\
 0 & -1 \\
 1 &  0 \\
 0 &  1 \\
 0 &  0 \\
 0 &  0
\end{bmatrix}
\quad (6 \times 2)
$$

- 线段 2：$\mathbf{f}_{L,2} = [x_2\!-\!x_1,\; y_2\!-\!y_1]$

$$
A_{L,2} = \begin{bmatrix}
 0 &  0 \\
 0 &  0 \\
-1 &  0 \\
 0 & -1 \\
 1 &  0 \\
 0 &  1
\end{bmatrix}
\quad (6 \times 2)
$$

各线段代价为 $\|\mathbf{d}_j\|^2 = \mathbf{X}^\top (A_{L,j}\, A_{L,j}^\top) \, \mathbf{X}$，总代价：

$$
\mathrm{Cost}_{L,3} = \mathbf{X}^\top \underbrace{\bigl(A_{L,1}\,A_{L,1}^\top + A_{L,2}\,A_{L,2}^\top\bigr)}_{P_L} \mathbf{X}
$$

计算得 $P_L \in \mathbb{R}^{6 \times 6}$ 的 $2 \times 2$ 分块形式为：

$$
P_L = \begin{bmatrix}
 I_2  & -I_2 &  0 \\
-I_2  &  2I_2 & -I_2 \\
 0    & -I_2 &  I_2
\end{bmatrix}
$$

对应标量系数矩阵：

$$
\alpha_L^{(3)} = \begin{bmatrix} 1 & -1 & 0 \\ -1 & 2 & -1 \\ 0 & -1 & 1 \end{bmatrix}
\quad (3 \times 3)
$$

##### 推广到 $n$ 个点

$n$ 个点有 $n - 1$ 条线段。第 $j$ 条线段 $(j = 0, 1, \ldots, n\!-\!2)$ 对应 $A_{L,j} \in \mathbb{R}^{2n \times 2}$，其结构与三点情形相同，仅在第 $j$ 和 $j+1$ 个点对应的行有非零值。

$$
P_L = \sum_{j=0}^{n-2} A_{L,j}\, A_{L,j}^\top \;\in \mathbb{R}^{2n \times 2n}
$$

$P_L$ 为**三对角**块矩阵，标量系数矩阵为：

$$
\alpha_L = \begin{bmatrix}
 1 & -1 &     &        \\
-1 &  2 & -1  &        \\
   & -1 &  2  & \ddots \\
   &    & \ddots & \ddots & -1 \\
   &    &     & -1     &  1
\end{bmatrix}
\quad (n \times n)
$$

即：主对角线首末为 $1$，其余为 $2$；次对角线全部为 $-1$。

#### 3.2.3 几何相似代价（Geometric Similarity Cost）

##### 三点情形

几何相似代价约束优化后的点不偏离原始参考线点太远：

$$
\mathrm{Cost}_{D,3} = \sum_{i=0}^{2} \|p_i - p_i^{\mathrm{ref}}\|^2 = \|\mathbf{X} - \mathbf{X}_{\mathrm{ref}}\|^2
$$

展开：

$$
\mathrm{Cost}_{D,3} = (\mathbf{X} - \mathbf{X}_{\mathrm{ref}})^\top (\mathbf{X} - \mathbf{X}_{\mathrm{ref}})
= \mathbf{X}^\top I_6\, \mathbf{X} - 2\,\mathbf{X}_{\mathrm{ref}}^\top \mathbf{X} + \underbrace{\mathbf{X}_{\mathrm{ref}}^\top \mathbf{X}_{\mathrm{ref}}}_{\text{常数}}
$$

其中 $I_6$ 为 $6 \times 6$ 单位矩阵。忽略常数项：

$$
\mathrm{Cost}_{D,3} = \mathbf{X}^\top \underbrace{I_6}_{P_D}\, \mathbf{X} - 2\,\mathbf{X}_{\mathrm{ref}}^\top \mathbf{X}
$$

注意：几何相似代价中除了二次项 $\mathbf{X}^\top P_D\, \mathbf{X}$ 外，还包含一个**一次项** $-2\,\mathbf{X}_{\mathrm{ref}}^\top \mathbf{X}$，它将在后续转化为 QP 的 $\mathbf{q}$ 向量。

标量系数矩阵即单位矩阵：$\alpha_D^{(3)} = I_3$。

##### 推广到 $n$ 个点

$$
\mathrm{Cost}_D = \mathbf{X}^\top I_{2n}\, \mathbf{X} - 2\,\mathbf{X}_{\mathrm{ref}}^\top \mathbf{X} + \text{const}
$$

$$
P_D = I_{2n}, \quad \alpha_D = I_n
$$

### 3.3 总代价函数

将三个代价按权重加权求和：

$$
f(\mathbf{X}) = w_1 \cdot \mathrm{Cost}_S + w_2 \cdot \mathrm{Cost}_L + w_3 \cdot \mathrm{Cost}_D
$$

$$
= \mathbf{X}^\top \bigl(w_1\, P_S + w_2\, P_L + w_3\, I_{2n}\bigr) \mathbf{X} - 2\,w_3\, \mathbf{X}_{\mathrm{ref}}^\top \mathbf{X} + \text{const}
$$

### 3.4 转化为 QP 标准形式

QP 标准形式为 $\min\;\frac{1}{2}\,\mathbf{X}^\top P\,\mathbf{X} + \mathbf{q}^\top \mathbf{X}$。将总代价函数配凑为该形式：

$$
f(\mathbf{X}) = \frac{1}{2}\,\mathbf{X}^\top \underbrace{2(w_1\, P_S + w_2\, P_L + w_3\, I_{2n})}_{P}\,\mathbf{X} + \underbrace{(-2\,w_3\, \mathbf{X}_{\mathrm{ref}})^\top}_{\mathbf{q}^\top}\,\mathbf{X} + \text{const}
$$

得到：

$$
\boxed{P = 2\bigl(w_1\, P_S + w_2\, P_L + w_3\, I_{2n}\bigr) \;\in \mathbb{R}^{2n \times 2n}}
$$

$$
\boxed{\mathbf{q} = -2\,w_3\, \mathbf{X}_{\mathrm{ref}} \;\in \mathbb{R}^{2n}}
$$

> **代码注意**：当前实现中 $w_3 = 1.0$，因此代码中直接写为 `q = -2.0 * X`。

### 3.5 $P$ 矩阵的分块结构与代码对应

$P$ 矩阵按 $2 \times 2$ 分块后，第 $(i,j)$ 号块为：

$$
P_{\mathrm{block}}(i,j) = 2\,w_1 \cdot \alpha_S(i,j) \cdot I_2 + 2\,w_2 \cdot \alpha_L(i,j) \cdot I_2 + 2\,w_3 \cdot \delta_{ij} \cdot I_2
$$

定义代码中的 $2 \times 2$ 权重矩阵：

$$
W_1 = 2\,w_1\, I_2, \quad W_2 = 2\,w_2\, I_2, \quad W_3 = 2\,w_3\, I_2
$$

则：

$$
P_{\mathrm{block}}(i,j) = \alpha_S(i,j) \cdot W_1 + \alpha_L(i,j) \cdot W_2 + \delta_{ij} \cdot W_3
$$

下表列出各位置的系数和对应的代码 block 变量（对于 $n \ge 4$）：

**对角块** $(i, i)$：

| 位置 | $\alpha_S$ | $\alpha_L$ | $\delta$ | 组合表达式 | 代码变量 |
|:---|:---:|:---:|:---:|:---|:---|
| $i = 0$ 或 $i = n\!-\!1$（首 / 末点） | $1$ | $1$ | $1$ | $W_1 + W_2 + W_3$ | `block1` |
| $i = 1$ 或 $i = n\!-\!2$（第 2 / 倒数第 2 点） | $5$ | $2$ | $1$ | $5W_1 + 2W_2 + W_3$ | `block4` |
| $2 \le i \le n\!-\!3$（中间点） | $6$ | $2$ | $1$ | $6W_1 + 2W_2 + W_3$ | `block5` |

**次对角块** $(i, i\!+\!1)$：

| 位置 | $\alpha_S$ | $\alpha_L$ | 组合表达式 | 代码变量 |
|:---|:---:|:---:|:---|:---|
| $i = 0$ 或 $i = n\!-\!2$（首 / 末端） | $-2$ | $-1$ | $-2W_1 - W_2$ | `block2` |
| $1 \le i \le n\!-\!3$（中间） | $-4$ | $-1$ | $-4W_1 - W_2$ | `block3` |

**次次对角块** $(i, i\!+\!2)$：

| 位置 | $\alpha_S$ | $\alpha_L$ | 组合表达式 | 代码变量 |
|:---|:---:|:---:|:---|:---|
| 所有有效 $i$（$0 \le i \le n\!-\!3$） | $1$ | $0$ | $W_1$ | `W1`（直接赋值） |

> **特例 $n = 3$**：只有 1 个内部点，$\alpha_S(1,1) = 4$（非 $6$），对应 `block6` $= 4W_1 + 2W_2 + W_3$。

代码中 $P$ 矩阵先填写上三角，然后通过 `P = P^T + P - diag(P)` 对称化得到完整矩阵。

### 3.6 约束条件

约束矩阵 $A = I_{2n}$（单位矩阵），所以约束退化为简单的**箱约束（Box Constraint）**：

$$
\mathbf{X}_{\mathrm{ref}} - \boldsymbol{\epsilon} \le \mathbf{X} \le \mathbf{X}_{\mathrm{ref}} + \boldsymbol{\epsilon}
$$

其中 $\boldsymbol{\epsilon}$ 为每个坐标分量的容差缓冲。代码实现中：

- 中间点：$\epsilon = 0.2\,\mathrm{m}$（`buff = 0.2`），允许各点在原始位置的 $\pm 0.2\,\mathrm{m}$ 范围内移动。
- 首末点：$\epsilon = 0$（`buff = 0.0`），即**起点和终点固定**不变，保证参考线端点与全局路径对齐。

### 3.7 代码中的权重参数

代码中三个权重定义于 `ReferenceLineSmoother` 类的成员变量：

| 权重 | 值 | 控制目标 | 含义 |
|:---|:---:|:---|:---|
| `w1` | $100.0$ | 平滑性（二阶差分） | 权重最大，强调参考线的**曲率连续**和**平滑过渡** |
| `w2` | $10.0$ | 均匀性（一阶差分） | 中等权重，约束相邻点**间距均匀**，防止局部过密或过疏 |
| `w3` | $1.0$ | 几何相似性（原始偏差） | 权重最小，允许适度偏移原始位置，但不过度偏离 |

权重的大小关系 $w_1 \gg w_2 \gg w_3$ 体现了设计优先级：**平滑性优先，均匀性其次，几何保真度最后**。

### 3.8 求解流程总览

```text
输入: 原始参考线点列 {(x_i, y_i)}，共 n 个点（n >= 3）

1. 组装优化变量向量 X_ref = [x0, y0, x1, y1, ..., x_{n-1}, y_{n-1}]^T

2. 构建 P 矩阵 (2n × 2n):
   - 利用 block1 ~ block6 按行填充上三角
   - 对称化: P = P^T + P - diag(P)

3. 构建 q 向量 (2n × 1):
   - q = -2 * X_ref

4. 构建约束:
   - A = I (单位矩阵)
   - l = X_ref - buff,  u = X_ref + buff
   - 首末点 buff = 0 (固定), 其余 buff = 0.2m

5. 调用 OSQP 求解:
   min  0.5 * X^T P X + q^T X
   s.t. l <= X <= u

6. 将解 X* 写回参考线各点坐标
```

---

## 参考文献

- Baidu Apollo, *"Reference Line Smoother"*, [https://github.com/ApolloAuto/apollo](https://github.com/ApolloAuto/apollo)
- OSQP: Operator Splitting Quadratic Program Solver, [https://osqp.org/](https://osqp.org/)
- S. Boyd & L. Vandenberghe, *Convex Optimization*, Cambridge University Press, 2004
