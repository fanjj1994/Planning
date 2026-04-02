# 仿真地图创建（PNC Map Creation）

## 1. 概述

`pnc_map_creator` 模块负责为规划系统生成仿真用的道路地图，即 **PNC Map**（Planning & Control Map）。PNC Map 是一张极简的双车道道路模型，整个地图仅由三条折线组成：

| 字段 | 含义 | RViz 显示颜色 |
|------|------|--------------|
| `pncMap.midline` | 两车道之间的车道分隔线（中心虚线） | 黄色 |
| `pncMap.left_boundary` | 道路左侧外边界 | 白色 |
| `pncMap.right_boundary` | 道路右侧外边界 | 白色 |

目前支持两种道路形状：**直道（Straight）** 和 **S 形弯道（STurn）**，通过配置文件中的 `type` 字段切换。

---

## 2. 配置参数

配置文件位于 `src/planning_core/config/` 目录下：

- `scenario_config.yaml`：场景选择配置，通过 `type` 字段切换场景（详见 [ProjectConfigIntroduction.md](../Algorithm/ProjectConfigIntroduction.md)）
- 静态场景（type=0/1）：加载 `planning_static_tps_config.yaml`
- 动态场景（type=3）：加载 `planning_dynamic_tps_config.yaml`

`pnc_map` 节点的相关字段如下（以静态配置为例）：

```yaml
pnc_map:
  frame: "map"        # 坐标系名称
  type: 1             # 地图类型枚举：0=直道，1=S形弯道
  road_length: 250.0  # 道路总长度（单位：m）
  lane_width: 4.0     # 单车道宽度（单位：m）
  segment_len: 0.5    # 相邻地图点的间距（单位：m）
  speed_limit: 1.0    # 限速（预留字段，暂未启用）
```

对应的 C++ 结构体定义（`config_reader.h`）：

```cpp
struct PNCMapStruct {
    std_string frame_;       // 坐标系
    uint8      type_;        // 地图类型
    float32    road_length_; // 道路长度
    float32    lane_width_;  // 车道宽度
    float32    segment_len_; // 采样间距
    float32    speed_limit_; // 限速（预留）
};
```

---

## 3. 道路横截面布局

无论是直道还是弯道，在任意一个采样点处，道路的横截面布局均遵循以下规则（以正方向行驶时的 y 轴横向坐标为例）：

```
     y 轴（横向）
      ^
      |
 y=+6 │ ══════════════════  ← left_boundary（左外边界）
      │     [ 左 车 道 ]
 y=+2 │ - - - - - - - - - -  ← midline（车道分隔线，黄色虚线）
      │     [ 右 车 道 ]（本车道）
 y=-2 │ ══════════════════  ← right_boundary（右外边界）
      │
      +──────────────────────> x 轴（纵向）
```

**数值推导（lane_width = 4.0 m）：**

-  **车道分隔线**的起始坐标pCenter 设置为 `(x=-3.0, y=lane_width/2=2.0)`，即自车初始的位置点；
- 每个采样点：
  - `pLeft.y  = pCenter.y + lane_width = 2.0 + 4.0 = +6.0`（左外边界）
  - `pRight.y = pCenter.y - lane_width = 2.0 - 4.0 = -2.0`（右外边界）
- 道路总宽 = `2 × lane_width = 8.0 m`（双车道，每条 4.0 m）；
- 右侧车道（本车道）行驶中心线位于 `y = 0.0`，即后续全局路径规划的目标轨迹。

> **注意：** `midline` 字段在 PNC Map 中实际对应的是**双车道之间的分隔线**，而非道路的几何中心。全局路径规划器会利用 `midline` 与 `right_boundary` 的平均值来得到右侧车道的行驶中心线，详见 [NormalGlobalPath.md](NormalGlobalPath.md)。

---

## 4. 直道地图（PNCMapCreatorStraight）

**类文件：** [pnc_map_creator_straight](../../src/planning_core/src/pnc_map_creator/pnc_map_straight/pnc_map_creator_straight.cpp)

### 4.1 初始化

```cpp
pCenter.x = -3.0;
pCenter.y = pncMapConfig->getPNCMap().lane_width_ / 2.0;  // = 2.0
pCenter.z = 0.0;
lengthStep = pncMapConfig->getPNCMap().segment_len_;        // = 0.5 m
```

### 4.2 核心绘制函数 `drawStraightX`

```cpp
void PNCMapCreatorStraight::drawStraightX(const float64& length,
                                           const float64& plusFlag,
                                           const float64& ratio)
{
    float64 lenRuler{ 0.0 };
    while (lenRuler < length)
    {
        pLeft.x  = pCenter.x;
        pLeft.y  = pCenter.y + lane_width;   // +4.0 → y=+6.0
        pRight.x = pCenter.x;
        pRight.y = pCenter.y - lane_width;   // -4.0 → y=-2.0

        pncMap.midline.points.emplace_back(pCenter);
        pncMap.left_boundary.points.emplace_back(pLeft);
        pncMap.right_boundary.points.emplace_back(pRight);

        lenRuler   += lengthStep * ratio;
        pCenter.x  += lengthStep * plusFlag; // 沿 +x 方向步进
    }
}
```

**调用方式：**

```cpp
// 以步长 0.5m 绘制 road_length=250m 的直线段
drawStraightX(pncMap.road_length, /* plusFlag= */ 1.0);
```

**结果：** 共生成约 $250 / 0.5 = 500$ 个采样点，整条道路平行于 x 轴延伸。

### 4.3 俯视示意图

```
 y
 ^
+6│══════════════════════════════════════════════  left_boundary
  │
+2│─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─  midline（车道分隔线）
  │
-2│══════════════════════════════════════════════  right_boundary
  +────────────────────────────────────────────────> x
   start(-3,·)                            end(247,·)
```

---

## 5. S 形弯道地图（PNCMapCreatorSTurn）

**类文件：** [pnc_map_creator_sturn](../../src/planning_core/src/pnc_map_creator/pnc_map_sturn/pnc_map_creator_sturn.cpp)

### 5.1 初始化

在构造函数中，除了与直道相同的 `pCenter`、`lengthStep` 设置外，还初始化了弧线角度步长：

```cpp
thetaStep = 0.01;  // 每步转过的角度增量（弧度）
```

`thetaCurrent` 初始值为 `0.0`，表示当前行进方向角（相对于 x 轴）。

### 5.2 地图生成流程

```cpp
base_msgs::msg::PNCMap PNCMapCreatorSTurn::createPNCMap()
{
    drawStraightX(road_length / 3.0, 1.0);   // ① 直线段
    drawArc(M_PI / 2.0,  1.0);               // ② 顺时针弧（左转 90°）
    drawArc(M_PI / 2.0, -1.0);               // ③ 逆时针弧（右转 90°）
    ...
}
```

三段合成完整的 S 形弯道：

| 段 | 类型 | 长度 / 角度 | 转向效果 |
|----|------|------------|---------|
| ① 直线段 | 直道 | $250/3 \approx 83.3$ m | 沿 +x 方向平直行驶 |
| ② 顺时针弧 | 圆弧 | 90°，弧长 $\approx 78.5$ m | 向左（+y 方向）弯曲 |
| ③ 逆时针弧 | 圆弧 | 90°，弧长 $\approx 78.5$ m | 向右弯曲，恢复 +x 方向 |

### 5.3 弧线绘制函数 `drawArc`

弧线通过**沿当前行进方向逐步前进并旋转方向角**的方式生成：

```cpp
void PNCMapCreatorSTurn::drawArc(const float64& angle,
                                  const float64& plusFlag,
                                  const float64& ratio)
{
    float64 thetaRuler{ 0.0 };
    while (thetaRuler < angle)
    {
        // 计算左右边界点（垂直于当前方向，向两侧偏移 lane_width）
        pLeft.x  = pCenter.x - lane_width * sin(thetaCurrent);
        pLeft.y  = pCenter.y + lane_width * cos(thetaCurrent);
        pRight.x = pCenter.x + lane_width * sin(thetaCurrent);
        pRight.y = pCenter.y - lane_width * cos(thetaCurrent);

        // 记录当前点
        pncMap.midline.points.emplace_back(pCenter);
        pncMap.left_boundary.points.emplace_back(pLeft);
        pncMap.right_boundary.points.emplace_back(pRight);

        // 沿当前方向前进一步
        pCenter.x += lengthStep * cos(thetaCurrent);
        pCenter.y += lengthStep * sin(thetaCurrent);

        // 更新角度计数器和当前方向角
        thetaRuler   += thetaStep * ratio;
        thetaCurrent += thetaStep * plusFlag * ratio;
    }
}
```

**几何原理：**

在方向角为 $\theta$ 时：
- 前进方向向量：$(\cos\theta, \sin\theta)$
- 左侧垂线方向（旋转 +90°）：$(-\sin\theta, \cos\theta)$
- 右侧垂线方向（旋转 -90°）：$(\sin\theta, -\cos\theta)$

因此：

$$
p_\text{left}  = p_\text{center} + \text{lane\_width} \cdot (-\sin\theta,\ \cos\theta)
$$

$$
p_\text{right} = p_\text{center} + \text{lane\_width} \cdot (\sin\theta,\ -\cos\theta)
$$

弧线的等效曲率半径约为：

$$
R \approx \frac{\text{lengthStep}}{\text{thetaStep}} = \frac{0.5}{0.01} = 50 \text{ m}
$$

### 5.4 俯视示意图

```
 y
 ^
  │                      ╭────────────────────────
  │                     ╱
  │          ╭──────────╯
  │         ╱
  │─────────╯
  +─────────────────────────────────────────────> x
  start(-3,·)
  ←— ①直线段 —→←── ②左转90°弧 ──→←── ③右转90°弧 ──→
```

> S 形弯道的总长度约为 $83.3 + 78.5 + 78.5 \approx 240$ m，终点行进方向仍为 +x，但横向位置（y 值）相较起点有所偏移。

---

## 6. ROS 2 通信架构

`PNCMapServer` 节点（`pnc_map_server.cpp`）封装了地图创建逻辑，提供以下接口：

```
┌──────────────────────────────────────────┐
│           pnc_map_server_node            │
│                                          │
│  服务（Service）:                         │
│    /pnc_map_server                       │
│    类型: base_msgs/srv/PNCMapService      │
│    请求字段: map_type (0=Straight,        │
│             1=STurn)                     │
│                                          │
│  发布（Publish）:                         │
│    /pnc_map                              │
│       类型: base_msgs/msg/PNCMap          │
│       用途: 供规划模块订阅使用             │
│                                          │
│    /pnc_map_marker_array                 │
│       类型: visualization_msgs/          │
│             msg/MarkerArray              │
│       用途: RViz 可视化显示               │
└──────────────────────────────────────────┘
```

**工作流程：**

1. 规划主流程发起 `/pnc_map_server` 服务请求，携带 `map_type`；
2. `PNCMapServer` 根据 `map_type` 实例化 `PNCMapCreatorStraight` 或 `PNCMapCreatorSTurn`；
3. 调用 `createPNCMap()` 完成地图绘制；
4. 响应服务请求，同时向 `/pnc_map` 和 `/pnc_map_marker_array` 发布消息。

---

## 7. 类继承关系

```
PNCMapCreatorBase（抽象基类）
├── PNCMapCreatorStraight  （直道，type=0）
└── PNCMapCreatorSTurn     （S形弯道，type=1）
```

`PNCMapCreatorBase` 定义了公共接口 `createPNCMap()` 以及共享成员 `pncMap`、`pCenter`、`pLeft`、`pRight`、`thetaCurrent`、`lengthStep`、`thetaStep`。子类通过 `ConfigReader` 读取配置，实现具体的绘制逻辑。
