# 普通全局路径规划（Normal Global Path Planning）

## 1. 概述

`global_planner` 模块负责根据 PNC Map 生成全局参考路径，供后续的局部路径规划器使用。当前项目实现了一种最简单的全局路径规划策略：**NormalGlobalPlanner**。

其核心思路为：直接对 PNC Map 中的**车道分隔线（midline）** 和**右侧边界（right_boundary）** 逐点取均值，得到本车道（右侧车道）的行驶中心线，作为全局路径返回。

---

## 2. PNC Map 到全局路径的几何关系

PNC Map 中三条折线的几何含义如下（以直道横截面为例）：

```
     y 轴（横向）
      ^
      |
 y=+6 │ ══════════════════  ← left_boundary（左外边界）
      │     [ 左 车 道 ]
 y=+2 │ - - - - - - - - -  ← midline（车道分隔线）
      │     [ 右 车 道 ]（本车道）
 y= 0 │ ·  ·  ·  ·  ·  ·  ← Global Path ★（midline 与 right_boundary 的均值）
      │
 y=-2 │ ══════════════════  ← right_boundary（右外边界）
      |
      +──────────────────────> x 轴（纵向）
```

全局路径点恰好落在右侧车道的几何中心，即：

$$
y_\text{global} = \frac{y_\text{midline} + y_\text{right\_boundary}}{2} = \frac{2.0 + (-2.0)}{2} = 0.0 \text{ m}
$$

---

## 3. 算法实现

### 3.1 类层次结构

```
GlobalPlannerBase（抽象基类）
├── GlobalPlannerNormal   （普通规划器，type=0）
└── GlobalPlannerAStar    （A* 规划器，type=1，待实现）
```

`GlobalPlannerBase` 定义了纯虚接口 `searchGlobalPath()`，子类必须实现。

### 3.2 `searchGlobalPath` 核心代码

**文件：** `src/global_planner/global_planner_normal/global_planner_normal.cpp`

```cpp
nav_msgs::msg::Path GlobalPlannerNormal::searchGlobalPath(
    const base_msgs::msg::PNCMap& pncMap)
{
    globalPath.header.frame_id = pncMap.header.frame_id;
    globalPath.header.stamp    = rclcpp::Clock().now();
    globalPath.poses.clear();

    geometry_msgs::msg::PoseStamped p_tmp;
    p_tmp.header = pncMap.header;
    // 姿态方向全部置零（恒等四元数分量均为 0）
    p_tmp.pose.orientation = {0.0, 0.0, 0.0, 0.0};

    const uint32 midLineSize = pncMap.midline.points.size();
    for (uint32 i = 0U; i < midLineSize; i++)
    {
        // 逐点取 midline 与 right_boundary 的均值
        p_tmp.pose.position.x =
            (pncMap.midline.points[i].x + pncMap.right_boundary.points[i].x) / 2.0;
        p_tmp.pose.position.y =
            (pncMap.midline.points[i].y + pncMap.right_boundary.points[i].y) / 2.0;
        globalPath.poses.emplace_back(p_tmp);
    }

    return globalPath;
}
```

### 3.3 算法说明

**输入：** `base_msgs::msg::PNCMap`，包含等长的 `midline.points` 和 `right_boundary.points` 点集（长度相同，逐点对应）。

**输出：** `nav_msgs::msg::Path`，每个 `PoseStamped` 的位置为对应索引点的**两线段中点**，方向四元数全为零。

**逐点计算公式：**

$$
x_i = \frac{x_{\text{midline},i} + x_{\text{right\_boundary},i}}{2}
$$

$$
y_i = \frac{y_{\text{midline},i} + y_{\text{right\_boundary},i}}{2}
$$

---

## 4. 各场景下的全局路径数值示例

### 4.1 直道场景（type=0）

直道绘制中，`midline` 与 `right_boundary` 的 x 坐标始终相同（均等于 `pCenter.x`），因此：

$$
x_{\text{global},i} = \frac{x_{\text{midline},i} + x_{\text{midline},i}}{2} = x_{\text{midline},i}
$$

$$
y_{\text{global},i} = \frac{(+2.0) + (-2.0)}{2} = 0.0 \text{ m（常量）}
$$

全局路径为平行于 x 轴的直线，y 恒为 0，即右侧车道中心：

```
  y
  ^
+2│- - - - - - - - - - - - - - - - - - -  midline (y=+2)
  │
 0│★ ★ ★ ★ ★ ★ ★ ★ ★ ★ ★ ★ ★ ★ ★ ★ ★  Global Path (y=0)
  │
-2│══════════════════════════════════════  right_boundary (y=-2)
  +────────────────────────────────────────> x
```

### 4.2 S 形弯道场景（type=1）

弯道中，`midline` 与 `right_boundary` 的坐标均随曲率变化，但它们始终相对于当前行进方向**法向对称**地偏移 `±lane_width`。`GlobalPlannerNormal` 取二者均值，同样得到本车道的几何中心线。

在弧线段上（方向角为 $\theta$），法向偏移量为 `lane_width * (±sin(θ), ∓cos(θ))`，均值恰好抵消，因此全局路径与 `midline`（车道分隔线）等距、位于右侧 `lane_width/2 = 2.0 m` 处。

```
 y                   ╭──────────────── global path ★
 ^                  ╱           ─ ─ ─ ─ midline
  │      ╭──────────╯
  │     ╱
  │─────╯
  +─────────────────────────────────────> x
```

---

## 5. 全局路径点数量

全局路径的点数与 PNC Map 的 `midline` 点数相同：

| 场景 | 计算公式 | 约等于 |
|------|---------|-------|
| 直道 | `road_length / segment_len` | $250 / 0.5 = 500$ 个点 |
| S 形弯道 | `road_length/3 / segment_len + 2 × (π/2 / thetaStep)` | $\approx 167 + 2 \times 157 = 481$ 个点 |

---

## 6. ROS 2 通信架构

`GlobalPathServer` 节点（`global_path_server.cpp`）封装了规划器调用逻辑：

```
┌──────────────────────────────────────────────┐
│         global_path_server_node              │
│                                              │
│  服务（Service）:                             │
│    /global_path_server                       │
│    类型: base_msgs/srv/GlobalPathService      │
│    请求字段:                                  │
│      global_planner_type (0=Normal, 1=AStar) │
│      pnc_map               （输入地图）        │
│    响应字段:                                  │
│      global_path           （输出路径）        │
│                                              │
│  发布（Publish）:                             │
│    /global_path                              │
│       类型: nav_msgs/msg/Path                │
│       用途: 供局部规划模块使用                 │
│                                              │
│    /global_path_rviz                         │
│       类型: visualization_msgs/msg/Marker    │
│       颜色: 红色（r=0.8, g=0.0, b=0.0）      │
│       用途: RViz 可视化显示                   │
└──────────────────────────────────────────────┘
```

**工作流程：**

1. 规划主流程发起 `/global_path_server` 服务请求，携带 `global_planner_type` 和已生成的 `pnc_map`；
2. `GlobalPathServer` 校验 `pnc_map.midline` 是否为空，若为空则中止并打印错误日志；
3. 根据 `global_planner_type` 实例化对应规划器（当前仅实现 `GlobalPlannerNormal`）；
4. 调用 `searchGlobalPath(pnc_map)` 生成全局路径；
5. 响应服务请求，同时向 `/global_path` 和 `/global_path_rviz` 发布消息。

---

## 7. 模块依赖关系

```
planning_static_tps_config.yaml
        │
        ▼
  ConfigReader::readPNCMapConfig()
        │
        ▼
  PNCMapCreatorStraight / PNCMapCreatorSTurn
        │  createPNCMap()
        ▼
  base_msgs::msg::PNCMap
  ├── midline.points[i]        ──────┐
  └── right_boundary.points[i] ─────┤
                                     ▼
                         GlobalPlannerNormal::searchGlobalPath()
                                     │
                                     ▼
                         nav_msgs::msg::Path（全局路径）
                                     │
                         ┌───────────┴────────────┐
                         ▼                        ▼
                    /global_path            /global_path_rviz
                  （供局部规划使用）         （RViz 红色折线显示）
```

---

## 8. 扩展说明

`GlobalPlannerBase` 中定义的枚举类型为未来扩展预留了接口：

```cpp
enum class GlobalPlannerType : uint8 {
    NORMAL = 0U,   // 当前实现：midline + right_boundary 均值
    ASTAR  = 1U,   // 待实现：A* 搜索全局路径
    DEFAULT = 255U
};
```

若需接入 A* 或其他算法，只需继承 `GlobalPlannerBase`，实现 `searchGlobalPath()` 虚函数，并在 `GlobalPathServer::responseGlobalPathCallback()` 中补充对应的 `case` 分支即可。
