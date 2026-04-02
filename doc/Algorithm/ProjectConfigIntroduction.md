# Project Configuration Introduction

本文档介绍 BSAP 项目中所有配置文件的用途、结构及切换方式。

---

## 1. 配置文件总览

所有配置文件位于 `src/planning_core/config/` 目录下：

| 文件 | 用途 |
|------|------|
| `scenario_config.yaml` | **场景选择**——决定当前运行哪种仿真场景 |
| `planning_static_tps_config.yaml` | 静态障碍物场景（场景 0/1）的完整规划参数 |
| `planning_dynamic_tps_config.yaml` | 动态障碍物场景（场景 3）的完整规划参数 |
| `planning_inlane_tps_config.yaml` | 同车道障碍物场景（场景 2,如超车、跟车、会车等场景）的规划参数 |

---

## 2. 场景配置（scenario_config.yaml）

```yaml
scenario:
  type: 1  # 0: follow lane; 1: static tp detour; 2: tp in ego lane; 3: dynamic tp detour
  tp_num: 3
```

| 字段 | 含义 |
|------|------|
| `type` | 场景类型枚举，见下表 |
| `tp_num` | 交通参与者（Traffic Participant）数量 |

### 场景类型枚举

| type | 场景名称 | 描述 | 加载的配置文件 |
|:----:|---------|------|--------------|
| 0 | Follow Lane | 无障碍物，自车沿车道行驶 | `planning_static_tps_config.yaml` |
| 1 | Static TP Detour | 静态障碍物绕行 | `planning_static_tps_config.yaml` |
| 2 | TP in Ego Lane | 同车道障碍物 | `planning_inlane_tps_config.yaml` |
| 3 | Dynamic TP Detour | 动态障碍物绕行 | `planning_dynamic_tps_config.yaml` |

对应的 C++ 枚举定义（`config_reader.h`）：

```cpp
enum class ScenarioType : uint8 {
    FOLLOW_LANE       = 0U,
    STATIC_TP_DETOUR  = 1U,
    TP_IN_EGO_LANE    = 2U,
    DYNAMIC_TP_DETOUR = 3U
};
```

`ConfigReader` 构造函数根据 `scenario.type_` 通过 `switch-case` 加载对应的配置文件，**无需重新编译**即可切换场景——只需修改 `scenario_config.yaml` 中的 `type` 值并重启节点。

---

## 3. 规划参数配置（planning_*_config.yaml）

每个场景配置文件具有相同的 YAML 结构，包含以下模块：

### 3.1 车辆参数（vehicle）

```yaml
vehicle:
  ego_car:
    id: 0
    frame: "base_footprint"
    length: 3.0          # 车身长度 (m)
    width: 1.5           # 车身宽度 (m)
    pose_x: 0.0          # 初始 x 坐标 (m)
    pose_y: 0.0          # 初始 y 坐标 (m)
    pose_theta: 0.0      # 初始航向角 (rad)
    set_speed: 1.0       # 设定速度 (m/s)
  tp_car1:
    id: 1
    frame: "base_footprint_tp1"
    length: 3.0
    width: 1.6
    pose_x: 60.0
    pose_y: 0.0
    pose_theta: 0.0
    set_speed: 0.0       # 0.0 表示静止
  # tp_car2, tp_car3 类似...
```

### 3.2 PNC 地图参数（pnc_map）

```yaml
pnc_map:
  frame: "map"
  type: 1             # 0: 直道, 1: S 形弯道
  road_length: 250.0  # 道路总长度 (m)
  lane_width: 4.0     # 单车道宽度 (m)
  segment_len: 0.5    # 采样间距 (m)
  speed_limit: 1.0    # 限速 (m/s)，预留字段
```

### 3.3 全局路径规划（global_path）

```yaml
global_path:
  type: 0  # 0: normal, 1: astar
```

### 3.4 参考线（reference_line）

```yaml
reference_line:
  type: 0          # 0: normal, 1: stitch
  front_size: 200  # 参考线前方采样点数
  back_size: 80    # 参考线后方采样点数
```

### 3.5 局部路径规划（local_path）

```yaml
local_path:
  curve_type: 2    # 0: 一阶, 1: 三阶, 2: 五阶多项式
  path_size: 80    # 路径采样点数
```

### 3.6 局部速度规划（local_speeds）

```yaml
local_speeds:
  speeds_size: 100  # 速度采样点数
```

### 3.7 决策参数（decision）

```yaml
decision:
  lat_safe_margin: 0.5   # 横向安全裕度 (m)
  long_safe_margin: 10.0  # 纵向安全裕度 (m)
```

### 3.8 规划主流程（planning_process）

```yaml
planning_process:
  perception_range: 100.0  # 感知范围 (m)
```

---

## 4. URDF 可视化配置（tp_car.xacro）

**文件：** `src/planning_core/urdf/tp_model/tp_car.xacro`

TP 车辆的 RViz 可视化模型通过 xacro 宏生成。该文件支持以下两个控制维度：

### 4.1 TP 车辆数量（tp_num）

`tp_car.xacro` 接受外部参数 `tp_num`，用于控制生成的 TP 车辆数量。该参数由 `planning_launch.py` 在启动时从 `scenario_config.yaml` 中读取并传入：

```xml
<xacro:arg name="tp_num" default="3"/>
```

| tp_num | 效果 |
|--------|------|
| `0` | 不生成任何 TP 车辆模型，同时 launch 文件不启动 TP 相关节点 |
| `1` | 仅生成 tp1 |
| `2` | 生成 tp1 + tp2 |
| `3` | 生成 tp1 + tp2 + tp3 |

> **说明：** `tp_num` 的值直接来源于 `scenario_config.yaml` 中的 `tp_num` 字段，无需手动修改 xacro 文件即可控制可视化车辆数量。

### 4.2 TP 车辆尺寸（use_dynamic_tps_config）

该文件中的 `use_dynamic_tps_config` 属性控制 TP 车辆的可视化尺寸：

```xml
<xacro:property name="use_dynamic_tps_config" value="false"/>
```

| Value | 效果 |
|-------|------|
| `false` | 所有 TP 车辆使用静态场景尺寸（3.0 × 1.6 × 1.2 m） |
| `true`  | TP 车辆使用动态场景尺寸（与 `planning_dynamic_tps_config.yaml` 中的 vehicle 尺寸一致） |

> **注意：** 切换场景时，应确保 `tp_car.xacro` 中的尺寸与 `scenario_config.yaml` 选择的场景一致，以避免可视化模型与规划逻辑不匹配。

---

## 5. 启动配置（planning_launch.py）

**文件：** `src/planning_core/launch/planning_launch.py`

launch 文件中有一个 Python 级别的调试开关：

```python
JOINT_GUI_DEBUG = False
```

| Value | 效果 |
|-------|------|
| `False` | 使用 `joint_state_publisher`（无界面，程序化控制关节） |
| `True` | 使用 `joint_state_publisher_gui`（弹出 GUI 滑条窗口，手动控制） |

此外，launch 文件在启动时会读取 `scenario_config.yaml` 中的 `tp_num` 字段：
- 将 `tp_num` 作为参数传递给 `xacro` 命令（`xacro tp_car.xacro tp_num:=N`），控制生成的 TP 车辆模型数量
- 当 `tp_num == 0` 时，整个 TP 车辆节点组（`robot_state_publisher` + `joint_state_publisher`）不会被启动

---

## 6. 如何切换场景

切换场景**无需重新编译**，只需修改配置文件并重启节点：

1. 编辑 `src/planning_core/config/scenario_config.yaml`，将 `type` 设为目标场景编号，`tp_num` 设为对应的 TP 数量：
    ```yaml
    scenario:
      type: 3  # 切换到动态障碍物绕行
      tp_num: 3
    ```
    `tp_num` 同时控制规划逻辑中的 TP 数量和 RViz 可视化模型数量（无需手动修改 xacro 文件）。例如循迹场景设为 `tp_num: 0`，则不会生成任何 TP 车辆。

2. 如果切换到动态场景（type=3），同步修改 `tp_car.xacro` 中的可视化尺寸开关：
    ```xml
    <xacro:property name="use_dynamic_tps_config" value="true"/>
    ```
    切换回静态场景（type=0/1）时设为 `false`。

3. 重新构建（仅当修改了 xacro 文件时需要；仅修改 `scenario_config.yaml` 中的 `tp_num` 无需重新构建）：
    ```sh
    colcon build --packages-select planning_core
    ```

4. 重新启动节点：
    ```sh
    source install/setup.bash
    ./scripts/start_launch.sh
    ```
