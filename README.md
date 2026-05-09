# Teleop GUI 🤖✨

一个面向遥操作主臂的可视化监控与交互工具。基于 ImGui + OpenGL + GLFW，支持机器人 3D 渲染、主臂传感器数据监控、手柄状态显示、WBC/错误监控与基础控制下发。

希望它是你现场调试时的“第二双眼睛” 👀

## 1. 主要能力 🚀

- 实时订阅主臂关节传感器数据（FastDDS/EmbOSA）
- 3D 机器人姿态渲染（支持固定底座、轨道相机）
- 关节诊断表：位置/速度/力矩/电流/限位/健康状态
- 关节波形：可按关节组与指标切换，带坐标轴与刻度数字
- 手柄监控：左右手柄摇杆与按键状态、按键历史记录
- OmniLink 桥接控制：RC 虚拟按键下发（如 `fix_height` 等）
- WBC 监控：`wbc_info` 组范数状态显示
- 错误监控：`error` 频道实时显示与关键故障联动
- 发布打包：自动收集运行依赖并生成可分发目录 📦

## 2. 仓库结构 🗂️

- `src/teleop_viewer/`：主应用核心（App、侧栏、场景、订阅发布、配置）
- `include/teleop_viewer/`：对应头文件
- `config/robot_viewer.yaml`：运行配置
- `docs/USER_GUIDE_zh.md`：面向操作人员的使用手册
- `build.sh` / `all_rebuild.sh`：开发构建脚本
- `auto_build.sh`：一键构建+打包+（可选）自检
- `scripts/collect_dep.sh`：收集依赖并生成 `teleop_gui_release`

## 3. 通信与消息 📡

当前 GUI 主要对接 `singorix_omnilink` 侧消息：

- 关节数据：
  - Topic: `singorix_omnilink/scaled_device_robot_data`
  - Proto: `galbot::singorix_proto::SingoriXSensor`
- 手柄数据：
  - Topic: `singorix_omnilink/joy`
  - Proto: `galbot::sensor_proto::Joy`
- OmniLink 状态：
  - Topic: `singorix_omnilink/states`
- WBC 信息：
  - Topic: `singorix/wbcs/wbc_info`
  - Proto: `galbot::singorix_proto::WBCInfo`
- 错误信息：
  - Topic: `singorix/wbcs/error`
  - Proto: `galbot::singorix_proto::SingoriXError`

说明：消息发布端在 `singorix_omnilink` 仓库中，GUI 按相同通信风格订阅/发布，开箱即可对齐现有链路 🔗

## 4. 依赖环境 🧩

本项目依赖公司内部与系统库，典型包括：

- C++17, CMake (>= 3.16)
- OpenGL, GLFW, GLEW, Assimp
- glog, Boost(thread), yaml-cpp
- `singorix_proto`, `omnilink_proto`, `singorix_common`, `trac_ik`
- EmbOSA/FastDDS 相关库（如 `embosa`, `fastcdr`, `fastrtps`, `protobuf` 等）

默认会从如下路径查找部分内部依赖：

- `/opt/galbot/devel/<arch>/...`

若环境中存在 `~/switch_env.sh`，推荐通过 `auto_build.sh` 自动切换，一步到位更省心 ✅

## 5. 开发构建与运行 🛠️

### 5.1 快速构建 ⚡

```bash
./build.sh
```

### 5.2 全量重编译 🔁

```bash
./all_rebuild.sh
```

### 5.3 启动 ▶️

```bash
./bin/robot_viewer
```

或指定配置：

```bash
./bin/robot_viewer ./config/robot_viewer.yaml
```

## 6. 一键打包发布（推荐）📦

目标：生成 `teleop_gui_release`，在另一台 Ubuntu 上无需额外安装第三方库（系统基础库除外）即可运行，拎包就走 🧳

### 6.1 执行 🏃

```bash
./auto_build.sh
```

常用参数：

```bash
./auto_build.sh -q            # 快速编译后打包
./auto_build.sh --all         # 全量重编译后打包（默认）
./auto_build.sh --self-test   # 打包后执行发布包依赖自检
./auto_build.sh --skip-env    # 跳过 switch_env.sh
```

### 6.2 产物 🎁

默认最终发布目录：

- `../../teleop_gui_release`（可用环境变量 `TELEOP_GUI_RELEASE_TARGET_DIR` 覆盖）

目录内包含：

- `bin/robot_viewer`
- `lib/*.so*`（打包收集的运行依赖）
- `config/`
- `shader/`
- `docs/`
- `run.sh`

### 6.3 在目标机运行 🖥️

```bash
cd teleop_gui_release
./run.sh
# 或
./run.sh ./config/robot_viewer.yaml
# 仅依赖检查
./run.sh --self-test
```

`run.sh` 会自动设置：

- `LD_LIBRARY_PATH=$PWD/lib:$LD_LIBRARY_PATH`

## 7. 关键配置（config/robot_viewer.yaml）⚙️

- `sensor.topic`：主臂关节数据 topic
- `joy.topic`：手柄 topic
- `robot.urdf_path`：URDF 路径
- `ui.side_panel_width`：侧栏宽度初始值
- `ui.sidebar_width_drag_speed`：侧栏拖拽灵敏度
- `ui.fix_base_like_mujoco`：固定底座显示
- `ui.only_show_master_arm_groups`：仅显示主臂关节组
- `ui.cjk_font_path` / `ui.cjk_font_size`：中文字体
- `ik.mode`：IK 求解模式，`single_chain`（单链）或 `full_body`（全身多链约束）
- `ik.full_body_iterations`：`full_body` 下每次求解的迭代轮次
- `ik.chains`：IK 链配置列表（`label/base_link/tip_link`）
- `ik.chains[].base_link_candidates` / `ik.chains[].tip_link_candidates`：可选候选 link 列表；程序会结合 URDF 自动选取首个存在的 link
- `omnilink_bridge.enable`：是否启用桥接控制
- `omnilink_bridge.rc_virtual_joy_topic`：RC 控制下发 topic
- `omnilink_bridge.wbc_info_topic` / `error_topic`：WBC 与错误通道
- `omnilink_bridge.auto_lock_on_critical_fault`：关键故障自动锁定

## 8. 常见问题 🩺

- 中文显示为问号：
  - 配置 `ui.cjk_font_path` 指向可用中文字体；
  - 打包脚本也会尝试自动拷贝常见 CJK 字体到 `teleop_gui_release/fonts`。
- 启动后无数据：
  - 检查 `singorix_omnilink` 发布进程是否在运行；
  - 核对 topic 与 proto 类型是否匹配。
- 目标机运行失败：
  - 在发布目录执行 `./run.sh --self-test` 查看缺失/越界依赖。

## 9. 相关文档 📚

- 操作手册：`docs/USER_GUIDE_zh.md`

## 10. RViz Interactive Marker（推荐做IK拖动）🕹️

如果你觉得 `robot_kinematic_viewer` 内置拖动手感不好，可以直接用 RViz 的 Interactive Marker 做 6DoF 拖动，再把目标位姿发布给 IK。

脚本位置：

- `scripts/rviz_ik_interactive_marker.py`

### 10.1 依赖

- ROS1 Noetic
- `interactive_markers`
- `rviz`

示例安装：

```bash
sudo apt install ros-noetic-interactive-markers ros-noetic-rviz
```

### 10.2 启动脚本

一键启动整套（`roscore + static tf + marker + rviz + robot_kinematic_viewer`）：

```bash
cd /home/yuxia/Workspace/SingoriX/OmniLink/teleop_gui
./scripts/start_rviz_ik_stack.sh
```

只启动 ROS+marker（不拉起 RViz 和 viewer）：

```bash
./scripts/start_rviz_ik_stack.sh --no-rviz --no-viewer
```

旧的手动方式：

```bash
source /opt/ros/noetic/setup.bash
rosrun <your_ros_pkg> rviz_ik_interactive_marker.py
```

若你不是按 ROS package 方式安装，可直接运行：

```bash
source /opt/ros/noetic/setup.bash
python3 ./scripts/rviz_ik_interactive_marker.py
```

默认发布目标位姿：

- Topic: `/teleop_gui/ik_target_pose`
- Type: `geometry_msgs/PoseStamped`
- Frame: `world`（可通过参数改）

### 10.3 RViz 配置

1. `Fixed Frame` 设为与脚本 `~frame_id` 相同（默认 `world`）。
2. 添加 `InteractiveMarkers` 显示，Topic 设为 `/teleop_gui_ik_marker/update`。
3. 拖动 marker 的轴/圆环即可输出实时位姿。

### 10.4 常用参数

```bash
rosrun <your_ros_pkg> rviz_ik_interactive_marker.py _frame_id:=torso_base_link _publish_topic:=/ik_target_pose _marker_scale:=0.35 _publish_rate_hz:=60
```
