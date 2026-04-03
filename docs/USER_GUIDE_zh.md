# SingoriX Teleop GUI 用户说明书

## 1. 软件用途

`robot_viewer` 是一个用于遥操作主臂的可视化监控工具，主要用于：

- 实时订阅主臂关节传感器数据（FastDDS + protobuf）
- 显示机器人模型姿态与关节状态
- 通过健康诊断快速发现异常关节
- 查看单关节实时波形曲线
- 管理告警事件（触发、确认、恢复）
- 录制会话数据到 CSV 用于复盘

---

## 2. 数据来源

- Topic: `singorix_omnilink/scaled_device_robot_data`
- 消息类型: `galbot::singorix_proto::SingoriXSensor`
- 通信方式: 与 `singorix_omnilink` 中 `omnilinkInterface` 同一套 EmbOSA/FastDDS 风格

---

## 3. 构建与启动

## 3.1 构建

在工程根目录执行：

```bash
cmake -S . -B build
cmake --build build --target robot_viewer -j4
```

## 3.2 启动

默认配置启动（从工程根目录）：

```bash
./bin/robot_viewer
```

指定配置文件启动：

```bash
./bin/robot_viewer /your/path/robot_viewer.yaml
```

---

## 4. 界面说明

## 4.1 3D 视图

- 左键拖动: 旋转（RViz Orbit）
- 中键拖动 或 `Shift + 左键`: 平移
- 右键拖动: Dolly 缩放
- 鼠标滚轮: 缩放

## 4.2 侧边栏（可折叠）

- 右上角箭头可折叠/展开侧边栏
- `Sidebar Width` 可拖动调节宽度

## 4.3 状态区

- `Status`:
- `waiting for data`: 还未收到消息
- `receiving`: 正常接收
- `stale`: 数据超时（超过 `stale_timeout_seconds`）
- `Msg Count`: 已收到消息计数

---

## 5. 功能模块

## 5.1 Health Overview（健康总览）

- `Input Rate (Hz)`: 输入消息频率估计
- `Data Age (s)`: 距离最新一帧数据的时间
- `Health Score`: 0-100 健康分
- `State`: `HEALTHY / WARNING / CRITICAL`

该区域用于快速判断系统整体是否稳定。

## 5.2 Session Tools（会话工具）

- `Capture Baseline`: 抓取当前姿态作为基线
- `Clear Baseline`: 清空基线
- `Start Recording CSV`: 开始录制
- `Stop Recording CSV`: 停止录制

录制文件默认输出到 `logs/`，文件名形如：

`teleop_session_YYYYMMDD_HHMMSS.csv`

## 5.3 Joint Diagnostic（关节诊断表）

字段说明：

- `Pos(rad)`: 当前弧度
- `Pos(deg)`: 当前角度
- `Delta(deg)`: 相对基线偏差（未抓取基线时显示 `-`）
- `Vel / Eff / Cur`: 速度/力矩/电流
- `Limit(rad)`: URDF 关节限位
- `Health`: `OK / INVALID / NO_URDF_MATCH / OUT_OF_RANGE`

诊断建议：

- `INVALID`: 上游数据异常（空值或非有限数）
- `NO_URDF_MATCH`: 消息关节名与模型未匹配
- `OUT_OF_RANGE`: 超过 URDF 限位（含容差）

## 5.4 Joint Waveform（关节波形）

- `Wave Group`: 关节组选择
- `Wave Joint`: 关节选择
- `Wave Metric`: 指标选择（位置/速度/力矩/电流）
- 下方曲线图: 实时历史波形

用于定位抖动、漂移、突变等动态问题。

## 5.5 Alarm Center（告警中心）

- 连续异常达到阈值后触发告警（抗抖设计）
- `Show only active alarms`: 仅看活跃告警
- `Ack All Active`: 批量确认
- `Clear Recovered`: 清理已恢复告警

告警表字段：

- `State`: `ACTIVE / RECOVERED`
- `Reason`: 告警原因
- `Count`: 触发次数
- `First(s) / Last(s)`: 首次/最近触发时间（相对程序启动）
- `Ack`: 告警确认状态

---

## 6. 配置文件说明

默认配置文件：`config/robot_viewer.yaml`

常用参数：

- `sensor.topic`: 订阅 topic
- `sensor.node_name`: 订阅节点名
- `ui.only_show_master_arm_groups`: 仅显示左右臂组
- `ui.fix_base_like_mujoco`: 固定底座
- `ui.stale_timeout_seconds`: stale 判定时间
- `ui.out_of_range_margin`: 越界容差
- `ui.waveform_history_size`: 波形缓存长度
- `ui.waveform_plot_height`: 波形区域高度
- `ui.alarm_trigger_frames`: 连续异常多少帧才触发告警
- `ui.baseline_warn_delta_deg`: 基线偏差高亮阈值（角度）
- `ui.record_output_dir`: 录制文件输出目录
- `ui.auto_start_recording`: 启动后自动录制

---

## 7. CSV 字段说明

录制 CSV 表头：

`time_s,group,joint,position,velocity,effort,current,invalid,no_urdf_match,out_of_range,health`

说明：

- `time_s`: 相对程序启动时间（秒）
- `invalid/no_urdf_match/out_of_range`: 0 或 1
- `health`: 每行关节对应的健康状态标签

---

## 8. 常见问题排查

## 8.1 一直显示 `waiting for data`

- 检查数据发布进程是否运行
- 检查 topic 是否与配置一致
- 检查网络/中间件环境是否正常

## 8.2 显示 `Sensor subscriber init failed`

- 检查 EmbOSA/FastDDS 运行环境与依赖库
- 检查运行时库路径和部署环境

## 8.3 大量 `NO_URDF_MATCH`

- 检查消息中的关节名与 URDF 关节名是否一致
- 检查加载的 URDF 是否为当前机器人版本

## 8.4 `OUT_OF_RANGE` 频繁出现

- 先确认 URDF 限位是否正确
- 再检查上游数据是否做了缩放/单位转换
- 可适度调整 `out_of_range_margin`

---

## 9. 使用建议（现场）

- 开机后先看 `Status` 和 `Input Rate`
- 执行动作前先 `Capture Baseline`
- 观察 `Delta(deg)` 与 `Alarm Center` 联动
- 关键测试阶段开启 CSV 录制，便于事后分析

