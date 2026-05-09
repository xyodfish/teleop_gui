# Trajectory 文件格式（简化版）

推荐使用紧凑格式：`joints + dt + values`。

```yaml
trajectory:
  version: 2
  format: compact
  joints: [joint_a, joint_b, joint_c]
  t0: 0.0
  dt: 0.1
  values:
    - [0.0, 0.1, -0.1]
    - [0.1, 0.2, -0.1]
    - [0.2, 0.1,  0.0]
```

说明：
- `joints`：列定义，后续每一列对应一个关节。
- `t0`：首帧时间（秒，可省略，默认 0）。
- `dt`：固定时间步长（秒）。
- `values`：每行一帧，只写关节值，不再重复关节名。

非等间隔轨迹可用：

```yaml
trajectory:
  version: 2
  format: compact
  joints: [joint_a, joint_b]
  samples:
    - [0.0,  0.1, -0.2]   # [t, joint_a, joint_b]
    - [0.35, 0.3, -0.1]
    - [0.9,  0.0,  0.0]
```

兼容性：
- 当前加载器兼容旧版 `keyframes: [{t, joints:{...}}]`。
- 保存时默认输出简化后的紧凑格式。

## CSV 格式（也支持）

```csv
time,joint_a,joint_b,joint_c
0.0,0.0,0.1,-0.1
0.1,0.1,0.2,-0.1
0.2,0.2,0.1,0.0
```

说明：
- 第一列必须是 `time`（或 `t`）。
- 后续列名是关节名。
- 每行是一帧数据，单位是弧度。

示例文件：
- `config/trajectory_playback_demo.csv`
