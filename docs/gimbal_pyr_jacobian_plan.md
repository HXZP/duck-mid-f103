# 云台 P-Y-R 结构 Jacobian 控制方案

## 目标

记录云台在位置环使用 IMU 姿态坐标系、速度环使用电机轴速度时的控制映射方案。

当前讨论的机械连接顺序为：

```text
Pitch -> Yaw -> Roll
```

末端轴为 Roll。位置环不能直接把 IMU 姿态误差分别当作三个电机的速度目标，需要先把姿态角速度目标转换成电机轴速度目标。

## 坐标假设

本文公式和仿真页面都直接使用实机 Z-up 右手坐标系：

```text
X = 前向，末端 Roll 初始轴方向
Y = 侧向，Pitch 固定轴方向
Z = 垂直地面，Yaw 初始轴方向
```

P-Y-R 是串联关节，不是三个固定世界轴。只有 Pitch 轴始终固定在基座 Y 轴；Yaw 轴会随 Pitch 变化；Roll 轴会随 Pitch 和 Yaw 变化：

```text
Pitch 轴 = 基座固定 Y 轴
Yaw 轴   = Pitch 后的局部 Z 轴
Roll 轴  = Pitch + Yaw 后的局部 X 轴
```

3D 仿真中的零位几何为：

```text
Pitch 输出连杆竖直向上
Yaw 关节连接在 Pitch 输出端
Pitch 输出连杆与 Yaw 后续连杆之间存在 90° 结构转折
Yaw 后续连杆沿局部 X 方向连接 Roll 负载
```

该 90° 转折是机构外观和连杆偏置，用于让仿真模型接近实物连接方式；当前 Jacobian 只描述三个转轴对姿态角速度的贡献，不包含末端位置线速度，因此该几何偏置不改变下面的轴向 Jacobian。

旋转顺序为：

```text
R = Ry(pitch) * Rz(yaw) * Rx(roll)
```

电机轴速度定义为：

```text
q_dot = [pitch_dot, yaw_dot, roll_dot]
```

姿态角速度目标定义在云台基座坐标系：

```text
omega_base = [wx, wy, wz]
```

如果 RK/IMU 输出坐标系不是云台基座坐标系，需要先增加 IMU 到云台基座的安装矩阵，把 IMU 姿态和角速度转换到该坐标系。

## 正运动学映射

三个电机轴在基座坐标系下的方向为：

```text
pitch_axis_base = [0, 1, 0]

yaw_axis_base = Ry(pitch) * [0, 0, 1]
              = [sin(pitch), 0, cos(pitch)]

roll_axis_base = Ry(pitch) * Rz(yaw) * [1, 0, 0]
               = [cos(pitch) * cos(yaw),
                  sin(yaw),
                 -sin(pitch) * cos(yaw)]
```

因此：

```text
omega_base = J(q) * q_dot
```

其中 Jacobian 按 `[pitch_dot, yaw_dot, roll_dot]` 排列：

```text
J =
[ 0, sin(pitch),  cos(pitch) * cos(yaw)  ]
[ 1, 0,           sin(yaw)                ]
[ 0, cos(pitch), -sin(pitch) * cos(yaw)  ]
```

展开后：

```text
wx = sin(pitch) * yaw_dot + cos(pitch) * cos(yaw) * roll_dot
wy = pitch_dot + sin(yaw) * roll_dot
wz = cos(pitch) * yaw_dot - sin(pitch) * cos(yaw) * roll_dot
```

## 反解电机轴速度

位置环输出的是姿态角速度目标 `omega_cmd_base`，需要反解为电机轴速度目标：

```text
roll_dot_cmd = (cos(pitch) * wx - sin(pitch) * wz) / cos(yaw)
yaw_dot_cmd = sin(pitch) * wx + cos(pitch) * wz
pitch_dot_cmd = wy - sin(yaw) * roll_dot_cmd
```

当 `cos(yaw)` 接近 0 时，反解存在奇异点，控制器应限制 Yaw 工作范围或在奇异区附近输出零电流并清 PID。

映射回当前代码的轴编号时，需要注意机械顺序和数组顺序不同：

```text
target_speed[GIMBAL_AXIS_PITCH] = pitch_dot_cmd
target_speed[GIMBAL_AXIS_YAW]   = yaw_dot_cmd
target_speed[GIMBAL_AXIS_ROLL]  = roll_dot_cmd
```

## 推荐控制链路

最小风险实现建议如下：

```text
目标 IMU 姿态
    -
反馈 IMU 姿态
    =
姿态误差

姿态误差进入三个位置环 PID
    ↓
omega_cmd_base = [wx, wy, wz]

读取当前电机角度 pitch、yaw
    ↓
P-Y-R Jacobian 反解
    ↓
target_motor_speed[PITCH/YAW/ROLL]

target_motor_speed[i] - motor_state[i].speed_mrad_s
    ↓
速度环 PID
    ↓
current_target
```

第一阶段建议速度反馈仍然使用电机速度，不直接用 IMU 角速度闭速度环。这样速度环仍然在电机轴坐标系内闭环，风险较低。

## 后续 IMU 速度反馈方案

如果后续速度反馈也要使用 IMU gyro，不能直接把 `imu_roll_speed`、`imu_pitch_speed`、`imu_yaw_speed` 分别当作三个电机轴速度。

应该先把 IMU gyro 转到云台基座坐标系：

```text
omega_feedback_base = R_imu_to_base * omega_imu
```

再通过同一套 P-Y-R Jacobian 反解为电机轴速度反馈：

```text
feedback_motor_speed[PITCH/YAW/ROLL]
```

最后速度环使用：

```text
speed_error[i] = target_motor_speed[i] - feedback_motor_speed[i]
```

## 实现注意事项

1. 当前代码轴枚举顺序是 `ROLL, PITCH, YAW`，机械顺序是 `PITCH, YAW, ROLL`，实现时不能直接按数组顺序套公式。
2. 需要确认三个电机正方向和 IMU/RK 姿态正方向是否一致，不一致时应在轴配置中增加方向系数 `+1/-1`。
3. 需要确认 IMU 安装坐标系和云台基座坐标系的关系，必要时增加安装矩阵。
4. 三角函数输入单位应统一为 rad 或 mrad 转换后的定点角度，不能直接把 mrad 传入普通 `sin/cos`。
5. 如果 F103 上使用浮点三角函数开销过大，优先考虑 CMSIS-DSP 或项目 lib 中的定点三角函数实现。
6. 电机角度 `pitch/yaw` 必须有效，否则不能计算 Jacobian，应输出零电流并清 PID。
7. 反解后的目标速度需要继续使用每轴速度限幅，避免姿态误差过大导致电机速度目标突变。
8. `cos(yaw)` 接近 0 时 Jacobian 反解奇异，实机控制必须设置保护阈值。
9. 当前公式依赖所列轴定义和旋转顺序，机械结构或坐标定义变化时必须重新推导。

## 建议落地步骤

1. 在代码中新增三轴联合控制函数，替代逐轴独立 `gimbal_control_axis`。
2. 每个控制周期先拷贝三轴电机状态，确认角度和速度有效。
3. 位置环根据 IMU 姿态误差输出 `omega_cmd_base`。
4. 用当前 `pitch/yaw` 电机角度反解电机轴目标速度。
5. 三个速度环继续使用电机速度反馈并输出电流目标。
6. 完成坐标方向标定后，再考虑把速度反馈源切到 IMU gyro。

## 3D 可视化仿真

仿真入口：

```text
project/duck-mid-f103/scripts/gimbal_pyr_3d_sim.html
```

该页面用于验证 P-Y-R 机械顺序下的轴方向、耦合关系和 Jacobian 正反解一致性。页面包含：

1. 固定底座、Pitch 关节、Yaw 连杆、Roll 末端负载组成的机械臂式 3D 结构。
2. Pitch 零位竖直向上、顶部 90° 转折连接 Yaw 的机构外观。
3. 实机 Z-up 右手坐标系标签。
4. 当前电机角度滑块。
5. 目标姿态滑块。
6. 位置环比例系数和仿真步长。
7. `omega_cmd`、反解电机速度和正反解误差显示。
8. 单步、运行、重置控制。

验证步骤：

1. 打开 `scripts/gimbal_pyr_3d_sim.html`。
2. 先确认坐标标签：`Z` 是垂直地面的竖直轴。
3. 分别只设置 `Target X`、`Target Y`、`Target Z`，观察对应主轴运动方向是否符合机械正方向。
4. 调整当前 Pitch/Yaw 角度，再观察同一个姿态目标下的电机速度耦合变化。
5. 检查页面中的正反解误差是否保持接近 0。
6. 如果某一轴方向与实机相反，在固件轴配置中增加方向系数，而不是修改 Jacobian 公式本身。

注意：该页面使用 Three.js CDN 加载 3D 渲染库，打开页面时需要网络访问。如果需要完全离线仿真，应把 Three.js 文件下载到本地并改为本地引用。
