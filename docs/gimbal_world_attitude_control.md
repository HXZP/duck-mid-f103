# 云台世界姿态控制理论说明

## 目标

本文记录云台使用世界坐标系姿态目标时的运动学关系。

需要区分两类量：

```text
世界姿态目标：描述末端在世界坐标系中的目标姿态
电机关节角：描述 Pitch/Yaw/Roll 三个电机轴各自转了多少
```

这两类量不能直接逐项相减。也就是说，不能把：

```text
world_roll  - motor_roll
world_pitch - motor_pitch
world_yaw   - motor_yaw
```

直接当作姿态误差。正确做法是先把二者都转换成姿态矩阵或四元数，再计算完整姿态误差。

## 坐标和姿态定义

当前约定实机使用 Z-up 右手坐标系：

```text
X = 前向
Y = 侧向
Z = 垂直地面
```

矩阵采用列向量约定：

```text
v_world = R * v_body
```

其中 `R` 表示末端坐标系到世界坐标系的姿态变换。`R` 的三列分别是：

```text
末端 X 轴在世界坐标中的方向
末端 Y 轴在世界坐标中的方向
末端 Z 轴在世界坐标中的方向
```

## 云台机构姿态

云台机械连接顺序为：

```text
Pitch -> Yaw -> Roll
```

三个电机轴定义为：

```text
Pitch 轴 = 基座固定 Y 轴
Yaw 轴   = Pitch 后的局部 Z 轴
Roll 轴  = Pitch + Yaw 后的局部 X 轴
```

因此云台相对基座的机构姿态为：

```text
R_gimbal = Ry(motor_pitch) * Rz(motor_yaw) * Rx(motor_roll)
```

这里的 `motor_roll` 是 Roll 电机的关节角，不等价于世界坐标系下的 `world_roll`。

## 世界姿态目标

世界坐标系下的 Roll/Pitch/Yaw 目标通常表达为：

```text
R_target_world = Rz(world_yaw) * Ry(world_pitch) * Rx(world_roll)
```

由于矩阵右侧先作用，上式等价于：

```text
先绕局部 X 形成 roll
再绕中间 Y 形成 pitch
最后绕世界 Z 形成 yaw
```

这个定义描述的是末端在世界坐标中的目标姿态，而不是三个电机应该到达的目标角。

因此，当：

```text
world_roll  = 0
world_pitch = 30 deg
world_yaw   变化
```

Roll 电机角不一定保持 0。为了让末端世界 Roll 继续保持 0，Roll 电机通常需要随 Pitch/Yaw 组合进行补偿。

## 姿态误差

设当前末端世界姿态为：

```text
R_current_world
```

目标世界姿态为：

```text
R_target_world
```

从当前姿态转到目标姿态的误差旋转为：

```text
R_err = R_target_world * R_current_world^-1
```

如果使用四元数：

```text
q_err = q_target_world * inverse(q_current_world)
```

姿态控制不能直接使用欧拉角差值，而应把 `q_err` 转换为旋转向量：

```text
e = Log(q_err)
```

其中：

```text
e = axis * angle
```

含义是：从当前姿态到目标姿态，最短需要绕 `axis` 转动 `angle`。

位置环输出可以写为：

```text
omega_cmd = Kp * e
```

这里的 `omega_cmd` 是姿态角速度目标。

## 四元数到旋转向量

设误差四元数为：

```text
q_err = [w, x, y, z]
```

先选择最短旋转。如果：

```text
w < 0
```

则取：

```text
q_err = -q_err
```

然后：

```text
sin_half = sqrt(x^2 + y^2 + z^2)
angle = 2 * atan2(sin_half, w)
axis = [x, y, z] / sin_half
e = axis * angle
```

当 `sin_half` 很小时，说明误差接近 0，旋转向量可以直接置 0 或使用小角度近似。

## 叉乘误差的关系

Mahony 姿态解算中常见的误差形式为：

```text
e = v_measured x v_estimated
```

它表示两个方向向量之间的最小旋转方向。小角度时：

```text
v_measured x v_estimated ~= 旋转误差向量
```

因此叉乘误差和 `Log(q_err)` 是同一类思想。

差异在于：

```text
单个方向向量叉乘：只能约束部分姿态，例如重力向量只能约束 roll/pitch
完整四元数误差：可以约束完整三轴姿态，包括 yaw
```

所以云台世界姿态控制建议使用：

```text
e = Log(q_target * inverse(q_current))
```

而不是只用单个向量叉乘。

## 底盘 IMU 参与时

如果底盘有 IMU，且底盘自身会运动，则末端世界姿态由两部分组成：

```text
R_base_world = 底盘 IMU 给出的基座世界姿态
R_gimbal     = 云台相对底盘基座的机构姿态
```

末端世界姿态为：

```text
R_ee_world = R_base_world * R_gimbal
```

展开后：

```text
R_ee_world =
    Rz(base_yaw) * Ry(base_pitch) * Rx(base_roll)
  * Ry(motor_pitch) * Rz(motor_yaw) * Rx(motor_roll)
```

世界目标姿态仍然是：

```text
R_target_world = Rz(world_yaw) * Ry(world_pitch) * Rx(world_roll)
```

完整误差为：

```text
R_err_world = R_target_world * R_ee_world^-1
```

如果由该误差得到的角速度目标表达在世界坐标系：

```text
omega_cmd_world = Kp * Log(R_err_world)
```

而 Jacobian 使用的是云台基座坐标系下的角速度：

```text
omega_base = J(q) * q_dot
```

则进入 Jacobian 前需要转换坐标：

```text
omega_cmd_base = R_base_world^-1 * omega_cmd_world
```

再进行反解：

```text
q_dot = J(q)^-1 * omega_cmd_base
```

## 控制链路总结

底盘会动时，推荐控制链路为：

```text
底盘 IMU 姿态
    -> R_base_world

电机角 pitch/yaw/roll
    -> R_gimbal

R_ee_world = R_base_world * R_gimbal

世界目标姿态
    -> R_target_world

R_err_world = R_target_world * R_ee_world^-1

omega_cmd_world = Kp * Log(R_err_world)

omega_cmd_base = R_base_world^-1 * omega_cmd_world

q_dot = J(q)^-1 * omega_cmd_base
```

其中：

```text
q_dot = [pitch_dot, yaw_dot, roll_dot]
```

最后再进入各电机速度环，输出电流目标。

## 实现注意事项

1. 目标世界姿态和电机关节姿态必须分开建模，不能直接逐项相减。
2. 姿态误差建议使用四元数误差再取旋转向量。
3. 需要确认姿态误差旋转向量所在坐标系，再决定是否转换到云台基座坐标系。
4. 底盘 IMU 参与时，必须先合成 `R_ee_world = R_base_world * R_gimbal`。
5. Jacobian 反解使用的角速度必须和 Jacobian 定义处于同一坐标系。
6. 当目标姿态误差接近 180 deg 时，四元数最短路径和控制限幅需要特别处理。
7. F103 上实现三角函数、四元数归一化、矩阵运算时，应优先考虑 CMSIS-DSP 或项目公共数学库。
