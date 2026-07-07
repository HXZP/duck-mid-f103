# 云台世界姿态控制理论说明

## 目标

本文记录云台使用世界坐标系姿态目标时的控制链路和数学原理。

需要区分三类量：

```text
世界姿态目标：描述末端在世界坐标系中的目标姿态
末端姿态误差：描述末端当前姿态到目标姿态还差的空间旋转
电机关节角：描述 Pitch/Yaw/Roll 三个电机轴各自转了多少
```

这三类量不能直接逐项相减。也就是说，不能把：

```text
world_roll  - motor_roll
world_pitch - motor_pitch
world_yaw   - motor_yaw
```

直接当作姿态误差。正确做法是：

```text
目标世界姿态和当前世界姿态
    -> 姿态误差
    -> 末端角速度目标
    -> 机构反解
    -> 电机速度目标
```

## 符号约定

坐标系符号：

```text
W = world，世界坐标系
C = chassis/base，底盘或云台基座坐标系
E = end effector，云台末端坐标系
T = target，目标姿态坐标系
```

姿态符号：

```text
R      = 旋转矩阵
q_att  = 姿态四元数
q_err  = 姿态误差四元数
e      = 姿态误差旋转向量
omega  = 角速度
```

电机关节角不要写成 `q`，避免和四元数混淆。本文统一写成：

$$
\boldsymbol{\theta}_{motor}
=
\begin{bmatrix}
\theta_{pitch} \\
\theta_{yaw} \\
\theta_{roll}
\end{bmatrix}
$$

电机速度写成：

$$
\dot{\boldsymbol{\theta}}_{motor}
=
\begin{bmatrix}
\dot{\theta}_{pitch} \\
\dot{\theta}_{yaw} \\
\dot{\theta}_{roll}
\end{bmatrix}
$$

当前约定实机使用 Z-up 右手坐标系：

```text
X = 前向
Y = 侧向
Z = 垂直地面
```

矩阵采用列向量约定：

$$
\mathbf{v}_{world}
=
\mathbf{R}
\mathbf{v}_{body}
$$

矩阵连乘时，右侧矩阵先作用到向量。

## 主体推导

### 1. 当前末端世界姿态

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

所以云台相对基座的机构姿态为：

$$
{}^{C}\mathbf{R}_{E}
=
\mathbf{R}_{y}(\theta_{pitch})
\mathbf{R}_{z}(\theta_{yaw})
\mathbf{R}_{x}(\theta_{roll})
$$

如果底盘不动，可以近似认为：

$$
{}^{W}\mathbf{R}_{E}
\approx
{}^{C}\mathbf{R}_{E}
$$

如果底盘会动，并且底盘 IMU 给出底盘世界姿态：

$$
{}^{W}\mathbf{R}_{C}
=
\mathbf{R}_{base\_world}
$$

则末端真实世界姿态为：

$$
{}^{W}\mathbf{R}_{E}
=
{}^{W}\mathbf{R}_{C}
{}^{C}\mathbf{R}_{E}
$$

也就是：

```text
末端世界姿态 = 底盘世界姿态 * 云台相对底盘姿态
```

### 2. 世界姿态目标

世界坐标系下的 Roll/Pitch/Yaw 目标不是电机目标，而是末端最终应该处于的世界姿态。

目标姿态矩阵为：

$$
{}^{W}\mathbf{R}_{T}
=
\mathbf{R}_{z}(world\_yaw)
\mathbf{R}_{y}(world\_pitch)
\mathbf{R}_{x}(world\_roll)
$$

这里采用的是 ZYX / yaw-pitch-roll 欧拉角语义：

```text
先定义绕世界 Z 轴的 yaw
再定义绕 yaw 后局部 Y 轴的 pitch
最后定义绕 yaw + pitch 后局部 X 轴的 roll
```

数学表达写成 `Rz * Ry * Rx`。在列向量矩阵乘法中，矩阵作用到向量时是右侧先计算，所以如果看“一个向量被这个总旋转矩阵变换”的计算过程，会表现为先 `Rx`，再 `Ry`，最后 `Rz`。

因此需要区分两个说法：

```text
欧拉角姿态语义：ZYX，也就是 yaw -> pitch -> roll
矩阵乘向量计算顺序：XYZ，也就是 roll -> pitch -> yaw
```

二者描述的是同一个姿态矩阵，不是两套不同的目标定义。

例如：

```text
world_roll  = 0 deg
world_pitch = 30 deg
world_yaw   = 任意角度
```

含义是：

```text
末端相对世界坐标系保持 roll = 0 deg
末端相对世界坐标系保持 pitch = 30 deg
末端相对世界坐标系跟随 yaw 目标旋转
```

这时 Roll 电机角不一定等于 0，因为 Roll 电机是在 Pitch/Yaw 之后的局部 X 轴上转动，控制器需要通过机构反解自动补偿。

### 3. 当前姿态到目标姿态的误差旋转

已知当前末端世界姿态：

$$
{}^{W}\mathbf{R}_{E}
$$

已知目标世界姿态：

$$
{}^{W}\mathbf{R}_{T}
$$

我们要找一个误差旋转矩阵：

$$
\mathbf{R}_{err}
$$

它的含义是：

```text
末端在世界坐标下，从当前姿态转到目标姿态，还需要补上的空间旋转
```

也就是：

$$
{}^{W}\mathbf{R}_{T}
=
\mathbf{R}_{err}
{}^{W}\mathbf{R}_{E}
$$

两边右乘当前姿态的逆矩阵：

$$
{}^{W}\mathbf{R}_{T}
\left({}^{W}\mathbf{R}_{E}\right)^{-1}
=
\mathbf{R}_{err}
{}^{W}\mathbf{R}_{E}
\left({}^{W}\mathbf{R}_{E}\right)^{-1}
$$

由于：

$$
{}^{W}\mathbf{R}_{E}
\left({}^{W}\mathbf{R}_{E}\right)^{-1}
=
\mathbf{I}
$$

所以：

$$
\mathbf{R}_{err}
=
{}^{W}\mathbf{R}_{T}
\left({}^{W}\mathbf{R}_{E}\right)^{-1}
$$

旋转矩阵的逆等于转置：

$$
\left({}^{W}\mathbf{R}_{E}\right)^{-1}
=
\left({}^{W}\mathbf{R}_{E}\right)^{T}
$$

最终得到：

$$
\mathbf{R}_{err}
=
{}^{W}\mathbf{R}_{T}
\left({}^{W}\mathbf{R}_{E}\right)^{T}
$$

如果当前姿态已经等于目标姿态：

$$
{}^{W}\mathbf{R}_{T}
=
{}^{W}\mathbf{R}_{E}
$$

则：

$$
\mathbf{R}_{err}
=
{}^{W}\mathbf{R}_{E}
\left({}^{W}\mathbf{R}_{E}\right)^{T}
=
\mathbf{I}
$$

说明没有姿态误差。

单轴例子：

$$
{}^{W}\mathbf{R}_{E}
=
\mathbf{R}_{z}(30^\circ)
$$

$$
{}^{W}\mathbf{R}_{T}
=
\mathbf{R}_{z}(80^\circ)
$$

代入误差公式：

$$
\mathbf{R}_{err}
=
\mathbf{R}_{z}(80^\circ)
\mathbf{R}_{z}(30^\circ)^{-1}
$$

由于：

$$
\mathbf{R}_{z}(30^\circ)^{-1}
=
\mathbf{R}_{z}(-30^\circ)
$$

所以：

$$
\mathbf{R}_{err}
=
\mathbf{R}_{z}(80^\circ)
\mathbf{R}_{z}(-30^\circ)
=
\mathbf{R}_{z}(50^\circ)
$$

也就是从当前姿态到目标姿态还差 50 deg。

注意：`R_err` 不是电机角度，也不是三个电机分别应该转多少。它只是末端整体在空间中还差的旋转。

### 4. 误差矩阵转换成误差四元数

姿态误差可以用旋转矩阵表示，也可以用四元数表示。二者描述的是同一个旋转，只是表示形式不同。

如果前面已经算出了：

$$
\mathbf{R}_{err}
$$

则可以把它转换成四元数：

$$
\mathbf{q}_{err}
=
Quaternion(\mathbf{R}_{err})
$$

这里的 `q_err` 是姿态误差四元数，和电机关节角没有关系。

如果当前姿态和目标姿态一开始就是四元数形式，也可以直接算：

$$
\mathbf{q}_{err}
=
\mathbf{q}_{target\_world}
\otimes
\mathbf{q}_{current\_world}^{-1}
$$

这和矩阵形式是同一个含义：

$$
\mathbf{R}_{err}
=
\mathbf{R}_{target\_world}
\mathbf{R}_{current\_world}^{-1}
$$

实际固件里通常更适合使用四元数做姿态误差，因为四元数计算量较小，也更容易做最短路径处理。

### 5. 误差四元数转换成旋转向量

误差四元数不能直接作为 PID 的三轴误差使用，需要转换成旋转向量：

$$
\mathbf{e}
=
\log(\mathbf{q}_{err})
$$

旋转向量的含义是：

$$
\mathbf{e}
=
\mathbf{axis}\theta
$$

其中：

```text
axis  = 旋转轴，单位向量
theta = 绕该旋转轴需要转动的角度，单位 rad
```

所以 `e` 是一个三维向量：

$$
\mathbf{e}
=
\begin{bmatrix}
e_x \\
e_y \\
e_z
\end{bmatrix}
$$

它的方向是旋转轴，长度是旋转角：

$$
\|\mathbf{e}\|
=
\theta
$$

如果 `R_err` 是左乘误差：

$$
\mathbf{R}_{err}
=
\mathbf{R}_{target}
\mathbf{R}_{current}^{-1}
$$

则得到的 `e` 表达在世界坐标系或基座方向上。后续进入 Jacobian 前，需要确认它和 Jacobian 使用的坐标系一致。

### 6. 姿态误差转换成末端角速度目标

角度环可以把旋转向量误差转换成末端角速度目标：

$$
\boldsymbol{\omega}_{cmd\_world}
=
K_p
\mathbf{e}_{world}
$$

其中：

```text
e_world 的单位             = rad
Kp 的单位                  = 1/s
omega_cmd_world 的单位     = rad/s
```

如果底盘不动，且 Jacobian 也定义在同一基座坐标系下，可以近似认为：

$$
\boldsymbol{\omega}_{cmd\_base}
\approx
\boldsymbol{\omega}_{cmd\_world}
$$

如果底盘会动，且姿态误差是在世界坐标系下得到的，则需要转换到底盘/基座坐标系：

$$
\boldsymbol{\omega}_{cmd\_base}
=
\left({}^{W}\mathbf{R}_{C}\right)^T
\boldsymbol{\omega}_{cmd\_world}
$$

这一步的目的很简单：

```text
姿态误差给的是末端在世界方向上应该怎么转
Jacobian 要的是末端在基座方向上应该怎么转
所以必须先把角速度目标转到基座坐标系
```

### 7. 机构反解：末端角速度到电机速度

机构反解不是四元数计算。这里已经离开姿态表示问题，进入机构速度映射问题。

电机角为：

$$
\boldsymbol{\theta}_{motor}
=
\begin{bmatrix}
\theta_{pitch} \\
\theta_{yaw} \\
\theta_{roll}
\end{bmatrix}
$$

电机速度为：

$$
\dot{\boldsymbol{\theta}}_{motor}
=
\begin{bmatrix}
\dot{\theta}_{pitch} \\
\dot{\theta}_{yaw} \\
\dot{\theta}_{roll}
\end{bmatrix}
$$

Jacobian 描述的是：

```text
三个电机轴分别以一定速度转动时，末端在基座坐标系下会产生什么角速度
```

数学形式为：

$$
\boldsymbol{\omega}_{base}
=
\mathbf{J}(\boldsymbol{\theta}_{motor})
\dot{\boldsymbol{\theta}}_{motor}
$$

控制时已知的是末端角速度目标：

$$
\boldsymbol{\omega}_{cmd\_base}
$$

要求的是三个电机速度，所以反过来：

$$
\dot{\boldsymbol{\theta}}_{motor}
=
\mathbf{J}(\boldsymbol{\theta}_{motor})^{-1}
\boldsymbol{\omega}_{cmd\_base}
$$

这里的 `theta_motor` 是电机关节角，不是四元数。

对 Pitch -> Yaw -> Roll 机构，三个电机轴在基座坐标系下的方向为：

$$
\mathbf{a}_{pitch}
=
\begin{bmatrix}
0 \\
1 \\
0
\end{bmatrix}
$$

$$
\mathbf{a}_{yaw}
=
\mathbf{R}_{y}(\theta_{pitch})
\begin{bmatrix}
0 \\
0 \\
1
\end{bmatrix}
=
\begin{bmatrix}
\sin\theta_{pitch} \\
0 \\
\cos\theta_{pitch}
\end{bmatrix}
$$

$$
\mathbf{a}_{roll}
=
\mathbf{R}_{y}(\theta_{pitch})
\mathbf{R}_{z}(\theta_{yaw})
\begin{bmatrix}
1 \\
0 \\
0
\end{bmatrix}
=
\begin{bmatrix}
\cos\theta_{pitch}\cos\theta_{yaw} \\
\sin\theta_{yaw} \\
-\sin\theta_{pitch}\cos\theta_{yaw}
\end{bmatrix}
$$

所以：

$$
\boldsymbol{\omega}_{base}
=
\mathbf{a}_{pitch}\dot{\theta}_{pitch}
+
\mathbf{a}_{yaw}\dot{\theta}_{yaw}
+
\mathbf{a}_{roll}\dot{\theta}_{roll}
$$

对应 Jacobian 为：

$$
\mathbf{J}(\boldsymbol{\theta}_{motor})
=
\begin{bmatrix}
0 & \sin\theta_{pitch} & \cos\theta_{pitch}\cos\theta_{yaw} \\
1 & 0 & \sin\theta_{yaw} \\
0 & \cos\theta_{pitch} & -\sin\theta_{pitch}\cos\theta_{yaw}
\end{bmatrix}
$$

列顺序为：

```text
第 1 列 = Pitch 轴方向
第 2 列 = Yaw 轴方向
第 3 列 = Roll 轴方向
```

若：

$$
\boldsymbol{\omega}_{cmd\_base}
=
\begin{bmatrix}
\omega_x \\
\omega_y \\
\omega_z
\end{bmatrix}
$$

则一个展开反解形式为：

$$
\dot{\theta}_{roll}
=
\frac{
\cos\theta_{pitch}\omega_x
-
\sin\theta_{pitch}\omega_z
}{
\cos\theta_{yaw}
}
$$

$$
\dot{\theta}_{yaw}
=
\sin\theta_{pitch}\omega_x
+
\cos\theta_{pitch}\omega_z
$$

$$
\dot{\theta}_{pitch}
=
\omega_y
-
\sin\theta_{yaw}
\dot{\theta}_{roll}
$$

当：

$$
\cos\theta_{yaw}
\approx
0
$$

反解会接近奇异，需要限幅、阻尼最小二乘或切换控制策略。

### 8. 完整控制链路总结

底盘会动时，推荐控制链路为：

```text
底盘 IMU 姿态
    -> R_base_world

电机角 theta_motor
    -> R_gimbal

R_current_world = R_base_world * R_gimbal

世界目标姿态
    -> R_target_world

R_err = R_target_world * R_current_world^-1

q_err = Quaternion(R_err)

e_world = Log(q_err)

omega_cmd_world = Kp * e_world

omega_cmd_base = R_base_world^T * omega_cmd_world

theta_motor_dot = J(theta_motor)^-1 * omega_cmd_base

theta_motor_dot
    -> 各电机速度环
    -> 各电机电流环
```

简化公式为：

$$
{}^{W}\mathbf{R}_{E}
=
{}^{W}\mathbf{R}_{C}
{}^{C}\mathbf{R}_{E}
$$

$$
\mathbf{R}_{err}
=
{}^{W}\mathbf{R}_{T}
\left({}^{W}\mathbf{R}_{E}\right)^{-1}
$$

$$
\mathbf{q}_{err}
=
Quaternion(\mathbf{R}_{err})
$$

$$
\mathbf{e}_{world}
=
\log(\mathbf{q}_{err})
$$

$$
\boldsymbol{\omega}_{cmd\_world}
=
K_p
\mathbf{e}_{world}
$$

$$
\boldsymbol{\omega}_{cmd\_base}
=
\left({}^{W}\mathbf{R}_{C}\right)^T
\boldsymbol{\omega}_{cmd\_world}
$$

$$
\dot{\boldsymbol{\theta}}_{motor}
=
\mathbf{J}(\boldsymbol{\theta}_{motor})^{-1}
\boldsymbol{\omega}_{cmd\_base}
$$

## 数学原理补充

### 旋转矩阵的含义

姿态矩阵经常写成：

$$
{}^{W}\mathbf{R}_{E}
$$

它读作：

```text
末端坐标系 E 相对于世界坐标系 W 的姿态
```

`{}^{W}R_E` 是一个 3x3 矩阵：

$$
{}^{W}\mathbf{R}_{E}
=
\begin{bmatrix}
| & | & | \\
{}^{W}\mathbf{x}_{E} & {}^{W}\mathbf{y}_{E} & {}^{W}\mathbf{z}_{E} \\
| & | & |
\end{bmatrix}
$$

也就是：

$$
{}^{W}\mathbf{R}_{E}
=
\begin{bmatrix}
r_{11} & r_{12} & r_{13} \\
r_{21} & r_{22} & r_{23} \\
r_{31} & r_{32} & r_{33}
\end{bmatrix}
$$

三列分别表示：

```text
第 1 列 = 末端 X 轴在世界坐标系中的方向
第 2 列 = 末端 Y 轴在世界坐标系中的方向
第 3 列 = 末端 Z 轴在世界坐标系中的方向
```

如果有一个向量 `v_E` 是在末端坐标系中描述的，则转换到世界坐标系：

$$
\mathbf{v}_{W}
=
{}^{W}\mathbf{R}_{E}
\mathbf{v}_{E}
$$

如果有一个向量 `v_W` 是在世界坐标系中描述的，则转换到末端坐标系：

$$
\mathbf{v}_{E}
=
\left({}^{W}\mathbf{R}_{E}\right)^T
\mathbf{v}_{W}
$$

这里用到了旋转矩阵的性质：

$$
\mathbf{R}^{-1}
=
\mathbf{R}^{T}
$$

### 基础旋转矩阵

绕 X 轴旋转 Roll 角 `r`：

$$
\mathbf{R}_{x}(r)
=
\begin{bmatrix}
1 & 0 & 0 \\
0 & \cos r & -\sin r \\
0 & \sin r & \cos r
\end{bmatrix}
$$

绕 Y 轴旋转 Pitch 角 `p`：

$$
\mathbf{R}_{y}(p)
=
\begin{bmatrix}
\cos p & 0 & \sin p \\
0 & 1 & 0 \\
-\sin p & 0 & \cos p
\end{bmatrix}
$$

绕 Z 轴旋转 Yaw 角 `y`：

$$
\mathbf{R}_{z}(y)
=
\begin{bmatrix}
\cos y & -\sin y & 0 \\
\sin y & \cos y & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

### 四元数到旋转向量

设误差四元数为：

$$
\mathbf{q}_{err}
=
\begin{bmatrix}
w & x & y & z
\end{bmatrix}
$$

先选择最短旋转。如果：

$$
w < 0
$$

则取：

$$
\mathbf{q}_{err}
=
-\mathbf{q}_{err}
$$

然后：

$$
sin\_half
=
\sqrt{x^2 + y^2 + z^2}
$$

$$
\theta
=
2\operatorname{atan2}(sin\_half, w)
$$

$$
\mathbf{axis}
=
\frac{1}{sin\_half}
\begin{bmatrix}
x \\
y \\
z
\end{bmatrix}
$$

$$
\mathbf{e}
=
\mathbf{axis}
\theta
$$

当 `sin_half` 很小时，说明误差接近 0，旋转向量可以直接置 0 或使用小角度近似。

### Log 和 Exp

`Log(R_err)` 或 `Log(q_err)` 的作用是：

```text
把一个姿态误差旋转转换成旋转向量 e
```

旋转向量为：

$$
\mathbf{e}
=
\mathbf{axis}
\theta
$$

`Exp` 是反过程：

```text
把旋转向量 e 转回旋转矩阵或四元数
```

对于旋转矩阵：

$$
\log(\mathbf{R}_{err})
=
[\mathbf{e}]_{\times}
$$

其中：

$$
[\mathbf{e}]_{\times}
=
\begin{bmatrix}
0 & -e_z & e_y \\
e_z & 0 & -e_x \\
-e_y & e_x & 0
\end{bmatrix}
$$

`[e]x` 是叉乘矩阵，它满足：

$$
[\mathbf{e}]_{\times}
\mathbf{v}
=
\mathbf{e}
\times
\mathbf{v}
$$

如果：

$$
\mathbf{R}_{err}
=
\begin{bmatrix}
R_{11} & R_{12} & R_{13} \\
R_{21} & R_{22} & R_{23} \\
R_{31} & R_{32} & R_{33}
\end{bmatrix}
$$

先计算误差旋转角：

$$
\theta
=
\arccos\left(
\frac{R_{11}+R_{22}+R_{33}-1}{2}
\right)
$$

再计算误差旋转向量：

$$
\mathbf{e}
=
\frac{\theta}{2\sin\theta}
\begin{bmatrix}
R_{32}-R_{23} \\
R_{13}-R_{31} \\
R_{21}-R_{12}
\end{bmatrix}
$$

当 `theta` 很小时，可以使用小角度近似：

$$
\mathbf{e}
\approx
\frac{1}{2}
\begin{bmatrix}
R_{32}-R_{23} \\
R_{13}-R_{31} \\
R_{21}-R_{12}
\end{bmatrix}
$$

### 左乘误差和右乘误差

本文主体使用的是左乘误差：

$$
\mathbf{R}_{err\_world}
=
\mathbf{R}_{target}
\mathbf{R}_{current}^{-1}
$$

它表达的是：

```text
在世界/基座方向上，还需要补一个什么旋转
```

另一种常见写法是右乘误差：

$$
\mathbf{R}_{err\_body}
=
\mathbf{R}_{current}^{-1}
\mathbf{R}_{target}
$$

它表达的是：

```text
在当前末端自身坐标系方向上，还需要补一个什么旋转
```

二者都可以使用，但后续必须和 Jacobian 的坐标系一致。

### 和 Mahony 叉乘误差的关系

Mahony 姿态解算中常见的误差形式为：

$$
\mathbf{e}
=
\mathbf{v}_{measured}
\times
\mathbf{v}_{estimated}
$$

它表示两个方向向量之间的最小旋转方向。小角度时：

$$
\mathbf{v}_{measured}
\times
\mathbf{v}_{estimated}
\approx
姿态误差向量
$$

叉乘误差和 `Log(q_err)` 是同一类思想。

差异在于：

```text
单个方向向量叉乘：只能约束部分姿态，例如重力向量只能约束 roll/pitch
完整四元数误差：可以约束完整三轴姿态，包括 yaw
```

云台世界姿态控制建议使用完整姿态误差：

$$
\mathbf{e}
=
\log\left(
\mathbf{q}_{target}
\otimes
\mathbf{q}_{current}^{-1}
\right)
$$

而不是只用单个向量叉乘。

## 实现注意事项

1. 目标世界姿态和电机关节姿态必须分开建模，不能直接逐项相减。
2. `R_err` 是末端姿态误差，不是电机角度。
3. `q_err` 是姿态误差四元数，不要和电机关节角混用同一个符号。
4. 电机关节角建议统一写成 `theta_motor`。
5. 姿态误差建议使用四元数误差再取旋转向量。
6. 需要确认姿态误差旋转向量所在坐标系，再决定是否转换到云台基座坐标系。
7. 底盘 IMU 参与时，必须先合成末端世界姿态。
8. Jacobian 反解使用的角速度必须和 Jacobian 定义处于同一坐标系。
9. 当目标姿态误差接近 180 deg 时，四元数最短路径和控制限幅需要特别处理。
10. 当 `cos(theta_yaw)` 接近 0 时，Pitch/Yaw/Roll 速度反解接近奇异。
11. F103 上实现三角函数、四元数归一化、矩阵运算时，应优先考虑 CMSIS-DSP 或项目公共数学库。
