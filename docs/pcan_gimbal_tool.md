# PCAN 云台调试上位机

## 1. 功能

`scripts/pcan_gimbal_tool.py` 用于通过 PCAN 调试 Duck Mid：

- 接收 `0x201`、`0x202`、`0x203` 三个电机主动上报帧。
- 在表格中显示三个电机的位置、速度、更新时间和收包数量。
- 按当前选中的轴绘制位置和速度折线图。
- 通过 `0x300` 设置当前轴目标角度。
- 通过 `0x300` 请求、`0x301` 响应读取和设置 mid 端云台 PID 参数。
- 支持关闭云台输出。

## 2. 启动

默认使用 PCAN USB1，也就是 `PCAN_USBBUS1 (0x51)`，波特率 `1M`：

```powershell
python project/duck-mid-f103/scripts/pcan_gimbal_tool.py
```

指定通道和波特率：

```powershell
python project/duck-mid-f103/scripts/pcan_gimbal_tool.py --channel PCAN_USBBUS1 --bitrate 1M
```

如果连接时报 `The value of a handle ... is invalid`，说明选择的 PCAN 通道号在当前电脑上不存在。可以先点击界面上的“扫描”，工具会尝试打开 `PCAN_USBBUS1~PCAN_USBBUS16` 并列出可用通道。

## 3. PID 字段

轴编号：

| 名称 | 编号 |
| --- | --- |
| `roll` | `0` |
| `pitch` | `1` |
| `yaw` | `2` |

PID 环：

| 名称 | 编号 |
| --- | --- |
| `position` | `0` |
| `speed` | `1` |

字段：

| 名称 | 编号 | 说明 |
| --- | --- | --- |
| `kp` | `0` | 比例增益 |
| `ki` | `1` | 积分增益 |
| `kd` | `2` | 微分增益 |
| `integral_limit_percent` | `3` | 积分限幅百分比 |
| `output_limit_percent` | `4` | 输出限幅百分比 |

界面上的“读取整组”和“设置整组”会一次处理 `kp`、`ki`、`kd`、`integral_limit_percent`、`output_limit_percent` 五个字段。底层仍按固件协议逐字段发送 CAN 帧。

`integral_limit_percent` 和 `output_limit_percent` 使用百分比输入，例如 `50` 表示 50%。

## 4. 目标和曲线

PID 面板里的轴选择同时决定三个行为：

- “设置目标”发送该轴的目标角度，单位 mrad。
- “读取整组”和“设置整组”操作该轴的 PID。
- 曲线图显示该轴对应电机的位置和速度。
- 曲线图上方的“窗口 s”用于设置最近多少秒的数据参与绘制，范围为 `0.2~120` 秒。

轴和电机节点对应关系：

| 轴 | 电机节点 | 上报 ID |
| --- | --- | --- |
| `pitch` | `0x01` | `0x201` |
| `yaw` | `0x02` | `0x202` |
| `roll` | `0x03` | `0x203` |

## 5. 依赖

该工具只依赖 Python 标准库和 PEAK PCAN-Basic 驱动。运行前需要确保系统已经安装 PEAK 驱动，并且 `PCANBasic.dll` 可以被 Windows 找到。
