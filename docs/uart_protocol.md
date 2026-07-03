# UART 协议说明

> 文档版本：`v0.2.0`
>
> 更新日期：`2026-07-04`
>
> 适用固件：`duck-mid-f103 0.2.0`

## 修订记录

| 版本 | 日期 | 说明 |
| --- | --- | --- |
| `v0.2.0` | `2026-07-04` | UART 协议改为 Zephyr driver，补充帧格式、CRC、响应规则和业务命令文档；固件版本响应更新为 `duck-mid-f103 0.2.0` |
| `v0.1.0` | 初始版本 | 提供 `PING` 和 `GET_VERSION` 基础命令 |

## 1. 概述

本项目使用 `hxzp,uart-protocol` driver 管理串口帧收发，当前挂载在 `usart1`，波特率由应用 overlay 配置为 `1000000`。

UART 协议用于上位机和 `duck-mid-f103` 主控通信。业务命令通过命令表注册到协议 driver，协议 driver 负责底层 UART 中断接收、帧解析、CRC 校验和响应帧发送。

当前 UART 线协议帧版本为 `0x01`。本次文档和业务版本升级为 `v0.2.0`，但帧格式没有变化，所以帧头中的 `version` 字段仍保持 `0x01`。

## 2. 设备树配置

当前工程中的协议设备配置如下：

```dts
uart_protocol0: uart-protocol {
	compatible = "hxzp,uart-protocol";
	status = "okay";
	uart = <&usart1>;
	rx-ring-size = <256>;
};
```

字段说明：

| 字段 | 含义 |
| --- | --- |
| `uart` | 底层 UART 控制器 phandle |
| `rx-ring-size` | 接收环形缓冲区大小，单位字节 |
| `echo-rx-raw` | 调试选项，打开后会回显接收到的原始字节，默认关闭 |

## 3. 帧格式

所有多字节字段均使用小端格式。完整帧由帧头、固定头部、负载和 CRC16 组成：

| 字段 | 长度 | 含义 |
| --- | --- | --- |
| `sof1` | 1 字节 | 固定为 `0xA5` |
| `sof2` | 1 字节 | 固定为 `0x5A` |
| `version` | 1 字节 | 线协议版本，当前为 `0x01` |
| `flags` | 1 字节 | 协议标志位，当前请求帧填 `0x00` |
| `cmd` | 2 字节 | 命令号 |
| `seq` | 2 字节 | 帧序号，由请求方维护，响应帧原样回显 |
| `payload_len` | 2 字节 | 负载长度，范围 `0~128` |
| `payload` | `payload_len` 字节 | 业务负载 |
| `crc16` | 2 字节 | CRC16-CCITT 校验值 |

CRC16 计算范围从 `version` 开始，到 `payload` 结束，不包含 `sof1`、`sof2` 和 `crc16` 字段。

CRC16 参数：

| 参数 | 值 |
| --- | --- |
| 多项式 | `0x1021` |
| 初始值 | `0xFFFF` |
| 输出字节序 | 小端 |

## 4. 响应规则

响应命令号为请求命令号按位或 `0x8000`：

```text
rsp_cmd = req_cmd | 0x8000
```

响应帧的 `seq` 与请求帧一致。响应负载固定以 2 字节状态码开头，状态码为 `int16` 小端格式：

| 字节 | 含义 |
| --- | --- |
| Byte0~Byte1 | 状态码，`int16` 小端 |
| Byte2~ByteN | 可选响应数据 |

常用状态码：

| 状态码 | 含义 |
| --- | --- |
| `0` | 成功 |
| `-EINVAL` | 请求参数非法 |
| `-ENOENT` | 命令未注册 |
| 其他负值 | Zephyr errno 错误码 |

## 5. 当前业务命令

### 5.1 PING

- 命令号：`0x0001`
- 方向：上位机 -> 主控
- 请求负载：任意数据，最大 `128` 字节
- 响应负载：`status + 原请求负载`

示例：

```text
REQ cmd=0x0001 seq=1 payload=11 22 33
RSP cmd=0x8001 seq=1 payload=00 00 11 22 33
```

### 5.2 获取固件版本

- 命令号：`0x0002`
- 方向：上位机 -> 主控
- 请求负载：空
- 响应负载：`status + ASCII 版本字符串`

当前版本响应字符串：

```text
duck-mid-f103 0.2.0
```

请求负载非空时，设备返回 `-EINVAL`。

## 6. Driver 使用关系

`uart_command.c` 只注册业务命令，不直接操作 UART 外设。当前初始化流程为：

1. `main.c` 调用 `uart_command_init()`。
2. `uart_command_init()` 获取 `uart_protocol0` 设备。
3. 业务命令表通过 `uart_protocol_register_handlers()` 注册到 UART 协议 driver。
4. UART 协议 driver 在接收到完整合法帧后调用对应命令处理函数。
5. 命令处理函数通过 `uart_protocol_send_response()` 返回响应帧。

这种分层和 CAN 侧保持一致：底层协议 driver 负责收发和分发，业务模块只关心命令语义。
