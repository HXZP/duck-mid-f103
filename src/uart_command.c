/**
 * @file uart_command.c
 * @brief 串口业务命令实现。
 */
#include "uart_command.h"

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart_protocol.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(uart_command, LOG_LEVEL_INF);

/** @brief PING 命令。 */
#define UART_CMD_PING 0x0001U

/** @brief 获取固件版本命令。 */
#define UART_CMD_GET_VERSION 0x0002U

/** @brief 固件协议版本字符串。 */
#define UART_COMMAND_PROTOCOL_VERSION_TEXT "0.2.0"

/** @brief 固件版本字符串。 */
#define UART_COMMAND_VERSION_TEXT "duck-mid-f103 " UART_COMMAND_PROTOCOL_VERSION_TEXT

/** @brief 串口协议设备节点。 */
#define UART_COMMAND_PROTOCOL_NODE DT_NODELABEL(uart_protocol0)

/** @brief 串口协议设备。 */
static const struct device *const uart_command_protocol_dev =
    DEVICE_DT_GET(UART_COMMAND_PROTOCOL_NODE);

/**
 * @brief 处理 PING 命令。
 * @param dev 串口协议设备。
 * @param frame 协议帧。
 * @param user_data 用户参数。
 * @return int 0 表示成功，负值表示失败。
 */
static int uart_command_handle_ping(const struct device *dev,
                                    const struct uart_protocol_frame *frame,
                                    void *user_data)
{
    ARG_UNUSED(user_data);

    return uart_protocol_send_response(dev,
                                       frame->cmd,
                                       frame->seq,
                                       0,
                                       frame->payload,
                                       frame->payload_len);
}

/**
 * @brief 处理获取固件版本命令。
 * @param dev 串口协议设备。
 * @param frame 协议帧。
 * @param user_data 用户参数。
 * @return int 0 表示成功，负值表示失败。
 */
static int uart_command_handle_get_version(const struct device *dev,
                                           const struct uart_protocol_frame *frame,
                                           void *user_data)
{
    const uint8_t version[] = UART_COMMAND_VERSION_TEXT;

    ARG_UNUSED(user_data);

    if (frame->payload_len != 0U)
    {
        return uart_protocol_send_response(dev, frame->cmd, frame->seq, -EINVAL, NULL, 0U);
    }

    return uart_protocol_send_response(dev,
                                       frame->cmd,
                                       frame->seq,
                                       0,
                                       version,
                                       (uint16_t)strlen((const char *)version));
}

/** @brief 串口业务命令表。 */
static const struct uart_protocol_cmd_entry uart_command_table[] = {
    {
        .cmd = UART_CMD_PING,
        .handler = uart_command_handle_ping,
        .user_data = NULL,
    },
    {
        .cmd = UART_CMD_GET_VERSION,
        .handler = uart_command_handle_get_version,
        .user_data = NULL,
    },
};

/**
 * @brief 初始化串口业务命令。
 * @return int 0 表示成功，负值表示失败。
 */
int uart_command_init(void)
{
    if (!device_is_ready(uart_command_protocol_dev))
    {
        return -ENODEV;
    }

    return uart_protocol_register_handlers(uart_command_protocol_dev,
                                           uart_command_table,
                                           ARRAY_SIZE(uart_command_table));
}
