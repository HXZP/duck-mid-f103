/**
 * @file uart_command.c
 * @brief 串口业务命令实现。
 */
#include "uart_command.h"

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "uart_protocol.h"

LOG_MODULE_REGISTER(uart_command, LOG_LEVEL_INF);

/** @brief PING 命令。 */
#define UART_CMD_PING 0x0001U

/** @brief 获取固件版本命令。 */
#define UART_CMD_GET_VERSION 0x0002U

/** @brief 固件版本字符串。 */
#define UART_COMMAND_VERSION_TEXT "duck-mid-f103 0.1.0"

/**
 * @brief 处理 PING 命令。
 * @param frame 协议帧。
 * @param user_data 用户参数。
 * @return int 0 表示成功，负值表示失败。
 */
static int uart_command_handle_ping(const struct uart_protocol_frame *frame,
                                    void *user_data)
{
    ARG_UNUSED(user_data);

    return uart_protocol_send_response(frame->cmd,
                                       frame->seq,
                                       0,
                                       frame->payload,
                                       frame->payload_len);
}

/**
 * @brief 处理获取固件版本命令。
 * @param frame 协议帧。
 * @param user_data 用户参数。
 * @return int 0 表示成功，负值表示失败。
 */
static int uart_command_handle_get_version(const struct uart_protocol_frame *frame,
                                           void *user_data)
{
    const uint8_t version[] = UART_COMMAND_VERSION_TEXT;

    ARG_UNUSED(user_data);

    if (frame->payload_len != 0U)
    {
        return uart_protocol_send_response(frame->cmd, frame->seq, -EINVAL, NULL, 0U);
    }

    return uart_protocol_send_response(frame->cmd,
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

int uart_command_init(void)
{
    return uart_protocol_init(uart_command_table, ARRAY_SIZE(uart_command_table));
}
