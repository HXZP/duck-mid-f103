/**
 * @file uart_command.c
 * @brief 串口业务命令实现。
 */
#include "uart_command.h"

#include "gimbal.h"

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart_protocol.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(uart_command, LOG_LEVEL_INF);

/** @brief PING 命令。 */
#define UART_CMD_PING 0x0001U

/** @brief 获取固件版本命令。 */
#define UART_CMD_GET_VERSION 0x0002U

/** @brief 设置云台 RPY 目标角度命令。 */
#define UART_CMD_SET_GIMBAL_RPY_TARGET 0x0101U

/** @brief 设置 RK 当前姿态反馈命令。 */
#define UART_CMD_SET_GIMBAL_ATTITUDE 0x0102U

/** @brief 同时设置云台 RPY 目标和 RK 当前姿态反馈命令。 */
#define UART_CMD_SET_GIMBAL_RPY_STATE 0x0103U

/** @brief 固件协议版本字符串。 */
#define UART_COMMAND_PROTOCOL_VERSION_TEXT "0.3.0"

/** @brief 固件版本字符串。 */
#define UART_COMMAND_VERSION_TEXT "duck-mid-f103 " UART_COMMAND_PROTOCOL_VERSION_TEXT

/** @brief RPY 三轴角度负载长度，单位字节。 */
#define UART_COMMAND_RPY_PAYLOAD_LEN 12U

/** @brief RPY 目标和姿态组合负载长度，单位字节。 */
#define UART_COMMAND_RPY_STATE_PAYLOAD_LEN 24U

/** @brief 串口协议设备节点。 */
#define UART_COMMAND_PROTOCOL_NODE DT_NODELABEL(uart_protocol0)

/** @brief 串口协议设备。 */
static const struct device *const uart_command_protocol_dev =
    DEVICE_DT_GET(UART_COMMAND_PROTOCOL_NODE);

/**
 * @brief 解析 RPY 三轴角度负载。
 * @param payload 负载数据。
 * @param payload_len 负载长度，单位字节。
 * @param roll_mrad 输出 Roll 角度，单位 mrad。
 * @param pitch_mrad 输出 Pitch 角度，单位 mrad。
 * @param yaw_mrad 输出 Yaw 角度，单位 mrad。
 * @return int 0 表示成功，负值表示失败。
 */
static int uart_command_parse_rpy_payload(const uint8_t *payload,
                                          uint16_t payload_len,
                                          int32_t *roll_mrad,
                                          int32_t *pitch_mrad,
                                          int32_t *yaw_mrad)
{
    if ((payload == NULL) ||
        (roll_mrad == NULL) ||
        (pitch_mrad == NULL) ||
        (yaw_mrad == NULL))
    {
        return -EINVAL;
    }

    if (payload_len < UART_COMMAND_RPY_PAYLOAD_LEN)
    {
        return -EMSGSIZE;
    }

    *roll_mrad = (int32_t)sys_get_le32(&payload[0]);
    *pitch_mrad = (int32_t)sys_get_le32(&payload[4]);
    *yaw_mrad = (int32_t)sys_get_le32(&payload[8]);

    return 0;
}

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
        return uart_protocol_send_response(dev,
                                           frame->cmd,
                                           frame->seq,
                                           -EINVAL,
                                           NULL,
                                           0U);
    }

    return uart_protocol_send_response(dev,
                                       frame->cmd,
                                       frame->seq,
                                       0,
                                       version,
                                       (uint16_t)strlen((const char *)version));
}

/**
 * @brief 处理设置云台 RPY 目标命令。
 * @param dev 串口协议设备。
 * @param frame 协议帧。
 * @param user_data 用户参数。
 * @return int 0 表示成功，负值表示失败。
 */
static int uart_command_handle_set_gimbal_target(const struct device *dev,
                                                 const struct uart_protocol_frame *frame,
                                                 void *user_data)
{
    int32_t roll_mrad;
    int32_t pitch_mrad;
    int32_t yaw_mrad;
    int ret;

    ARG_UNUSED(user_data);

    if (frame->payload_len != UART_COMMAND_RPY_PAYLOAD_LEN)
    {
        ret = -EINVAL;
    }
    else
    {
        ret = uart_command_parse_rpy_payload(frame->payload,
                                             frame->payload_len,
                                             &roll_mrad,
                                             &pitch_mrad,
                                             &yaw_mrad);

        if (ret == 0)
        {
            ret = gimbal_set_rpy_target(roll_mrad,
                                        pitch_mrad,
                                        yaw_mrad,
                                        GIMBAL_TARGET_SOURCE_RK_UART);
        }
    }

    return uart_protocol_send_response(dev,
                                       frame->cmd,
                                       frame->seq,
                                       ret,
                                       NULL,
                                       0U);
}

/**
 * @brief 处理设置 RK 当前姿态反馈命令。
 * @param dev 串口协议设备。
 * @param frame 协议帧。
 * @param user_data 用户参数。
 * @return int 0 表示成功，负值表示失败。
 */
static int uart_command_handle_set_gimbal_attitude(const struct device *dev,
                                                   const struct uart_protocol_frame *frame,
                                                   void *user_data)
{
    int32_t roll_mrad;
    int32_t pitch_mrad;
    int32_t yaw_mrad;
    int ret;

    ARG_UNUSED(user_data);

    if (frame->payload_len != UART_COMMAND_RPY_PAYLOAD_LEN)
    {
        ret = -EINVAL;
    }
    else
    {
        ret = uart_command_parse_rpy_payload(frame->payload,
                                             frame->payload_len,
                                             &roll_mrad,
                                             &pitch_mrad,
                                             &yaw_mrad);

        if (ret == 0)
        {
            ret = gimbal_set_attitude_feedback(roll_mrad,
                                               pitch_mrad,
                                               yaw_mrad);
        }
    }

    return uart_protocol_send_response(dev,
                                       frame->cmd,
                                       frame->seq,
                                       ret,
                                       NULL,
                                       0U);
}

/**
 * @brief 处理同时设置云台 RPY 目标和 RK 当前姿态反馈命令。
 * @param dev 串口协议设备。
 * @param frame 协议帧。
 * @param user_data 用户参数。
 * @return int 0 表示成功，负值表示失败。
 */
static int uart_command_handle_set_gimbal_state(const struct device *dev,
                                                const struct uart_protocol_frame *frame,
                                                void *user_data)
{
    int32_t target_roll_mrad;
    int32_t target_pitch_mrad;
    int32_t target_yaw_mrad;
    int32_t attitude_roll_mrad;
    int32_t attitude_pitch_mrad;
    int32_t attitude_yaw_mrad;
    int ret;

    ARG_UNUSED(user_data);

    if (frame->payload_len != UART_COMMAND_RPY_STATE_PAYLOAD_LEN)
    {
        ret = -EINVAL;
    }
    else
    {
        ret = uart_command_parse_rpy_payload(frame->payload,
                                             UART_COMMAND_RPY_PAYLOAD_LEN,
                                             &target_roll_mrad,
                                             &target_pitch_mrad,
                                             &target_yaw_mrad);

        if (ret == 0)
        {
            ret = uart_command_parse_rpy_payload(&frame->payload[UART_COMMAND_RPY_PAYLOAD_LEN],
                                                 UART_COMMAND_RPY_PAYLOAD_LEN,
                                                 &attitude_roll_mrad,
                                                 &attitude_pitch_mrad,
                                                 &attitude_yaw_mrad);
        }

        if (ret == 0)
        {
            ret = gimbal_set_attitude_feedback(attitude_roll_mrad,
                                               attitude_pitch_mrad,
                                               attitude_yaw_mrad);
        }

        if (ret == 0)
        {
            ret = gimbal_set_rpy_target(target_roll_mrad,
                                        target_pitch_mrad,
                                        target_yaw_mrad,
                                        GIMBAL_TARGET_SOURCE_RK_UART);
        }
    }

    return uart_protocol_send_response(dev,
                                       frame->cmd,
                                       frame->seq,
                                       ret,
                                       NULL,
                                       0U);
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
    {
        .cmd = UART_CMD_SET_GIMBAL_RPY_TARGET,
        .handler = uart_command_handle_set_gimbal_target,
        .user_data = NULL,
    },
    {
        .cmd = UART_CMD_SET_GIMBAL_ATTITUDE,
        .handler = uart_command_handle_set_gimbal_attitude,
        .user_data = NULL,
    },
    {
        .cmd = UART_CMD_SET_GIMBAL_RPY_STATE,
        .handler = uart_command_handle_set_gimbal_state,
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
