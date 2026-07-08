/**
 * @file can_command.c
 * @brief CAN 业务调试命令实现。
 */
#include "can_command.h"

#include "gimbal.h"

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can_protocol.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(can_command, LOG_LEVEL_INF);

/** @brief mid 端云台调试请求 CAN ID。 */
#define CAN_COMMAND_GIMBAL_REQUEST_ID 0x300U

/** @brief mid 端云台调试响应 CAN ID。 */
#define CAN_COMMAND_GIMBAL_RESPONSE_ID 0x301U

/** @brief CAN 业务调试响应负载长度。 */
#define CAN_COMMAND_RESPONSE_DLC 8U

/** @brief CAN 业务调试命令最小负载长度。 */
#define CAN_COMMAND_MIN_DLC 1U

/** @brief CAN 协议设备节点。 */
#define CAN_COMMAND_PROTOCOL_NODE DT_NODELABEL(can_protocol0)

/** @brief PID 字段编号中 loop 字段偏移。 */
#define CAN_COMMAND_PID_LOOP_SHIFT 4U

/** @brief PID 字段编号中 field 字段掩码。 */
#define CAN_COMMAND_PID_FIELD_MASK 0x0FU

/**
 * @brief CAN 业务调试命令码。
 */
enum can_command_gimbal_cmd
{
    /**< 设置单轴目标角度。 */
    CAN_COMMAND_GIMBAL_CMD_SET_AXIS_TARGET = 0x01,
    /**< 关闭云台输出。 */
    CAN_COMMAND_GIMBAL_CMD_DISABLE = 0x02,
    /**< 设置 PID 单个字段。 */
    CAN_COMMAND_GIMBAL_CMD_SET_PID_FIELD = 0x10,
    /**< 读取 PID 单个字段。 */
    CAN_COMMAND_GIMBAL_CMD_GET_PID_FIELD = 0x11,
};

/**
 * @brief CAN 业务调试状态码。
 */
enum can_command_status
{
    /**< 命令成功。 */
    CAN_COMMAND_STATUS_OK = 0x00,
    /**< 命令非法。 */
    CAN_COMMAND_STATUS_INVALID_CMD = 0x01,
    /**< 参数非法。 */
    CAN_COMMAND_STATUS_INVALID_PARAM = 0x02,
    /**< 执行失败。 */
    CAN_COMMAND_STATUS_EXEC_FAILED = 0x03,
};

/**
 * @brief PID 可调字段编号。
 */
enum can_command_pid_field
{
    /**< 比例增益。 */
    CAN_COMMAND_PID_FIELD_KP = 0x00,
    /**< 积分增益。 */
    CAN_COMMAND_PID_FIELD_KI = 0x01,
    /**< 微分增益。 */
    CAN_COMMAND_PID_FIELD_KD = 0x02,
    /**< 积分限幅百分比。 */
    CAN_COMMAND_PID_FIELD_INTEGRAL_LIMIT_PERCENT = 0x03,
    /**< 输出限幅百分比。 */
    CAN_COMMAND_PID_FIELD_OUTPUT_LIMIT_PERCENT = 0x04,
};

/** @brief CAN 协议设备。 */
static const struct device *const can_command_protocol_dev =
    DEVICE_DT_GET(CAN_COMMAND_PROTOCOL_NODE);

/** @brief CAN 业务调试过滤器句柄。 */
static int can_command_filter_handle = -1;

/**
 * @brief 将内部错误码转换为 CAN 业务调试状态码。
 * @param ret 内部错误码。
 * @return uint8_t CAN 业务调试状态码。
 */
static uint8_t can_command_status_from_ret(int ret)
{
    if (ret == 0)
    {
        return CAN_COMMAND_STATUS_OK;
    }

    if ((ret == -EINVAL) || (ret == -EMSGSIZE))
    {
        return CAN_COMMAND_STATUS_INVALID_PARAM;
    }

    return CAN_COMMAND_STATUS_EXEC_FAILED;
}

/**
 * @brief 判断云台轴编号是否有效。
 * @param axis_id 云台轴编号。
 * @return bool true 表示有效，false 表示无效。
 */
static bool can_command_axis_id_is_valid(uint8_t axis_id)
{
    if (axis_id >= (uint8_t)GIMBAL_AXIS_COUNT)
    {
        return false;
    }

    return true;
}

/**
 * @brief 判断 PID 环编号是否有效。
 * @param loop PID 环编号。
 * @return bool true 表示有效，false 表示无效。
 */
static bool can_command_pid_loop_is_valid(uint8_t loop)
{
    if ((loop == (uint8_t)GIMBAL_PID_LOOP_POSITION) ||
        (loop == (uint8_t)GIMBAL_PID_LOOP_SPEED))
    {
        return true;
    }

    return false;
}

/**
 * @brief 判断 PID 字段编号是否有效。
 * @param field PID 字段编号。
 * @return bool true 表示有效，false 表示无效。
 */
static bool can_command_pid_field_is_valid(uint8_t field)
{
    if (field <= (uint8_t)CAN_COMMAND_PID_FIELD_OUTPUT_LIMIT_PERCENT)
    {
        return true;
    }

    return false;
}

/**
 * @brief 从 CAN PID 字段描述中解析 PID 环和字段编号。
 * @param packed CAN PID 字段描述。
 * @param loop 输出 PID 环编号。
 * @param field 输出 PID 字段编号。
 * @return int 0 表示成功，负值表示失败。
 */
static int can_command_parse_pid_selector(uint8_t packed,
                                          enum gimbal_pid_loop *loop,
                                          uint8_t *field)
{
    uint8_t raw_loop;
    uint8_t raw_field;

    if ((loop == NULL) || (field == NULL))
    {
        return -EINVAL;
    }

    raw_loop = packed >> CAN_COMMAND_PID_LOOP_SHIFT;
    raw_field = packed & CAN_COMMAND_PID_FIELD_MASK;

    if (!can_command_pid_loop_is_valid(raw_loop))
    {
        return -EINVAL;
    }

    if (!can_command_pid_field_is_valid(raw_field))
    {
        return -EINVAL;
    }

    *loop = (enum gimbal_pid_loop)raw_loop;
    *field = raw_field;

    return 0;
}

/**
 * @brief 从 PID 百分比配置中读取指定字段。
 * @param config PID 百分比配置。
 * @param field PID 字段编号。
 * @param value 输出字段值。
 * @return int 0 表示成功，负值表示失败。
 */
static int can_command_get_pid_field_value(const struct gimbal_pid_percent_config *config,
                                           uint8_t field,
                                           int32_t *value)
{
    if ((config == NULL) || (value == NULL))
    {
        return -EINVAL;
    }

    switch (field)
    {
    case CAN_COMMAND_PID_FIELD_KP:
        *value = config->kp;
        return 0;

    case CAN_COMMAND_PID_FIELD_KI:
        *value = config->ki;
        return 0;

    case CAN_COMMAND_PID_FIELD_KD:
        *value = config->kd;
        return 0;

    case CAN_COMMAND_PID_FIELD_INTEGRAL_LIMIT_PERCENT:
        *value = (int32_t)config->integral_limit_percent;
        return 0;

    case CAN_COMMAND_PID_FIELD_OUTPUT_LIMIT_PERCENT:
        *value = (int32_t)config->output_limit_percent;
        return 0;

    default:
        break;
    }

    return -EINVAL;
}

/**
 * @brief 向 PID 百分比配置写入指定字段。
 * @param config PID 百分比配置。
 * @param field PID 字段编号。
 * @param value 字段值。
 * @return int 0 表示成功，负值表示失败。
 */
static int can_command_set_pid_field_value(struct gimbal_pid_percent_config *config,
                                           uint8_t field,
                                           int32_t value)
{
    if (config == NULL)
    {
        return -EINVAL;
    }

    switch (field)
    {
    case CAN_COMMAND_PID_FIELD_KP:
        config->kp = value;
        return 0;

    case CAN_COMMAND_PID_FIELD_KI:
        config->ki = value;
        return 0;

    case CAN_COMMAND_PID_FIELD_KD:
        config->kd = value;
        return 0;

    case CAN_COMMAND_PID_FIELD_INTEGRAL_LIMIT_PERCENT:
        if (value < 0)
        {
            return -EINVAL;
        }

        config->integral_limit_percent = (uint32_t)value;
        return 0;

    case CAN_COMMAND_PID_FIELD_OUTPUT_LIMIT_PERCENT:
        if (value < 0)
        {
            return -EINVAL;
        }

        config->output_limit_percent = (uint32_t)value;
        return 0;

    default:
        break;
    }

    return -EINVAL;
}

/**
 * @brief 发送 CAN 业务调试响应。
 * @param cmd 命令码。
 * @param status 状态码。
 * @param axis_id 云台轴编号。
 * @param packed_selector PID 字段描述或保留字段。
 * @param value 响应数值。
 * @return int 0 表示成功，负值表示失败。
 */
static int can_command_send_response(uint8_t cmd,
                                     uint8_t status,
                                     uint8_t axis_id,
                                     uint8_t packed_selector,
                                     int32_t value)
{
    uint8_t payload[CAN_COMMAND_RESPONSE_DLC] = {0};

    payload[0] = cmd;
    payload[1] = status;
    payload[2] = axis_id;
    payload[3] = packed_selector;
    sys_put_le32((uint32_t)value, &payload[4]);

    return can_protocol_send_data(can_command_protocol_dev,
                                  CAN_COMMAND_GIMBAL_RESPONSE_ID,
                                  payload,
                                  CAN_COMMAND_RESPONSE_DLC);
}

/**
 * @brief 处理设置单轴目标角度命令。
 * @param frame CAN 报文。
 * @return int 0 表示成功，负值表示失败。
 */
static int can_command_handle_set_axis_target(const struct can_frame *frame)
{
    uint8_t axis_id;
    int32_t target_mrad;
    int ret;

    if ((frame == NULL) || (frame->dlc < 6U))
    {
        return -EINVAL;
    }

    axis_id = frame->data[1];
    target_mrad = (int32_t)sys_get_le32(&frame->data[2]);

    if (!can_command_axis_id_is_valid(axis_id))
    {
        return -EINVAL;
    }

    ret = gimbal_set_axis_target((enum gimbal_axis_id)axis_id,
                                 target_mrad,
                                 GIMBAL_TARGET_SOURCE_EXTERNAL);

    if (ret < 0)
    {
        return ret;
    }

    return can_command_send_response(frame->data[0],
                                     CAN_COMMAND_STATUS_OK,
                                     axis_id,
                                     0U,
                                     target_mrad);
}

/**
 * @brief 处理关闭云台输出命令。
 * @param frame CAN 报文。
 * @return int 0 表示成功，负值表示失败。
 */
static int can_command_handle_disable(const struct can_frame *frame)
{
    int ret;

    if (frame == NULL)
    {
        return -EINVAL;
    }

    ret = gimbal_disable();

    if (ret < 0)
    {
        return ret;
    }

    return can_command_send_response(frame->data[0],
                                     CAN_COMMAND_STATUS_OK,
                                     0U,
                                     0U,
                                     0);
}

/**
 * @brief 处理设置 PID 单字段命令。
 * @param frame CAN 报文。
 * @return int 0 表示成功，负值表示失败。
 */
static int can_command_handle_set_pid_field(const struct can_frame *frame)
{
    struct gimbal_pid_percent_config config;
    enum gimbal_pid_loop loop;
    uint8_t axis_id;
    uint8_t packed_selector;
    uint8_t field;
    int32_t value;
    int ret;

    if ((frame == NULL) || (frame->dlc < 8U))
    {
        return -EINVAL;
    }

    axis_id = frame->data[1];
    packed_selector = frame->data[2];
    value = (int32_t)sys_get_le32(&frame->data[3]);

    if (!can_command_axis_id_is_valid(axis_id))
    {
        return -EINVAL;
    }

    ret = can_command_parse_pid_selector(packed_selector, &loop, &field);

    if (ret < 0)
    {
        return ret;
    }

    ret = gimbal_get_pid_percent_config((enum gimbal_axis_id)axis_id, loop, &config);

    if (ret < 0)
    {
        return ret;
    }

    ret = can_command_set_pid_field_value(&config, field, value);

    if (ret < 0)
    {
        return ret;
    }

    ret = gimbal_set_pid_percent_config((enum gimbal_axis_id)axis_id, loop, &config);

    if (ret < 0)
    {
        return ret;
    }

    return can_command_send_response(frame->data[0],
                                     CAN_COMMAND_STATUS_OK,
                                     axis_id,
                                     packed_selector,
                                     value);
}

/**
 * @brief 处理读取 PID 单字段命令。
 * @param frame CAN 报文。
 * @return int 0 表示成功，负值表示失败。
 */
static int can_command_handle_get_pid_field(const struct can_frame *frame)
{
    struct gimbal_pid_percent_config config;
    enum gimbal_pid_loop loop;
    uint8_t axis_id;
    uint8_t packed_selector;
    uint8_t field;
    int32_t value;
    int ret;

    if ((frame == NULL) || (frame->dlc < 3U))
    {
        return -EINVAL;
    }

    axis_id = frame->data[1];
    packed_selector = frame->data[2];

    if (!can_command_axis_id_is_valid(axis_id))
    {
        return -EINVAL;
    }

    ret = can_command_parse_pid_selector(packed_selector, &loop, &field);

    if (ret < 0)
    {
        return ret;
    }

    ret = gimbal_get_pid_percent_config((enum gimbal_axis_id)axis_id, loop, &config);

    if (ret < 0)
    {
        return ret;
    }

    ret = can_command_get_pid_field_value(&config, field, &value);

    if (ret < 0)
    {
        return ret;
    }

    return can_command_send_response(frame->data[0],
                                     CAN_COMMAND_STATUS_OK,
                                     axis_id,
                                     packed_selector,
                                     value);
}

/**
 * @brief 处理云台 CAN 业务调试报文。
 * @param frame CAN 报文。
 * @return int 0 表示成功，负值表示失败。
 */
static int can_command_handle_gimbal_frame(const struct can_frame *frame)
{
    if ((frame == NULL) || (frame->dlc < CAN_COMMAND_MIN_DLC))
    {
        return -EINVAL;
    }

    switch (frame->data[0])
    {
    case CAN_COMMAND_GIMBAL_CMD_SET_AXIS_TARGET:
        return can_command_handle_set_axis_target(frame);

    case CAN_COMMAND_GIMBAL_CMD_DISABLE:
        return can_command_handle_disable(frame);

    case CAN_COMMAND_GIMBAL_CMD_SET_PID_FIELD:
        return can_command_handle_set_pid_field(frame);

    case CAN_COMMAND_GIMBAL_CMD_GET_PID_FIELD:
        return can_command_handle_get_pid_field(frame);

    default:
        break;
    }

    return -ENOTSUP;
}

/**
 * @brief CAN 业务调试接收回调。
 * @param frame 接收到的 CAN 报文。
 * @param user_data 用户参数。
 * @return void
 */
static void can_command_rx_handler(const struct can_frame *frame, void *user_data)
{
    uint8_t status;
    uint8_t cmd = 0U;
    uint8_t axis_id = 0U;
    uint8_t packed_selector = 0U;
    int ret;

    ARG_UNUSED(user_data);

    if ((frame != NULL) && (frame->dlc > 0U))
    {
        cmd = frame->data[0];

        if (frame->dlc > 1U)
        {
            axis_id = frame->data[1];
        }

        if (frame->dlc > 2U)
        {
            packed_selector = frame->data[2];
        }
    }

    ret = can_command_handle_gimbal_frame(frame);

    if (ret < 0)
    {
        if (ret == -ENOTSUP)
        {
            status = CAN_COMMAND_STATUS_INVALID_CMD;
        }
        else
        {
            status = can_command_status_from_ret(ret);
        }

        can_command_send_response(cmd, status, axis_id, packed_selector, ret);
    }
}

/**
 * @brief 初始化 CAN 业务调试命令模块。
 * @return int 0 表示成功，负值表示失败。
 */
int can_command_init(void)
{
    struct can_protocol_filter_config filter = {
        .id = CAN_COMMAND_GIMBAL_REQUEST_ID,
        .mask = CAN_STD_ID_MASK,
        .flags = 0U,
    };

    if (!device_is_ready(can_command_protocol_dev))
    {
        return -ENODEV;
    }

    if (can_command_filter_handle >= 0)
    {
        return 0;
    }

    can_command_filter_handle = can_protocol_add_rx_filter(can_command_protocol_dev,
                                                           &filter,
                                                           can_command_rx_handler,
                                                           NULL);

    if (can_command_filter_handle < 0)
    {
        return can_command_filter_handle;
    }

    LOG_INF("CAN gimbal debug ready: req=0x%03x rsp=0x%03x",
            CAN_COMMAND_GIMBAL_REQUEST_ID,
            CAN_COMMAND_GIMBAL_RESPONSE_ID);

    return 0;
}
