/**
 * @file motor.c
 * @brief 电机控制层实现。
 */
#include "motor.h"

#include <errno.h>
#include <inttypes.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include "../can_protocol.h"

LOG_MODULE_REGISTER(motor_module, LOG_LEVEL_INF);

/** @brief 默认演示电机节点 ID。 */
#define MOTOR_DEFAULT_NODE_ID 0x10U

/** @brief 协议允许的最大节点 ID。 */
#define MOTOR_MAX_NODE_ID 0x7FU

/** @brief 主机命令帧 ID 基值。 */
#define MOTOR_HOST_CMD_BASE 0x100U

/** @brief 应答帧 ID 基值。 */
#define MOTOR_ACK_BASE 0x180U

/** @brief 上报帧 ID 基值。 */
#define MOTOR_REPORT_BASE 0x200U

/** @brief 电机协议过滤器组掩码。 */
#define MOTOR_ID_GROUP_MASK 0x780U

/**
 * @brief 电机协议命令码定义。
 */
enum motor_protocol_command
{
    /**< 设置运行使能和控制模式。 */
    MOTOR_CMD_SET_RUN_MODE = 0x01,
    /**< 设置电流目标值。 */
    MOTOR_CMD_SET_CURRENT_TARGET = 0x02,
    /**< 设置速度目标值。 */
    MOTOR_CMD_SET_SPEED_TARGET = 0x03,
    /**< 设置位置目标值。 */
    MOTOR_CMD_SET_POSITION_TARGET = 0x04,
    /**< 写速度环 PID 参数。 */
    MOTOR_CMD_SET_SPEED_PID = 0x10,
    /**< 读速度环 PID 参数。 */
    MOTOR_CMD_READ_SPEED_PID = 0x11,
    /**< 写位置环 PID 参数。 */
    MOTOR_CMD_SET_POSITION_PID = 0x12,
    /**< 读位置环 PID 参数。 */
    MOTOR_CMD_READ_POSITION_PID = 0x13,
    /**< 设置节点 ID。 */
    MOTOR_CMD_SET_NODE_ID = 0x20,
    /**< 读取当前节点 ID。 */
    MOTOR_CMD_READ_NODE_ID = 0x21,
    /**< 触发零点校准。 */
    MOTOR_CMD_ZERO_CALIBRATION = 0x30,
    /**< 读取零点信息。 */
    MOTOR_CMD_READ_ZERO = 0x31,
};

/**
 * @brief 电机协议应答状态码。
 */
enum motor_protocol_status_code
{
    /**< 命令执行成功。 */
    MOTOR_STATUS_OK = 0x00,
};

/**
 * @brief 电机协议主动上报命令码。
 */
enum motor_report_code
{
    /**< 位置上报。 */
    MOTOR_REPORT_POSITION = 0x80,
    /**< 速度上报。 */
    MOTOR_REPORT_SPEED = 0x81,
};

/**
 * @brief 接收帧分类。
 */
enum motor_frame_kind
{
    /**< 未知帧。 */
    MOTOR_FRAME_UNKNOWN = 0,
    /**< 应答帧。 */
    MOTOR_FRAME_ACK,
    /**< 主动上报帧。 */
    MOTOR_FRAME_REPORT,
};

/**
 * @brief 电机对象定义。
 */
struct motor_device
{
    /**< 电机在注册表中的索引。 */
    size_t index;
    /**< 电机默认节点 ID。 */
    uint8_t default_node_id;
};

/** @brief 电机对象注册表。 */
static const struct motor_device motor_devices[] = {
    {
        .index = 0U,
        .default_node_id = MOTOR_DEFAULT_NODE_ID,
    },
};

/** @brief 电机运行状态表。 */
static struct motor_state motor_states[] = {
    {
        .node_id = MOTOR_DEFAULT_NODE_ID,
        .requested_node_id = MOTOR_DEFAULT_NODE_ID,
        .requested_mode = MOTOR_CONTROL_MODE_SPEED,
    },
};

/** @brief 电机控制层互斥锁。 */
K_MUTEX_DEFINE(motor_mutex);

/** @brief 电机控制层是否已初始化。 */
static bool motor_initialized;

/**
 * @brief 判断电机对象是否有效。
 * @param motor 电机对象指针。
 * @return bool `true` 表示有效，`false` 表示无效。
 */
static bool motor_device_is_valid(const struct motor_device *motor)
{
    if (motor == NULL)
    {
        return false;
    }

    if (motor->index >= ARRAY_SIZE(motor_devices))
    {
        return false;
    }

    return (&motor_devices[motor->index] == motor);
}

/**
 * @brief 判断节点 ID 是否有效。
 * @param node_id 节点 ID。
 * @return bool `true` 表示有效，`false` 表示无效。
 */
static bool motor_node_id_is_valid(uint8_t node_id)
{
    return node_id <= MOTOR_MAX_NODE_ID;
}

/**
 * @brief 生成主机命令帧 ID。
 * @param node_id 目标节点 ID。
 * @return uint32_t 主机命令帧 ID。
 */
static uint32_t motor_host_cmd_id(uint8_t node_id)
{
    return MOTOR_HOST_CMD_BASE + node_id;
}

/**
 * @brief 重置全部电机状态。
 * @return void
 */
static void motor_reset_states_locked(void)
{
    size_t i;

    for (i = 0U; i < ARRAY_SIZE(motor_states); i++)
    {
        memset(&motor_states[i], 0, sizeof(motor_states[i]));
        motor_states[i].node_id = motor_devices[i].default_node_id;
        motor_states[i].requested_node_id = motor_devices[i].default_node_id;
        motor_states[i].requested_mode = MOTOR_CONTROL_MODE_SPEED;
    }
}

/**
 * @brief 获取电机对象索引。
 * @param motor 电机对象指针。
 * @param index 输出电机索引。
 * @return int 0 表示成功，负值表示失败。
 */
static int motor_get_index(const struct motor_device *motor, size_t *index)
{
    if (!motor_device_is_valid(motor) || (index == NULL))
    {
        return -EINVAL;
    }

    *index = motor->index;
    return 0;
}

/**
 * @brief 按当前节点或待切换节点 ID 查找电机状态。
 * @param node_id 节点 ID。
 * @return struct motor_state * 找到时返回状态指针，未找到时返回 NULL。
 */
static struct motor_state *motor_find_state_by_any_node_locked(uint8_t node_id)
{
    size_t i;

    for (i = 0U; i < ARRAY_SIZE(motor_states); i++)
    {
        if (motor_states[i].node_id == node_id)
        {
            return &motor_states[i];
        }

        if (motor_states[i].pending_node_id_change &&
            (motor_states[i].requested_node_id == node_id))
        {
            return &motor_states[i];
        }
    }

    return NULL;
}

/**
 * @brief 判断节点 ID 是否已被占用。
 * @param node_id 待检查的节点 ID。
 * @param ignore_index 检查时忽略的电机索引。
 * @return bool `true` 表示已占用，`false` 表示可用。
 */
static bool motor_node_id_is_reserved_locked(uint8_t node_id, size_t ignore_index)
{
    size_t i;

    for (i = 0U; i < ARRAY_SIZE(motor_states); i++)
    {
        if (i == ignore_index)
        {
            continue;
        }

        if (motor_states[i].node_id == node_id)
        {
            return true;
        }

        if (motor_states[i].pending_node_id_change &&
            (motor_states[i].requested_node_id == node_id))
        {
            return true;
        }
    }

    return false;
}

/**
 * @brief 解码接收帧 ID。
 * @param frame_id 接收帧 ID。
 * @param kind 输出帧分类。
 * @param node_id 输出节点 ID。
 * @return bool `true` 表示成功，`false` 表示不属于当前电机协议。
 */
static bool motor_decode_frame_id(uint32_t frame_id,
                                  enum motor_frame_kind *kind,
                                  uint8_t *node_id)
{
    if ((kind == NULL) || (node_id == NULL))
    {
        return false;
    }

    if ((frame_id >= MOTOR_ACK_BASE) &&
        (frame_id <= (MOTOR_ACK_BASE + MOTOR_MAX_NODE_ID)))
    {
        *kind = MOTOR_FRAME_ACK;
        *node_id = (uint8_t)(frame_id - MOTOR_ACK_BASE);
        return true;
    }

    if ((frame_id >= MOTOR_REPORT_BASE) &&
        (frame_id <= (MOTOR_REPORT_BASE + MOTOR_MAX_NODE_ID)))
    {
        *kind = MOTOR_FRAME_REPORT;
        *node_id = (uint8_t)(frame_id - MOTOR_REPORT_BASE);
        return true;
    }

    *kind = MOTOR_FRAME_UNKNOWN;
    *node_id = 0U;
    return false;
}

/**
 * @brief 记录发送结果。
 * @param index 电机索引。
 * @param ret 发送结果。
 * @return void
 */
static void motor_record_send_result_locked(size_t index, int ret)
{
    if (ret < 0)
    {
        motor_states[index].last_can_error = ret;
        return;
    }

    motor_states[index].last_can_error = 0;
    motor_states[index].tx_count++;
}

/**
 * @brief 按指定节点 ID 发送协议负载。
 * @param index 电机索引。
 * @param node_id 发送时使用的节点 ID。
 * @param payload 待发送的 8 字节协议负载。
 * @return int 0 表示成功，负值表示失败。
 */
static int motor_send_payload_for_index(size_t index,
                                        uint8_t node_id,
                                        const uint8_t payload[CAN_MAX_DLC])
{
    int ret;

    ret = can_protocol_send_data(motor_host_cmd_id(node_id), payload, CAN_MAX_DLC);

    k_mutex_lock(&motor_mutex, K_FOREVER);
    motor_record_send_result_locked(index, ret);
    k_mutex_unlock(&motor_mutex);

    return ret;
}

/**
 * @brief 获取发送所需的索引和节点 ID。
 * @param motor 电机对象指针。
 * @param index 输出电机索引。
 * @param node_id 输出当前节点 ID。
 * @return int 0 表示成功，负值表示失败。
 */
static int motor_prepare_send(const struct motor_device *motor,
                              size_t *index,
                              uint8_t *node_id)
{
    int ret;

    if ((index == NULL) || (node_id == NULL))
    {
        return -EINVAL;
    }

    k_mutex_lock(&motor_mutex, K_FOREVER);
    ret = motor_get_index(motor, index);

    if (ret == 0)
    {
        *node_id = motor_states[*index].node_id;
    }

    k_mutex_unlock(&motor_mutex);
    return ret;
}

/**
 * @brief 发送无参数命令。
 * @param motor 电机对象指针。
 * @param cmd 命令码。
 * @return int 0 表示成功，负值表示失败。
 */
static int motor_send_simple(const struct motor_device *motor, uint8_t cmd)
{
    size_t index;
    uint8_t node_id;
    uint8_t payload[CAN_MAX_DLC] = {0};
    int ret;

    ret = motor_prepare_send(motor, &index, &node_id);

    if (ret < 0)
    {
        return ret;
    }

    payload[0] = cmd;
    return motor_send_payload_for_index(index, node_id, payload);
}

/**
 * @brief 发送带参数编号和 int32 值的命令。
 * @param motor 电机对象指针。
 * @param cmd 命令码。
 * @param param 参数编号。
 * @param value 参数值。
 * @return int 0 表示成功，负值表示失败。
 */
static int motor_send_param_write(const struct motor_device *motor,
                                  uint8_t cmd,
                                  enum motor_pid_param param,
                                  int32_t value)
{
    size_t index;
    uint8_t node_id;
    uint8_t payload[CAN_MAX_DLC] = {0};
    int ret;

    ret = motor_prepare_send(motor, &index, &node_id);

    if (ret < 0)
    {
        return ret;
    }

    payload[0] = cmd;
    payload[1] = (uint8_t)param;
    sys_put_le32((uint32_t)value, &payload[2]);
    return motor_send_payload_for_index(index, node_id, payload);
}

/**
 * @brief 发送带参数编号的读取命令。
 * @param motor 电机对象指针。
 * @param cmd 命令码。
 * @param param 参数编号。
 * @return int 0 表示成功，负值表示失败。
 */
static int motor_send_param_read(const struct motor_device *motor,
                                 uint8_t cmd,
                                 enum motor_pid_param param)
{
    size_t index;
    uint8_t node_id;
    uint8_t payload[CAN_MAX_DLC] = {0};
    int ret;

    ret = motor_prepare_send(motor, &index, &node_id);

    if (ret < 0)
    {
        return ret;
    }

    payload[0] = cmd;
    payload[1] = (uint8_t)param;
    return motor_send_payload_for_index(index, node_id, payload);
}

/**
 * @brief 处理接收到的应答帧。
 * @param state 电机状态指针。
 * @param frame 应答帧指针。
 * @return void
 */
static void motor_handle_ack_locked(struct motor_state *state,
                                    const struct can_frame *frame)
{
    size_t payload_size;

    state->last_ack.cmd = frame->data[0];
    state->last_ack.status = frame->data[1];
    state->ack_valid = true;
    state->ack_count++;

    memset(state->last_ack.payload, 0, sizeof(state->last_ack.payload));

    if (frame->dlc > 2U)
    {
        payload_size = MIN((size_t)frame->dlc - 2U,
                           sizeof(state->last_ack.payload));
        memcpy(state->last_ack.payload, &frame->data[2], payload_size);
    }

    if ((frame->data[0] == MOTOR_CMD_SET_NODE_ID) &&
        state->pending_node_id_change)
    {
        if (frame->data[1] == MOTOR_STATUS_OK)
        {
            state->node_id = state->requested_node_id;
        }
        else
        {
            state->requested_node_id = state->node_id;
        }

        state->pending_node_id_change = false;
    }

    if ((frame->data[0] == MOTOR_CMD_READ_NODE_ID) &&
        (frame->data[1] == MOTOR_STATUS_OK))
    {
        state->reported_node_id = state->last_ack.payload[0];
        state->reported_node_id_valid = true;
    }
}

/**
 * @brief 处理接收到的主动上报帧。
 * @param state 电机状态指针。
 * @param frame 上报帧指针。
 * @return void
 */
static void motor_handle_report_locked(struct motor_state *state,
                                       const struct can_frame *frame)
{
    switch (frame->data[0])
    {
    case MOTOR_REPORT_POSITION:
        state->position_mrad = (int32_t)sys_get_le32(&frame->data[1]);
        state->position_valid = true;
        state->report_count++;
        break;

    case MOTOR_REPORT_SPEED:
        state->speed_mrad_s = (int32_t)sys_get_le32(&frame->data[1]);
        state->speed_valid = true;
        state->report_count++;
        break;

    default:
        LOG_WRN("Unknown report cmd=0x%02x", frame->data[0]);
        break;
    }
}

/**
 * @brief 处理来自 CAN 传输层的接收帧。
 * @param frame 接收到的 CAN 报文指针。
 * @param user_data 用户参数。
 * @return void
 */
static void motor_handle_can_frame(const struct can_frame *frame, void *user_data)
{
    enum motor_frame_kind kind;
    struct motor_state *state;
    uint8_t source_node_id;

    ARG_UNUSED(user_data);

    if (frame == NULL)
    {
        return;
    }

    if ((frame->flags & (CAN_FRAME_IDE | CAN_FRAME_RTR | CAN_FRAME_FDF)) != 0U)
    {
        return;
    }

    if (!motor_decode_frame_id(frame->id, &kind, &source_node_id))
    {
        return;
    }

    k_mutex_lock(&motor_mutex, K_FOREVER);

    if (!motor_initialized)
    {
        k_mutex_unlock(&motor_mutex);
        return;
    }

    state = motor_find_state_by_any_node_locked(source_node_id);

    if (state == NULL)
    {
        k_mutex_unlock(&motor_mutex);
        return;
    }

    state->rx_count++;

    if (kind == MOTOR_FRAME_ACK)
    {
        if (frame->dlc < 2U)
        {
            k_mutex_unlock(&motor_mutex);
            return;
        }

        motor_handle_ack_locked(state, frame);
        k_mutex_unlock(&motor_mutex);
        return;
    }

    if (frame->dlc < 5U)
    {
        k_mutex_unlock(&motor_mutex);
        return;
    }

    motor_handle_report_locked(state, frame);
    k_mutex_unlock(&motor_mutex);
}

int motor_init(void)
{
    static const struct can_protocol_filter_config filters[] = {
        {
            .id = MOTOR_ACK_BASE,
            .mask = MOTOR_ID_GROUP_MASK,
            .flags = 0U,
        },
        {
            .id = MOTOR_REPORT_BASE,
            .mask = MOTOR_ID_GROUP_MASK,
            .flags = 0U,
        },
    };
    int ret;

    k_mutex_lock(&motor_mutex, K_FOREVER);
    motor_reset_states_locked();
    motor_initialized = true;
    k_mutex_unlock(&motor_mutex);

    ret = can_protocol_init(filters,
                            ARRAY_SIZE(filters),
                            motor_handle_can_frame,
                            NULL);

    if (ret < 0)
    {
        k_mutex_lock(&motor_mutex, K_FOREVER);
        motor_initialized = false;
        motor_reset_states_locked();
        k_mutex_unlock(&motor_mutex);
        return ret;
    }

    return 0;
}

void motor_deinit(void)
{
    can_protocol_deinit();

    k_mutex_lock(&motor_mutex, K_FOREVER);
    motor_initialized = false;
    motor_reset_states_locked();
    k_mutex_unlock(&motor_mutex);
}

const struct motor_device *motor_get_default(void)
{
    return &motor_devices[0];
}

const struct motor_device *motor_get_by_index(size_t index)
{
    if (index >= ARRAY_SIZE(motor_devices))
    {
        return NULL;
    }

    return &motor_devices[index];
}

size_t motor_get_count(void)
{
    return ARRAY_SIZE(motor_devices);
}

int motor_get_current_node_id(const struct motor_device *motor, uint8_t *node_id)
{
    size_t index;
    int ret;

    if (node_id == NULL)
    {
        return -EINVAL;
    }

    k_mutex_lock(&motor_mutex, K_FOREVER);
    ret = motor_get_index(motor, &index);

    if (ret == 0)
    {
        *node_id = motor_states[index].node_id;
    }

    k_mutex_unlock(&motor_mutex);
    return ret;
}

int motor_copy_state(const struct motor_device *motor, struct motor_state *state)
{
    size_t index;
    int ret;

    if (state == NULL)
    {
        return -EINVAL;
    }

    k_mutex_lock(&motor_mutex, K_FOREVER);
    ret = motor_get_index(motor, &index);

    if (ret == 0)
    {
        memcpy(state, &motor_states[index], sizeof(*state));
    }

    k_mutex_unlock(&motor_mutex);
    return ret;
}

void motor_log_state(const struct motor_device *motor)
{
    struct motor_state state;
    int ret;

    ret = motor_copy_state(motor, &state);

    if (ret < 0)
    {
        LOG_WRN("No motor state available: %d", ret);
        return;
    }

    LOG_INF("node=0x%02x speed=%" PRId32 " pos=%" PRId32,
            state.node_id,
            state.speed_valid ? state.speed_mrad_s : 0,
            state.position_valid ? state.position_mrad : 0);
}

int motor_set_run_mode(const struct motor_device *motor,
                       bool enable,
                       enum motor_control_mode mode)
{
    size_t index;
    uint8_t node_id;
    uint8_t payload[CAN_MAX_DLC] = {0};
    bool old_enable;
    enum motor_control_mode old_mode;
    int ret;

    if (mode > MOTOR_CONTROL_MODE_POSITION)
    {
        return -EINVAL;
    }

    payload[0] = MOTOR_CMD_SET_RUN_MODE;
    payload[1] = enable ? 1U : 0U;
    payload[2] = (uint8_t)mode;

    k_mutex_lock(&motor_mutex, K_FOREVER);
    ret = motor_get_index(motor, &index);

    if (ret < 0)
    {
        k_mutex_unlock(&motor_mutex);
        return ret;
    }

    node_id = motor_states[index].node_id;
    old_enable = motor_states[index].requested_enable;
    old_mode = motor_states[index].requested_mode;
    motor_states[index].requested_enable = enable;
    motor_states[index].requested_mode = mode;
    k_mutex_unlock(&motor_mutex);

    ret = motor_send_payload_for_index(index, node_id, payload);

    if (ret < 0)
    {
        k_mutex_lock(&motor_mutex, K_FOREVER);
        motor_states[index].requested_enable = old_enable;
        motor_states[index].requested_mode = old_mode;
        k_mutex_unlock(&motor_mutex);
    }

    return ret;
}

/**
 * @brief 设置电机电流目标。
 * @param motor 电机对象指针。
 * @param current_target 目标电流控制量。
 * @return int 0 表示成功，负值表示失败。
 */
int motor_set_current_target(const struct motor_device *motor, int32_t current_target)
{
    size_t index;
    uint8_t node_id;
    uint8_t payload[CAN_MAX_DLC] = {0};
    int32_t old_target;
    int ret;

    payload[0] = MOTOR_CMD_SET_CURRENT_TARGET;
    sys_put_le32((uint32_t)current_target, &payload[1]);

    k_mutex_lock(&motor_mutex, K_FOREVER);
    ret = motor_get_index(motor, &index);

    if (ret < 0)
    {
        k_mutex_unlock(&motor_mutex);
        return ret;
    }

    node_id = motor_states[index].node_id;
    old_target = motor_states[index].requested_current_target;
    motor_states[index].requested_current_target = current_target;
    k_mutex_unlock(&motor_mutex);

    ret = motor_send_payload_for_index(index, node_id, payload);

    if (ret < 0)
    {
        k_mutex_lock(&motor_mutex, K_FOREVER);
        motor_states[index].requested_current_target = old_target;
        k_mutex_unlock(&motor_mutex);
    }

    return ret;
}

int motor_set_speed_target(const struct motor_device *motor, int32_t speed_mrad_s)
{
    size_t index;
    uint8_t node_id;
    uint8_t payload[CAN_MAX_DLC] = {0};
    int32_t old_target;
    int ret;

    payload[0] = MOTOR_CMD_SET_SPEED_TARGET;
    sys_put_le32((uint32_t)speed_mrad_s, &payload[1]);

    k_mutex_lock(&motor_mutex, K_FOREVER);
    ret = motor_get_index(motor, &index);

    if (ret < 0)
    {
        k_mutex_unlock(&motor_mutex);
        return ret;
    }

    node_id = motor_states[index].node_id;
    old_target = motor_states[index].requested_speed_target_mrad_s;
    motor_states[index].requested_speed_target_mrad_s = speed_mrad_s;
    k_mutex_unlock(&motor_mutex);

    ret = motor_send_payload_for_index(index, node_id, payload);

    if (ret < 0)
    {
        k_mutex_lock(&motor_mutex, K_FOREVER);
        motor_states[index].requested_speed_target_mrad_s = old_target;
        k_mutex_unlock(&motor_mutex);
    }

    return ret;
}

int motor_set_position_target(const struct motor_device *motor, int32_t position_mrad)
{
    size_t index;
    uint8_t node_id;
    uint8_t payload[CAN_MAX_DLC] = {0};
    int32_t old_target;
    int ret;

    payload[0] = MOTOR_CMD_SET_POSITION_TARGET;
    sys_put_le32((uint32_t)position_mrad, &payload[1]);

    k_mutex_lock(&motor_mutex, K_FOREVER);
    ret = motor_get_index(motor, &index);

    if (ret < 0)
    {
        k_mutex_unlock(&motor_mutex);
        return ret;
    }

    node_id = motor_states[index].node_id;
    old_target = motor_states[index].requested_position_target_mrad;
    motor_states[index].requested_position_target_mrad = position_mrad;
    k_mutex_unlock(&motor_mutex);

    ret = motor_send_payload_for_index(index, node_id, payload);

    if (ret < 0)
    {
        k_mutex_lock(&motor_mutex, K_FOREVER);
        motor_states[index].requested_position_target_mrad = old_target;
        k_mutex_unlock(&motor_mutex);
    }

    return ret;
}

int motor_set_speed_pid(const struct motor_device *motor,
                        enum motor_pid_param param,
                        int32_t value)
{
    if (param > MOTOR_PID_OUT_MAX)
    {
        return -EINVAL;
    }

    return motor_send_param_write(motor, MOTOR_CMD_SET_SPEED_PID, param, value);
}

int motor_request_speed_pid(const struct motor_device *motor,
                            enum motor_pid_param param)
{
    if (param > MOTOR_PID_OUT_MAX)
    {
        return -EINVAL;
    }

    return motor_send_param_read(motor, MOTOR_CMD_READ_SPEED_PID, param);
}

int motor_set_position_pid(const struct motor_device *motor,
                           enum motor_pid_param param,
                           int32_t value)
{
    if (param > MOTOR_PID_OUT_MAX)
    {
        return -EINVAL;
    }

    return motor_send_param_write(motor, MOTOR_CMD_SET_POSITION_PID, param, value);
}

int motor_request_position_pid(const struct motor_device *motor,
                               enum motor_pid_param param)
{
    if (param > MOTOR_PID_OUT_MAX)
    {
        return -EINVAL;
    }

    return motor_send_param_read(motor, MOTOR_CMD_READ_POSITION_PID, param);
}

int motor_set_node_id(const struct motor_device *motor, uint8_t new_node_id)
{
    size_t index;
    uint8_t node_id;
    uint8_t payload[CAN_MAX_DLC] = {0};
    uint8_t old_requested_node_id;
    bool old_pending;
    int ret;

    if (!motor_node_id_is_valid(new_node_id))
    {
        return -EINVAL;
    }

    payload[0] = MOTOR_CMD_SET_NODE_ID;
    payload[1] = new_node_id;

    k_mutex_lock(&motor_mutex, K_FOREVER);
    ret = motor_get_index(motor, &index);

    if (ret < 0)
    {
        k_mutex_unlock(&motor_mutex);
        return ret;
    }

    if (motor_node_id_is_reserved_locked(new_node_id, index))
    {
        k_mutex_unlock(&motor_mutex);
        return -EEXIST;
    }

    node_id = motor_states[index].node_id;
    old_requested_node_id = motor_states[index].requested_node_id;
    old_pending = motor_states[index].pending_node_id_change;
    motor_states[index].requested_node_id = new_node_id;
    motor_states[index].pending_node_id_change =
        (new_node_id != motor_states[index].node_id);
    k_mutex_unlock(&motor_mutex);

    ret = motor_send_payload_for_index(index, node_id, payload);

    if (ret < 0)
    {
        k_mutex_lock(&motor_mutex, K_FOREVER);
        motor_states[index].requested_node_id = old_requested_node_id;
        motor_states[index].pending_node_id_change = old_pending;
        k_mutex_unlock(&motor_mutex);
    }

    return ret;
}

int motor_request_node_id(const struct motor_device *motor)
{
    return motor_send_simple(motor, MOTOR_CMD_READ_NODE_ID);
}

int motor_zero_calibration(const struct motor_device *motor)
{
    return motor_send_simple(motor, MOTOR_CMD_ZERO_CALIBRATION);
}

int motor_request_zero(const struct motor_device *motor)
{
    return motor_send_simple(motor, MOTOR_CMD_READ_ZERO);
}
