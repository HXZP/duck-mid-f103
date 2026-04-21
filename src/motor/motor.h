/**
 * @file motor.h
 * @brief 电机控制层接口。
 */
#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief 电机 PID 增益参数缩放倍数。 */
#define MOTOR_PID_GAIN_SCALE 1000

/**
 * @brief 电机控制模式定义。
 */
enum motor_control_mode
{
    /**< 电流控制模式。 */
    MOTOR_CONTROL_MODE_CURRENT = 0,
    /**< 速度控制模式。 */
    MOTOR_CONTROL_MODE_SPEED = 1,
    /**< 位置控制模式。 */
    MOTOR_CONTROL_MODE_POSITION = 2,
};

/**
 * @brief 电机 PID 参数编号定义。
 */
enum motor_pid_param
{
    /**< 比例参数 P。 */
    MOTOR_PID_P = 0,
    /**< 积分参数 I。 */
    MOTOR_PID_I = 1,
    /**< 微分参数 D。 */
    MOTOR_PID_D = 2,
    /**< 积分累计上限。 */
    MOTOR_PID_I_ACC_MAX = 3,
    /**< 输出限幅。 */
    MOTOR_PID_OUT_MAX = 4,
};

/**
 * @brief 电机对象前向声明。
 */
struct motor_device;

/**
 * @brief 电机最近一次应答帧内容。
 */
struct motor_ack
{
    /**< 应答帧中的命令码。 */
    uint8_t cmd;
    /**< 应答状态码。 */
    uint8_t status;
    /**< 附加返回数据。 */
    uint8_t payload[6];
};

/**
 * @brief 电机状态快照。
 */
struct motor_state
{
    /**< 当前生效的节点 ID。 */
    uint8_t node_id;
    /**< 最近一次请求设置的节点 ID。 */
    uint8_t requested_node_id;
    /**< 通过读节点 ID 命令获取到的节点 ID。 */
    uint8_t reported_node_id;
    /**< 读取到的节点 ID 是否有效。 */
    bool reported_node_id_valid;
    /**< 是否存在待完成的节点 ID 切换。 */
    bool pending_node_id_change;
    /**< 最近一次请求的使能状态。 */
    bool requested_enable;
    /**< 最近一次请求的电流目标。 */
    int32_t requested_current_target;
    /**< 最近一次请求的控制模式。 */
    enum motor_control_mode requested_mode;
    /**< 最近一次请求的速度目标。 */
    int32_t requested_speed_target_mrad_s;
    /**< 最近一次请求的位置目标。 */
    int32_t requested_position_target_mrad;
    /**< 最近一次接收到的位置上报值。 */
    int32_t position_mrad;
    /**< 最近一次接收到的速度上报值。 */
    int32_t speed_mrad_s;
    /**< 位置上报是否有效。 */
    bool position_valid;
    /**< 速度上报是否有效。 */
    bool speed_valid;
    /**< 最近一次应答帧内容。 */
    struct motor_ack last_ack;
    /**< 最近一次应答帧是否有效。 */
    bool ack_valid;
    /**< 最近一次 CAN 错误码。 */
    int last_can_error;
    /**< 已发送报文数量。 */
    uint32_t tx_count;
    /**< 已接收报文数量。 */
    uint32_t rx_count;
    /**< 已解析应答帧数量。 */
    uint32_t ack_count;
    /**< 已解析上报帧数量。 */
    uint32_t report_count;
};

/**
 * @brief 初始化电机控制层。
 * @return int 0 表示成功，负值表示失败。
 */
int motor_init(void);

/**
 * @brief 反初始化电机控制层。
 * @return void
 */
void motor_deinit(void);

/**
 * @brief 获取默认演示电机对象。
 * @return const struct motor_device * 指向默认电机对象的只读指针。
 */
const struct motor_device *motor_get_default(void);

/**
 * @brief 按索引获取电机对象。
 * @param index 电机索引。
 * @return const struct motor_device * 找到时返回只读电机对象指针，未找到时返回 NULL。
 */
const struct motor_device *motor_get_by_index(size_t index);

/**
 * @brief 获取电机对象数量。
 * @return size_t 已注册的电机数量。
 */
size_t motor_get_count(void);

/**
 * @brief 获取电机当前节点 ID。
 * @param motor 电机对象指针。
 * @param node_id 输出当前节点 ID。
 * @return int 0 表示成功，负值表示失败。
 */
int motor_get_current_node_id(const struct motor_device *motor, uint8_t *node_id);

/**
 * @brief 拷贝指定电机的状态快照。
 * @param motor 电机对象指针。
 * @param state 输出状态快照。
 * @return int 0 表示成功，负值表示失败。
 */
int motor_copy_state(const struct motor_device *motor, struct motor_state *state);

/**
 * @brief 打印指定电机的位置和速度状态。
 * @param motor 电机对象指针。
 * @return void
 */
void motor_log_state(const struct motor_device *motor);

/**
 * @brief 设置电机运行模式和使能状态。
 * @param motor 电机对象指针。
 * @param enable `true` 表示使能，`false` 表示停机。
 * @param mode 控制模式。
 * @return int 0 表示成功，负值表示失败。
 */
int motor_set_run_mode(const struct motor_device *motor,
                       bool enable,
                       enum motor_control_mode mode);

/**
 * @brief 设置电机电流目标。
 * @param motor 电机对象指针。
 * @param current_target 目标电流控制量。
 * @return int 0 表示成功，负值表示失败。
 */
int motor_set_current_target(const struct motor_device *motor, int32_t current_target);

/**
 * @brief 设置电机速度目标。
 * @param motor 电机对象指针。
 * @param speed_mrad_s 目标速度，单位 mrad/s。
 * @return int 0 表示成功，负值表示失败。
 */
int motor_set_speed_target(const struct motor_device *motor, int32_t speed_mrad_s);

/**
 * @brief 设置电机位置目标。
 * @param motor 电机对象指针。
 * @param position_mrad 目标位置，单位 mrad。
 * @return int 0 表示成功，负值表示失败。
 */
int motor_set_position_target(const struct motor_device *motor, int32_t position_mrad);

/**
 * @brief 设置电机速度环 PID 参数。
 * @param motor 电机对象指针。
 * @param param PID 参数编号。
 * @param value PID 参数值。
 * @return int 0 表示成功，负值表示失败。
 */
int motor_set_speed_pid(const struct motor_device *motor,
                        enum motor_pid_param param,
                        int32_t value);

/**
 * @brief 请求读取电机速度环 PID 参数。
 * @param motor 电机对象指针。
 * @param param PID 参数编号。
 * @return int 0 表示成功，负值表示失败。
 */
int motor_request_speed_pid(const struct motor_device *motor,
                            enum motor_pid_param param);

/**
 * @brief 设置电机位置环 PID 参数。
 * @param motor 电机对象指针。
 * @param param PID 参数编号。
 * @param value PID 参数值。
 * @return int 0 表示成功，负值表示失败。
 */
int motor_set_position_pid(const struct motor_device *motor,
                           enum motor_pid_param param,
                           int32_t value);

/**
 * @brief 请求读取电机位置环 PID 参数。
 * @param motor 电机对象指针。
 * @param param PID 参数编号。
 * @return int 0 表示成功，负值表示失败。
 */
int motor_request_position_pid(const struct motor_device *motor,
                               enum motor_pid_param param);

/**
 * @brief 设置电机节点 ID。
 * @param motor 电机对象指针。
 * @param new_node_id 新节点 ID。
 * @return int 0 表示成功，负值表示失败。
 */
int motor_set_node_id(const struct motor_device *motor, uint8_t new_node_id);

/**
 * @brief 请求读取电机节点 ID。
 * @param motor 电机对象指针。
 * @return int 0 表示成功，负值表示失败。
 */
int motor_request_node_id(const struct motor_device *motor);

/**
 * @brief 触发电机零点校准。
 * @param motor 电机对象指针。
 * @return int 0 表示成功，负值表示失败。
 */
int motor_zero_calibration(const struct motor_device *motor);

/**
 * @brief 请求读取电机零点信息。
 * @param motor 电机对象指针。
 * @return int 0 表示成功，负值表示失败。
 */
int motor_request_zero(const struct motor_device *motor);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H_ */
