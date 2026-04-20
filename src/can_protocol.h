/**
 * @file can_protocol.h
 * @brief 上位机 CAN 电机控制协议接口。
 */
#ifndef CAN_PROTOCOL_H_
#define CAN_PROTOCOL_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/drivers/can.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 协议规定的默认 CAN 波特率，单位 bit/s。 */
#define CAN_PROTOCOL_BAUDRATE        500000U
/** @brief 协议默认节点 ID。 */
#define CAN_PROTOCOL_DEFAULT_NODE_ID 0x10U
/** @brief PID 增益类参数的整数缩放倍数。 */
#define CAN_PROTOCOL_PID_GAIN_SCALE  1000
/** @brief 当前实现支持的最大电机数量。 */
#define CAN_PROTOCOL_MAX_MOTORS      5U
/** @brief 协议允许的最大节点 ID。 */
#define CAN_PROTOCOL_MAX_NODE_ID     0x7FU

/**
 * @brief 协议命令码定义。
 */
enum can_protocol_command {
	/**< 设置运行使能和控制模式。 */
	CAN_PROTOCOL_CMD_SET_RUN_MODE = 0x01,
	/**< 设置速度目标值。 */
	CAN_PROTOCOL_CMD_SET_SPEED_TARGET = 0x02,
	/**< 设置位置目标值。 */
	CAN_PROTOCOL_CMD_SET_POSITION_TARGET = 0x03,
	/**< 写速度环 PID 参数。 */
	CAN_PROTOCOL_CMD_SET_SPEED_PID = 0x10,
	/**< 读速度环 PID 参数。 */
	CAN_PROTOCOL_CMD_READ_SPEED_PID = 0x11,
	/**< 写位置环 PID 参数。 */
	CAN_PROTOCOL_CMD_SET_POSITION_PID = 0x12,
	/**< 读位置环 PID 参数。 */
	CAN_PROTOCOL_CMD_READ_POSITION_PID = 0x13,
	/**< 设置目标节点 ID。 */
	CAN_PROTOCOL_CMD_SET_NODE_ID = 0x20,
	/**< 读取当前节点 ID。 */
	CAN_PROTOCOL_CMD_READ_NODE_ID = 0x21,
	/**< 触发零点校准。 */
	CAN_PROTOCOL_CMD_ZERO_CALIBRATION = 0x30,
	/**< 读取零点信息。 */
	CAN_PROTOCOL_CMD_READ_ZERO = 0x31,
};

/**
 * @brief 电机控制模式定义。
 */
enum can_protocol_control_mode {
	/**< 速度控制模式。 */
	CAN_PROTOCOL_MODE_SPEED = 0x00,
	/**< 位置控制模式。 */
	CAN_PROTOCOL_MODE_POSITION = 0x01,
};

/**
 * @brief 协议应答状态码。
 */
enum can_protocol_status_code {
	/**< 命令执行成功。 */
	CAN_PROTOCOL_STATUS_OK = 0x00,
	/**< 命令码非法。 */
	CAN_PROTOCOL_STATUS_INVALID_COMMAND = 0x01,
	/**< 参数非法。 */
	CAN_PROTOCOL_STATUS_INVALID_PARAMETER = 0x02,
	/**< 当前模式不支持该命令。 */
	CAN_PROTOCOL_STATUS_INVALID_MODE = 0x03,
	/**< CAN 总线或发送过程发生错误。 */
	CAN_PROTOCOL_STATUS_CAN_ERROR = 0x04,
};

/**
 * @brief PID 参数索引定义。
 */
enum can_protocol_pid_param {
	/**< 比例参数 P。 */
	CAN_PROTOCOL_PID_P = 0x00,
	/**< 积分参数 I。 */
	CAN_PROTOCOL_PID_I = 0x01,
	/**< 微分参数 D。 */
	CAN_PROTOCOL_PID_D = 0x02,
	/**< 积分累加上限。 */
	CAN_PROTOCOL_PID_I_ACC_MAX = 0x03,
	/**< 输出限幅。 */
	CAN_PROTOCOL_PID_OUT_MAX = 0x04,
};

/**
 * @brief 主动上报帧命令码。
 */
enum can_protocol_report_code {
	/**< 位置上报。 */
	CAN_PROTOCOL_REPORT_POSITION = 0x80,
	/**< 速度上报。 */
	CAN_PROTOCOL_REPORT_SPEED = 0x81,
};

/**
 * @brief 缓存最近一次应答帧的内容。
 */
struct can_protocol_ack {
	/**< 应答帧回显的命令码。 */
	uint8_t cmd;
	/**< 应答状态码。 */
	uint8_t status;
	/**< 附加返回数据区，来自 Byte2~Byte7。 */
	uint8_t payload[6];
};

/**
 * @brief 单个电机节点的运行状态缓存。
 */
struct can_protocol_motor_state {
	/**< 当前生效的节点 ID。 */
	uint8_t node_id;
	/**< 最近一次请求设置的节点 ID。 */
	uint8_t requested_node_id;
	/**< 通过读节点 ID 命令获取到的节点 ID。 */
	uint8_t reported_node_id;
	/**< `reported_node_id` 是否有效。 */
	bool reported_node_id_valid;
	/**< 是否存在待完成的节点 ID 切换。 */
	bool pending_node_id_change;
	/**< 最近一次请求的使能状态。 */
	bool requested_enable;
	/**< 最近一次请求的控制模式。 */
	enum can_protocol_control_mode requested_mode;
	/**< 最近一次请求的速度目标，单位 mrad/s。 */
	int32_t requested_speed_target_mrad_s;
	/**< 最近一次请求的位置目标，单位 mrad。 */
	int32_t requested_position_target_mrad;
	/**< 最近一次解析到的位置上报值，单位 mrad。 */
	int32_t position_mrad;
	/**< 最近一次解析到的速度上报值，单位 mrad/s。 */
	int32_t speed_mrad_s;
	/**< 是否已接收到有效位置上报。 */
	bool position_valid;
	/**< 是否已接收到有效速度上报。 */
	bool speed_valid;
	/**< 最近一次有效应答帧。 */
	struct can_protocol_ack last_ack;
	/**< 最近一次应答帧缓存是否有效。 */
	bool ack_valid;
	/**< 最近一次 CAN 发送错误码。 */
	int last_can_error;
	/**< 已成功发送的报文数量。 */
	uint32_t tx_count;
	/**< 已接收并进入解析流程的报文数量。 */
	uint32_t rx_count;
	/**< 已解析的应答帧数量。 */
	uint32_t ack_count;
	/**< 已解析的主动上报帧数量。 */
	uint32_t report_count;
};

/**
 * @brief 上位机侧缓存的协议运行状态。
 */
struct can_protocol_state {
	/**< 当前注册到协议栈中的电机数量。 */
	size_t motor_count;
	/**< 所有已注册电机的状态缓存。 */
	struct can_protocol_motor_state motors[CAN_PROTOCOL_MAX_MOTORS];
	/**< 最近一次记录到的 CAN 控制器状态。 */
	enum can_state bus_state;
	/**< `bus_state` 是否有效。 */
	bool bus_state_valid;
	/**< 最近一次全局 CAN 错误码。 */
	int last_can_error;
	/**< 已成功发送的报文总数。 */
	uint32_t tx_count;
	/**< 已接收并进入解析流程的报文总数。 */
	uint32_t rx_count;
	/**< 已解析的应答帧总数。 */
	uint32_t ack_count;
	/**< 已解析的主动上报帧总数。 */
	uint32_t report_count;
};

/**
 * @brief 初始化 CAN 协议主机侧接口。
 *
 * @param node_ids 电机节点 ID 列表。
 * @param motor_count 列表中的节点数量。
 *
 * @retval 0 初始化成功。
 * @retval -EINVAL 节点列表为空、数量超限或节点 ID 非法/重复。
 * @retval -ENODEV CAN 控制器未就绪。
 * @retval other Zephyr CAN 驱动返回的错误码。
 */
int can_protocol_init(const uint8_t *node_ids, size_t motor_count);

/**
 * @brief 反初始化协议接口并移除接收过滤器。
 */
void can_protocol_deinit(void);

/**
 * @brief 轮询处理接收队列中的所有 CAN 报文。
 *
 * @return 本次调用中处理的报文数量。
 */
int can_protocol_process(void);

/**
 * @brief 解析单个 CAN 报文并更新状态缓存。
 *
 * @param frame 待解析的 CAN 报文。
 *
 * @retval 0 解析成功。
 * @retval -ENOMSG 报文 ID 不属于当前协议实例。
 * @retval -ENOENT 报文节点不在当前管理列表中。
 * @retval -EMSGSIZE 报文长度不满足协议要求。
 * @retval -ENOTSUP 报文格式或命令码不支持。
 * @retval -EINVAL 输入参数无效。
 */
int can_protocol_parse_frame(const struct can_frame *frame);

/**
 * @brief 拷贝当前协议状态快照。
 * @param state 输出的协议状态快照指针。
 * @return void
 */
void can_protocol_copy_state(struct can_protocol_state *state);

/**
 * @brief 获取当前协议状态快照。
 *
 * @return 指向内部状态对象的只读指针。
 */
const struct can_protocol_state *can_protocol_get_state(void);

/**
 * @brief 按节点 ID 获取电机状态。
 *
 * 该接口会同时匹配当前节点 ID 和待切换中的目标节点 ID。
 *
 * @param node_id 电机节点 ID。
 *
 * @return 匹配到的状态对象指针，未找到则返回 `NULL`。
 */
const struct can_protocol_motor_state *can_protocol_get_motor_state(uint8_t node_id);

/**
 * @brief 按索引获取电机状态。
 *
 * @param index 电机索引，范围为 `[0, motor_count)`。
 *
 * @return 匹配到的状态对象指针，未找到则返回 `NULL`。
 */
const struct can_protocol_motor_state *can_protocol_get_motor_state_by_index(size_t index);

/**
 * @brief 设置运行使能和控制模式。
 *
 * @param node_id 目标节点 ID。
 * @param enable `true` 表示运行，`false` 表示停机。
 * @param mode 目标控制模式。
 *
 * @return 0 表示发送成功，负值表示发送失败。
 */
int can_protocol_set_run_mode(uint8_t node_id, bool enable,
			      enum can_protocol_control_mode mode);

/**
 * @brief 设置速度目标。
 *
 * @param node_id 目标节点 ID。
 * @param speed_mrad_s 目标速度，单位 mrad/s。
 *
 * @return 0 表示发送成功，负值表示发送失败。
 */
int can_protocol_set_speed_target(uint8_t node_id, int32_t speed_mrad_s);

/**
 * @brief 设置位置目标。
 *
 * @param node_id 目标节点 ID。
 * @param position_mrad 目标位置，单位 mrad。
 *
 * @return 0 表示发送成功，负值表示发送失败。
 */
int can_protocol_set_position_target(uint8_t node_id, int32_t position_mrad);

/**
 * @brief 写入速度环 PID 参数。
 *
 * @param node_id 目标节点 ID。
 * @param param PID 参数编号。
 * @param value 参数值。`P/I/D` 需按 `CAN_PROTOCOL_PID_GAIN_SCALE` 放大。
 *
 * @return 0 表示发送成功，负值表示发送失败。
 */
int can_protocol_set_speed_pid(uint8_t node_id, enum can_protocol_pid_param param,
			       int32_t value);

/**
 * @brief 请求读取速度环 PID 参数。
 *
 * @param node_id 目标节点 ID。
 * @param param PID 参数编号。
 *
 * @return 0 表示发送成功，负值表示发送失败。
 */
int can_protocol_request_speed_pid(uint8_t node_id, enum can_protocol_pid_param param);

/**
 * @brief 写入位置环 PID 参数。
 *
 * @param node_id 目标节点 ID。
 * @param param PID 参数编号。
 * @param value 参数值。`P/I/D` 需按 `CAN_PROTOCOL_PID_GAIN_SCALE` 放大。
 *
 * @return 0 表示发送成功，负值表示发送失败。
 */
int can_protocol_set_position_pid(uint8_t node_id, enum can_protocol_pid_param param,
				  int32_t value);

/**
 * @brief 请求读取位置环 PID 参数。
 *
 * @param node_id 目标节点 ID。
 * @param param PID 参数编号。
 *
 * @return 0 表示发送成功，负值表示发送失败。
 */
int can_protocol_request_position_pid(uint8_t node_id, enum can_protocol_pid_param param);

/**
 * @brief 请求修改电机节点 ID。
 *
 * @param node_id 当前节点 ID。
 * @param new_node_id 目标新节点 ID。
 *
 * @return 0 表示发送成功，负值表示发送失败。
 */
int can_protocol_set_node_id(uint8_t node_id, uint8_t new_node_id);

/**
 * @brief 请求读取当前节点 ID。
 *
 * @param node_id 目标节点 ID。
 *
 * @return 0 表示发送成功，负值表示发送失败。
 */
int can_protocol_request_node_id(uint8_t node_id);

/**
 * @brief 触发零点校准命令。
 *
 * @param node_id 目标节点 ID。
 *
 * @return 0 表示发送成功，负值表示发送失败。
 */
int can_protocol_zero_calibration(uint8_t node_id);

/**
 * @brief 请求读取零点信息。
 *
 * @param node_id 目标节点 ID。
 *
 * @return 0 表示发送成功，负值表示发送失败。
 */
int can_protocol_request_zero(uint8_t node_id);

#ifdef __cplusplus
}
#endif

#endif /* CAN_PROTOCOL_H_ */
