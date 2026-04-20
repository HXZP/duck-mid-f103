/**
 * @file can_protocol.c
 * @brief 上位机 CAN 电机控制协议实现。
 */
#include "can_protocol.h"

#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(can_protocol, LOG_LEVEL_INF);

#define CAN_PROTOCOL_HOST_CMD_BASE 0x100U
#define CAN_PROTOCOL_ACK_BASE      0x180U
#define CAN_PROTOCOL_REPORT_BASE   0x200U
#define CAN_PROTOCOL_ID_GROUP_MASK 0x780U
#define CAN_PROTOCOL_RX_QUEUE_LEN  16
#define CAN_PROTOCOL_TX_TIMEOUT    K_MSEC(100)
#define CAN_PROTOCOL_INVALID_FILTER_ID (-1)
#define CAN_PROTOCOL_THREAD_STACK_SIZE 1024
#define CAN_PROTOCOL_THREAD_PRIORITY   5

/** @brief 接收过滤器投递到主循环的 CAN 报文队列。 */
CAN_MSGQ_DEFINE(can_protocol_rx_msgq, CAN_PROTOCOL_RX_QUEUE_LEN);

/** @brief 当前使用的 CAN 控制器设备实例。 */
static const struct device *const can_dev = DEVICE_DT_GET(DT_NODELABEL(can1));

/**
 * @brief 接收过滤器管理状态。
 *
 * 多电机场景下，这里仅保留两组范围过滤器：
 * - `0x180 + node_id` 对应所有应答帧
 * - `0x200 + node_id` 对应所有主动上报帧
 */
struct can_protocol_filter_bank {
	/**< 应答帧过滤器 ID。 */
	int ack_filter_id;
	/**< 主动上报帧过滤器 ID。 */
	int report_filter_id;
};

/**
 * @brief 接收报文的协议类别。
 */
enum can_protocol_frame_kind {
	/**< 未知报文。 */
	CAN_PROTOCOL_FRAME_UNKNOWN = 0,
	/**< 电机应答帧。 */
	CAN_PROTOCOL_FRAME_ACK,
	/**< 电机主动上报帧。 */
	CAN_PROTOCOL_FRAME_REPORT,
};

/** @brief 协议运行时状态缓存。 */
static struct can_protocol_state protocol_state;

/** @brief 协议接收过滤器管理对象。 */
static struct can_protocol_filter_bank filter_bank = {
	.ack_filter_id = CAN_PROTOCOL_INVALID_FILTER_ID,
	.report_filter_id = CAN_PROTOCOL_INVALID_FILTER_ID,
};

/** @brief 协议栈是否已完成初始化。 */
static bool can_protocol_initialized;

/** @brief 协议解析线程是否已启动。 */
static bool can_protocol_thread_started;

/** @brief 协议状态互斥锁。 */
K_MUTEX_DEFINE(can_protocol_mutex);

/** @brief 对外提供的协议状态快照。 */
static struct can_protocol_state protocol_state_snapshot;

/** @brief 协议解析线程栈。 */
K_THREAD_STACK_DEFINE(can_protocol_thread_stack, CAN_PROTOCOL_THREAD_STACK_SIZE);

/** @brief 协议解析线程控制块。 */
static struct k_thread can_protocol_thread_data;

/**
 * @brief 检查节点 ID 是否符合当前协议约束。
 *
 * @param node_id 节点 ID。
 *
 * @retval true 节点 ID 有效。
 * @retval false 节点 ID 超出支持范围。
 */
static bool can_protocol_node_id_is_valid(uint8_t node_id)
{
	return node_id <= CAN_PROTOCOL_MAX_NODE_ID;
}

/**
 * @brief 生成主机下发命令帧 ID。
 *
 * @param node_id 目标节点 ID。
 *
 * @return 标准帧 ID。
 */
static uint32_t can_protocol_host_cmd_id(uint8_t node_id)
{
	return CAN_PROTOCOL_HOST_CMD_BASE + node_id;
}

/**
 * @brief 重置运行时状态缓存。
 */
static void can_protocol_reset_state(void)
{
	memset(&protocol_state, 0, sizeof(protocol_state));
}

/**
 * @brief 判断节点 ID 是否已被其他电机占用。
 *
 * @param node_id 待检查的节点 ID。
 * @param ignore_index 检查时忽略的电机索引。
 *
 * @retval true 节点 ID 已存在。
 * @retval false 节点 ID 可用。
 */
static bool can_protocol_node_id_is_reserved(uint8_t node_id, size_t ignore_index)
{
	size_t i;

	for (i = 0; i < protocol_state.motor_count; i++) {
		const struct can_protocol_motor_state *motor = &protocol_state.motors[i];

		if (i == ignore_index) {
			continue;
		}

		if (motor->node_id == node_id) {
			return true;
		}

		if (motor->pending_node_id_change && motor->requested_node_id == node_id) {
			return true;
		}
	}

	return false;
}

/**
 * @brief 按当前节点 ID 查找电机状态。
 *
 * @param node_id 当前节点 ID。
 *
 * @return 匹配到的状态对象指针，未找到则返回 `NULL`。
 */
static struct can_protocol_motor_state *can_protocol_find_motor_by_current_node(uint8_t node_id)
{
	size_t i;

	for (i = 0; i < protocol_state.motor_count; i++) {
		if (protocol_state.motors[i].node_id == node_id) {
			return &protocol_state.motors[i];
		}
	}

	return NULL;
}

/**
 * @brief 按当前节点或待切换节点查找电机状态。
 *
 * @param node_id 接收报文中的节点 ID。
 *
 * @return 匹配到的状态对象指针，未找到则返回 `NULL`。
 */
static struct can_protocol_motor_state *can_protocol_find_motor_by_any_node(uint8_t node_id)
{
	size_t i;

	for (i = 0; i < protocol_state.motor_count; i++) {
		struct can_protocol_motor_state *motor = &protocol_state.motors[i];

		if (motor->node_id == node_id) {
			return motor;
		}

		if (motor->pending_node_id_change && motor->requested_node_id == node_id) {
			return motor;
		}
	}

	return NULL;
}

/**
 * @brief 获取电机状态在数组中的索引。
 *
 * @param motor 电机状态对象指针。
 *
 * @return 数组索引。
 */
static size_t can_protocol_motor_index(const struct can_protocol_motor_state *motor)
{
	return (size_t)(motor - protocol_state.motors);
}

/**
 * @brief 移除一个已安装的 CAN 接收过滤器。
 *
 * @param filter_id 过滤器 ID 指针。
 */
static void can_protocol_remove_filter(int *filter_id)
{
	if (*filter_id >= 0) {
		can_remove_rx_filter(can_dev, *filter_id);
		*filter_id = CAN_PROTOCOL_INVALID_FILTER_ID;
	}
}

/**
 * @brief 添加一个范围匹配的标准帧过滤器。
 *
 * @param filter_id 输出过滤器 ID。
 * @param id 过滤器基准 ID。
 *
 * @retval 0 添加成功。
 * @retval other Zephyr CAN 驱动返回的错误码。
 */
static int can_protocol_add_group_filter(int *filter_id, uint32_t id)
{
	const struct can_filter filter = {
		.id = id,
		.mask = CAN_PROTOCOL_ID_GROUP_MASK,
		.flags = 0U,
	};
	int ret;

	ret = can_add_rx_filter_msgq(can_dev, &can_protocol_rx_msgq, &filter);
	if (ret < 0) {
		return ret;
	}

	*filter_id = ret;
	return 0;
}

/**
 * @brief 配置多电机协议接收过滤器。
 *
 * @retval 0 配置成功。
 * @retval other Zephyr CAN 驱动返回的错误码。
 */
static int can_protocol_configure_filters(void)
{
	int ret;

	can_protocol_remove_filter(&filter_bank.ack_filter_id);
	can_protocol_remove_filter(&filter_bank.report_filter_id);

	ret = can_protocol_add_group_filter(&filter_bank.ack_filter_id, CAN_PROTOCOL_ACK_BASE);
	if (ret < 0) {
		return ret;
	}

	ret = can_protocol_add_group_filter(&filter_bank.report_filter_id,
					    CAN_PROTOCOL_REPORT_BASE);
	if (ret < 0) {
		can_protocol_remove_filter(&filter_bank.ack_filter_id);
		return ret;
	}

	return 0;
}

/**
 * @brief 解码报文 ID 所属的协议类别和节点。
 *
 * @param frame_id 接收到的标准帧 ID。
 * @param kind 输出报文类别。
 * @param node_id 输出节点 ID。
 *
 * @retval true 解码成功。
 * @retval false 该 ID 不属于当前协议。
 */
static bool can_protocol_decode_frame_id(uint32_t frame_id,
					 enum can_protocol_frame_kind *kind,
					 uint8_t *node_id)
{
	if ((frame_id >= CAN_PROTOCOL_ACK_BASE) &&
	    (frame_id <= (CAN_PROTOCOL_ACK_BASE + CAN_PROTOCOL_MAX_NODE_ID))) {
		*kind = CAN_PROTOCOL_FRAME_ACK;
		*node_id = (uint8_t)(frame_id - CAN_PROTOCOL_ACK_BASE);
		return true;
	}

	if ((frame_id >= CAN_PROTOCOL_REPORT_BASE) &&
	    (frame_id <= (CAN_PROTOCOL_REPORT_BASE + CAN_PROTOCOL_MAX_NODE_ID))) {
		*kind = CAN_PROTOCOL_FRAME_REPORT;
		*node_id = (uint8_t)(frame_id - CAN_PROTOCOL_REPORT_BASE);
		return true;
	}

	*kind = CAN_PROTOCOL_FRAME_UNKNOWN;
	*node_id = 0U;
	return false;
}

/**
 * @brief CAN 控制器状态变化回调。
 *
 * @param dev CAN 设备。
 * @param state 新状态。
 * @param err_cnt 当前错误计数。
 * @param user_data 用户参数，当前未使用。
 */
static void can_protocol_state_change_cb(const struct device *dev, enum can_state state,
					 struct can_bus_err_cnt err_cnt, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	protocol_state.bus_state = state;
	protocol_state.bus_state_valid = true;
	protocol_state.last_can_error = state;

	LOG_WRN("CAN state changed: state=%d tx_err=%u rx_err=%u",
		state, err_cnt.tx_err_cnt, err_cnt.rx_err_cnt);
}

/**
 * @brief 发送完整 8 字节协议命令帧。
 *
 * @param node_id 目标节点 ID。
 * @param payload 协议负载，Byte0 为命令码。
 *
 * @retval 0 发送成功。
 * @retval -ENODEV 协议未初始化。
 * @retval -ENOENT 目标节点未注册。
 * @retval other Zephyr CAN 驱动返回的错误码。
 */
static int can_protocol_send_frame(uint8_t node_id, const uint8_t payload[CAN_MAX_DLC])
{
	struct can_protocol_motor_state *motor;
	struct can_frame frame = {
		.dlc = CAN_MAX_DLC,
		.flags = 0U,
	};
	int ret;

	if (!can_protocol_initialized) {
		return -ENODEV;
	}

	motor = can_protocol_find_motor_by_current_node(node_id);
	if (motor == NULL) {
		return -ENOENT;
	}

	frame.id = can_protocol_host_cmd_id(motor->node_id);
	memcpy(frame.data, payload, CAN_MAX_DLC);

	ret = can_send(can_dev, &frame, CAN_PROTOCOL_TX_TIMEOUT, NULL, NULL);
	if (ret < 0) {
		motor->last_can_error = ret;
		protocol_state.last_can_error = ret;
		LOG_ERR("Failed to send CAN cmd 0x%02x to node 0x%02x: %d",
			payload[0], node_id, ret);
		return ret;
	}

	motor->last_can_error = 0;
	motor->tx_count++;
	protocol_state.tx_count++;
	return 0;
}

/**
 * @brief 发送无参数命令。
 *
 * @param node_id 目标节点 ID。
 * @param cmd 命令码。
 *
 * @return 发送结果。
 */
static int can_protocol_send_simple(uint8_t node_id, uint8_t cmd)
{
	uint8_t payload[CAN_MAX_DLC] = {0};

	payload[0] = cmd;
	return can_protocol_send_frame(node_id, payload);
}

/**
 * @brief 发送携带单个 `int32` 参数的命令。
 *
 * @param node_id 目标节点 ID。
 * @param cmd 命令码。
 * @param value 参数值。
 *
 * @return 发送结果。
 */
static int can_protocol_send_s32(uint8_t node_id, uint8_t cmd, int32_t value)
{
	uint8_t payload[CAN_MAX_DLC] = {0};

	payload[0] = cmd;
	sys_put_le32((uint32_t)value, &payload[1]);
	return can_protocol_send_frame(node_id, payload);
}

/**
 * @brief 发送 PID 参数写命令。
 *
 * @param node_id 目标节点 ID。
 * @param cmd 命令码。
 * @param param PID 参数编号。
 * @param value 参数值。
 *
 * @return 发送结果。
 */
static int can_protocol_send_param_write(uint8_t node_id, uint8_t cmd,
					 enum can_protocol_pid_param param, int32_t value)
{
	uint8_t payload[CAN_MAX_DLC] = {0};

	payload[0] = cmd;
	payload[1] = (uint8_t)param;
	sys_put_le32((uint32_t)value, &payload[2]);
	return can_protocol_send_frame(node_id, payload);
}

/**
 * @brief 发送 PID 参数读命令。
 *
 * @param node_id 目标节点 ID。
 * @param cmd 命令码。
 * @param param PID 参数编号。
 *
 * @return 发送结果。
 */
static int can_protocol_send_param_read(uint8_t node_id, uint8_t cmd,
					enum can_protocol_pid_param param)
{
	uint8_t payload[CAN_MAX_DLC] = {0};

	payload[0] = cmd;
	payload[1] = (uint8_t)param;
	return can_protocol_send_frame(node_id, payload);
}

/**
 * @brief 更新对外提供的协议状态快照。
 * @return void
 * @note 调用前必须已经持有协议状态互斥锁。
 */
static void can_protocol_update_snapshot_locked(void)
{
	memcpy(&protocol_state_snapshot, &protocol_state, sizeof(protocol_state_snapshot));
}

/**
 * @brief 在已持锁状态下解析单个 CAN 报文。
 * @param frame 待解析的 CAN 报文。
 * @retval 0 解析成功。
 * @retval -ENOMSG 报文 ID 不属于当前协议实例。
 * @retval -ENOENT 报文节点不在当前管理列表中。
 * @retval -EMSGSIZE 报文长度不满足协议要求。
 * @retval -ENOTSUP 报文格式或命令码不支持。
 * @retval -EINVAL 输入参数无效。
 * @note 调用前必须已经持有协议状态互斥锁。
 */
static int can_protocol_parse_frame_locked(const struct can_frame *frame)
{
	enum can_protocol_frame_kind kind;
	struct can_protocol_motor_state *motor;
	uint8_t source_node_id = 0U;

	if (frame == NULL)
	{
		return -EINVAL;
	}

	if ((frame->flags & (CAN_FRAME_IDE | CAN_FRAME_RTR | CAN_FRAME_FDF)) != 0U)
	{
		return -ENOTSUP;
	}

	if (!can_protocol_decode_frame_id(frame->id, &kind, &source_node_id))
	{
		return -ENOMSG;
	}

	motor = can_protocol_find_motor_by_any_node(source_node_id);

	if (motor == NULL)
	{
		return -ENOENT;
	}

	motor->rx_count++;
	protocol_state.rx_count++;

	if (kind == CAN_PROTOCOL_FRAME_ACK)
	{
		if (frame->dlc < 2U)
		{
			return -EMSGSIZE;
		}

		motor->last_ack.cmd = frame->data[0];
		motor->last_ack.status = frame->data[1];
		motor->ack_valid = true;
		motor->ack_count++;
		protocol_state.ack_count++;
		memset(motor->last_ack.payload, 0, sizeof(motor->last_ack.payload));

		if (frame->dlc > 2U)
		{
			memcpy(motor->last_ack.payload,
			       &frame->data[2],
			       MIN((size_t)frame->dlc - 2U,
				   sizeof(motor->last_ack.payload)));
		}

		if ((frame->data[0] == CAN_PROTOCOL_CMD_SET_NODE_ID) &&
		    motor->pending_node_id_change)
		{
			if (frame->data[1] == CAN_PROTOCOL_STATUS_OK)
			{
				motor->node_id = motor->requested_node_id;
			}
			else
			{
				motor->requested_node_id = motor->node_id;
			}

			motor->pending_node_id_change = false;
		}

		if ((frame->data[0] == CAN_PROTOCOL_CMD_READ_NODE_ID) &&
		    (frame->data[1] == CAN_PROTOCOL_STATUS_OK))
		{
			motor->reported_node_id = motor->last_ack.payload[0];
			motor->reported_node_id_valid = true;
		}

		LOG_INF("ACK node=0x%02x cmd=0x%02x status=0x%02x",
			source_node_id, frame->data[0], frame->data[1]);
		return 0;
	}

	if (frame->dlc < 5U)
	{
		return -EMSGSIZE;
	}

	switch (frame->data[0])
	{
	case CAN_PROTOCOL_REPORT_POSITION:
		motor->position_mrad = (int32_t)sys_get_le32(&frame->data[1]);
		motor->position_valid = true;
		motor->report_count++;
		protocol_state.report_count++;
		return 0;

	case CAN_PROTOCOL_REPORT_SPEED:
		motor->speed_mrad_s = (int32_t)sys_get_le32(&frame->data[1]);
		motor->speed_valid = true;
		motor->report_count++;
		protocol_state.report_count++;
		return 0;

	default:
		LOG_WRN("Unknown report from node=0x%02x cmd=0x%02x",
			source_node_id, frame->data[0]);
		return -ENOTSUP;
	}
}

/**
 * @brief 在已持锁状态下处理接收队列中的待解析报文。
 * @return int 本次调用中处理的报文数量。
 * @note 调用前必须已经持有协议状态互斥锁。
 */
static int can_protocol_process_pending_locked(void)
{
	struct can_frame frame;
	int processed = 0;

	while (k_msgq_get(&can_protocol_rx_msgq, &frame, K_NO_WAIT) == 0)
	{
		can_protocol_parse_frame_locked(&frame);
		processed++;
	}

	return processed;
}

/**
 * @brief CAN 协议解析线程入口。
 * @param arg1 未使用的线程参数 1。
 * @param arg2 未使用的线程参数 2。
 * @param arg3 未使用的线程参数 3。
 * @return void
 * @note 线程阻塞等待接收队列，当 ISR 投递报文后在中断退出后被唤醒执行解析。
 */
static void can_protocol_thread_entry(void *arg1, void *arg2, void *arg3)
{
	struct can_frame frame;

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	while (1)
	{
		if (k_msgq_get(&can_protocol_rx_msgq, &frame, K_FOREVER) != 0)
		{
			continue;
		}

		k_mutex_lock(&can_protocol_mutex, K_FOREVER);

		if (can_protocol_initialized)
		{
			can_protocol_parse_frame_locked(&frame);
			can_protocol_process_pending_locked();
			can_protocol_update_snapshot_locked();
		}

		k_mutex_unlock(&can_protocol_mutex);
	}
}

/**
 * @brief 启动协议解析线程。
 * @return int 0 表示成功，负值表示失败。
 */
static int can_protocol_start_thread(void)
{
	if (can_protocol_thread_started)
	{
		return 0;
	}

	k_mutex_lock(&can_protocol_mutex, K_FOREVER);
	can_protocol_update_snapshot_locked();
	k_mutex_unlock(&can_protocol_mutex);

	k_thread_create(&can_protocol_thread_data,
			can_protocol_thread_stack,
			K_THREAD_STACK_SIZEOF(can_protocol_thread_stack),
			can_protocol_thread_entry,
			NULL,
			NULL,
			NULL,
			CAN_PROTOCOL_THREAD_PRIORITY,
			0,
			K_NO_WAIT);

	can_protocol_thread_started = true;
	return 0;
}

/**
 * @brief 拷贝当前协议状态快照。
 * @param state 输出的协议状态快照指针。
 * @return void
 */
void can_protocol_copy_state(struct can_protocol_state *state)
{
	if (state == NULL)
	{
		return;
	}

	k_mutex_lock(&can_protocol_mutex, K_FOREVER);
	can_protocol_update_snapshot_locked();
	memcpy(state, &protocol_state_snapshot, sizeof(*state));
	k_mutex_unlock(&can_protocol_mutex);
}

/**
 * @brief 获取当前协议状态快照。
 * @return const struct can_protocol_state * 指向内部快照缓冲区的只读指针。
 */
const struct can_protocol_state *can_protocol_get_state(void)
{
	k_mutex_lock(&can_protocol_mutex, K_FOREVER);
	can_protocol_update_snapshot_locked();
	k_mutex_unlock(&can_protocol_mutex);

	return &protocol_state_snapshot;
}

/**
 * @brief 按节点 ID 获取电机状态快照。
 * @param node_id 电机节点 ID。
 * @return const struct can_protocol_motor_state * 匹配到的状态快照指针，未找到则返回 NULL。
 */
const struct can_protocol_motor_state *can_protocol_get_motor_state(uint8_t node_id)
{
	size_t i;

	k_mutex_lock(&can_protocol_mutex, K_FOREVER);
	can_protocol_update_snapshot_locked();

	for (i = 0; i < protocol_state_snapshot.motor_count; i++)
	{
		struct can_protocol_motor_state *motor = &protocol_state_snapshot.motors[i];

		if (motor->node_id == node_id)
		{
			k_mutex_unlock(&can_protocol_mutex);
			return motor;
		}

		if (motor->pending_node_id_change && (motor->requested_node_id == node_id))
		{
			k_mutex_unlock(&can_protocol_mutex);
			return motor;
		}
	}

	k_mutex_unlock(&can_protocol_mutex);
	return NULL;
}

/**
 * @brief 按索引获取电机状态快照。
 * @param index 电机索引。
 * @return const struct can_protocol_motor_state * 匹配到的状态快照指针，未找到则返回 NULL。
 */
const struct can_protocol_motor_state *can_protocol_get_motor_state_by_index(size_t index)
{
	k_mutex_lock(&can_protocol_mutex, K_FOREVER);
	can_protocol_update_snapshot_locked();

	if (index >= protocol_state_snapshot.motor_count)
	{
		k_mutex_unlock(&can_protocol_mutex);
		return NULL;
	}

	k_mutex_unlock(&can_protocol_mutex);
	return &protocol_state_snapshot.motors[index];
}

int can_protocol_init(const uint8_t *node_ids, size_t motor_count)
{
	size_t i;
	int ret;

	if ((node_ids == NULL) || (motor_count == 0U) ||
	    (motor_count > CAN_PROTOCOL_MAX_MOTORS))
	{
		return -EINVAL;
	}

	for (i = 0; i < motor_count; i++)
	{
		size_t j;

		if (!can_protocol_node_id_is_valid(node_ids[i]))
		{
			return -EINVAL;
		}

		for (j = i + 1U; j < motor_count; j++)
		{
			if (node_ids[i] == node_ids[j])
			{
				return -EINVAL;
			}
		}
	}

	if (!device_is_ready(can_dev))
	{
		return -ENODEV;
	}

	ret = can_protocol_start_thread();

	if (ret != 0)
	{
		return ret;
	}

	if (can_protocol_initialized)
	{
		can_protocol_deinit();
	}

	k_mutex_lock(&can_protocol_mutex, K_FOREVER);
	k_msgq_purge(&can_protocol_rx_msgq);
	can_protocol_reset_state();
	protocol_state.motor_count = motor_count;

	for (i = 0; i < motor_count; i++)
	{
		struct can_protocol_motor_state *motor = &protocol_state.motors[i];

		motor->node_id = node_ids[i];
		motor->requested_node_id = node_ids[i];
		motor->requested_mode = CAN_PROTOCOL_MODE_SPEED;
	}
	can_protocol_update_snapshot_locked();
	k_mutex_unlock(&can_protocol_mutex);

	ret = can_stop(can_dev);

	if ((ret < 0) && (ret != -EALREADY))
	{
		return ret;
	}

	ret = can_set_mode(can_dev, CAN_MODE_NORMAL);

	if (ret < 0)
	{
		return ret;
	}

	ret = can_set_bitrate(can_dev, CAN_PROTOCOL_BAUDRATE);

	if (ret < 0)
	{
		return ret;
	}

	can_set_state_change_callback(can_dev, can_protocol_state_change_cb, NULL);

	ret = can_start(can_dev);

	if (ret < 0)
	{
		return ret;
	}

	ret = can_protocol_configure_filters();

	if (ret < 0)
	{
		(void)can_stop(can_dev);
		return ret;
	}

	k_mutex_lock(&can_protocol_mutex, K_FOREVER);
	can_protocol_initialized = true;
	can_protocol_update_snapshot_locked();
	k_mutex_unlock(&can_protocol_mutex);

	LOG_INF("CAN host init done: motors=%u bitrate=%u",
		(unsigned int)motor_count, CAN_PROTOCOL_BAUDRATE);
	return 0;
}

void can_protocol_deinit(void)
{
	k_mutex_lock(&can_protocol_mutex, K_FOREVER);

	if (!can_protocol_initialized)
	{
		k_mutex_unlock(&can_protocol_mutex);
		return;
	}

	can_protocol_initialized = false;
	can_protocol_update_snapshot_locked();
	k_mutex_unlock(&can_protocol_mutex);

	can_protocol_remove_filter(&filter_bank.ack_filter_id);
	can_protocol_remove_filter(&filter_bank.report_filter_id);
	can_set_state_change_callback(can_dev, NULL, NULL);
	(void)can_stop(can_dev);

	k_msgq_purge(&can_protocol_rx_msgq);

	k_mutex_lock(&can_protocol_mutex, K_FOREVER);
	can_protocol_reset_state();
	can_protocol_update_snapshot_locked();
	k_mutex_unlock(&can_protocol_mutex);
}

int can_protocol_parse_frame(const struct can_frame *frame)
{
	int ret;

	k_mutex_lock(&can_protocol_mutex, K_FOREVER);
	ret = can_protocol_parse_frame_locked(frame);
	can_protocol_update_snapshot_locked();
	k_mutex_unlock(&can_protocol_mutex);

	return ret;
}

int can_protocol_process(void)
{
	int processed;

	k_mutex_lock(&can_protocol_mutex, K_FOREVER);
	processed = can_protocol_process_pending_locked();
	can_protocol_update_snapshot_locked();
	k_mutex_unlock(&can_protocol_mutex);

	return processed;
}

int can_protocol_set_run_mode(uint8_t node_id, bool enable,
			      enum can_protocol_control_mode mode)
{
	struct can_protocol_motor_state *motor;
	uint8_t payload[CAN_MAX_DLC] = {0};
	int ret;

	if (mode > CAN_PROTOCOL_MODE_POSITION)
	{
		return -EINVAL;
	}

	k_mutex_lock(&can_protocol_mutex, K_FOREVER);

	motor = can_protocol_find_motor_by_current_node(node_id);

	if (motor == NULL)
	{
		k_mutex_unlock(&can_protocol_mutex);
		return -ENOENT;
	}

	payload[0] = CAN_PROTOCOL_CMD_SET_RUN_MODE;
	payload[1] = enable ? 1U : 0U;
	payload[2] = (uint8_t)mode;

	ret = can_protocol_send_frame(node_id, payload);

	if (ret == 0)
	{
		motor->requested_enable = enable;
		motor->requested_mode = mode;
	}

	can_protocol_update_snapshot_locked();
	k_mutex_unlock(&can_protocol_mutex);

	return ret;
}

int can_protocol_set_speed_target(uint8_t node_id, int32_t speed_mrad_s)
{
	struct can_protocol_motor_state *motor;
	int ret;

	k_mutex_lock(&can_protocol_mutex, K_FOREVER);

	motor = can_protocol_find_motor_by_current_node(node_id);

	if (motor == NULL)
	{
		k_mutex_unlock(&can_protocol_mutex);
		return -ENOENT;
	}

	ret = can_protocol_send_s32(node_id, CAN_PROTOCOL_CMD_SET_SPEED_TARGET,
				    speed_mrad_s);

	if (ret == 0)
	{
		motor->requested_speed_target_mrad_s = speed_mrad_s;
	}

	can_protocol_update_snapshot_locked();
	k_mutex_unlock(&can_protocol_mutex);

	return ret;
}

int can_protocol_set_position_target(uint8_t node_id, int32_t position_mrad)
{
	struct can_protocol_motor_state *motor;
	int ret;

	k_mutex_lock(&can_protocol_mutex, K_FOREVER);

	motor = can_protocol_find_motor_by_current_node(node_id);

	if (motor == NULL)
	{
		k_mutex_unlock(&can_protocol_mutex);
		return -ENOENT;
	}

	ret = can_protocol_send_s32(node_id, CAN_PROTOCOL_CMD_SET_POSITION_TARGET,
				    position_mrad);

	if (ret == 0)
	{
		motor->requested_position_target_mrad = position_mrad;
	}

	can_protocol_update_snapshot_locked();
	k_mutex_unlock(&can_protocol_mutex);

	return ret;
}

int can_protocol_set_speed_pid(uint8_t node_id, enum can_protocol_pid_param param,
			       int32_t value)
{
	int ret;

	if (param > CAN_PROTOCOL_PID_OUT_MAX)
	{
		return -EINVAL;
	}

	k_mutex_lock(&can_protocol_mutex, K_FOREVER);
	ret = can_protocol_send_param_write(node_id, CAN_PROTOCOL_CMD_SET_SPEED_PID,
					    param, value);
	can_protocol_update_snapshot_locked();
	k_mutex_unlock(&can_protocol_mutex);

	return ret;
}

int can_protocol_request_speed_pid(uint8_t node_id, enum can_protocol_pid_param param)
{
	int ret;

	if (param > CAN_PROTOCOL_PID_OUT_MAX)
	{
		return -EINVAL;
	}

	k_mutex_lock(&can_protocol_mutex, K_FOREVER);
	ret = can_protocol_send_param_read(node_id, CAN_PROTOCOL_CMD_READ_SPEED_PID,
					   param);
	can_protocol_update_snapshot_locked();
	k_mutex_unlock(&can_protocol_mutex);

	return ret;
}

int can_protocol_set_position_pid(uint8_t node_id, enum can_protocol_pid_param param,
				  int32_t value)
{
	int ret;

	if (param > CAN_PROTOCOL_PID_OUT_MAX)
	{
		return -EINVAL;
	}

	k_mutex_lock(&can_protocol_mutex, K_FOREVER);
	ret = can_protocol_send_param_write(node_id,
					    CAN_PROTOCOL_CMD_SET_POSITION_PID,
					    param, value);
	can_protocol_update_snapshot_locked();
	k_mutex_unlock(&can_protocol_mutex);

	return ret;
}

int can_protocol_request_position_pid(uint8_t node_id,
				      enum can_protocol_pid_param param)
{
	int ret;

	if (param > CAN_PROTOCOL_PID_OUT_MAX)
	{
		return -EINVAL;
	}

	k_mutex_lock(&can_protocol_mutex, K_FOREVER);
	ret = can_protocol_send_param_read(node_id,
					   CAN_PROTOCOL_CMD_READ_POSITION_PID,
					   param);
	can_protocol_update_snapshot_locked();
	k_mutex_unlock(&can_protocol_mutex);

	return ret;
}

int can_protocol_set_node_id(uint8_t node_id, uint8_t new_node_id)
{
	struct can_protocol_motor_state *motor;
	uint8_t payload[CAN_MAX_DLC] = {0};
	size_t motor_index;
	int ret;

	if (!can_protocol_node_id_is_valid(new_node_id))
	{
		return -EINVAL;
	}

	k_mutex_lock(&can_protocol_mutex, K_FOREVER);

	motor = can_protocol_find_motor_by_current_node(node_id);

	if (motor == NULL)
	{
		k_mutex_unlock(&can_protocol_mutex);
		return -ENOENT;
	}

	motor_index = can_protocol_motor_index(motor);

	if (can_protocol_node_id_is_reserved(new_node_id, motor_index))
	{
		k_mutex_unlock(&can_protocol_mutex);
		return -EEXIST;
	}

	payload[0] = CAN_PROTOCOL_CMD_SET_NODE_ID;
	payload[1] = new_node_id;

	ret = can_protocol_send_frame(node_id, payload);

	if (ret < 0)
	{
		can_protocol_update_snapshot_locked();
		k_mutex_unlock(&can_protocol_mutex);
		return ret;
	}

	motor->requested_node_id = new_node_id;
	motor->pending_node_id_change = (new_node_id != motor->node_id);

	can_protocol_update_snapshot_locked();
	k_mutex_unlock(&can_protocol_mutex);
	return 0;
}

int can_protocol_request_node_id(uint8_t node_id)
{
	int ret;

	k_mutex_lock(&can_protocol_mutex, K_FOREVER);
	ret = can_protocol_send_simple(node_id, CAN_PROTOCOL_CMD_READ_NODE_ID);
	can_protocol_update_snapshot_locked();
	k_mutex_unlock(&can_protocol_mutex);

	return ret;
}

int can_protocol_zero_calibration(uint8_t node_id)
{
	int ret;

	k_mutex_lock(&can_protocol_mutex, K_FOREVER);
	ret = can_protocol_send_simple(node_id,
				       CAN_PROTOCOL_CMD_ZERO_CALIBRATION);
	can_protocol_update_snapshot_locked();
	k_mutex_unlock(&can_protocol_mutex);

	return ret;
}

int can_protocol_request_zero(uint8_t node_id)
{
	int ret;

	k_mutex_lock(&can_protocol_mutex, K_FOREVER);
	ret = can_protocol_send_simple(node_id, CAN_PROTOCOL_CMD_READ_ZERO);
	can_protocol_update_snapshot_locked();
	k_mutex_unlock(&can_protocol_mutex);

	return ret;
}
