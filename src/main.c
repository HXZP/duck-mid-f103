/**
 * @file main.c
 * @brief CAN 协议多电机主机侧示例入口。
 */
#include <inttypes.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "can_protocol.h"
#include "led/led.h"

LOG_MODULE_REGISTER(main_module, LOG_LEVEL_INF);

/** @brief 电机状态日志打印周期，单位毫秒。 */
#define LOG_INTERVAL_MS 1000

/** @brief 主循环休眠周期，单位毫秒。 */
#define MAIN_LOOP_SLEEP_MS 20

/** @brief 示例中的 5 台电机节点 ID。 */
static const uint8_t motor_node_ids[] = {
	0x10,
	0x11,
	0x12,
	0x13,
	0x14,
};

/**
 * @brief 打印电机状态。
 * @return void
 * @note 使用协议层快照，避免直接读取实时状态。
 */
static void log_motor_states(void)
{
	size_t i;
	struct can_protocol_state state;

	can_protocol_copy_state(&state);

	for (i = 0; i < state.motor_count; i++)
	{
		const struct can_protocol_motor_state *motor = &state.motors[i];

		LOG_INF("m%u node=0x%02x pending=%d req_en=%d mode=%u "
			"tgt_speed=%" PRId32 " tgt_pos=%" PRId32
			" speed=%" PRId32 " pos=%" PRId32
			" ack=%s tx=%u rx=%u",
			(unsigned int)i,
			motor->node_id,
			motor->pending_node_id_change,
			motor->requested_enable,
			motor->requested_mode,
			motor->requested_speed_target_mrad_s,
			motor->requested_position_target_mrad,
			motor->speed_valid ? motor->speed_mrad_s : 0,
			motor->position_valid ? motor->position_mrad : 0,
			motor->ack_valid ? "yes" : "no",
			motor->tx_count,
			motor->rx_count);
	}
}

/**
 * @brief 示例主循环。
 * @return int 始终返回 0。
 * @note 协议解析在线程中自动完成，主循环负责应用层初始化和日志打印。
 */
int main(void)
{
	size_t i;
	int ret;
	int64_t last_log_ms = 0;

	LOG_INF("Hello, Duck!!!");

	ret = led_init();

	if (ret < 0)
	{
		LOG_WRN("LED init failed: %d", ret);
	}
	else
	{
		led_boot_blink();
	}

	ret = can_protocol_init(motor_node_ids, ARRAY_SIZE(motor_node_ids));

	if (ret < 0)
	{
		LOG_ERR("CAN protocol init failed: %d", ret);
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(motor_node_ids); i++)
	{
		ret = can_protocol_request_node_id(motor_node_ids[i]);

		if (ret < 0)
		{
			LOG_WRN("Initial node-id request failed for node 0x%02x: %d",
				motor_node_ids[i],
				ret);
		}
	}

	while (1)
	{
		int64_t now_ms = k_uptime_get();

		if ((now_ms - last_log_ms) >= LOG_INTERVAL_MS)
		{
			last_log_ms = now_ms;
			log_motor_states();
		}

		k_sleep(K_MSEC(MAIN_LOOP_SLEEP_MS));
	}

	return 0;
}
