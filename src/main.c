/**
 * @file main.c
 * @brief Duck Mid 应用入口。
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

#include "led.h"
#include "uart_command.h"

LOG_MODULE_REGISTER(main_module, LOG_LEVEL_INF);

/** @brief 主循环休眠周期，单位毫秒。 */
#define MAIN_LOOP_SLEEP_MS 20

/** @brief gen-motor 设备节点。 */
#define MAIN_GEN_MOTOR_NODE DT_NODELABEL(motor_pyr)

/** @brief gen-motor 设备实例。 */
static const struct device *const main_gen_motor_dev = DEVICE_DT_GET(MAIN_GEN_MOTOR_NODE);

/**
 * @brief 应用主入口。
 * @return int 始终返回 0。
 * @note 主循环只负责保持系统运行，电机 demo 已移除。
 */
int main(void)
{
    int ret;

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

    ret = uart_command_init();

    if (ret < 0)
    {
        LOG_WRN("UART command init failed: %d", ret);
    }

    if (!device_is_ready(main_gen_motor_dev))
    {
        LOG_WRN("Gen motor device not ready");
    }

    while (1)
    {
        k_sleep(K_MSEC(MAIN_LOOP_SLEEP_MS));
    }

    return 0;
}
