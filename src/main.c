/**
 * @file main.c
 * @brief Duck Mid 应用入口。
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "gimbal.h"
#include "led.h"
#include "motor.h"
#include "uart_command.h"

LOG_MODULE_REGISTER(main_module, LOG_LEVEL_INF);

/** @brief 主循环休眠周期，单位毫秒。 */
#define MAIN_LOOP_SLEEP_MS 20

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

    ret = motor_init();

    if (ret < 0)
    {
        LOG_WRN("Motor init failed: %d", ret);
    }

    ret = gimbal_init();

    if (ret < 0)
    {
        LOG_WRN("Gimbal init failed: %d", ret);
    }

    while (1)
    {
        k_sleep(K_MSEC(MAIN_LOOP_SLEEP_MS));
    }

    return 0;
}
