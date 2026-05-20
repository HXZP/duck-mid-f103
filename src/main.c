/**
 * @file main.c
 * @brief CAN 单电机主机侧演示入口。
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "led/led.h"
#include "motor/motor.h"
#include "uart_command.h"

LOG_MODULE_REGISTER(main_module, LOG_LEVEL_INF);

/** @brief 电机状态日志打印周期，单位毫秒。 */
#define LOG_INTERVAL_MS 100

/** @brief 主循环休眠周期，单位毫秒。 */
#define MAIN_LOOP_SLEEP_MS 20

/** @brief 速度环比例参数。 */
#define SPEED_PID_KP 2000

/** @brief 速度环输出限幅位数。 */
#define OUT_MAX_BIT 15

/** @brief 速度环输出限幅值。 */
#define OUT_MAX ((1 << OUT_MAX_BIT) - 1)

/** @brief 速度模式目标速度，单位 mrad/s。 */
#define SPEED_TARGET_MRAD_S 10000

/** @brief 零点校准等待时间，单位毫秒。 */
#define ZERO_CALIBRATION_WAIT_MS 1000

/** @brief 速度模式运行时间，单位毫秒。 */
#define SPEED_RUN_DURATION_MS 5000

/** @brief 运行结束后下发的电流目标值。 */
#define STOP_CURRENT_TARGET 0

/** @brief 控制命令间隔，单位毫秒。 */
#define COMMAND_SETTLE_MS 100

/** @brief 串口调试消息发送周期，单位毫秒。 */

/** @brief 串口调试消息内容。 */

/**
 * @brief 执行默认电机的演示流程。
 * @param motor 电机对象指针。
 * @return int 0 表示成功，负值表示失败。
 * @note 演示流程包括读取节点 ID、零点校准、设置速度环参数和速度模式运行。
 */
static int app_run_motor_demo(const struct motor_device *motor)
{
    int ret;

    ret = motor_request_node_id(motor);

    if (ret < 0)
    {
        LOG_ERR("Node-id request failed: %d", ret);
        return ret;
    }

    k_sleep(K_MSEC(COMMAND_SETTLE_MS));

    ret = motor_zero_calibration(motor);

    if (ret < 0)
    {
        LOG_ERR("Zero calibration failed: %d", ret);
        return ret;
    }

    LOG_INF("Zero calibration started");
    k_sleep(K_MSEC(ZERO_CALIBRATION_WAIT_MS));

    ret = motor_set_speed_pid(motor, MOTOR_PID_P, SPEED_PID_KP);

    if (ret < 0)
    {
        LOG_ERR("Speed PID Kp set failed: %d", ret);
        return ret;
    }

    ret = motor_set_speed_pid(motor, MOTOR_PID_OUT_MAX, OUT_MAX);

    if (ret < 0)
    {
        LOG_ERR("Speed PID OUT_MAX set failed: %d", ret);
        return ret;
    }

    k_sleep(K_MSEC(COMMAND_SETTLE_MS));

    ret = motor_set_speed_target(motor, SPEED_TARGET_MRAD_S);

    if (ret < 0)
    {
        LOG_ERR("Speed target set failed: %d", ret);
        return ret;
    }

    ret = motor_set_run_mode(motor, true, MOTOR_CONTROL_MODE_SPEED);

    if (ret < 0)
    {
        LOG_ERR("Speed mode start failed: %d", ret);
        return ret;
    }

    LOG_INF("Speed mode running for %d ms", SPEED_RUN_DURATION_MS);
    k_sleep(K_MSEC(SPEED_RUN_DURATION_MS));

    ret = motor_set_run_mode(motor, false, MOTOR_CONTROL_MODE_SPEED);

    if (ret < 0)
    {
        LOG_ERR("Speed mode stop failed: %d", ret);
        return ret;
    }

    k_sleep(K_MSEC(COMMAND_SETTLE_MS));

    ret = motor_set_run_mode(motor, true, MOTOR_CONTROL_MODE_CURRENT);

    if (ret < 0)
    {
        LOG_ERR("Current mode start failed: %d", ret);
        return ret;
    }

    k_sleep(K_MSEC(COMMAND_SETTLE_MS));

    ret = motor_set_current_target(motor, STOP_CURRENT_TARGET);

    if (ret < 0)
    {
        LOG_ERR("Current target clear failed: %d", ret);
        return ret;
    }

    k_sleep(K_MSEC(COMMAND_SETTLE_MS));

    ret = motor_set_run_mode(motor, false, MOTOR_CONTROL_MODE_CURRENT);

    if (ret < 0)
    {
        LOG_ERR("Current mode stop failed: %d", ret);
        return ret;
    }

    LOG_INF("Motor demo completed");
    return 0;
}

/**
 * @brief 应用主入口。
 * @return int 始终返回 0。
 * @note 主循环负责设备初始化、演示控制和状态日志输出。
 */
int main(void)
{
    const struct motor_device *demo_motor;
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

    ret = uart_command_init();

    if (ret < 0)
    {
        LOG_WRN("UART command init failed: %d", ret);
    }

    // ret = motor_init();

    // if (ret < 0)
    // {
    //     LOG_ERR("Motor init failed: %d", ret);
    //     return 0;
    // }

    // demo_motor = motor_get_default();

    // ret = app_run_motor_demo(demo_motor);

    // if (ret < 0)
    // {
    //     LOG_WRN("Motor demo execution failed: %d", ret);
    // }

    while (1)
    {
        // if ((now_ms - last_log_ms) >= LOG_INTERVAL_MS)
        // {
        //     last_log_ms = now_ms;
        //     motor_log_state(demo_motor);
        // }

        k_sleep(K_MSEC(MAIN_LOOP_SLEEP_MS));
    }

    return 0;
}
