/**
 * @file motor.c
 * @brief Duck Mid 电机应用管理实现。
 */
#include "motor.h"

#include <errno.h>
#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gen_motor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(motor_app, LOG_LEVEL_INF);

/** @brief 电机上报频率，单位 Hz。 */
#define MOTOR_APP_REPORT_HZ 500U

/** @brief 电机上报周期，单位毫秒。 */
#define MOTOR_APP_REPORT_PERIOD_MS (1000U / MOTOR_APP_REPORT_HZ)

/** @brief 电机配置重试周期，单位毫秒。 */
#define MOTOR_APP_CONFIG_RETRY_MS 500U

/** @brief 电机配置线程栈大小，单位字节。 */
#define MOTOR_APP_CONFIG_STACK_SIZE 1024U

/** @brief 电机配置线程优先级。 */
#define MOTOR_APP_CONFIG_PRIORITY 8

/** @brief gen-motor 设备节点。 */
#define MOTOR_APP_GEN_MOTOR_NODE DT_NODELABEL(motor_pyr)

/** @brief gen-motor 设备实例。 */
static const struct device *const motor_app_gen_motor_dev =
    DEVICE_DT_GET(MOTOR_APP_GEN_MOTOR_NODE);

/** @brief 电机配置线程栈。 */
K_THREAD_STACK_DEFINE(motor_app_config_stack, MOTOR_APP_CONFIG_STACK_SIZE);

/** @brief 电机配置线程控制块。 */
static struct k_thread motor_app_config_thread;

/** @brief 电机配置线程是否已经启动。 */
static bool motor_app_config_thread_started;

/**
 * @brief 判断电机版本号是否已经读回。
 * @param motor 电机对象。
 * @return bool true 表示版本号有效，false 表示版本号无效。
 */
static bool motor_app_version_is_ready(const struct gen_motor *motor)
{
    struct gen_motor_state state;
    int ret;

    ret = gen_motor_copy_state(motor, &state);

    if (ret < 0)
    {
        return false;
    }

    return state.app_version_valid;
}

/**
 * @brief 判断电机主动上报配置是否已经生效。
 * @param motor 电机对象。
 * @return bool true 表示主动上报配置已生效，false 表示仍需配置。
 */
static bool motor_app_report_config_is_ready(const struct gen_motor *motor)
{
    struct gen_motor_state state;
    int ret;

    ret = gen_motor_copy_state(motor, &state);

    if (ret < 0)
    {
        return false;
    }

    if (!state.report_config_valid)
    {
        return false;
    }

    if (!state.report_enabled)
    {
        return false;
    }

    if (state.report_period_ms != MOTOR_APP_REPORT_PERIOD_MS)
    {
        return false;
    }

    return true;
}

/**
 * @brief 请求读取单个电机版本号。
 * @param motor 电机对象。
 * @param index 电机索引。
 * @return void
 */
static void motor_app_request_version_if_needed(const struct gen_motor *motor, size_t index)
{
    int ret;

    if (motor_app_version_is_ready(motor))
    {
        return;
    }

    ret = gen_motor_request_app_version(motor);

    if (ret < 0)
    {
        LOG_WRN("Motor %u version request failed: %d", (unsigned int)index, ret);
    }
}

/**
 * @brief 设置单个电机主动上报配置。
 * @param motor 电机对象。
 * @param index 电机索引。
 * @return void
 */
static void motor_app_set_report_if_needed(const struct gen_motor *motor, size_t index)
{
    int ret;

    if (motor_app_report_config_is_ready(motor))
    {
        return;
    }

    ret = gen_motor_set_report_config(motor, true, MOTOR_APP_REPORT_PERIOD_MS);

    if (ret < 0)
    {
        LOG_WRN("Motor %u set report %u Hz failed: %d",
                (unsigned int)index,
                (unsigned int)MOTOR_APP_REPORT_HZ,
                ret);
    }
}

/**
 * @brief 判断单个电机启动配置是否完成。
 * @param motor 电机对象。
 * @return bool true 表示配置完成，false 表示仍需重试。
 */
static bool motor_app_motor_is_ready(const struct gen_motor *motor)
{
    if (!motor_app_version_is_ready(motor))
    {
        return false;
    }

    if (!motor_app_report_config_is_ready(motor))
    {
        return false;
    }

    return true;
}

/**
 * @brief 执行一轮电机启动配置。
 * @param motor_count 电机数量。
 * @return bool true 表示全部电机配置完成，false 表示仍需重试。
 */
static bool motor_app_configure_once(size_t motor_count)
{
    const struct gen_motor *motor;
    bool all_ready = true;
    size_t i;

    for (i = 0U; i < motor_count; i++)
    {
        motor = gen_motor_get_by_index(motor_app_gen_motor_dev, i);

        if (motor == NULL)
        {
            LOG_WRN("Motor %u is not registered", (unsigned int)i);
            all_ready = false;
            continue;
        }

        motor_app_request_version_if_needed(motor, i);
        motor_app_set_report_if_needed(motor, i);

        if (!motor_app_motor_is_ready(motor))
        {
            all_ready = false;
        }
    }

    return all_ready;
}

/**
 * @brief 电机启动配置线程入口。
 * @param arg1 线程参数 1。
 * @param arg2 线程参数 2。
 * @param arg3 线程参数 3。
 * @return void
 */
static void motor_app_config_thread_entry(void *arg1, void *arg2, void *arg3)
{
    size_t motor_count;

    ARG_UNUSED(arg1);
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    if (!device_is_ready(motor_app_gen_motor_dev))
    {
        LOG_WRN("gen-motor device is not ready");
        return;
    }

    motor_count = gen_motor_get_count(motor_app_gen_motor_dev);
    LOG_INF("Configure %u motors: report=%u Hz period=%u ms",
            (unsigned int)motor_count,
            (unsigned int)MOTOR_APP_REPORT_HZ,
            (unsigned int)MOTOR_APP_REPORT_PERIOD_MS);

    while (1)
    {
        if (motor_app_configure_once(motor_count))
        {
            LOG_INF("Motor boot config done");
            return;
        }

        k_sleep(K_MSEC(MOTOR_APP_CONFIG_RETRY_MS));
    }
}

/**
 * @brief 初始化电机应用管理模块。
 * @return int 0 表示成功，负值表示失败。
 */
int motor_init(void)
{
    if (motor_app_config_thread_started)
    {
        return 0;
    }

    k_thread_create(&motor_app_config_thread,
                    motor_app_config_stack,
                    K_THREAD_STACK_SIZEOF(motor_app_config_stack),
                    motor_app_config_thread_entry,
                    NULL,
                    NULL,
                    NULL,
                    MOTOR_APP_CONFIG_PRIORITY,
                    0,
                    K_NO_WAIT);

    if (IS_ENABLED(CONFIG_THREAD_NAME))
    {
        k_thread_name_set(&motor_app_config_thread, "motor_config");
    }

    motor_app_config_thread_started = true;

    return 0;
}
