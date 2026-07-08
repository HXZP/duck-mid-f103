/**
 * @file debug_shell.c
 * @brief Duck Mid 调试 shell 命令。
 */
#include <errno.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gen_motor.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include "gimbal.h"

/** @brief gen-motor 设备节点。 */
#define DEBUG_SHELL_GEN_MOTOR_NODE DT_NODELABEL(motor_pyr)

/** @brief 调试 shell 使用的 gen-motor 设备。 */
static const struct device *const debug_shell_gen_motor_dev =
    DEVICE_DT_GET(DEBUG_SHELL_GEN_MOTOR_NODE);

/**
 * @brief 解析有符号 32 位整数。
 * @param text 输入字符串。
 * @param value 输出数值。
 * @return int 0 表示成功，负值表示失败。
 */
static int debug_shell_parse_i32(const char *text, int32_t *value)
{
    int64_t parsed = 0;
    int sign = 1;
    size_t i = 0U;

    if ((text == NULL) || (value == NULL))
    {
        return -EINVAL;
    }

    if (text[0] == '-')
    {
        sign = -1;
        i = 1U;
    }

    if (text[i] == '\0')
    {
        return -EINVAL;
    }

    while (text[i] != '\0')
    {
        if ((text[i] < '0') || (text[i] > '9'))
        {
            return -EINVAL;
        }

        parsed = (parsed * 10) + (text[i] - '0');

        if ((sign > 0) && (parsed > INT32_MAX))
        {
            return -ERANGE;
        }

        if ((sign < 0) && (-parsed < INT32_MIN))
        {
            return -ERANGE;
        }

        i++;
    }

    *value = (int32_t)(parsed * sign);

    return 0;
}

/**
 * @brief 解析无符号 32 位整数。
 * @param text 输入字符串。
 * @param value 输出数值。
 * @return int 0 表示成功，负值表示失败。
 */
static int debug_shell_parse_u32(const char *text, uint32_t *value)
{
    uint64_t parsed = 0U;
    size_t i = 0U;

    if ((text == NULL) || (value == NULL))
    {
        return -EINVAL;
    }

    if (text[0] == '\0')
    {
        return -EINVAL;
    }

    while (text[i] != '\0')
    {
        if ((text[i] < '0') || (text[i] > '9'))
        {
            return -EINVAL;
        }

        parsed = (parsed * 10U) + (uint64_t)(text[i] - '0');

        if (parsed > UINT32_MAX)
        {
            return -ERANGE;
        }

        i++;
    }

    *value = (uint32_t)parsed;

    return 0;
}

/**
 * @brief 按名称解析云台轴。
 * @param text 输入字符串。
 * @param axis_id 输出云台轴编号。
 * @return int 0 表示成功，负值表示失败。
 */
static int debug_shell_parse_axis(const char *text, enum gimbal_axis_id *axis_id)
{
    if ((text == NULL) || (axis_id == NULL))
    {
        return -EINVAL;
    }

    if (strcmp(text, "roll") == 0)
    {
        *axis_id = GIMBAL_AXIS_ROLL;
        return 0;
    }

    if (strcmp(text, "pitch") == 0)
    {
        *axis_id = GIMBAL_AXIS_PITCH;
        return 0;
    }

    if (strcmp(text, "yaw") == 0)
    {
        *axis_id = GIMBAL_AXIS_YAW;
        return 0;
    }

    return -EINVAL;
}

/**
 * @brief 按名称解析 PID 环。
 * @param text 输入字符串。
 * @param loop 输出 PID 环类型。
 * @return int 0 表示成功，负值表示失败。
 */
static int debug_shell_parse_pid_loop(const char *text, enum gimbal_pid_loop *loop)
{
    if ((text == NULL) || (loop == NULL))
    {
        return -EINVAL;
    }

    if (strcmp(text, "pos") == 0)
    {
        *loop = GIMBAL_PID_LOOP_POSITION;
        return 0;
    }

    if (strcmp(text, "position") == 0)
    {
        *loop = GIMBAL_PID_LOOP_POSITION;
        return 0;
    }

    if (strcmp(text, "speed") == 0)
    {
        *loop = GIMBAL_PID_LOOP_SPEED;
        return 0;
    }

    return -EINVAL;
}

/**
 * @brief 获取轴名称。
 * @param axis_id 云台轴编号。
 * @return const char * 轴名称。
 */
static const char *debug_shell_axis_name(enum gimbal_axis_id axis_id)
{
    if (axis_id == GIMBAL_AXIS_ROLL)
    {
        return "roll";
    }

    if (axis_id == GIMBAL_AXIS_PITCH)
    {
        return "pitch";
    }

    if (axis_id == GIMBAL_AXIS_YAW)
    {
        return "yaw";
    }

    return "unknown";
}

/**
 * @brief 获取 PID 环名称。
 * @param loop PID 环类型。
 * @return const char * PID 环名称。
 */
static const char *debug_shell_pid_loop_name(enum gimbal_pid_loop loop)
{
    if (loop == GIMBAL_PID_LOOP_POSITION)
    {
        return "pos";
    }

    if (loop == GIMBAL_PID_LOOP_SPEED)
    {
        return "speed";
    }

    return "unknown";
}

/**
 * @brief 打印单个电机状态。
 * @param shell shell 对象。
 * @param index 电机索引。
 * @return int 0 表示成功，负值表示失败。
 */
static int debug_shell_print_motor_state(const struct shell *shell, size_t index)
{
    const struct gen_motor *motor;
    struct gen_motor_state state;
    int ret;

    motor = gen_motor_get_by_index(debug_shell_gen_motor_dev, index);

    if (motor == NULL)
    {
        shell_error(shell, "m%u missing", (unsigned int)index);
        return -ENODEV;
    }

    ret = gen_motor_copy_state(motor, &state);

    if (ret < 0)
    {
        shell_error(shell, "m%u read %d", (unsigned int)index, ret);
        return ret;
    }

    shell_print(shell,
                "motor %u: pos=%" PRId32 " pos_valid=%u speed=%" PRId32 " speed_valid=%u",
                (unsigned int)index,
                state.position_valid ? state.position_mrad : 0,
                state.position_valid ? 1U : 0U,
                state.speed_valid ? state.speed_mrad_s : 0,
                state.speed_valid ? 1U : 0U);

    return 0;
}

/**
 * @brief shell 命令：打印三个电机位置和速度。
 * @param shell shell 对象。
 * @param argc 参数数量。
 * @param argv 参数数组。
 * @return int 0 表示成功，负值表示失败。
 */
static int debug_shell_cmd_motor_pos(const struct shell *shell, size_t argc, char **argv)
{
    size_t motor_count;
    size_t i;

    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    if (!device_is_ready(debug_shell_gen_motor_dev))
    {
        shell_error(shell, "motor dev not ready");
        return -ENODEV;
    }

    motor_count = gen_motor_get_count(debug_shell_gen_motor_dev);

    for (i = 0U; i < motor_count; i++)
    {
        debug_shell_print_motor_state(shell, i);
    }

    return 0;
}

/**
 * @brief shell 命令：设置云台目标角。
 * @param shell shell 对象。
 * @param argc 参数数量。
 * @param argv 参数数组。
 * @return int 0 表示成功，负值表示失败。
 */
static int debug_shell_cmd_gimbal_target(const struct shell *shell, size_t argc, char **argv)
{
    int32_t roll_mrad;
    int32_t pitch_mrad;
    int32_t yaw_mrad;
    int ret;

    ARG_UNUSED(argc);

    ret = debug_shell_parse_i32(argv[1], &roll_mrad);

    if (ret < 0)
    {
        shell_error(shell, "bad roll");
        return ret;
    }

    ret = debug_shell_parse_i32(argv[2], &pitch_mrad);

    if (ret < 0)
    {
        shell_error(shell, "bad pitch");
        return ret;
    }

    ret = debug_shell_parse_i32(argv[3], &yaw_mrad);

    if (ret < 0)
    {
        shell_error(shell, "bad yaw");
        return ret;
    }

    ret = gimbal_set_rpy_target(roll_mrad,
                                pitch_mrad,
                                yaw_mrad,
                                GIMBAL_TARGET_SOURCE_EXTERNAL);

    if (ret < 0)
    {
        shell_error(shell, "target %d", ret);
        return ret;
    }

    shell_print(shell,
                "gimbal target set: roll=%" PRId32 " pitch=%" PRId32 " yaw=%" PRId32,
                roll_mrad,
                pitch_mrad,
                yaw_mrad);

    return 0;
}

/**
 * @brief shell 命令：关闭云台输出。
 * @param shell shell 对象。
 * @param argc 参数数量。
 * @param argv 参数数组。
 * @return int 0 表示成功，负值表示失败。
 */
static int debug_shell_cmd_gimbal_disable(const struct shell *shell, size_t argc, char **argv)
{
    int ret;

    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    ret = gimbal_disable();

    if (ret < 0)
    {
        shell_error(shell, "disable %d", ret);
        return ret;
    }

    shell_print(shell, "gimbal disabled");

    return 0;
}

/**
 * @brief 打印指定 PID 配置。
 * @param shell shell 对象。
 * @param axis_id 云台轴编号。
 * @param loop PID 环类型。
 * @param config PID 百分比配置。
 * @return void
 */
static void debug_shell_print_pid_config(const struct shell *shell,
                                         enum gimbal_axis_id axis_id,
                                         enum gimbal_pid_loop loop,
                                         const struct gimbal_pid_percent_config *config)
{
    if (config == NULL)
    {
        return;
    }

    shell_print(shell,
                "pid %s %s: kp=%" PRId32 " ki=%" PRId32 " kd=%" PRId32
                " integral_limit=%u%% output_limit=%u%%",
                debug_shell_axis_name(axis_id),
                debug_shell_pid_loop_name(loop),
                config->kp,
                config->ki,
                config->kd,
                (unsigned int)config->integral_limit_percent,
                (unsigned int)config->output_limit_percent);
}

/**
 * @brief shell 命令：获取 PID 配置。
 * @param shell shell 对象。
 * @param argc 参数数量。
 * @param argv 参数数组。
 * @return int 0 表示成功，负值表示失败。
 */
static int debug_shell_cmd_gimbal_pid_get(const struct shell *shell, size_t argc, char **argv)
{
    struct gimbal_pid_percent_config config;
    enum gimbal_axis_id axis_id;
    enum gimbal_pid_loop loop;
    int ret;

    ARG_UNUSED(argc);

    ret = debug_shell_parse_axis(argv[1], &axis_id);

    if (ret < 0)
    {
        shell_error(shell, "bad axis");
        return ret;
    }

    ret = debug_shell_parse_pid_loop(argv[2], &loop);

    if (ret < 0)
    {
        shell_error(shell, "bad loop");
        return ret;
    }

    ret = gimbal_get_pid_percent_config(axis_id, loop, &config);

    if (ret < 0)
    {
        shell_error(shell, "pid get %d", ret);
        return ret;
    }

    debug_shell_print_pid_config(shell, axis_id, loop, &config);

    return 0;
}

/**
 * @brief shell 命令：设置 PID 配置。
 * @param shell shell 对象。
 * @param argc 参数数量。
 * @param argv 参数数组。
 * @return int 0 表示成功，负值表示失败。
 */
static int debug_shell_cmd_gimbal_pid_set(const struct shell *shell, size_t argc, char **argv)
{
    struct gimbal_pid_percent_config config;
    enum gimbal_axis_id axis_id;
    enum gimbal_pid_loop loop;
    int ret;

    ARG_UNUSED(argc);

    ret = debug_shell_parse_axis(argv[1], &axis_id);

    if (ret < 0)
    {
        shell_error(shell, "bad axis");
        return ret;
    }

    ret = debug_shell_parse_pid_loop(argv[2], &loop);

    if (ret < 0)
    {
        shell_error(shell, "bad loop");
        return ret;
    }

    ret = debug_shell_parse_i32(argv[3], &config.kp);

    if (ret < 0)
    {
        shell_error(shell, "bad kp");
        return ret;
    }

    ret = debug_shell_parse_i32(argv[4], &config.ki);

    if (ret < 0)
    {
        shell_error(shell, "bad ki");
        return ret;
    }

    ret = debug_shell_parse_i32(argv[5], &config.kd);

    if (ret < 0)
    {
        shell_error(shell, "bad kd");
        return ret;
    }

    ret = debug_shell_parse_u32(argv[6], &config.integral_limit_percent);

    if (ret < 0)
    {
        shell_error(shell, "bad ilimit");
        return ret;
    }

    ret = debug_shell_parse_u32(argv[7], &config.output_limit_percent);

    if (ret < 0)
    {
        shell_error(shell, "bad olimit");
        return ret;
    }

    ret = gimbal_set_pid_percent_config(axis_id, loop, &config);

    if (ret < 0)
    {
        shell_error(shell, "pid set %d", ret);
        return ret;
    }

    debug_shell_print_pid_config(shell, axis_id, loop, &config);

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(debug_shell_motor_cmds,
    SHELL_CMD_ARG(pos, NULL, NULL, debug_shell_cmd_motor_pos, 1, 0),
    SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(debug_shell_gimbal_pid_cmds,
    SHELL_CMD_ARG(get,
                  NULL,
                  NULL,
                  debug_shell_cmd_gimbal_pid_get,
                  3,
                  0),
    SHELL_CMD_ARG(set,
                  NULL,
                  NULL,
                  debug_shell_cmd_gimbal_pid_set,
                  8,
                  0),
    SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(debug_shell_gimbal_cmds,
    SHELL_CMD_ARG(pid,
                  &debug_shell_gimbal_pid_cmds,
                  NULL,
                  NULL,
                  1,
                  0),
    SHELL_CMD_ARG(target,
                  NULL,
                  NULL,
                  debug_shell_cmd_gimbal_target,
                  4,
                  0),
    SHELL_CMD_ARG(disable, NULL, NULL, debug_shell_cmd_gimbal_disable, 1, 0),
    SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(motor, &debug_shell_motor_cmds, NULL, NULL);
SHELL_CMD_REGISTER(gimbal, &debug_shell_gimbal_cmds, NULL, NULL);
