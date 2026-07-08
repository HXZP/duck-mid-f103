/**
 * @file gimbal.c
 * @brief 云台控制模块实现。
 */
#include "gimbal.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gen_motor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(gimbal, LOG_LEVEL_INF);

/** @brief 云台控制线程栈大小，单位字节。 */
#define GIMBAL_CONTROL_STACK_SIZE 1024U

/** @brief 云台控制线程优先级。 */
#define GIMBAL_CONTROL_PRIORITY 7

/** @brief PID 增益缩放倍数。 */
#define GIMBAL_PID_GAIN_SCALE 1000

/** @brief gen-motor 设备节点。 */
#define GIMBAL_GEN_MOTOR_NODE DT_NODELABEL(motor_pyr)

/**
 * @brief 云台位置反馈来源。
 */
typedef enum
{
    /**< 使用 RK Roll 姿态反馈。 */
    GIMBAL_POSITION_FEEDBACK_ATTITUDE_ROLL = 0,
    /**< 使用 RK Pitch 姿态反馈。 */
    GIMBAL_POSITION_FEEDBACK_ATTITUDE_PITCH,
    /**< 使用 RK Yaw 姿态反馈。 */
    GIMBAL_POSITION_FEEDBACK_ATTITUDE_YAW,
    /**< 使用电机角度反馈。 */
    GIMBAL_POSITION_FEEDBACK_MOTOR,
} Gimbal_PositionFeedbackSource_t;

/**
 * @brief 云台速度反馈来源。
 */
typedef enum
{
    /**< 使用 IMU Roll 角速度反馈。 */
    GIMBAL_SPEED_FEEDBACK_IMU_ROLL = 0,
    /**< 使用 IMU Pitch 角速度反馈。 */
    GIMBAL_SPEED_FEEDBACK_IMU_PITCH,
    /**< 使用 IMU Yaw 角速度反馈。 */
    GIMBAL_SPEED_FEEDBACK_IMU_YAW,
    /**< 使用电机速度反馈。 */
    GIMBAL_SPEED_FEEDBACK_MOTOR,
} Gimbal_SpeedFeedbackSource_t;

/**
 * @brief PID 配置。
 */
typedef struct
{
    /**< 比例增益，按 GIMBAL_PID_GAIN_SCALE 缩放。 */
    int32_t kp;
    /**< 积分增益，按 GIMBAL_PID_GAIN_SCALE 缩放。 */
    int32_t ki;
    /**< 微分增益，按 GIMBAL_PID_GAIN_SCALE 缩放。 */
    int32_t kd;
    /**< 积分限幅，0 表示不积分。 */
    int32_t integral_limit;
    /**< 输出限幅，0 表示不限制。 */
    int32_t output_limit;
} Gimbal_PidConfig_t;

/**
 * @brief PID 运行对象。
 */
typedef struct
{
    /**< PID 配置。 */
    Gimbal_PidConfig_t config;
    /**< 积分项。 */
    int32_t integral;
    /**< 上一次误差。 */
    int32_t last_error;
    /**< 上一次误差是否有效。 */
    bool last_error_valid;
} Gimbal_Pid_t;

/**
 * @brief 云台控制轴配置。
 */
typedef struct
{
    /**< 轴名称。 */
    const char *name;
    /**< gen-motor 中的电机索引。 */
    size_t motor_index;
    /**< 位置反馈来源。 */
    Gimbal_PositionFeedbackSource_t position_feedback_source;
    /**< 速度反馈来源。 */
    Gimbal_SpeedFeedbackSource_t speed_feedback_source;
    /**< 位置环 PID 配置。 */
    Gimbal_PidConfig_t position_pid;
    /**< 速度环 PID 配置。 */
    Gimbal_PidConfig_t speed_pid;
} Gimbal_AxisConfig_t;

/**
 * @brief 云台控制轴对象。
 */
typedef struct
{
    /**< 轴配置。 */
    const Gimbal_AxisConfig_t *config;
    /**< 电机对象。 */
    const struct gen_motor *motor;
    /**< 目标角度，单位 mrad。 */
    int32_t target_mrad;
    /**< 位置环 PID。 */
    Gimbal_Pid_t position_pid;
    /**< 速度环 PID。 */
    Gimbal_Pid_t speed_pid;
    /**< 位置环输出的目标速度，单位 mrad/s。 */
    int32_t target_speed_mrad_s;
    /**< 最近一次位置环更新时间，单位毫秒。 */
    uint32_t last_position_update_ms;
    /**< 最近一次电流目标。 */
    int32_t last_current_target;
    /**< 是否已经设置电流模式。 */
    bool current_mode_requested;
    /**< 目标是否有效。 */
    bool target_valid;
    /**< 位置环输出目标速度是否有效。 */
    bool target_speed_valid;
} Gimbal_Axis_t;

/**
 * @brief RK 姿态反馈对象。
 */
typedef struct
{
    /**< 当前 Roll 姿态角，单位 mrad。 */
    int32_t roll_mrad;
    /**< 当前 Pitch 姿态角，单位 mrad。 */
    int32_t pitch_mrad;
    /**< 当前 Yaw 姿态角，单位 mrad。 */
    int32_t yaw_mrad;
    /**< 当前 Roll 角速度，单位 mrad/s。 */
    int32_t roll_speed_mrad_s;
    /**< 当前 Pitch 角速度，单位 mrad/s。 */
    int32_t pitch_speed_mrad_s;
    /**< 当前 Yaw 角速度，单位 mrad/s。 */
    int32_t yaw_speed_mrad_s;
    /**< 姿态反馈更新时间，单位毫秒。 */
    uint32_t update_ms;
    /**< 姿态反馈是否有效。 */
    bool valid;
    /**< 角速度反馈是否有效。 */
    bool speed_valid;
} Gimbal_Attitude_t;

/**
 * @brief 云台控制器配置。
 */
typedef struct
{
    /**< 控制器名称。 */
    const char *name;
    /**< gen-motor 设备。 */
    const struct device *gen_motor_dev;
    /**< 速度环控制周期，单位毫秒。 */
    uint32_t speed_control_period_ms;
    /**< 位置环控制周期，单位毫秒。 */
    uint32_t position_control_period_ms;
    /**< RK 姿态反馈超时时间，单位毫秒。 */
    uint32_t attitude_timeout_ms;
    /**< 角度整圈范围，单位 mrad。 */
    int32_t full_circle_mrad;
    /**< 位置环积分限幅百分比基准，单位 mrad。 */
    int32_t position_integral_limit_base;
    /**< 位置环输出限幅百分比基准，单位 mrad/s。 */
    int32_t position_output_limit_base;
    /**< 速度环积分限幅百分比基准，单位 mrad/s。 */
    int32_t speed_integral_limit_base;
    /**< 速度环输出限幅百分比基准，单位电流控制量。 */
    int32_t speed_output_limit_base;
    /**< 轴配置表。 */
    const Gimbal_AxisConfig_t *axis_configs;
    /**< 轴数量。 */
    size_t axis_count;
} Gimbal_ControllerConfig_t;

/**
 * @brief 云台控制器对象。
 */
typedef struct
{
    /**< 控制器配置。 */
    const Gimbal_ControllerConfig_t *config;
    /**< 轴对象表。 */
    Gimbal_Axis_t axes[GIMBAL_AXIS_COUNT];
    /**< RK 姿态反馈。 */
    Gimbal_Attitude_t attitude;
    /**< 对象互斥锁。 */
    struct k_mutex lock;
    /**< 控制线程控制块。 */
    struct k_thread control_thread;
    /**< 最新目标来源。 */
    enum gimbal_target_source target_source;
    /**< 控制线程是否已经启动。 */
    bool thread_started;
    /**< 控制器是否已经初始化。 */
    bool initialized;
} Gimbal_Controller_t;

/** @brief 云台控制轴配置表。 */
static const Gimbal_AxisConfig_t gimbal_axis_configs[GIMBAL_AXIS_COUNT] = {
    {
        .name = "roll",
        .motor_index = 2U,
        .position_feedback_source = GIMBAL_POSITION_FEEDBACK_MOTOR,
        .speed_feedback_source = GIMBAL_SPEED_FEEDBACK_MOTOR,
        .position_pid = {
            .kp = 0 ,
            .ki = 0,
            .kd = 0,
            .integral_limit = 0,
            .output_limit = 0,
        },
        .speed_pid = {
            .kp = 0,
            .ki = 0,
            .kd = 0,
            .integral_limit = 0,
            .output_limit = 0,
        },
    },
    {
        .name = "pitch",
        .motor_index = 0U,
        .position_feedback_source = GIMBAL_POSITION_FEEDBACK_MOTOR,
        .speed_feedback_source = GIMBAL_SPEED_FEEDBACK_MOTOR,
        .position_pid = {
            .kp = 0,
            .ki = 0,
            .kd = 0,
            .integral_limit = 0,
            .output_limit = 0,
        },
        .speed_pid = {
            .kp = 0,
            .ki = 0,
            .kd = 0,
            .integral_limit = 0,
            .output_limit = 0,
        },
    },
    {
        .name = "yaw",
        .motor_index = 1U,
        .position_feedback_source = GIMBAL_POSITION_FEEDBACK_MOTOR,
        .speed_feedback_source = GIMBAL_SPEED_FEEDBACK_MOTOR,
        .position_pid = {
            .kp = 0,
            .ki = 0,
            .kd = 0,
            .integral_limit = 0,
            .output_limit = 0,
        },
        .speed_pid = {
            .kp = 0,
            .ki = 0,
            .kd = 0,
            .integral_limit = 0,
            .output_limit = 0,
        },
    },
};

/** @brief 默认云台控制器配置。 */
static const Gimbal_ControllerConfig_t gimbal_default_config = {
    .name = "gimbal",
    .gen_motor_dev = DEVICE_DT_GET(GIMBAL_GEN_MOTOR_NODE),
    .speed_control_period_ms = 2U,
    .position_control_period_ms = 10U,
    .attitude_timeout_ms = 100U,
    .full_circle_mrad = 6283,
    .position_integral_limit_base = 6283,
    .position_output_limit_base = 6283,
    .speed_integral_limit_base = 6283,
    .speed_output_limit_base = 32767,
    .axis_configs = gimbal_axis_configs,
    .axis_count = GIMBAL_AXIS_COUNT,
};

/** @brief 云台控制线程栈。 */
K_THREAD_STACK_DEFINE(gimbal_control_stack, GIMBAL_CONTROL_STACK_SIZE);

/** @brief 默认云台控制器对象。 */
static Gimbal_Controller_t gimbal_controller;

/**
 * @brief 判断云台控制器是否可用。
 * @param controller 云台控制器对象。
 * @return bool true 表示可用，false 表示不可用。
 */
static bool gimbal_controller_is_ready(const Gimbal_Controller_t *controller)
{
    if (controller == NULL)
    {
        return false;
    }

    if (!controller->initialized)
    {
        return false;
    }

    if (controller->config == NULL)
    {
        return false;
    }

    return true;
}

/**
 * @brief 将 64 位数值限幅为 32 位范围。
 * @param value 输入数值。
 * @param min 最小值。
 * @param max 最大值。
 * @return int32_t 限幅后的数值。
 */
static int32_t gimbal_clamp_i64(int64_t value, int32_t min, int32_t max)
{
    if (value < min)
    {
        return min;
    }

    if (value > max)
    {
        return max;
    }

    return (int32_t)value;
}

/**
 * @brief 将有符号数值按对称限幅处理。
 * @param value 输入数值。
 * @param limit 正向限幅值，0 表示不限幅。
 * @return int32_t 限幅后的数值。
 */
static int32_t gimbal_clamp_symmetric_i64(int64_t value, int32_t limit)
{
    if (limit <= 0)
    {
        return (int32_t)value;
    }

    return gimbal_clamp_i64(value, -limit, limit);
}

/**
 * @brief 将百分比换算为实际限幅。
 * @param percent 百分比数值。
 * @param base 百分比基准值。
 * @return int32_t 实际限幅值，0 表示关闭或不限幅。
 */
static int32_t gimbal_percent_to_limit(uint32_t percent, int32_t base)
{
    int64_t value;

    if (percent == 0U)
    {
        return 0;
    }

    if (base <= 0)
    {
        return 0;
    }

    value = ((int64_t)base * percent) / 100;

    return gimbal_clamp_i64(value, 0, INT32_MAX);
}

/**
 * @brief 将实际限幅反算为百分比。
 * @param limit 实际限幅值。
 * @param base 百分比基准值。
 * @return uint32_t 百分比数值。
 */
static uint32_t gimbal_limit_to_percent(int32_t limit, int32_t base)
{
    int64_t percent;

    if (limit <= 0)
    {
        return 0U;
    }

    if (base <= 0)
    {
        return 0U;
    }

    percent = ((int64_t)limit * 100) / base;

    if (percent < 0)
    {
        return 0U;
    }

    if (percent > UINT32_MAX)
    {
        return UINT32_MAX;
    }

    return (uint32_t)percent;
}

/**
 * @brief 复位 PID 运行状态。
 * @param pid PID 对象。
 * @return void
 */
static void gimbal_pid_reset(Gimbal_Pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral = 0;
    pid->last_error = 0;
    pid->last_error_valid = false;
}

/**
 * @brief 初始化 PID 对象。
 * @param pid PID 对象。
 * @param config PID 配置。
 * @return int 0 表示成功，负值表示失败。
 */
static int gimbal_pid_init(Gimbal_Pid_t *pid, const Gimbal_PidConfig_t *config)
{
    if ((pid == NULL) || (config == NULL))
    {
        return -EINVAL;
    }

    memset(pid, 0, sizeof(*pid));
    pid->config = *config;

    return 0;
}

/**
 * @brief 判断轴编号是否有效。
 * @param axis_id 轴编号。
 * @return bool true 表示有效，false 表示无效。
 */
static bool gimbal_axis_id_is_valid(enum gimbal_axis_id axis_id)
{
    if ((int)axis_id < 0)
    {
        return false;
    }

    if (axis_id >= GIMBAL_AXIS_COUNT)
    {
        return false;
    }

    return true;
}

/**
 * @brief 获取指定轴的指定 PID 环。
 * @param axis 轴对象。
 * @param loop PID 环类型。
 * @return Gimbal_Pid_t * 成功返回 PID 对象，失败返回 NULL。
 */
static Gimbal_Pid_t *gimbal_axis_get_pid(Gimbal_Axis_t *axis, enum gimbal_pid_loop loop)
{
    if (axis == NULL)
    {
        return NULL;
    }

    if (loop == GIMBAL_PID_LOOP_POSITION)
    {
        return &axis->position_pid;
    }

    if (loop == GIMBAL_PID_LOOP_SPEED)
    {
        return &axis->speed_pid;
    }

    return NULL;
}

/**
 * @brief 获取指定 PID 环的积分限幅百分比基准。
 * @param config 云台控制器配置。
 * @param loop PID 环类型。
 * @return int32_t 百分比基准值。
 */
static int32_t gimbal_get_integral_limit_base(const Gimbal_ControllerConfig_t *config,
                                              enum gimbal_pid_loop loop)
{
    if (config == NULL)
    {
        return 0;
    }

    if (loop == GIMBAL_PID_LOOP_POSITION)
    {
        return config->position_integral_limit_base;
    }

    if (loop == GIMBAL_PID_LOOP_SPEED)
    {
        return config->speed_integral_limit_base;
    }

    return 0;
}

/**
 * @brief 获取指定 PID 环的输出限幅百分比基准。
 * @param config 云台控制器配置。
 * @param loop PID 环类型。
 * @return int32_t 百分比基准值。
 */
static int32_t gimbal_get_output_limit_base(const Gimbal_ControllerConfig_t *config,
                                            enum gimbal_pid_loop loop)
{
    if (config == NULL)
    {
        return 0;
    }

    if (loop == GIMBAL_PID_LOOP_POSITION)
    {
        return config->position_output_limit_base;
    }

    if (loop == GIMBAL_PID_LOOP_SPEED)
    {
        return config->speed_output_limit_base;
    }

    return 0;
}

/**
 * @brief 将 PID 百分比配置换算为内部 PID 配置。
 * @param controller_config 云台控制器配置。
 * @param loop PID 环类型。
 * @param percent_config PID 百分比配置。
 * @param pid_config 输出内部 PID 配置。
 * @return int 0 表示成功，负值表示失败。
 */
static int gimbal_pid_config_from_percent(const Gimbal_ControllerConfig_t *controller_config,
                                          enum gimbal_pid_loop loop,
                                          const struct gimbal_pid_percent_config *percent_config,
                                          Gimbal_PidConfig_t *pid_config)
{
    if ((controller_config == NULL) || (percent_config == NULL) || (pid_config == NULL))
    {
        return -EINVAL;
    }

    if ((loop != GIMBAL_PID_LOOP_POSITION) && (loop != GIMBAL_PID_LOOP_SPEED))
    {
        return -EINVAL;
    }

    pid_config->kp = percent_config->kp;
    pid_config->ki = percent_config->ki;
    pid_config->kd = percent_config->kd;
    pid_config->integral_limit =
        gimbal_percent_to_limit(percent_config->integral_limit_percent,
                                gimbal_get_integral_limit_base(controller_config, loop));
    pid_config->output_limit =
        gimbal_percent_to_limit(percent_config->output_limit_percent,
                                gimbal_get_output_limit_base(controller_config, loop));

    return 0;
}

/**
 * @brief 将内部 PID 配置换算为百分比配置。
 * @param controller_config 云台控制器配置。
 * @param loop PID 环类型。
 * @param pid_config 内部 PID 配置。
 * @param percent_config 输出 PID 百分比配置。
 * @return int 0 表示成功，负值表示失败。
 */
static int gimbal_pid_config_to_percent(const Gimbal_ControllerConfig_t *controller_config,
                                        enum gimbal_pid_loop loop,
                                        const Gimbal_PidConfig_t *pid_config,
                                        struct gimbal_pid_percent_config *percent_config)
{
    if ((controller_config == NULL) || (pid_config == NULL) || (percent_config == NULL))
    {
        return -EINVAL;
    }

    if ((loop != GIMBAL_PID_LOOP_POSITION) && (loop != GIMBAL_PID_LOOP_SPEED))
    {
        return -EINVAL;
    }

    percent_config->kp = pid_config->kp;
    percent_config->ki = pid_config->ki;
    percent_config->kd = pid_config->kd;
    percent_config->integral_limit_percent =
        gimbal_limit_to_percent(pid_config->integral_limit,
                                gimbal_get_integral_limit_base(controller_config, loop));
    percent_config->output_limit_percent =
        gimbal_limit_to_percent(pid_config->output_limit,
                                gimbal_get_output_limit_base(controller_config, loop));

    return 0;
}

/**
 * @brief 执行一次 PID 计算。
 * @param pid PID 对象。
 * @param error 当前误差。
 * @return int32_t PID 输出。
 */
static int32_t gimbal_pid_update(Gimbal_Pid_t *pid, int32_t error)
{
    int32_t derivative = 0;
    int64_t output;

    if (pid == NULL)
    {
        return 0;
    }

    if (pid->config.integral_limit > 0)
    {
        pid->integral = gimbal_clamp_symmetric_i64((int64_t)pid->integral + error,
                                                   pid->config.integral_limit);
    }
    else
    {
        pid->integral = 0;
    }

    if (pid->last_error_valid)
    {
        derivative = error - pid->last_error;
    }

    pid->last_error = error;
    pid->last_error_valid = true;

    output =
        (((int64_t)error * pid->config.kp) +
         ((int64_t)pid->integral * pid->config.ki) +
         ((int64_t)derivative * pid->config.kd)) /
        GIMBAL_PID_GAIN_SCALE;

    return gimbal_clamp_symmetric_i64(output, pid->config.output_limit);
}

/**
 * @brief 将角度误差折算到半圈范围内。
 * @param controller 云台控制器对象。
 * @param error_mrad 原始角度误差，单位 mrad。
 * @return int32_t 折算后的角度误差，单位 mrad。
 */
static int32_t gimbal_wrap_angle_error(const Gimbal_Controller_t *controller, int32_t error_mrad)
{
    int32_t full_circle_mrad;
    int32_t half_circle_mrad;

    if ((controller == NULL) || (controller->config == NULL))
    {
        return error_mrad;
    }

    full_circle_mrad = controller->config->full_circle_mrad;

    if (full_circle_mrad <= 0)
    {
        return error_mrad;
    }

    half_circle_mrad = full_circle_mrad / 2;

    if ((error_mrad >= full_circle_mrad) ||
        (error_mrad <= -full_circle_mrad))
    {
        error_mrad %= full_circle_mrad;
    }

    if (error_mrad > half_circle_mrad)
    {
        error_mrad -= full_circle_mrad;
    }

    if (error_mrad < -half_circle_mrad)
    {
        error_mrad += full_circle_mrad;
    }

    return error_mrad;
}

/**
 * @brief 判断姿态反馈是否新鲜。
 * @param controller 云台控制器对象。
 * @param now_ms 当前时间，单位毫秒。
 * @return bool true 表示姿态反馈可用，false 表示姿态反馈超时。
 */
static bool gimbal_attitude_is_fresh_locked(const Gimbal_Controller_t *controller, uint32_t now_ms)
{
    if ((controller == NULL) || (controller->config == NULL))
    {
        return false;
    }

    if (!controller->attitude.valid)
    {
        return false;
    }

    if ((uint32_t)(now_ms - controller->attitude.update_ms) >
        controller->config->attitude_timeout_ms)
    {
        return false;
    }

    return true;
}

/**
 * @brief 获取单轴姿态位置反馈。
 * @param controller 云台控制器对象。
 * @param axis 轴对象。
 * @param now_ms 当前时间，单位毫秒。
 * @param feedback_mrad 输出位置反馈，单位 mrad。
 * @return bool true 表示反馈有效，false 表示反馈无效。
 */
static bool gimbal_get_attitude_feedback_locked(const Gimbal_Controller_t *controller,
                                                const Gimbal_Axis_t *axis,
                                                uint32_t now_ms,
                                                int32_t *feedback_mrad)
{
    if ((controller == NULL) || (axis == NULL) || (axis->config == NULL) ||
        (feedback_mrad == NULL))
    {
        return false;
    }

    if (!gimbal_attitude_is_fresh_locked(controller, now_ms))
    {
        return false;
    }

    if (axis->config->position_feedback_source == GIMBAL_POSITION_FEEDBACK_ATTITUDE_ROLL)
    {
        *feedback_mrad = controller->attitude.roll_mrad;
        return true;
    }

    if (axis->config->position_feedback_source == GIMBAL_POSITION_FEEDBACK_ATTITUDE_PITCH)
    {
        *feedback_mrad = controller->attitude.pitch_mrad;
        return true;
    }

    if (axis->config->position_feedback_source == GIMBAL_POSITION_FEEDBACK_ATTITUDE_YAW)
    {
        *feedback_mrad = controller->attitude.yaw_mrad;
        return true;
    }

    return false;
}

/**
 * @brief 计算位置环输出的目标速度。
 * @param controller 云台控制器对象。
 * @param axis 轴对象。
 * @param target_mrad 目标位置，单位 mrad。
 * @param feedback_mrad 反馈位置，单位 mrad。
 * @return int32_t 目标速度，单位 mrad/s。
 */
static int32_t gimbal_axis_calc_target_speed(Gimbal_Controller_t *controller,
                                             Gimbal_Axis_t *axis,
                                             int32_t target_mrad,
                                             int32_t feedback_mrad)
{
    int32_t error_mrad;

    if ((controller == NULL) || (axis == NULL))
    {
        return 0;
    }

    error_mrad = gimbal_wrap_angle_error(controller, target_mrad - feedback_mrad);

    return gimbal_pid_update(&axis->position_pid, error_mrad);
}

/**
 * @brief 计算速度环输出的电流目标。
 * @param axis 轴对象。
 * @param target_speed_mrad_s 目标速度，单位 mrad/s。
 * @param feedback_speed_mrad_s 反馈速度，单位 mrad/s。
 * @return int32_t 电流目标控制量。
 */
static int32_t gimbal_axis_calc_current_target(Gimbal_Axis_t *axis,
                                               int32_t target_speed_mrad_s,
                                               int32_t feedback_speed_mrad_s)
{
    int32_t speed_error;

    if (axis == NULL)
    {
        return 0;
    }

    speed_error = target_speed_mrad_s - feedback_speed_mrad_s;

    return gimbal_pid_update(&axis->speed_pid, speed_error);
}

/**
 * @brief 确保指定轴电机处于电流模式。
 * @param axis 轴对象。
 * @return int 0 表示成功，负值表示失败。
 */
static int gimbal_axis_ensure_current_mode(Gimbal_Axis_t *axis)
{
    int ret;

    if (axis->current_mode_requested)
    {
        return 0;
    }

    ret = gen_motor_set_run_mode(axis->motor, true, GEN_MOTOR_CONTROL_MODE_CURRENT);

    if (ret < 0)
    {
        return ret;
    }

    axis->current_mode_requested = true;

    return 0;
}

/**
 * @brief 向指定轴输出零电流并清空积分。
 * @param axis 轴对象。
 * @return void
 */
static void gimbal_axis_output_zero_current(Gimbal_Axis_t *axis)
{
    if (axis == NULL)
    {
        return;
    }

    gimbal_pid_reset(&axis->position_pid);
    gimbal_pid_reset(&axis->speed_pid);
    axis->target_speed_mrad_s = 0;
    axis->target_speed_valid = false;
    axis->last_position_update_ms = 0U;

    if (!axis->current_mode_requested)
    {
        return;
    }

    if (axis->last_current_target == 0)
    {
        return;
    }

    if (gen_motor_set_current_target(axis->motor, 0) == 0)
    {
        axis->last_current_target = 0;
    }
}

/**
 * @brief 清除单轴控制目标和 PID 状态。
 * @param axis 云台单轴对象。
 * @return void
 */
static void gimbal_axis_clear_control_state(Gimbal_Axis_t *axis)
{
    if (axis == NULL)
    {
        return;
    }

    axis->target_mrad = 0;
    axis->target_valid = false;
    axis->target_speed_mrad_s = 0;
    axis->target_speed_valid = false;
    axis->last_position_update_ms = 0U;
    gimbal_pid_reset(&axis->position_pid);
    gimbal_pid_reset(&axis->speed_pid);
}

/**
 * @brief 关闭单轴电机输出并清除输出状态。
 * @param axis 云台单轴对象。
 * @return int 0 表示成功，负值表示失败。
 */
static int gimbal_axis_disable_output(Gimbal_Axis_t *axis)
{
    int first_ret = 0;
    int ret;

    if (axis == NULL)
    {
        return -EINVAL;
    }

    if (axis->motor == NULL)
    {
        return -ENODEV;
    }

    ret = gen_motor_set_current_target(axis->motor, 0);

    if (ret < 0)
    {
        first_ret = ret;
    }
    else
    {
        axis->last_current_target = 0;
    }

    ret = gen_motor_set_run_mode(axis->motor, false, GEN_MOTOR_CONTROL_MODE_CURRENT);

    if ((ret < 0) && (first_ret == 0))
    {
        first_ret = ret;
    }

    if (ret == 0)
    {
        axis->last_current_target = 0;
        axis->current_mode_requested = false;
        axis->target_speed_mrad_s = 0;
        axis->target_speed_valid = false;
        axis->last_position_update_ms = 0U;
    }

    return first_ret;
}

/**
 * @brief 从电机状态获取位置反馈。
 * @param axis 轴对象。
 * @param motor_state 电机状态快照。
 * @param feedback_mrad 输出位置反馈，单位 mrad。
 * @return bool true 表示反馈有效，false 表示反馈无效。
 */
static bool gimbal_axis_get_motor_position_feedback(const Gimbal_Axis_t *axis,
                                                    const struct gen_motor_state *motor_state,
                                                    int32_t *feedback_mrad)
{
    if ((axis == NULL) || (axis->config == NULL) ||
        (motor_state == NULL) || (feedback_mrad == NULL))
    {
        return false;
    }

    if (axis->config->position_feedback_source != GIMBAL_POSITION_FEEDBACK_MOTOR)
    {
        return false;
    }

    if (!motor_state->position_valid)
    {
        return false;
    }

    *feedback_mrad = motor_state->position_mrad;

    return true;
}

/**
 * @brief 获取单轴位置反馈。
 * @param controller 云台控制器对象。
 * @param axis 轴对象。
 * @param motor_state 电机状态快照。
 * @param now_ms 当前时间，单位毫秒。
 * @param feedback_mrad 输出位置反馈，单位 mrad。
 * @return bool true 表示反馈有效，false 表示反馈无效。
 */
static bool gimbal_axis_get_position_feedback(Gimbal_Controller_t *controller,
                                              Gimbal_Axis_t *axis,
                                              const struct gen_motor_state *motor_state,
                                              uint32_t now_ms,
                                              int32_t *feedback_mrad)
{
    if ((controller == NULL) || (axis == NULL) || (axis->config == NULL))
    {
        return false;
    }

    if ((axis->config->position_feedback_source == GIMBAL_POSITION_FEEDBACK_ATTITUDE_ROLL) ||
        (axis->config->position_feedback_source == GIMBAL_POSITION_FEEDBACK_ATTITUDE_PITCH) ||
        (axis->config->position_feedback_source == GIMBAL_POSITION_FEEDBACK_ATTITUDE_YAW))
    {
        return gimbal_get_attitude_feedback_locked(controller,
                                                   axis,
                                                   now_ms,
                                                   feedback_mrad);
    }

    return gimbal_axis_get_motor_position_feedback(axis, motor_state, feedback_mrad);
}

/**
 * @brief 从电机状态获取速度反馈。
 * @param axis 轴对象。
 * @param motor_state 电机状态快照。
 * @param feedback_speed_mrad_s 输出速度反馈，单位 mrad/s。
 * @return bool true 表示反馈有效，false 表示反馈无效。
 */
static bool gimbal_axis_get_motor_speed_feedback(const Gimbal_Axis_t *axis,
                                                 const struct gen_motor_state *motor_state,
                                                 int32_t *feedback_speed_mrad_s)
{
    if ((axis == NULL) || (axis->config == NULL) ||
        (motor_state == NULL) || (feedback_speed_mrad_s == NULL))
    {
        return false;
    }

    if (axis->config->speed_feedback_source != GIMBAL_SPEED_FEEDBACK_MOTOR)
    {
        return false;
    }

    if (!motor_state->speed_valid)
    {
        return false;
    }

    *feedback_speed_mrad_s = motor_state->speed_mrad_s;

    return true;
}

/**
 * @brief 获取单轴 IMU 速度反馈。
 * @param controller 云台控制器对象。
 * @param axis 轴对象。
 * @param now_ms 当前时间，单位毫秒。
 * @param feedback_speed_mrad_s 输出速度反馈，单位 mrad/s。
 * @return bool true 表示反馈有效，false 表示反馈无效。
 */
static bool gimbal_get_imu_speed_feedback_locked(const Gimbal_Controller_t *controller,
                                                 const Gimbal_Axis_t *axis,
                                                 uint32_t now_ms,
                                                 int32_t *feedback_speed_mrad_s)
{
    if ((controller == NULL) || (axis == NULL) || (axis->config == NULL) ||
        (feedback_speed_mrad_s == NULL))
    {
        return false;
    }

    if (!gimbal_attitude_is_fresh_locked(controller, now_ms))
    {
        return false;
    }

    if (!controller->attitude.speed_valid)
    {
        return false;
    }

    if (axis->config->speed_feedback_source == GIMBAL_SPEED_FEEDBACK_IMU_ROLL)
    {
        *feedback_speed_mrad_s = controller->attitude.roll_speed_mrad_s;
        return true;
    }

    if (axis->config->speed_feedback_source == GIMBAL_SPEED_FEEDBACK_IMU_PITCH)
    {
        *feedback_speed_mrad_s = controller->attitude.pitch_speed_mrad_s;
        return true;
    }

    if (axis->config->speed_feedback_source == GIMBAL_SPEED_FEEDBACK_IMU_YAW)
    {
        *feedback_speed_mrad_s = controller->attitude.yaw_speed_mrad_s;
        return true;
    }

    return false;
}

/**
 * @brief 获取单轴速度反馈。
 * @param controller 云台控制器对象。
 * @param axis 轴对象。
 * @param motor_state 电机状态快照。
 * @param now_ms 当前时间，单位毫秒。
 * @param feedback_speed_mrad_s 输出速度反馈，单位 mrad/s。
 * @return bool true 表示反馈有效，false 表示反馈无效。
 */
static bool gimbal_axis_get_speed_feedback(Gimbal_Controller_t *controller,
                                           Gimbal_Axis_t *axis,
                                           const struct gen_motor_state *motor_state,
                                           uint32_t now_ms,
                                           int32_t *feedback_speed_mrad_s)
{
    if ((controller == NULL) || (axis == NULL) || (axis->config == NULL))
    {
        return false;
    }

    if (axis->config->speed_feedback_source == GIMBAL_SPEED_FEEDBACK_MOTOR)
    {
        return gimbal_axis_get_motor_speed_feedback(axis,
                                                    motor_state,
                                                    feedback_speed_mrad_s);
    }

    return gimbal_get_imu_speed_feedback_locked(controller,
                                                axis,
                                                now_ms,
                                                feedback_speed_mrad_s);
}

/**
 * @brief 判断当前是否需要更新位置环。
 * @param controller 云台控制器对象。
 * @param axis 轴对象。
 * @param now_ms 当前时间，单位毫秒。
 * @return bool true 表示需要更新，false 表示暂不更新。
 */
static bool gimbal_axis_position_update_is_due(const Gimbal_Controller_t *controller,
                                               const Gimbal_Axis_t *axis,
                                               uint32_t now_ms)
{
    if ((controller == NULL) || (controller->config == NULL) || (axis == NULL))
    {
        return false;
    }

    if (!axis->target_speed_valid)
    {
        return true;
    }

    if ((now_ms - axis->last_position_update_ms) >=
        controller->config->position_control_period_ms)
    {
        return true;
    }

    return false;
}

/**
 * @brief 执行单轴控制。
 * @param controller 云台控制器对象。
 * @param axis 轴对象。
 * @param now_ms 当前时间，单位毫秒。
 * @return void
 */
static void gimbal_control_axis(Gimbal_Controller_t *controller,
                                Gimbal_Axis_t *axis,
                                uint32_t now_ms)
{
    struct gen_motor_state motor_state;
    int32_t target_mrad;
    int32_t feedback_position_mrad;
    int32_t feedback_speed_mrad_s;
    int32_t target_speed_mrad_s;
    int32_t current_target;
    bool target_valid;
    bool position_feedback_valid;
    bool speed_feedback_valid;
    bool update_position_loop;
    int ret;

    if ((controller == NULL) || (axis == NULL) || (axis->motor == NULL))
    {
        return;
    }

    ret = gen_motor_copy_state(axis->motor, &motor_state);

    if (ret < 0)
    {
        k_mutex_lock(&controller->lock, K_FOREVER);
        gimbal_axis_output_zero_current(axis);
        k_mutex_unlock(&controller->lock);
        return;
    }

    k_mutex_lock(&controller->lock, K_FOREVER);
    target_mrad = axis->target_mrad;
    target_valid = axis->target_valid;
    update_position_loop = gimbal_axis_position_update_is_due(controller, axis, now_ms);

    if (update_position_loop)
    {
        position_feedback_valid = gimbal_axis_get_position_feedback(controller,
                                                                    axis,
                                                                    &motor_state,
                                                                    now_ms,
                                                                    &feedback_position_mrad);
    }
    else
    {
        position_feedback_valid = axis->target_speed_valid;
        feedback_position_mrad = 0;
    }

    speed_feedback_valid = gimbal_axis_get_speed_feedback(controller,
                                                          axis,
                                                          &motor_state,
                                                          now_ms,
                                                          &feedback_speed_mrad_s);

    if ((!target_valid) || (!position_feedback_valid) || (!speed_feedback_valid))
    {
        gimbal_axis_output_zero_current(axis);
        k_mutex_unlock(&controller->lock);
        return;
    }

    ret = gimbal_axis_ensure_current_mode(axis);

    if (ret < 0)
    {
        k_mutex_unlock(&controller->lock);
        return;
    }

    if (update_position_loop)
    {
        target_speed_mrad_s = gimbal_axis_calc_target_speed(controller,
                                                            axis,
                                                            target_mrad,
                                                            feedback_position_mrad);
        axis->target_speed_mrad_s = target_speed_mrad_s;
        axis->target_speed_valid = true;
        axis->last_position_update_ms = now_ms;
    }
    else
    {
        target_speed_mrad_s = axis->target_speed_mrad_s;
    }

    current_target = gimbal_axis_calc_current_target(axis,
                                                     target_speed_mrad_s,
                                                     feedback_speed_mrad_s);

    ret = gen_motor_set_current_target(axis->motor, current_target);

    if (ret == 0)
    {
        axis->last_current_target = current_target;
    }

    k_mutex_unlock(&controller->lock);
}

/**
 * @brief 云台控制线程入口。
 * @param arg1 线程参数 1。
 * @param arg2 线程参数 2。
 * @param arg3 线程参数 3。
 * @return void
 */
static void gimbal_control_thread_entry(void *arg1, void *arg2, void *arg3)
{
    Gimbal_Controller_t *controller = arg1;
    uint32_t now_ms;
    size_t i;

    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    if (controller == NULL)
    {
        return;
    }

    while (1)
    {
        now_ms = k_uptime_get_32();

        for (i = 0U; i < controller->config->axis_count; i++)
        {
            gimbal_control_axis(controller, &controller->axes[i], now_ms);
        }

        k_sleep(K_MSEC(controller->config->speed_control_period_ms));
    }
}

/**
 * @brief 设置云台三轴目标角度。
 * @param controller 云台控制器对象。
 * @param roll_mrad Roll 目标角度，单位 mrad。
 * @param pitch_mrad Pitch 目标角度，单位 mrad。
 * @param yaw_mrad Yaw 目标角度，单位 mrad。
 * @param source 目标来源。
 * @return int 0 表示成功，负值表示失败。
 */
static int gimbal_controller_set_rpy_target(Gimbal_Controller_t *controller,
                                            int32_t roll_mrad,
                                            int32_t pitch_mrad,
                                            int32_t yaw_mrad,
                                            enum gimbal_target_source source)
{
    if (!gimbal_controller_is_ready(controller))
    {
        return -ENODEV;
    }

    k_mutex_lock(&controller->lock, K_FOREVER);
    controller->axes[GIMBAL_AXIS_ROLL].target_mrad = roll_mrad;
    controller->axes[GIMBAL_AXIS_ROLL].target_valid = true;
    controller->axes[GIMBAL_AXIS_PITCH].target_mrad = pitch_mrad;
    controller->axes[GIMBAL_AXIS_PITCH].target_valid = true;
    controller->axes[GIMBAL_AXIS_YAW].target_mrad = yaw_mrad;
    controller->axes[GIMBAL_AXIS_YAW].target_valid = true;
    controller->target_source = source;
    k_mutex_unlock(&controller->lock);

    return 0;
}

/**
 * @brief 设置云台单轴目标角度。
 * @param controller 云台控制器对象。
 * @param axis_id 云台轴编号。
 * @param target_mrad 目标角度，单位 mrad。
 * @param source 目标来源。
 * @return int 0 表示成功，负值表示失败。
 */
static int gimbal_controller_set_axis_target(Gimbal_Controller_t *controller,
                                             enum gimbal_axis_id axis_id,
                                             int32_t target_mrad,
                                             enum gimbal_target_source source)
{
    if (!gimbal_controller_is_ready(controller))
    {
        return -ENODEV;
    }

    if (!gimbal_axis_id_is_valid(axis_id))
    {
        return -EINVAL;
    }

    k_mutex_lock(&controller->lock, K_FOREVER);
    controller->axes[axis_id].target_mrad = target_mrad;
    controller->axes[axis_id].target_valid = true;
    controller->target_source = source;
    k_mutex_unlock(&controller->lock);

    return 0;
}

/**
 * @brief 关闭云台输出并清除控制目标。
 * @param controller 云台控制器对象。
 * @return int 0 表示成功，负值表示失败。
 */
static int gimbal_controller_disable(Gimbal_Controller_t *controller)
{
    int first_ret = 0;
    int ret;
    size_t i;

    if (!gimbal_controller_is_ready(controller))
    {
        return -ENODEV;
    }

    k_mutex_lock(&controller->lock, K_FOREVER);

    for (i = 0U; i < controller->config->axis_count; i++)
    {
        gimbal_axis_clear_control_state(&controller->axes[i]);
    }

    controller->target_source = GIMBAL_TARGET_SOURCE_UNKNOWN;

    for (i = 0U; i < controller->config->axis_count; i++)
    {
        ret = gimbal_axis_disable_output(&controller->axes[i]);

        if ((ret < 0) && (first_ret == 0))
        {
            first_ret = ret;
        }
    }

    k_mutex_unlock(&controller->lock);

    return first_ret;
}

/**
 * @brief 初始化单轴对象。
 * @param controller 云台控制器对象。
 * @param axis_id 轴编号。
 * @return int 0 表示成功，负值表示失败。
 */
static int gimbal_axis_init(Gimbal_Controller_t *controller, enum gimbal_axis_id axis_id)
{
    const Gimbal_AxisConfig_t *axis_config;
    Gimbal_Axis_t *axis;
    const struct gen_motor *motor;
    int ret;

    if ((controller == NULL) || (controller->config == NULL) ||
        (axis_id >= GIMBAL_AXIS_COUNT))
    {
        return -EINVAL;
    }

    axis_config = &controller->config->axis_configs[axis_id];
    axis = &controller->axes[axis_id];
    motor = gen_motor_get_by_index(controller->config->gen_motor_dev,
                                   axis_config->motor_index);

    if (motor == NULL)
    {
        return -ENODEV;
    }

    memset(axis, 0, sizeof(*axis));
    axis->config = axis_config;
    axis->motor = motor;

    ret = gimbal_pid_init(&axis->position_pid, &axis_config->position_pid);

    if (ret < 0)
    {
        return ret;
    }

    ret = gimbal_pid_init(&axis->speed_pid, &axis_config->speed_pid);

    if (ret < 0)
    {
        return ret;
    }

    axis->last_current_target = 0;

    return 0;
}

/**
 * @brief 初始化云台控制器对象。
 * @param controller 云台控制器对象。
 * @param config 云台控制器配置。
 * @return int 0 表示成功，负值表示失败。
 */
static int gimbal_controller_init(Gimbal_Controller_t *controller,
                                  const Gimbal_ControllerConfig_t *config)
{
    size_t i;
    int ret;

    if ((controller == NULL) || (config == NULL) ||
        (config->gen_motor_dev == NULL) ||
        (config->axis_configs == NULL) ||
        (config->axis_count != GIMBAL_AXIS_COUNT))
    {
        return -EINVAL;
    }

    if (controller->initialized)
    {
        return 0;
    }

    if (!device_is_ready(config->gen_motor_dev))
    {
        return -ENODEV;
    }

    memset(controller, 0, sizeof(*controller));
    controller->config = config;
    k_mutex_init(&controller->lock);

    for (i = 0U; i < config->axis_count; i++)
    {
        ret = gimbal_axis_init(controller, (enum gimbal_axis_id)i);

        if (ret < 0)
        {
            return ret;
        }
    }

    return 0;
}

/**
 * @brief 启动云台控制线程。
 * @param controller 云台控制器对象。
 * @return int 0 表示成功，负值表示失败。
 */
static int gimbal_controller_start(Gimbal_Controller_t *controller)
{
    if (controller == NULL)
    {
        return -EINVAL;
    }

    if (controller->thread_started)
    {
        return 0;
    }

    k_thread_create(&controller->control_thread,
                    gimbal_control_stack,
                    K_THREAD_STACK_SIZEOF(gimbal_control_stack),
                    gimbal_control_thread_entry,
                    controller,
                    NULL,
                    NULL,
                    GIMBAL_CONTROL_PRIORITY,
                    0,
                    K_NO_WAIT);

    if (IS_ENABLED(CONFIG_THREAD_NAME))
    {
        k_thread_name_set(&controller->control_thread, "gimbal_ctrl");
    }

    controller->thread_started = true;

    return 0;
}

/**
 * @brief 初始化云台控制模块。
 * @return int 0 表示成功，负值表示失败。
 */
int gimbal_init(void)
{
    Gimbal_Controller_t *controller = &gimbal_controller;
    int ret;

    ret = gimbal_controller_init(controller, &gimbal_default_config);

    if (ret < 0)
    {
        return ret;
    }

    ret = gimbal_controller_start(controller);

    if (ret < 0)
    {
        return ret;
    }

    controller->initialized = true;
    LOG_DBG("gimbal ready: speed_period=%u ms position_period=%u ms",
            (unsigned int)controller->config->speed_control_period_ms,
            (unsigned int)controller->config->position_control_period_ms);

    return 0;
}

/**
 * @brief 设置云台 RPY 目标角度。
 * @param roll_mrad Roll 目标角度，单位 mrad。
 * @param pitch_mrad Pitch 目标角度，单位 mrad。
 * @param yaw_mrad Yaw 目标角度，单位 mrad。
 * @param source 目标来源。
 * @return int 0 表示成功，负值表示失败。
 */
int gimbal_set_rpy_target(int32_t roll_mrad,
                          int32_t pitch_mrad,
                          int32_t yaw_mrad,
                          enum gimbal_target_source source)
{
    Gimbal_Controller_t *controller = &gimbal_controller;

    return gimbal_controller_set_rpy_target(controller,
                                            roll_mrad,
                                            pitch_mrad,
                                            yaw_mrad,
                                            source);
}

/**
 * @brief 设置云台单轴目标角度。
 * @param axis_id 云台轴编号。
 * @param target_mrad 目标角度，单位 mrad。
 * @param source 目标来源。
 * @return int 0 表示成功，负值表示失败。
 */
int gimbal_set_axis_target(enum gimbal_axis_id axis_id,
                           int32_t target_mrad,
                           enum gimbal_target_source source)
{
    Gimbal_Controller_t *controller = &gimbal_controller;

    return gimbal_controller_set_axis_target(controller,
                                             axis_id,
                                             target_mrad,
                                             source);
}

/**
 * @brief 关闭云台输出并清除控制目标和 PID 状态。
 * @return int 0 表示成功，负值表示失败。
 */
int gimbal_disable(void)
{
    Gimbal_Controller_t *controller = &gimbal_controller;

    return gimbal_controller_disable(controller);
}

/**
 * @brief 设置指定轴指定环的 PID 百分比配置。
 * @param axis_id 云台轴编号。
 * @param loop PID 环类型。
 * @param config PID 百分比配置。
 * @return int 0 表示成功，负值表示失败。
 */
int gimbal_set_pid_percent_config(enum gimbal_axis_id axis_id,
                                  enum gimbal_pid_loop loop,
                                  const struct gimbal_pid_percent_config *config)
{
    Gimbal_Controller_t *controller = &gimbal_controller;
    Gimbal_PidConfig_t pid_config;
    Gimbal_Pid_t *pid;
    int ret;

    if (!gimbal_controller_is_ready(controller))
    {
        return -ENODEV;
    }

    if ((!gimbal_axis_id_is_valid(axis_id)) || (config == NULL))
    {
        return -EINVAL;
    }

    ret = gimbal_pid_config_from_percent(controller->config, loop, config, &pid_config);

    if (ret < 0)
    {
        return ret;
    }

    k_mutex_lock(&controller->lock, K_FOREVER);
    pid = gimbal_axis_get_pid(&controller->axes[axis_id], loop);

    if (pid == NULL)
    {
        k_mutex_unlock(&controller->lock);
        return -EINVAL;
    }

    pid->config = pid_config;
    gimbal_pid_reset(pid);
    k_mutex_unlock(&controller->lock);

    return 0;
}

/**
 * @brief 获取指定轴指定环的 PID 百分比配置。
 * @param axis_id 云台轴编号。
 * @param loop PID 环类型。
 * @param config 输出 PID 百分比配置。
 * @return int 0 表示成功，负值表示失败。
 */
int gimbal_get_pid_percent_config(enum gimbal_axis_id axis_id,
                                  enum gimbal_pid_loop loop,
                                  struct gimbal_pid_percent_config *config)
{
    Gimbal_Controller_t *controller = &gimbal_controller;
    Gimbal_PidConfig_t pid_config;
    Gimbal_Pid_t *pid;
    int ret;

    if (!gimbal_controller_is_ready(controller))
    {
        return -ENODEV;
    }

    if ((!gimbal_axis_id_is_valid(axis_id)) || (config == NULL))
    {
        return -EINVAL;
    }

    k_mutex_lock(&controller->lock, K_FOREVER);
    pid = gimbal_axis_get_pid(&controller->axes[axis_id], loop);

    if (pid == NULL)
    {
        k_mutex_unlock(&controller->lock);
        return -EINVAL;
    }

    pid_config = pid->config;
    k_mutex_unlock(&controller->lock);

    ret = gimbal_pid_config_to_percent(controller->config, loop, &pid_config, config);

    if (ret < 0)
    {
        return ret;
    }

    return 0;
}

/**
 * @brief 设置 RK 下发的当前姿态角。
 * @param roll_mrad 当前 Roll 姿态角，单位 mrad。
 * @param pitch_mrad 当前 Pitch 姿态角，单位 mrad。
 * @param yaw_mrad 当前 Yaw 姿态角，单位 mrad。
 * @return int 0 表示成功，负值表示失败。
 * @note Roll 和 Pitch 位置环使用该姿态反馈；Yaw 位置环使用电机角度反馈。
 */
int gimbal_set_attitude_feedback(int32_t roll_mrad,
                                 int32_t pitch_mrad,
                                 int32_t yaw_mrad)
{
    Gimbal_Controller_t *controller = &gimbal_controller;

    if (!gimbal_controller_is_ready(controller))
    {
        return -ENODEV;
    }

    k_mutex_lock(&controller->lock, K_FOREVER);
    controller->attitude.roll_mrad = roll_mrad;
    controller->attitude.pitch_mrad = pitch_mrad;
    controller->attitude.yaw_mrad = yaw_mrad;
    controller->attitude.update_ms = k_uptime_get_32();
    controller->attitude.valid = true;
    controller->attitude.speed_valid = false;
    k_mutex_unlock(&controller->lock);

    return 0;
}

/**
 * @brief 设置 IMU 姿态角和角速度反馈。
 * @param roll_mrad 当前 Roll 姿态角，单位 mrad。
 * @param pitch_mrad 当前 Pitch 姿态角，单位 mrad。
 * @param yaw_mrad 当前 Yaw 姿态角，单位 mrad。
 * @param roll_speed_mrad_s 当前 Roll 角速度，单位 mrad/s。
 * @param pitch_speed_mrad_s 当前 Pitch 角速度，单位 mrad/s。
 * @param yaw_speed_mrad_s 当前 Yaw 角速度，单位 mrad/s。
 * @return int 0 表示成功，负值表示失败。
 */
int gimbal_set_imu_feedback(int32_t roll_mrad,
                            int32_t pitch_mrad,
                            int32_t yaw_mrad,
                            int32_t roll_speed_mrad_s,
                            int32_t pitch_speed_mrad_s,
                            int32_t yaw_speed_mrad_s)
{
    Gimbal_Controller_t *controller = &gimbal_controller;

    if (!gimbal_controller_is_ready(controller))
    {
        return -ENODEV;
    }

    k_mutex_lock(&controller->lock, K_FOREVER);
    controller->attitude.roll_mrad = roll_mrad;
    controller->attitude.pitch_mrad = pitch_mrad;
    controller->attitude.yaw_mrad = yaw_mrad;
    controller->attitude.roll_speed_mrad_s = roll_speed_mrad_s;
    controller->attitude.pitch_speed_mrad_s = pitch_speed_mrad_s;
    controller->attitude.yaw_speed_mrad_s = yaw_speed_mrad_s;
    controller->attitude.update_ms = k_uptime_get_32();
    controller->attitude.valid = true;
    controller->attitude.speed_valid = true;
    k_mutex_unlock(&controller->lock);

    return 0;
}
