/**
 * @file gimbal.h
 * @brief 云台控制模块接口。
 */
#ifndef GIMBAL_H_
#define GIMBAL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief 云台控制轴编号。
 */
enum gimbal_axis_id
{
    /**< Roll 轴。 */
    GIMBAL_AXIS_ROLL = 0,
    /**< Pitch 轴。 */
    GIMBAL_AXIS_PITCH,
    /**< Yaw 轴。 */
    GIMBAL_AXIS_YAW,
    /**< 云台轴数量。 */
    GIMBAL_AXIS_COUNT,
};

/**
 * @brief 云台目标来源。
 */
enum gimbal_target_source
{
    /**< 未知来源。 */
    GIMBAL_TARGET_SOURCE_UNKNOWN = 0,
    /**< RK 串口来源。 */
    GIMBAL_TARGET_SOURCE_RK_UART,
    /**< 外部业务接口来源。 */
    GIMBAL_TARGET_SOURCE_EXTERNAL,
};

/**
 * @brief 云台 PID 环类型。
 */
enum gimbal_pid_loop
{
    /**< 位置环。 */
    GIMBAL_PID_LOOP_POSITION = 0,
    /**< 速度环。 */
    GIMBAL_PID_LOOP_SPEED,
};

/**
 * @brief 云台 PID 百分比配置。
 */
struct gimbal_pid_percent_config
{
    /**< 比例增益，按控制器内部缩放系数配置。 */
    int32_t kp;
    /**< 积分增益，按控制器内部缩放系数配置。 */
    int32_t ki;
    /**< 微分增益，按控制器内部缩放系数配置。 */
    int32_t kd;
    /**< 积分限幅百分比，0 表示关闭积分。 */
    uint32_t integral_limit_percent;
    /**< 输出限幅百分比，0 表示不限制输出。 */
    uint32_t output_limit_percent;
};

/**
 * @brief 初始化云台控制模块。
 * @return int 0 表示成功，负值表示失败。
 */
int gimbal_init(void);

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
                          enum gimbal_target_source source);

/**
 * @brief 设置云台单轴目标角度。
 * @param axis_id 云台轴编号。
 * @param target_mrad 目标角度，单位 mrad。
 * @param source 目标来源。
 * @return int 0 表示成功，负值表示失败。
 */
int gimbal_set_axis_target(enum gimbal_axis_id axis_id,
                           int32_t target_mrad,
                           enum gimbal_target_source source);

/**
 * @brief 关闭云台输出并清除控制目标和 PID 状态。
 * @return int 0 表示成功，负值表示失败。
 */
int gimbal_disable(void);

/**
 * @brief 设置指定轴指定环的 PID 百分比配置。
 * @param axis_id 云台轴编号。
 * @param loop PID 环类型。
 * @param config PID 百分比配置。
 * @return int 0 表示成功，负值表示失败。
 */
int gimbal_set_pid_percent_config(enum gimbal_axis_id axis_id,
                                  enum gimbal_pid_loop loop,
                                  const struct gimbal_pid_percent_config *config);

/**
 * @brief 获取指定轴指定环的 PID 百分比配置。
 * @param axis_id 云台轴编号。
 * @param loop PID 环类型。
 * @param config 输出 PID 百分比配置。
 * @return int 0 表示成功，负值表示失败。
 */
int gimbal_get_pid_percent_config(enum gimbal_axis_id axis_id,
                                  enum gimbal_pid_loop loop,
                                  struct gimbal_pid_percent_config *config);

/**
 * @brief 设置 RK 下发的当前姿态角。
 * @param roll_mrad 当前 Roll 姿态角，单位 mrad。
 * @param pitch_mrad 当前 Pitch 姿态角，单位 mrad。
 * @param yaw_mrad 当前 Yaw 姿态角，单位 mrad。
 * @return int 0 表示成功，负值表示失败。
 * @note Roll 和 Pitch 位置环使用该姿态反馈；Yaw 位置环使用电机角度反馈。
 * @note 该接口不更新角速度反馈，速度源配置为 IMU 时请使用 gimbal_set_imu_feedback。
 */
int gimbal_set_attitude_feedback(int32_t roll_mrad,
                                 int32_t pitch_mrad,
                                 int32_t yaw_mrad);

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
                            int32_t yaw_speed_mrad_s);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_H_ */
