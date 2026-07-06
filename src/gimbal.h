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
 * @brief 设置 RK 下发的当前姿态角。
 * @param roll_mrad 当前 Roll 姿态角，单位 mrad。
 * @param pitch_mrad 当前 Pitch 姿态角，单位 mrad。
 * @param yaw_mrad 当前 Yaw 姿态角，单位 mrad。
 * @return int 0 表示成功，负值表示失败。
 * @note Roll 和 Pitch 位置环使用该姿态反馈；Yaw 位置环使用电机角度反馈。
 */
int gimbal_set_attitude_feedback(int32_t roll_mrad,
                                 int32_t pitch_mrad,
                                 int32_t yaw_mrad);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_H_ */
