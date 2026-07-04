/**
 * @file motor.h
 * @brief Duck Mid 电机应用管理接口。
 */
#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化电机应用管理模块。
 * @return int 0 表示成功，负值表示失败。
 */
int motor_init(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
